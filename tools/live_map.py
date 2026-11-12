#!/usr/bin/env python3
"""Živý 360° webový radar ETA Raggio přes CP2102 na Raspberry Pi 4.

Pouze čte sériová data. Neposílá žádné příkazy LiDARu ani neřídí motor.
Web běží samostatně na portu 8765, bez vazby na služby HanzHubu.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import struct
import threading
import time
from urllib.parse import urlsplit

from decode_lidar import crc8, FRAME_LEN, HEADER, POINTS_PER_FRAME

WEB_FILE = Path(__file__).resolve().parent.parent / "web" / "index.html"
BIN_COUNT = 720  # Rozlišení mapy: 0,5 stupně.
POINT_TTL = 1.8  # Staré body nezůstávají jako falešné překážky.


class ScanState:
    def __init__(self):
        self.lock = threading.Lock()
        self.buffer = bytearray()
        self.bins = [(0, 0, 0.0)] * BIN_COUNT
        self.frames = 0
        self.bad_crc = 0
        self.total_bytes = 0
        self.last_frame = 0.0
        self.rpm = 0.0
        self.connected = False
        self.error = "Čekám na připojení LiDARu."

    def set_connection(self, connected, error=""):
        with self.lock:
            self.connected = connected
            self.error = error

    def feed(self, data):
        # Volá pouze jedno čtecí vlákno.
        self.buffer.extend(data)
        with self.lock:
            self.total_bytes += len(data)
        while True:
            pos = self.buffer.find(HEADER)
            if pos < 0:
                self.buffer[:] = b"\x54" if self.buffer.endswith(b"\x54") else b""
                return
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < FRAME_LEN:
                return
            frame = bytes(self.buffer[:FRAME_LEN])
            if crc8(frame[:-1]) != frame[-1]:
                with self.lock:
                    self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:FRAME_LEN]

            speed, start = struct.unpack_from("<HH", frame, 2)
            end, _timestamp = struct.unpack_from("<HH", frame, 42)
            span = (end - start) % 36000
            now = time.monotonic()
            with self.lock:
                self.frames += 1
                self.last_frame = now
                self.rpm = speed / 6.0
                for i in range(POINTS_PER_FRAME):
                    distance, intensity = struct.unpack_from("<HB", frame, 6 + i * 3)
                    angle_hundredths = (start + span * i / (POINTS_PER_FRAME - 1)) % 36000
                    bin_index = round(angle_hundredths / 50.0) % BIN_COUNT
                    # I nulový odraz aktualizuje paprsek, aby nezůstal starý bod.
                    self.bins[bin_index] = (distance, intensity, now)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            points = [
                [i / 2.0, distance, intensity]
                for i, (distance, intensity, stamp) in enumerate(self.bins)
                if distance > 0 and now - stamp <= POINT_TTL
            ]
            streaming = self.connected and now - self.last_frame < 2.5
            return {
                "streaming": streaming,
                "connected": self.connected,
                "error": self.error,
                "rpm": round(self.rpm, 1) if streaming else 0,
                "frames": self.frames,
                "bad_crc": self.bad_crc,
                "bytes": self.total_bytes,
                "points": points,
                "nearest_mm": min((p[1] for p in points), default=0),
            }


def serial_reader(state, device, baud):
    try:
        import serial
    except ImportError:
        state.set_connection(False, "Chybí pyserial: sudo apt install python3-serial")
        return
    while True:
        try:
            with serial.Serial(device, baudrate=baud, timeout=0.2,
                               xonxoff=False, rtscts=False, dsrdtr=False) as uart:
                state.set_connection(True, "")
                print(f"[UART] Připojeno: {device}, {baud} baud", flush=True)
                while True:
                    data = uart.read(4096)
                    if data:
                        state.feed(data)
        except (serial.SerialException, OSError) as exc:
            state.set_connection(False, f"UART: {exc}")
            print(f"[UART] {exc}; opakuji za 2 s", flush=True)
            time.sleep(2)


def replay_reader(state, source):
    """Offline náhled, nikoli emulace skutečných časových charakteristik snímače."""
    try:
        data = source.read_bytes()
        if not data:
            raise ValueError("Prázdný soubor")
    except (OSError, ValueError) as exc:
        state.set_connection(False, f"Replay: {exc}")
        return
    state.set_connection(True, "")
    print(f"[REPLAY] {source} ({len(data)} B)", flush=True)
    while True:
        for i in range(0, len(data), 512):
            state.feed(data[i:i + 512])
            time.sleep(0.05)


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                try:
                    payload = WEB_FILE.read_bytes()
                except OSError as exc:
                    return self.respond(500, str(exc).encode(), "text/plain; charset=utf-8")
                return self.respond(200, payload, "text/html; charset=utf-8")
            if path == "/api/scan":
                payload = json.dumps(state.snapshot(), separators=(",", ":")).encode()
                return self.respond(200, payload, "application/json; charset=utf-8")
            return self.respond(404, b"Not found", "text/plain; charset=utf-8")

        def respond(self, code, payload, content_type):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, fmt, *args):
            pass  # Žádné logování při každém webovém dotazu.

    return Handler


def main():
    parser = argparse.ArgumentParser(description="Živá 360° webová mapa LiDARu ETA Raggio")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="CP2102 (/dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="0.0.0.0", help="Web: 0.0.0.0 pro LAN nebo 127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--replay", type=Path, help="Volitelný offline .bin soubor namísto UART")
    args = parser.parse_args()
    if args.baud <= 0 or not 1 <= args.http_port <= 65535:
        parser.error("Neplatný baudrate nebo HTTP port")

    state = ScanState()
    if args.replay:
        target, target_args = replay_reader, (state, args.replay)
    else:
        target, target_args = serial_reader, (state, args.port, args.baud)
    threading.Thread(target=target, args=target_args, daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(state))
    print(f"Webová mapa: http://<IP_RPI4>:{args.http_port} (lokální síť)", flush=True)
    print("Ctrl+C ukončí. UART je pouze pro čtení; TX příkazy se neposílají.", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
