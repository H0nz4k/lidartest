#!/usr/bin/env python3
"""Webový 2D SLAM ETA Raggio na Raspberry Pi 4, samostatně od HUBu.

HTTP 8766: / (SLAM), /radar (stávající radar), /api/slam,
           /api/scan, /api/map.pgm, /api/trajectory.csv, POST /api/reset
Čte /dev/ttyUSB0 pasivně. Do LiDARu nepíše.
"""
import argparse
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
from queue import Empty, Full, Queue
import struct
import threading
import time
from urllib.parse import urlsplit

from decode_lidar import HEADER, FRAME_LEN, POINTS_PER_FRAME, crc8
from live_map import ScanState
from slam_core import Slam2D
from imu_fusion import ImuAngleTracker, imu_reader

ROOT = Path(__file__).resolve().parent.parent
PAGE_SLAM = ROOT / "web" / "slam.html"
PAGE_RADAR = ROOT / "web" / "index.html"


class RevolutionAssembler:
    """CRC-validní LDROBOT rámce -> úplné 360° skeny (min. 80 bodů)."""

    def __init__(self):
        self.buffer = bytearray()
        self.prev_start = None
        self.current = []
        self.frames = 0
        self.bad_crc = 0
        self.rpm = 0.0

    def feed(self, data):
        scans = []
        self.buffer.extend(data)
        while True:
            pos = self.buffer.find(HEADER)
            if pos < 0:
                self.buffer[:] = b"\x54" if self.buffer.endswith(b"\x54") else b""
                return scans
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < FRAME_LEN:
                return scans
            frame = bytes(self.buffer[:FRAME_LEN])
            if crc8(frame[:-1]) != frame[-1]:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:FRAME_LEN]

            self.frames += 1
            speed, start = struct.unpack_from("<HH", frame, 2)
            end, _timestamp = struct.unpack_from("<HH", frame, 42)
            self.rpm = speed / 6.0

            if self.prev_start is not None and start < self.prev_start - 15000:
                if len(self.current) >= 80:
                    scans.append(self.current)
                self.current = []
            self.prev_start = start

            span = (end - start) % 36000
            for idx in range(POINTS_PER_FRAME):
                distance, intensity = struct.unpack_from("<HB", frame, 6 + idx * 3)
                angle = ((start + span * idx / (POINTS_PER_FRAME - 1)) % 36000) / 100.0
                self.current.append((angle, distance, intensity))
            if len(self.current) > 550:
                # Při ztrátě synchronizace neakumulovat nekonečný sken.
                self.current.clear()


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.online = False
        self.last_error = "Čekám na CP2102."
        self.last_data = 0.0

    def set(self, online, error=""):
        with self.lock:
            self.online = online
            self.last_error = error
            if online:
                self.last_data = time.monotonic()

    def snapshot(self):
        with self.lock:
            age = time.monotonic() - self.last_data
            return {
                "online": self.online and age < 3,
                "error": self.last_error,
            }


def uart_reader(port, baud, scan_q, radar, telemetry, imu=None, gyro_sign=1):
    try:
        import serial
    except ImportError:
        telemetry.set(False, "Chybí pyserial: sudo apt install python3-serial")
        return
    assembler = RevolutionAssembler()
    last_enqueued = 0.0

    while True:
        try:
            with serial.Serial(port, baudrate=baud, timeout=0.2,
                               xonxoff=False, rtscts=False, dsrdtr=False) as device:
                telemetry.set(True, "")
                radar.set_connection(True, "")
                print(f"[UART] {port} @ {baud} baud", flush=True)
                assembler.buffer.clear()
                assembler.current.clear()
                assembler.prev_start = None
                while True:
                    data = device.read(2048)
                    if not data:
                        continue
                    telemetry.set(True, "")
                    radar.feed(data)
                    completed = assembler.feed(data)
                    for scan in completed:
                        now = time.monotonic()
                        if now - last_enqueued < 0.24:
                            continue
                        last_enqueued = now
                        # Gyro odečítat při dokončení skenu, nikoli až po
                        # čekání ve frontě na pomalejší scan matcher.
                        imu_data = imu.snapshot() if imu is not None else None
                        heading = (gyro_sign * imu_data["heading_deg"]
                                   if imu_data and imu_data["ready"] else None)
                        item = (scan, heading)
                        try:
                            scan_q.put_nowait(item)
                        except Full:
                            try:
                                scan_q.get_nowait()
                            except Empty:
                                pass
                            scan_q.put_nowait(item)
        except (serial.SerialException, OSError) as exc:
            message = f"UART: {exc}"
            telemetry.set(False, message)
            radar.set_connection(False, message)
            print(f"[UART] {message}; znovu za 2 s", flush=True)
            time.sleep(2)


def mapping_worker(scan_q, slam):
    while True:
        scan, imu_heading = scan_q.get()
        try:
            slam.process_scan(scan, imu_heading_deg=imu_heading)
        except Exception as exc:
            # Zabránit tichému konci vlákna a zachovat sériový odposlech.
            print(f"[SLAM] Chyba mapování: {exc}", flush=True)
        finally:
            scan_q.task_done()


def handler_for(slam, radar, telemetry, imu=None):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, code, content, mime, filename=None):
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if filename:
                self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            route = urlsplit(self.path).path
            try:
                if route in ("/", "/slam"):
                    return self.respond(200, PAGE_SLAM.read_bytes(), "text/html; charset=utf-8")
                if route == "/radar":
                    return self.respond(200, PAGE_RADAR.read_bytes(), "text/html; charset=utf-8")
                if route == "/api/scan":
                    return self.respond(200, json.dumps(radar.snapshot()).encode(), "application/json")
                if route == "/api/slam":
                    state = slam.snapshot()
                    scan_status = radar.snapshot()
                    imu_data = imu.snapshot() if imu is not None else {
                        "ready": False, "heading_deg": None, "rate_dps": None,
                        "error": "IMU vypnutá (spusť s --imu)."
                    }
                    state.update({
                        "imu": imu_data,
                        "rpm": scan_status["rpm"],
                        "streaming": scan_status["streaming"],
                        "online": telemetry.snapshot()["online"],
                        "error": telemetry.snapshot()["error"],
                        "frames": scan_status["frames"],
                        "crc_errors": scan_status["bad_crc"],
                    })
                    return self.respond(200, json.dumps(state, separators=(",", ":")).encode(),
                                        "application/json")
                if route == "/api/map.pgm":
                    return self.respond(200, slam.pgm(), "image/x-portable-graymap", "lidartest-map.pgm")
                if route == "/api/trajectory.csv":
                    return self.respond(200, slam.trajectory_csv().encode(),
                                        "text/csv; charset=utf-8", "lidartest-path.csv")
                return self.respond(404, b"Not found", "text/plain")
            except (OSError, ValueError) as exc:
                return self.respond(500, str(exc).encode(), "text/plain; charset=utf-8")

        def do_POST(self):
            if urlsplit(self.path).path != "/api/reset":
                return self.respond(404, b"Not found", "text/plain")
            origin = self.headers.get("Origin")
            host = self.headers.get("Host", "")
            if origin is not None and origin != f"http://{host}":
                return self.respond(403, b"Origin not allowed", "text/plain")
            if self.headers.get("Content-Length", "0") != "0":
                return self.respond(400, b"Expected empty request", "text/plain")
            slam.reset()  # Jen vnitřní mapa; žádný zápis do LiDARu.
            return self.respond(200, b'{"ok":true}', "application/json")

        def log_message(self, _fmt, *_args):
            pass

    return Handler


def main():
    p = argparse.ArgumentParser(description="ETA Raggio 2D SLAM a živý 360° radar")
    p.add_argument("--port", default="/dev/ttyUSB0", help="CP2102")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--http-port", type=int, default=8766)
    p.add_argument("--imu", action="store_true", help="Aktivovat MPU6050 na I²C-1")
    p.add_argument("--imu-address", type=lambda x: int(x, 0), default=0x68,
                   choices=(0x68, 0x69))
    p.add_argument("--imu-yaw-sign", type=int, choices=(-1, 1), default=1,
                   help="Znaménko gyra Z vůči LiDAR souřadnicím (+1/-1)")
    args = p.parse_args()
    if args.baud <= 0 or not 1 <= args.http_port <= 65535:
        p.error("Neplatný baudrate či HTTP port")

    try:
        import numpy  # noqa: F401 - ověření závislosti před spuštěním
        import serial  # noqa: F401
    except ImportError as exc:
        p.error(f"Chybí závislost: {exc}. sudo apt install python3-numpy python3-serial")

    slam = Slam2D()
    radar = ScanState()
    telemetry = Telemetry()
    scan_q = Queue(maxsize=2)
    imu = ImuAngleTracker() if args.imu else None
    if imu is not None:
        threading.Thread(target=imu_reader,
                         args=(imu, 1, args.imu_address),
                         daemon=True).start()
    threading.Thread(target=uart_reader,
                     args=(args.port, args.baud, scan_q, radar, telemetry,
                           imu, args.imu_yaw_sign),
                     daemon=True).start()
    threading.Thread(target=mapping_worker, args=(scan_q, slam), daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.http_port),
                                 handler_for(slam, radar, telemetry, imu))
    print(f"[SLAM] http://<IP_RPI4>:{args.http_port}/", flush=True)
    print(f"[RADAR] http://<IP_RPI4>:{args.http_port}/radar", flush=True)
    print(f"[IMU] {'zapnuto' if args.imu else 'vypnuto'}"
          f" (gyro Z sign: {args.imu_yaw_sign:+d}); čtení MPU nikdy neposílá"
          " příkazy do LiDARu.", flush=True)
    print("Pouze lokální síť. Ctrl+C ukončí. Do LiDARu neposíláme příkazy.", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
