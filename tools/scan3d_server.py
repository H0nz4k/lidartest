#!/usr/bin/env python3
"""Stacionární 3D skener ETA Raggio + MPU-6050, bez ROS a bez HUBu.

Pro pevný mechanický pivot a jedinou osu náklonu. Čte UART pasivně.
Pouze ONE proces smí číst /dev/ttyUSB0; 2D slam_server.py musí být
před spuštěním tohoto 3D serveru ukončený. Kamera není požadována.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

from cloud3d import StationaryCloud3D
from imu_fusion import ImuAngleTracker, imu_reader
from live_map import ScanState
from slam_server import RevolutionAssembler

PAGE = Path(__file__).resolve().parent.parent / "web" / "cloud3d.html"


class Status:
    def __init__(self):
        self.lock = threading.Lock()
        self.live_at = 0.0
        self.error = "Čekám na LiDAR."
        self.frames = 0
        self.crc_errors = 0
        self.rpm = 0.0
        self.revolutions = 0
        self.last_imu_offset_ms = None

    def on_scan(self, asm, t, sample=None):
        with self.lock:
            self.live_at = t
            self.error = ""
            self.frames = asm.frames
            self.crc_errors = asm.bad_crc
            self.rpm = round(asm.rpm, 1)
            self.revolutions += 1
            self.last_imu_offset_ms = (
                round(sample["sync_ms"], 1) if sample else None
            )

    def set_error(self, error):
        with self.lock:
            self.error = str(error)
            self.live_at = 0

    def snapshot(self):
        with self.lock:
            return {
                "online": self.live_at > 0 and time.monotonic() - self.live_at < 3,
                "error": self.error,
                "frames": self.frames,
                "crc_errors": self.crc_errors,
                "rpm": self.rpm,
                "revolutions": self.revolutions,
                "imu_sync_ms": self.last_imu_offset_ms,
            }


def lidar_worker(port, baud, cloud, imu, status, radar):
    try:
        import serial
    except ImportError:
        status.set_error("Chybí python3-serial.")
        return

    assembler = RevolutionAssembler()
    while True:
        try:
            with serial.Serial(port, baudrate=baud, timeout=0.05,
                               xonxoff=False, rtscts=False, dsrdtr=False) as device:
                assembler.buffer.clear()
                assembler.current.clear()
                assembler.prev_start = None
                radar.set_connection(True, "")
                print(f"[3D] LiDAR {port} @ {baud}. Otáčej pouze náklonem základny.", flush=True)
                while True:
                    data = device.read(256)
                    if not data:
                        continue
                    stamp = time.monotonic()
                    radar.feed(data)
                    for points, end_time in assembler.feed_events(data, received_at=stamp):
                        # Průměrná orientace během otáčky lépe odpovídá
                        # středu skenu než okamžiku posledního bodu.
                        rpm = assembler.rpm
                        mid_time = end_time - (30.0 / rpm if rpm > 60 else 0.05)
                        sample = imu.sample_at(mid_time, max_age=0.15)
                        status.on_scan(assembler, end_time, sample)
                        cloud.add_scan(points, sample)
        except (serial.SerialException, OSError) as exc:
            status.set_error(f"UART: {exc}")
            radar.set_connection(False, str(exc))
            print(f"[3D] {exc}; další pokus za 2 sekundy.", flush=True)
            time.sleep(2)


def http_handler(cloud, imu, status, radar):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, code, data, mime="application/json", filename=None):
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if filename:
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{filename}"')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path in ("/", "/3d"):
                return self.respond(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            if path == "/api/points":
                result = cloud.snapshot()
                result.update({"imu": imu.snapshot(), "lidar": status.snapshot()})
                return self.respond(200, json.dumps(result,
                                     separators=(",", ":")).encode())
            if path == "/api/points.ply":
                return self.respond(200, cloud.ply(), "application/octet-stream",
                                    "lidartest-stationary3d.ply")
            if path == "/api/radar":
                return self.respond(200, json.dumps(radar.snapshot()).encode())
            return self.respond(404, b"Not found", "text/plain")

        def do_POST(self):
            if urlsplit(self.path).path != "/api/reset":
                return self.respond(404, b"Not found", "text/plain")
            origin = self.headers.get("Origin")
            host = self.headers.get("Host", "")
            if origin is not None and origin != f"http://{host}":
                return self.respond(403, b"Origin not allowed", "text/plain")
            if self.headers.get("Content-Length", "0") != "0":
                return self.respond(400, b"Expected empty body", "text/plain")
            cloud.reset()
            return self.respond(200, b'{"ok":true}')

        def log_message(self, _fmt, *_args):
            return

    return Handler


def main():
    p = argparse.ArgumentParser(description="ETA Raggio: fixed-pivot 3D tilt scanner")
    p.add_argument("--port", default="/dev/ttyUSB0", help="CP2102 USB-UART")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--http-port", type=int, default=8767)
    p.add_argument("--tilt-axis", choices=("x", "y"), default="y",
                   help="x=roll MPU, y=pitch MPU; zvol podle skutečné mechaniky")
    p.add_argument("--tilt-sign", type=int, choices=(-1, 1), default=1)
    p.add_argument("--tilt-step-deg", type=float, default=2.0)
    p.add_argument("--max-points", type=int, default=160000)
    p.add_argument("--pivot-offset-x", type=float, default=0.0,
                   help="Vzdálenost osy pivotu od středu LiDARu v metrech")
    p.add_argument("--pivot-offset-y", type=float, default=0.0)
    p.add_argument("--pivot-offset-z", type=float, default=0.0)
    args = p.parse_args()
    if args.baud <= 0 or not 1 <= args.http_port <= 65535:
        p.error("invalid baud or HTTP port")

    try:
        import serial  # noqa F401
        from smbus import SMBus  # noqa F401
    except ImportError as exc:
        p.error(f"Chybí závislost: {exc}. sudo apt install python3-serial python3-smbus")
    try:
        cloud = StationaryCloud3D(
            axis=args.tilt_axis, tilt_sign=args.tilt_sign,
            tilt_step_deg=args.tilt_step_deg,
            max_points=args.max_points,
            lever=(args.pivot_offset_x, args.pivot_offset_y, args.pivot_offset_z),
        )
    except ValueError as exc:
        p.error(str(exc))
    imu = ImuAngleTracker()
    status, radar = Status(), ScanState()

    threading.Thread(target=imu_reader, args=(imu,), daemon=True).start()
    threading.Thread(target=lidar_worker,
                     args=(args.port, args.baud, cloud, imu, status, radar),
                     daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.http_port),
                                 http_handler(cloud, imu, status, radar))
    print(f"[3D] http://<IP_RPI4>:{args.http_port}/", flush=True)
    print(f"[3D] Osa {args.tilt_axis}, znaménko {args.tilt_sign:+d}, "
          f"krok {args.tilt_step_deg}°. Základna musí stát na místě.", flush=True)
    print("[3D] Nehýbej MPU 2,5 sekundy při startu. "
          "Potom pomalu a pravidelně nakláněj konstrukci.", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
