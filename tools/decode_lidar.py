#!/usr/bin/env python3
"""Dekodér 47b rámců LiDARu LD00/LD06/LD19 (0x54 0x2C).

Podporuje živý pasivní odposlech CP2102 i offline .bin dump.
NIKDY neodesílá data ani povely k motoru.
"""
import argparse
import csv
from pathlib import Path
import struct
import sys
import time

HEADER = b"\x54\x2c"
FRAME_LEN = 47
POINTS_PER_FRAME = 12


def crc8(data):
    """CRC-8, poly 0x4D, init 0, dle protokolu LDROBOT."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ (0x4D if crc & 0x80 else 0)) & 0xFF
    return crc


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.frames = 0
        self.bad_crc = 0
        self.points = 0
        self.nonzero = 0
        self.rpm_sum = 0.0
        self.rpm_min = float("inf")
        self.rpm_max = 0.0

    def feed(self, data, writer=None):
        self.buffer.extend(data)
        while True:
            pos = self.buffer.find(HEADER)
            if pos < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(b"\x54") else b""
                return
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < FRAME_LEN:
                return
            frame = bytes(self.buffer[:FRAME_LEN])
            if crc8(frame[:46]) != frame[46]:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:FRAME_LEN]
            self.frames += 1
            speed, start = struct.unpack_from("<HH", frame, 2)
            end, stamp = struct.unpack_from("<HH", frame, 42)
            start /= 100.0
            end /= 100.0
            span = (end - start) % 360.0
            rpm = speed / 6.0
            self.rpm_sum += rpm
            self.rpm_min = min(self.rpm_min, rpm)
            self.rpm_max = max(self.rpm_max, rpm)
            for i in range(POINTS_PER_FRAME):
                distance, intensity = struct.unpack_from("<HB", frame, 6 + i * 3)
                angle = (start + span * i / (POINTS_PER_FRAME - 1)) % 360.0
                self.points += 1
                self.nonzero += bool(distance)
                if writer is not None:
                    writer.writerow((f"{angle:.2f}", distance, intensity, f"{rpm:.2f}", stamp))


def main():
    p = argparse.ArgumentParser(description="ETA Raggio LiDAR: UART -> úhel, vzdálenost, intenzita")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--input", type=Path, help="Dříve uložený binární UART záznam")
    src.add_argument("--port", default="/dev/ttyUSB0", help="CP2102, výchozí /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200, help="Výchozí 115200 (ověřeno na našem LiDARu)")
    p.add_argument("--seconds", type=float, default=10, help="Délka živého záznamu")
    p.add_argument("--csv", type=Path, help="Volitelný CSV výstup všech bodů")
    a = p.parse_args()

    if a.seconds <= 0 or a.baud <= 0:
        p.error("--seconds i --baud musí být kladné")
    d = Decoder()
    if a.csv:
        a.csv.parent.mkdir(parents=True, exist_ok=True)

    try:
        with (a.csv.open("w", newline="", encoding="utf-8") if a.csv else open("/dev/null", "w")) as out:
            writer = csv.writer(out) if a.csv else None
            if writer:
                writer.writerow(("angle_deg", "distance_mm", "intensity", "rpm", "timestamp_ms"))
            if a.input:
                with a.input.open("rb") as fh:
                    for chunk in iter(lambda: fh.read(4096), b""):
                        d.feed(chunk, writer)
            else:
                try:
                    import serial
                except ImportError:
                    print("Nainstaluj: sudo apt install python3-serial", file=sys.stderr)
                    return 2
                print(f"Čtu {a.port}, {a.baud} baud, {a.seconds:g} s; pouze RX.", flush=True)
                with serial.Serial(a.port, a.baud, timeout=0.2, xonxoff=False,
                                   rtscts=False, dsrdtr=False) as ser:
                    end_time = time.monotonic() + a.seconds
                    while time.monotonic() < end_time:
                        data = ser.read(4096)
                        if data:
                            d.feed(data, writer)
    except KeyboardInterrupt:
        print("\nPřerušeno uživatelem.")
    except (OSError, Exception) as exc:
        print(f"Chyba čtení: {exc}", file=sys.stderr)
        return 1

    print(f"Rámců CRC OK: {d.frames} | CRC chyby: {d.bad_crc} | "
          f"Bodů: {d.points} | Nenulových vzdáleností: {d.nonzero}")
    if d.frames:
        print(f"Otáčky: {d.rpm_sum/d.frames:.1f} rpm "
              f"(rozsah {d.rpm_min:.1f}–{d.rpm_max:.1f})")
    if a.csv:
        print(f"CSV: {a.csv}")
    return 0 if d.frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
