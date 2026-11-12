#!/usr/bin/env python3
"""Pasivní čtení UART dat LiDARu ETA Raggio přes USB–UART CP2102.

Skript NIKDY neposílá data do LiDARu ani neovládá motor.
"""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

try:
    import serial
except ImportError:
    print("Chybí pyserial: sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(2)


def main() -> int:
    parser = argparse.ArgumentParser(description="Pasivní UART odposlech LiDARu ETA Raggio")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Sériový port (výchozí /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (výchozí 115200, neověřeno)")
    parser.add_argument("--seconds", type=float, default=15, help="Doba odposlechu v sekundách")
    parser.add_argument("--output", type=Path, help="Volitelné uložení syrových bajtů do .bin souboru")
    args = parser.parse_args()

    if args.baud <= 0 or args.seconds <= 0:
        parser.error("--baud a --seconds musí být kladné hodnoty")

    print(f"LiDAR UART: port={args.port}, baud={args.baud}, čas={args.seconds:g} s")
    print("Pouze příjem (RX). Nic se neodesílá, motor se neovládá.", flush=True)

    total = 0
    chunks = 0
    started = time.monotonic()
    raw = None
    try:
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            raw = args.output.open("wb")

        with serial.Serial(args.port, args.baud, timeout=0.25,
                           xonxoff=False, rtscts=False, dsrdtr=False) as uart:
            while time.monotonic() - started < args.seconds:
                data = uart.read(256)
                if not data:
                    continue
                total += len(data)
                chunks += 1
                if raw is not None:
                    raw.write(data)
                timestamp = datetime.now().astimezone().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{timestamp}] {len(data):3d} B: {data.hex(' ')}", flush=True)
    except KeyboardInterrupt:
        print("\nPřerušeno uživatelem.")
    except (serial.SerialException, OSError) as exc:
        print(f"Chyba UART / souboru: {exc}", file=sys.stderr)
        return 1
    finally:
        if raw is not None:
            raw.close()

    print(f"Hotovo: {total} bajtů v {chunks} blocích.")
    if total == 0:
        print("Žádná data. Zkontroluj propojení, úrovně, baudrate a zda LiDAR vůbec vysílá.")
        print("Nulový výsledek sám o sobě neznamená vadný LiDAR.")
    elif args.output:
        print(f"Syrová data: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
