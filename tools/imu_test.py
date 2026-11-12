#!/usr/bin/env python3
"""Bezpečný první test MPU-6050 na Raspberry Pi 4 (I²C).

Čte 6osá data a vypisuje odhad relativního otočení kolem osy Z.
Gyro musí být při úvodní kalibraci bez pohybu. Neovládá LiDAR ani HUB.
"""
import argparse
from contextlib import closing
import statistics
import struct
import sys
import time

try:
    from smbus import SMBus
except ImportError:
    print("Chybí python3-smbus: sudo apt install -y python3-smbus", file=sys.stderr)
    raise SystemExit(2)

GYRO_LSB_PER_DPS = 131.0  # +/-250 stupňů/s
ACCEL_LSB_PER_G = 16384.0  # +/-2g


def read_sample(bus, address, state=None):
    """Čti 14 registrů; při nespolehlivém burst read přejdi na jednotlivé bajty.

    Některé moduly/sběrnice odpoví na WHO_AM_I, ale 14b burst selže EIO.
    Fallback problém s I²C neodstraní; rozliší chybu burst přenosu
    od úplného výpadku a umožní první zkoušku gyroskopu.
    """
    if state is None:
        state = {}
    if state.get("bytewise"):
        b = [bus.read_byte_data(address, reg) for reg in range(0x3B, 0x49)]
    else:
        try:
            b = bus.read_i2c_block_data(address, 0x3B, 14)
            if len(b) != 14:
                raise OSError(f"Nekompletní I²C blok: {len(b)} z 14 bajtů")
        except OSError as exc:
            print(f"Varování: 14b I²C čtení selhalo ({exc}); zkouším "
                  "čtení jednotlivých registrů. Pokud selže i to, "
                  "ověř napětí SDA/SCL, kabel a kontakty.", file=sys.stderr, flush=True)
            state["bytewise"] = True
            b = [bus.read_byte_data(address, reg) for reg in range(0x3B, 0x49)]
    ax, ay, az, temp, gx, gy, gz = struct.unpack(">hhhhhhh", bytes(b))
    return {
        "acc_g": (ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G),
        "gyro_dps": (gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS),
        "temp_c": temp / 340.0 + 36.53,
    }


def find_address(bus, choice):
    candidates = [choice] if choice is not None else [0x68, 0x69]
    for addr in candidates:
        try:
            who = bus.read_byte_data(addr, 0x75)
            if who == 0x68:
                return addr
            print(f"I²C 0x{addr:02X}: WHO_AM_I=0x{who:02X}, očekáváno 0x68", file=sys.stderr)
        except OSError:
            pass
    raise OSError("MPU-6050 nenalezen na 0x68/0x69; ověř I²C, napájení a vodiče SDA/SCL")


def main():
    p = argparse.ArgumentParser(description="MPU-6050: akcelerometr, gyro a relativní yaw")
    p.add_argument("--bus", type=int, default=1, help="I²C sběrnice RPi4 (výchozí 1)")
    p.add_argument("--address", type=lambda s: int(s, 0), default=None,
                   help="Např. 0x68 nebo 0x69; výchozí autodetekce")
    p.add_argument("--seconds", type=float, default=10, help="Doba čtení po kalibraci")
    p.add_argument("--calibration", type=float, default=2.5,
                   help="Doba bez pohybu pro nulování gyroskopu v sekundách")
    a = p.parse_args()
    if a.seconds <= 0 or a.calibration < 1 or a.bus < 0:
        p.error("Čas musí být kladný, kalibrace >=1 s, I²C bus >=0")
    if a.address is not None and a.address not in (0x68, 0x69):
        p.error("Podporovaná adresa: 0x68 nebo 0x69")

    try:
        with closing(SMBus(a.bus)) as bus:
            addr = find_address(bus, a.address)
            print(f"MPU-6050 nalezen: I²C-{a.bus} @ 0x{addr:02X}", flush=True)
            # Probuzení a gyro referenční hodiny.
            bus.write_byte_data(addr, 0x6B, 0x01)
            # Low-pass filtr přibližně 42 Hz, rozsah +/-250 dps, +/-2 g.
            bus.write_byte_data(addr, 0x1A, 0x03)
            bus.write_byte_data(addr, 0x1B, 0x00)
            bus.write_byte_data(addr, 0x1C, 0x00)
            bus.write_byte_data(addr, 0x19, 0x09)
            time.sleep(0.15)

            print(f"Nehýbej senzorem {a.calibration:g} s – kalibrace nulového gyra Z.", flush=True)
            z_samples = []
            read_state = {}
            finish = time.monotonic() + a.calibration
            while time.monotonic() < finish:
                z_samples.append(read_sample(bus, addr, read_state)["gyro_dps"][2])
                time.sleep(0.01)
            bias = statistics.mean(z_samples)
            noise = statistics.pstdev(z_samples)
            print(f"Offset Z: {bias:+.3f} °/s; šum: {noise:.3f} °/s "
                  f"({len(z_samples)} vzorků)", flush=True)
            if noise > 1.0:
                print("POZOR: během kalibrace byl pravděpodobně pohyb; výsledek může driftovat.")

            print("Teď pomalu otáčej celou konstrukcí vodorovně kolem svislé osy Z.", flush=True)
            print("Tisk každých ~0,25 s: Z gyro / relativní úhel / akcelerace / teplota.", flush=True)
            yaw = 0.0
            started = prev = last_print = time.monotonic()
            while time.monotonic() - started < a.seconds:
                sample = read_sample(bus, addr, read_state)
                now = time.monotonic()
                dt = now - prev
                prev = now
                yaw += (sample["gyro_dps"][2] - bias) * dt
                if now - last_print > 0.25:
                    x, y, z = sample["acc_g"]
                    print(f"gyroZ={sample['gyro_dps'][2]-bias:+7.2f} °/s | "
                          f"yaw={yaw:+8.2f}° | acc=({x:+.2f},{y:+.2f},{z:+.2f})g | "
                          f"temp={sample['temp_c']:.1f}°C", flush=True)
                    last_print = now
                time.sleep(0.01)
            print(f"Konec: relativní otočení yaw={yaw:+.2f}°. Absolutní sever neznáme.")
    except KeyboardInterrupt:
        print("\nPřerušeno.")
    except OSError as exc:
        print(f"Chyba I²C: {exc}. Detekce 0x68 sama neznamená stabilní "
              "čtení dat; ověř SDA/SCL (max. 3,3 V), kontakty a napájení.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
