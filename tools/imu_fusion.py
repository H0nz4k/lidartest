#!/usr/bin/env python3
"""MPU-6050 -> časovaný relativní azimut pro 2D LiDAR SLAM.

Samostatné vlákno I²C snímá gyro Z na Raspberry Pi 4 (cca 80-100 Hz).
Offset se kalibruje při nehybném modulu a integruje se úhlová rychlost.
Do LiDARu neposílá žádná data. MPU-6050 nemá magnetometr: výsledný
azimut je relativní a bez průběžné korekce mapou driftuje.
"""
from contextlib import closing
import math
import statistics
import threading
import time

from sensor_sync import TimedHistory


class ImuAngleTracker:
    """Thread-safe stav azimutu a zdravotního stavu IMU."""

    def __init__(self):
        self.lock = threading.RLock()
        self.history = TimedHistory(capacity=650)
        self._session = 0
        self._ready = False
        self._last_time = None
        self._heading_rad = 0.0
        self._roll_rad = 0.0
        self._pitch_rad = 0.0
        self._tilt_ready = False
        self._gyro_xy_bias = (0.0, 0.0)
        self._bias_dps = 0.0
        self._speed_dps = 0.0
        self._noise_dps = 0.0
        self._samples = 0
        self._error = "Čekám na MPU-6050."

    @staticmethod
    def inclination(acc):
        """Směr vektoru zdánlivé gravitace v tělesových osách MPU.

        Předpoklad: při vodorovném modulu čip ukazuje accZ přibližně +1 g.
        Vrací (roll, pitch) v rad; před rotací platformy musí být
        osy MPU známy vůči osám měření LiDARu.
        """
        ax, ay, az = (float(x) for x in acc)
        return math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))

    def calibrate(self, bias_dps, noise_dps, now=None, accel_g=None,
                  gyro_xy_bias_dps=(0.0, 0.0)):
        if now is None:
            now = time.monotonic()
        with self.lock:
            self.history.clear()
            self._session += 1
            self._bias_dps = float(bias_dps)
            self._noise_dps = float(noise_dps)
            self._heading_rad = 0.0
            self._gyro_xy_bias = tuple(float(v) for v in gyro_xy_bias_dps)
            self._tilt_ready = accel_g is not None
            if self._tilt_ready:
                self._roll_rad, self._pitch_rad = self.inclination(accel_g)
            else:
                self._roll_rad = self._pitch_rad = 0.0
            self._last_time = float(now)
            self._ready = True
            self._speed_dps = 0.0
            self._samples = 0
            self._error = ""

    def add_sample(self, gyro_z_dps, now=None, gyro_xy_dps=None, accel_g=None):
        if now is None:
            now = time.monotonic()
        with self.lock:
            if not self._ready:
                return
            dt = max(0.0, float(now) - self._last_time)
            self._last_time = float(now)
            velocity = float(gyro_z_dps) - self._bias_dps
            # Po výpadku se vyhnout extrapolaci přes neznámý pohyb.
            if dt <= 0.15:
                self._heading_rad += math.radians(velocity) * dt
                if self._tilt_ready and gyro_xy_dps is not None:
                    gx = float(gyro_xy_dps[0]) - self._gyro_xy_bias[0]
                    gy = float(gyro_xy_dps[1]) - self._gyro_xy_bias[1]
                    self._roll_rad += math.radians(gx) * dt
                    self._pitch_rad += math.radians(gy) * dt
                    if accel_g is not None:
                        norm = math.sqrt(sum(float(v) ** 2 for v in accel_g))
                        if 0.75 < norm < 1.35:
                            roll_acc, pitch_acc = self.inclination(accel_g)
                            # Komplementární filtr: gyro krátkodobě,
                            # gravitace dlouhodobě. Nepoužívat jako
                            # plnou 6DoF inerciální odometrii.
                            alpha = math.exp(-dt / 0.6)
                            roll_err = (roll_acc - self._roll_rad + math.pi) % (2 * math.pi) - math.pi
                            pitch_err = (pitch_acc - self._pitch_rad + math.pi) % (2 * math.pi) - math.pi
                            self._roll_rad += (1 - alpha) * roll_err
                            self._pitch_rad += (1 - alpha) * pitch_err
            self._speed_dps = velocity
            self._samples += 1
            self.history.add(float(now), {
                "heading_deg": math.degrees(self._heading_rad),
                "roll_deg": math.degrees(self._roll_rad) if self._tilt_ready else None,
                "pitch_deg": math.degrees(self._pitch_rad) if self._tilt_ready else None,
                "rate_dps": velocity, "session": self._session,
            })

    def set_error(self, reason):
        with self.lock:
            self._ready = False
            self.history.clear()
            self._speed_dps = 0.0
            self._last_time = None
            self._error = str(reason)

    def sample_at(self, timestamp, max_age=0.10):
        """Yaw přesněji přiřazený ke konci LiDAR otáčky.

        Během chyby I²C/kalibrace vrací None. Pouze software timestamp.
        """
        with self.lock:
            if not self._ready:
                return None
            return self.history.at(timestamp, max_age=max_age, max_gap=0.12)

    def snapshot(self, max_age=0.4):
        now = time.monotonic()
        with self.lock:
            age = float("inf") if self._last_time is None else now - self._last_time
            ready = self._ready and age <= max_age
            return {
                "ready": ready,
                "heading_deg": math.degrees(self._heading_rad) if ready else None,
                "rate_dps": round(self._speed_dps, 3) if ready else None,
                "roll_deg": round(math.degrees(self._roll_rad), 2) if ready and self._tilt_ready else None,
                "pitch_deg": round(math.degrees(self._pitch_rad), 2) if ready and self._tilt_ready else None,
                "tilt_ready": bool(ready and self._tilt_ready),
                "bias_dps": round(self._bias_dps, 3) if self._samples else None,
                "noise_dps": round(self._noise_dps, 3) if self._samples else None,
                "samples": self._samples,
                "session": self._session,
                "error": "" if ready else (self._error or "IMU neodpovídá."),
            }


def imu_reader(tracker, bus_id=1, address=0x68, calibration_s=2.5,
               sample_interval=0.01, run_forever=True):
    """Čtecí smyčka I²C (připojuje se znovu po výpadku).

    Používá stejné funkce jako potvrzený hardware test imu_test.py.
    Import až ve vláknu: LiDAR-only režim funguje i bez smbus modulu.
    """
    try:
        from smbus import SMBus
        from imu_test import find_address, read_sample
    except ImportError as exc:
        tracker.set_error(f"MPU-6050: chybí python3-smbus ({exc})")
        return

    while True:
        try:
            tracker.set_error("MPU-6050: připojuji I²C.")
            with closing(SMBus(bus_id)) as bus:
                addr = find_address(bus, address)
                bus.write_byte_data(addr, 0x6B, 0x01)  # gyro reference clock
                bus.write_byte_data(addr, 0x1A, 0x03)  # DLPF
                bus.write_byte_data(addr, 0x1B, 0x00)  # +-250°/s
                bus.write_byte_data(addr, 0x1C, 0x00)  # +-2g
                bus.write_byte_data(addr, 0x19, 0x09)  # sample divisor
                time.sleep(0.15)

                print("[IMU] Kalibrace gyroskopu: nech konstrukci 2,5 s v klidu.",
                      flush=True)
                state = {}
                readings = []
                accelerations = []
                finish = time.monotonic() + calibration_s
                while time.monotonic() < finish:
                    sample = read_sample(bus, addr, state)
                    readings.append(sample["gyro_dps"])
                    accelerations.append(sample["acc_g"])
                    time.sleep(sample_interval)
                if len(readings) < 30:
                    raise OSError("Příliš málo gyro vzorků při kalibraci")
                bias_xyz = tuple(statistics.mean(row[k] for row in readings)
                                 for k in range(3))
                noise_xyz = tuple(statistics.pstdev(row[k] for row in readings)
                                  for k in range(3))
                bias = bias_xyz[2]
                noise = max(noise_xyz)
                gravity = tuple(statistics.mean(row[k] for row in accelerations)
                                for k in range(3))
                g = math.sqrt(sum(v * v for v in gravity))
                if not 0.75 < g < 1.35:
                    raise OSError(f"Kalibrace náklonu: nestandardní gravitace {g:.2f}g")
                if noise > 1.5:
                    tracker.set_error(
                        f"Kalibrace je nestabilní ({noise:.2f} °/s): nehýbej konstrukcí."
                    )
                    print("[IMU] Kalibrace nestabilní, opakuji.", flush=True)
                    time.sleep(0.5)
                    if not run_forever:
                        return
                    continue

                tracker.calibrate(bias, noise, accel_g=gravity,
                                  gyro_xy_bias_dps=bias_xyz[:2])
                print(f"[IMU] Aktivní 0x{addr:02x}, bias Z={bias:+.3f} °/s, "
                      f"šum={noise:.3f} °/s", flush=True)
                while True:
                    sample = read_sample(bus, addr, state)
                    gx, gy, gz = sample["gyro_dps"]
                    tracker.add_sample(gz, gyro_xy_dps=(gx, gy),
                                       accel_g=sample["acc_g"])
                    time.sleep(sample_interval)

        except OSError as exc:
            tracker.set_error(f"I²C chyba: {exc}")
            print(f"[IMU] {exc}; za 2 s nový pokus.", flush=True)
            if not run_forever:
                return
            time.sleep(2)
