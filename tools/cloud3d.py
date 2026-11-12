#!/usr/bin/env python3
"""Stacionární 3D mračno z 2D ETA Raggio + jediné osy náklonu.

FIXNÍ základna; osa naklápění x (MPU roll) nebo y (MPU pitch).
Všechny 360° LiDAR skeny se překládají do XYZ pouze za předpokladu,
že se neposouvá základna, orientace MPU vůči LiDARu je pevná a
osový úhel odpovídá mechanice. Není to volně pohyblivý 6DoF SLAM.
"""
import math
import threading

MIN_MM = 150
MAX_MM = 6000
MIN_INTENSITY = 3


def point_xyz(angle_deg, distance_mm, tilt_deg, axis="x",
              lever=(0.0, 0.0, 0.0)):
    """Lokální (sinθ,cosθ,0) -> world XYZ kolem pevné osy náklonu.

    lever = vektor od bodu otáčení k LiDAR středu v nulté pozici.
    Sken XYZ je relativní k počáteční poloze LiDAR středu:
    p_world = R*p_local + R*lever - lever.
    """
    if axis not in ("x", "y"):
        raise ValueError("axis must be x or y")
    radius = distance_mm / 1000.0
    a = math.radians(angle_deg)
    tilt = math.radians(tilt_deg)
    x, y, z = math.sin(a) * radius, math.cos(a) * radius, 0.0
    lx, ly, lz = map(float, lever)
    ct, st = math.cos(tilt), math.sin(tilt)
    if axis == "x":
        return (x,
                ct * (y + ly) - st * (z + lz) - ly,
                st * (y + ly) + ct * (z + lz) - lz)
    return (ct * (x + lx) + st * (z + lz) - lx,
            y,
            -st * (x + lx) + ct * (z + lz) - lz)


class StationaryCloud3D:
    def __init__(self, axis="x", tilt_sign=1, tilt_step_deg=2.0,
                 max_points=160000, lever=(0.0, 0.0, 0.0)):
        if axis not in ("x", "y") or tilt_sign not in (-1, 1):
            raise ValueError("invalid axis or sign")
        if not 0.25 <= tilt_step_deg <= 30 or max_points < 100:
            raise ValueError("invalid tilt step or points limit")
        self.axis = axis
        self.tilt_sign = tilt_sign
        self.tilt_step_deg = float(tilt_step_deg)
        self.max_points = int(max_points)
        self.lever = tuple(float(v) for v in lever)
        self.lock = threading.RLock()
        self.reset()

    def reset(self):
        with self.lock:
            self.points = []
            self.tilt_zero = None
            self.other_zero = None
            self.last_tilt = None
            self.session = None
            self.scans_saved = 0
            self.scans_skipped = 0
            self.scans_bad_pose = 0
            self.full = False
            self.last_error = "Čekám na kalibrovaný náklon MPU-6050."
            self.min_tilt = self.max_tilt = None

    def add_scan(self, scan, imu_sample):
        """Přidává vždy jednu celou 360° otáčku, nikoli jednotlivé UART rámce.

        Pro náklon používáme session gyro+gravity filtru. Po jeho restartu
        ani po výpadku I2C se skeny nemíchají do jednoho mračna.
        """
        with self.lock:
            if self.full:
                self.scans_skipped += 1
                self.last_error = "Limit bodů dosažen. Ulož PLY a spusť novou mapu."
                return False
            if not imu_sample or not all(
                imu_sample.get(k) is not None for k in ("roll_deg", "pitch_deg", "session")
            ):
                self.scans_bad_pose += 1
                self.last_error = "IMU nemá platný roll/pitch; sken vynechán."
                return False
            roll = float(imu_sample["roll_deg"])
            pitch = float(imu_sample["pitch_deg"])
            if not (math.isfinite(roll) and math.isfinite(pitch)):
                self.scans_bad_pose += 1
                return False
            session = imu_sample["session"]
            if self.session is None:
                self.session = session
            if session != self.session:
                self.scans_bad_pose += 1
                self.last_error = "IMU byla znovu kalibrována. Ulož PLY a resetuj 3D mapu."
                return False
            tilt_raw = roll if self.axis == "x" else pitch
            other = pitch if self.axis == "x" else roll
            if self.tilt_zero is None:
                self.tilt_zero, self.other_zero = tilt_raw, other
            # Při náklonu kolem jedné osy by druhá osa neměla měnit směr.
            if abs(other - self.other_zero) > 12:
                self.scans_bad_pose += 1
                self.last_error = "Velký náklon na vedlejší ose (>12°). Drž pevnou osu."
                return False
            tilt = self.tilt_sign * (tilt_raw - self.tilt_zero)
            if abs(tilt) > 85:
                self.scans_bad_pose += 1
                self.last_error = "Náklon >85° – méně spolehlivé měření."
                return False
            if (self.last_tilt is not None and
                    abs(tilt - self.last_tilt) < self.tilt_step_deg):
                self.scans_skipped += 1
                return False
            valid = [(angle, mm, intensity) for angle, mm, intensity in scan
                     if MIN_MM <= mm <= MAX_MM and intensity >= MIN_INTENSITY]
            if len(valid) < 40:
                self.scans_bad_pose += 1
                self.last_error = "Nedostatek platných LiDAR bodů."
                return False
            remaining = self.max_points - len(self.points)
            if remaining <= 0:
                self.full = True
                return False
            transformed = []
            for angle, mm, intensity in valid[:remaining]:
                x, y, z = point_xyz(angle, mm, tilt, self.axis, self.lever)
                transformed.append((x, y, z, int(intensity)))
            self.points.extend(transformed)
            self.scans_saved += 1
            self.last_tilt = tilt
            self.min_tilt = tilt if self.min_tilt is None else min(self.min_tilt, tilt)
            self.max_tilt = tilt if self.max_tilt is None else max(self.max_tilt, tilt)
            self.full = len(self.points) >= self.max_points
            self.last_error = ""
            return True

    def snapshot(self, preview_limit=8000):
        with self.lock:
            n = len(self.points)
            stride = max(1, math.ceil(n / preview_limit))
            preview = [
                [round(x, 3), round(y, 3), round(z, 3), intensity]
                for x, y, z, intensity in self.points[::stride]
            ]
            return {
                "points": preview,
                "point_count": n,
                "preview_count": len(preview),
                "axis": self.axis,
                "tilt_deg": round(self.last_tilt, 2) if self.last_tilt is not None else None,
                "tilt_min_deg": round(self.min_tilt, 2) if self.min_tilt is not None else None,
                "tilt_max_deg": round(self.max_tilt, 2) if self.max_tilt is not None else None,
                "tilt_step_deg": self.tilt_step_deg,
                "scans_saved": self.scans_saved,
                "scans_skipped": self.scans_skipped,
                "scans_bad_pose": self.scans_bad_pose,
                "full": self.full,
                "error": self.last_error,
                "mode": "stationary_tilt",
                "warning": "Výsledek je platný jen s pevnou základnou, známou osou a bez posunu XYZ.",
            }

    def ply(self):
        with self.lock:
            header = ("ply\nformat ascii 1.0\n"
                      "comment ETA Raggio experimental stationary 2D-LiDAR tilt cloud\n"
                      "comment Origin: LiDAR centre at first accepted scan\n"
                      f"element vertex {len(self.points)}\n"
                      "property float x\nproperty float y\nproperty float z\n"
                      "property uchar intensity\nend_header\n")
            lines = (f"{x:.4f} {y:.4f} {z:.4f} {inten}\n"
                     for x, y, z, inten in self.points)
            return (header + "".join(lines)).encode("ascii")
