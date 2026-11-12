#!/usr/bin/env python3
"""Experimentální 2D LiDAR SLAM bez odometrie (scan-to-map matching).

Vstup: úhel [stupně], vzdálenost [mm], intenzita. Výstup: 2D occupancy mapa
+ odhad polohy LiDARu. Pouze relativní lokalizace; zatím bez loop closure.
Nepoužívá žádné UART příkazy ani motor-control.

Vyžaduje numpy (Raspberry Pi: sudo apt install python3-numpy).
"""
import math
import threading

import numpy as np

GRID_SIZE = 400
CELL_M = 0.05
MAX_RANGE_MM = 6000
MIN_RANGE_MM = 150


class Slam2D:
    def __init__(self, size=GRID_SIZE, resolution=CELL_M):
        self.size = size
        self.resolution = resolution
        self.lock = threading.RLock()
        self.reset()

    def reset(self):
        with self.lock:
            self.grid = np.zeros((self.size, self.size), dtype=np.int8)
            self.pose = (0.0, 0.0, 0.0)  # x, y [m], rotace [radiány]
            self.path = [(0.0, 0.0)]
            self.initialized = False
            self.scans = 0
            self.accepted = 0
            self.rejected = 0
            self.quality = 0.0
            self.last_scan = []
            # Heading z IMU vždy vztahujeme k poslednímu přijatému skenu.
            self.imu_anchor_deg = None
            self.imu_anchor_session = None
            self.imu_used = False
            self.imu_delta_deg = 0.0
            self.visual_anchor = None
            self.visual_used = False
            self.visual_delta_deg = 0.0
            self.visual_translation_used = False

    @staticmethod
    def _scan_points(scan):
        # Úhlové koše 2°; každý koš reprezentuje nejbližší odraz.
        buckets = {}
        for angle_deg, distance_mm, intensity in scan:
            if not MIN_RANGE_MM <= distance_mm <= MAX_RANGE_MM or intensity < 3:
                continue
            bin_id = int((angle_deg % 360) // 2)
            if bin_id not in buckets or distance_mm < buckets[bin_id][1]:
                buckets[bin_id] = (angle_deg % 360, distance_mm, intensity)
        if len(buckets) < 40:
            return None
        reduced = [buckets[k] for k in sorted(buckets)]
        angle = np.deg2rad(np.array([row[0] for row in reduced], dtype=np.float64))
        distance = np.array([row[1] for row in reduced], dtype=np.float64) / 1000.0
        # 0° dopředu (nahoru v mapě), pozitivní úhly doprava.
        xy = np.stack([np.sin(angle) * distance, np.cos(angle) * distance], axis=1)
        return xy, reduced

    def _likelihood(self):
        occupied = self.grid >= 3
        likelihood = np.zeros(self.grid.shape, dtype=np.int8)
        likelihood[occupied] = 10
        # 1 a 2 buňky tolerance pro pohyb, rotaci a šum.
        for radius, weight in ((1, 6), (2, 2)):
            padded = np.pad(occupied, radius, mode="constant", constant_values=False)
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx == 0 and dy == 0:
                        continue
                    view = padded[
                        radius + dy:radius + dy + self.size,
                        radius + dx:radius + dx + self.size
                    ]
                    likelihood[view] = np.maximum(likelihood[view], weight)
        return likelihood

    def _score(self, xy, likelihood, pose, prior):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        world_x = x + c * xy[:, 0] - s * xy[:, 1]
        world_y = y + s * xy[:, 0] + c * xy[:, 1]
        ix = np.rint(world_x / self.resolution).astype(np.int32) + self.size // 2
        iy = np.rint(world_y / self.resolution).astype(np.int32) + self.size // 2
        valid = (ix >= 0) & (ix < self.size) & (iy >= 0) & (iy < self.size)
        if valid.sum() < 30:
            return -1e9
        matched = float(likelihood[iy[valid], ix[valid]].sum()) / len(xy)
        # Omezení driftu v plochých či symetrických scénách: preferuj malé pohyby.
        dx = x - prior[0]
        dy = y - prior[1]
        da = (yaw - prior[2] + math.pi) % (2 * math.pi) - math.pi
        penalty = 0.45 * math.hypot(dx, dy) / 0.10 + 0.12 * abs(math.degrees(da)) / 2.0
        return matched - penalty

    def _match(self, xy, imu_delta_deg=None, translation_xy=None):
        # Predikce ze senzorů pouze zužuje oblast pro LiDAR korelaci.
        # Kamera se používá jen po explicitní kalibraci měřítka.
        x, y, yaw = self.pose
        if translation_xy is not None:
            x += translation_xy[0]
            y += translation_xy[1]
        if imu_delta_deg is not None:
            yaw += math.radians(imu_delta_deg)
        prior = (x, y, yaw)
        likelihood = self._likelihood()
        # 90 bodů postačuje pro korelaci; hustší paprsky používáme k mapování.
        samples = xy[::2]
        best_pose = prior
        best_score = self._score(samples, likelihood, prior, prior)
        # Hrubé prohledání: +-24 cm a +-6°.
        for dtheta in ((-3, -1.5, 0, 1.5, 3) if imu_delta_deg is not None
                       else (-6, -3, 0, 3, 6)):
            yaw = prior[2] + math.radians(dtheta)
            for dx_cm in (-24, -12, 0, 12, 24):
                for dy_cm in (-24, -12, 0, 12, 24):
                    candidate = (prior[0] + dx_cm / 100, prior[1] + dy_cm / 100, yaw)
                    score = self._score(samples, likelihood, candidate, prior)
                    if score > best_score:
                        best_score, best_pose = score, candidate

        coarse = best_pose
        # Jemné vyhledávání vůči vítězi, krok 3cm a 0,75°.
        for dtheta in (-1.5, -0.75, 0, 0.75, 1.5):
            yaw = coarse[2] + math.radians(dtheta)
            for dx_cm in (-6, -3, 0, 3, 6):
                for dy_cm in (-6, -3, 0, 3, 6):
                    candidate = (coarse[0] + dx_cm / 100, coarse[1] + dy_cm / 100, yaw)
                    score = self._score(samples, likelihood, candidate, prior)
                    if score > best_score:
                        best_score, best_pose = score, candidate
        return best_pose, best_score

    def _integrate(self, xy):
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        sx = int(round(x / self.resolution)) + self.size // 2
        sy = int(round(y / self.resolution)) + self.size // 2
        for lx, ly in xy:
            wx = x + c * lx - s * ly
            wy = y + s * lx + c * ly
            ex = int(round(wx / self.resolution)) + self.size // 2
            ey = int(round(wy / self.resolution)) + self.size // 2
            if not (0 <= ex < self.size and 0 <= ey < self.size):
                continue

            # Bresenham: procházej volné buňky paprsku, bez koncového zásahu.
            dx, dy = abs(ex - sx), abs(ey - sy)
            stepx = 1 if sx < ex else -1
            stepy = 1 if sy < ey else -1
            err = dx - dy
            cx, cy = sx, sy
            while (cx, cy) != (ex, ey):
                if not (0 <= cx < self.size and 0 <= cy < self.size):
                    break
                value = int(self.grid[cy, cx])
                self.grid[cy, cx] = max(-12, value - 1)
                e2 = err * 2
                if e2 > -dy:
                    err -= dy
                    cx += stepx
                if e2 < dx:
                    err += dx
                    cy += stepy
            self.grid[ey, ex] = min(12, int(self.grid[ey, ex]) + 5)

    def process_scan(self, scan, imu_heading_deg=None, visual=None, camera_yaw_offset_deg=0.0,
                     imu_session=None):
        """Integrace jednoho skenu. IMU heading = relativní gyro Z [°].

        IMU a volitelná vizuální odometrie z kamery jsou pouze priory
        vůči poslednímu přijatému skenu; konečné body určuje LiDAR.
        """
        result = self._scan_points(scan)
        with self.lock:
            self.scans += 1
            if result is None:
                self.rejected += 1
                return False
            xy, self.last_scan = result
            imu_heading = None
            if imu_heading_deg is not None and math.isfinite(imu_heading_deg):
                imu_heading = float(imu_heading_deg)
            visual_data = None
            if visual and visual.get("ready") and visual.get("mode") == "downward":
                heading = visual.get("heading_deg")
                session = visual.get("session")
                if isinstance(heading, (int, float)) and math.isfinite(heading) and isinstance(session, int):
                    visual_data = dict(visual)
            if not self.initialized:
                self._integrate(xy)
                self.initialized = True
                self.accepted += 1
                self.quality = 10.0
                self.imu_anchor_deg = imu_heading
                self.imu_anchor_session = imu_session
                self.visual_anchor = visual_data
                return True

            # Po výpadku IMU nesmíme integrovat neznámé otočení:
            # první návrat IMU pouze synchronizuje referenci.
            if imu_heading is None or imu_session != self.imu_anchor_session:
                self.imu_anchor_deg = None
            delta = None
            if imu_heading is not None and self.imu_anchor_deg is not None:
                raw_delta = (imu_heading - self.imu_anchor_deg + 180) % 360 - 180
                if abs(raw_delta) <= 65:
                    delta = raw_delta
            # Korelace LiDARu má poslední slovo. Vizuální yaw lze
            # použít pouze pro kolmo dolů otočenou, pevně uchycenou kameru.
            visual_delta = None
            camera_translation = None
            if visual_data is None:
                self.visual_anchor = None
            elif self.visual_anchor is not None and (
                visual_data["session"] == self.visual_anchor["session"]
            ):
                raw = (visual_data["heading_deg"] -
                       self.visual_anchor["heading_deg"] + 180) % 360 - 180
                if abs(raw) <= 45:
                    visual_delta = raw
                    if visual_data.get("scale_calibrated") and self.visual_anchor.get("scale_calibrated"):
                        x0, y0 = self.visual_anchor.get("x_m"), self.visual_anchor.get("y_m")
                        x1, y1 = visual_data.get("x_m"), visual_data.get("y_m")
                        if all(isinstance(v, (int, float)) and math.isfinite(v)
                               for v in (x0, y0, x1, y1)):
                            dx, dy = x1 - x0, y1 - y0
                            if math.hypot(dx, dy) <= 0.4:
                                angle = self.pose[2] + math.radians(camera_yaw_offset_deg)
                                c, sn = math.cos(angle), math.sin(angle)
                                camera_translation = (c * dx - sn * dy, sn * dx + c * dy)

            use_visual = False
            # Konzistentní IMU a kamera: 80 % gyro, 20 % kamera.
            # Pokud si odporují o více než 10°, kameru ignorujeme.
            if delta is not None and visual_delta is not None:
                if abs(delta - visual_delta) <= 10:
                    delta = 0.8 * delta + 0.2 * visual_delta
                    use_visual = True
            elif delta is None and visual_delta is not None:
                delta = visual_delta
                use_visual = True
            pose, score = self._match(xy, delta, camera_translation)
            self.quality = round(score, 2)
            # Nedostatečná shoda se starou mapou -> nezapisovat nové body
            # do nesprávného místa. Pomalejší posun pomáhá obnovit tracking.
            if score < 1.8:
                self.rejected += 1
                return False

            x, y, yaw = pose
            if not (-self.size * self.resolution * 0.47 < x < self.size * self.resolution * 0.47 and
                    -self.size * self.resolution * 0.47 < y < self.size * self.resolution * 0.47):
                self.rejected += 1
                return False
            self.pose = (x, y, (yaw + math.pi) % (2 * math.pi) - math.pi)
            self.imu_used = delta is not None and imu_heading is not None and self.imu_anchor_deg is not None
            self.imu_delta_deg = round(delta if delta is not None else 0.0, 2)
            self.visual_used = use_visual
            self.visual_delta_deg = round(visual_delta if visual_delta is not None else 0.0, 2)
            self.visual_translation_used = camera_translation is not None
            self.imu_anchor_deg = imu_heading
            self.imu_anchor_session = imu_session
            self.visual_anchor = visual_data
            self.accepted += 1
            self._integrate(xy)
            if math.hypot(x - self.path[-1][0], y - self.path[-1][1]) > 0.025:
                self.path.append((x, y))
                if len(self.path) > 3000:
                    self.path = self.path[::2]
            return True

    def snapshot(self):
        with self.lock:
            # Posíláme pouze navštívené buňky (hodnoty <= -2 nebo >= +2).
            iy, ix = np.nonzero(np.abs(self.grid.astype(np.int16)) >= 2)
            values = self.grid[iy, ix]
            # Limit JSON při dlouhém mapování.
            stride = max(1, int(math.ceil(len(ix) / 55000)))
            cells = [[int(x), int(y), int(v)]
                     for x, y, v in zip(ix[::stride], iy[::stride], values[::stride])]
            x, y, yaw = self.pose
            return {
                "size": self.size,
                "resolution_m": self.resolution,
                "cells": cells,
                "pose": [round(x, 3), round(y, 3), round(math.degrees(yaw), 2)],
                "path": [[round(px, 3), round(py, 3)] for px, py in self.path],
                "scan": self.last_scan,
                "scans": self.scans,
                "accepted": self.accepted,
                "rejected": self.rejected,
                "quality": self.quality,
                "initialized": self.initialized,
                "imu_used": self.imu_used,
                "imu_delta_deg": self.imu_delta_deg,
                "visual_used": self.visual_used,
                "visual_delta_deg": self.visual_delta_deg,
                "visual_translation_used": self.visual_translation_used,
            }

    def pgm(self):
        """Grayscale image: obsazené černé, volné bílé, neznámé šedé."""
        with self.lock:
            pixels = np.full(self.grid.shape, 205, dtype=np.uint8)
            pixels[self.grid <= -2] = 252
            pixels[self.grid >= 3] = 18
            pixels = np.flipud(pixels)
            return (
                f"P5\n# origin=center resolution={self.resolution:.4f}m\n"
                f"{self.size} {self.size}\n255\n"
            ).encode("ascii") + pixels.tobytes()

    def trajectory_csv(self):
        with self.lock:
            return "x_m,y_m\n" + "".join(f"{x:.3f},{y:.3f}\n" for x, y in self.path)
