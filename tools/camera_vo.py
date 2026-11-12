#!/usr/bin/env python3
"""CSI kamera -> optický tok pro experimentální 2D SLAM.

Bez --camera pouze nečinná knihovna. Kameru otevírá jen explicitní
--camera. Režim 'monitor' měří vizuální pohyb bez zásahu do SLAM.
Režim 'downward' předpokládá pohled kolmo na rovnou podlahu,
pevnou montáž a dodává relativní yaw; XY pouze po výslovné kalibraci
metrického měřítka (m/px). Monokulární kamera neměří absolutní škálu.
"""
import math
import threading
import time


class CameraPreview:
    """Nejnovější JPEG pouze v RAM; aktivní jen při --camera-preview.

    JPEG jde po HTTP do důvěryhodné LAN bez autentizace. Nepoužívat
    mimo soukromou síť a nikdy nezapisovat snímky na disk.
    """
    def __init__(self, quality=70, max_fps=5):
        self.lock = threading.Lock()
        self.jpeg = None
        self.last_timestamp = 0.0
        self.last_encoded = 0.0
        self.seq = 0
        self.quality = int(quality)
        self.min_interval = 1.0 / max(1, int(max_fps))

    def update(self, frame, timestamp=None):
        now = time.monotonic() if timestamp is None else float(timestamp)
        with self.lock:
            if now - self.last_encoded < self.min_interval:
                return
            self.last_encoded = now
        import cv2
        import numpy as np
        rgb = np.ascontiguousarray(frame)
        if rgb.ndim != 3 or rgb.shape[2] not in (3, 4):
            return
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR if rgb.shape[2] == 3
                           else cv2.COLOR_RGBA2BGR)
        ok, encoded = cv2.imencode(".jpg", bgr,
                                  [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        with self.lock:
            self.jpeg = encoded.tobytes()
            self.last_timestamp = now
            self.seq += 1

    def latest(self, max_age=2.0):
        with self.lock:
            if self.jpeg is None or time.monotonic() - self.last_timestamp > max_age:
                return None
            return self.jpeg


class VisualOdometry:
    def __init__(self, downward=False, metres_per_pixel=0.0):
        self.lock = threading.RLock()
        self.downward = bool(downward)
        self.metres_per_pixel = float(metres_per_pixel)
        self.previous = None
        self.previous_t = None
        self.heading_deg = 0.0
        self.x_m = 0.0
        self.y_m = 0.0
        self.x_px = 0.0
        self.y_px = 0.0
        self.frames = 0
        self.good_frames = 0
        self.features = 0
        self.inliers = 0
        self.last_flow = 0.0
        self.last_good_t = 0.0
        self.error = "Kamera se ještě nepřipojila."
        self.opened = False
        self.session = 0

    def set_error(self, message):
        with self.lock:
            self.error = str(message)
            self.opened = False
            self.last_good_t = 0.0
            self.previous = None
            self.previous_t = None
            self.session += 1

    def set_opened(self):
        with self.lock:
            self.opened = True
            self.error = ""

    def process_frame(self, frame, timestamp=None):
        import cv2
        import numpy as np

        now = time.monotonic() if timestamp is None else float(timestamp)
        if len(frame.shape) == 3:
            if frame.shape[2] == 4:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY)
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        else:
            gray = frame
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        with self.lock:
            self.frames += 1
            if self.previous is None:
                self.previous, self.previous_t = gray, now
                return

            dt = now - self.previous_t
            previous = self.previous
            self.previous, self.previous_t = gray, now
            if dt <= 0 or dt > 0.8 or previous.shape != gray.shape:
                self.last_good_t = 0.0
                self.session += 1
                return

            corners = cv2.goodFeaturesToTrack(
                previous, maxCorners=240, qualityLevel=0.015,
                minDistance=8, blockSize=7
            )
            if corners is None or len(corners) < 25:
                self.features, self.inliers, self.last_good_t = 0, 0, 0.0
                self.session += 1
                return
            nxt, status, _ = cv2.calcOpticalFlowPyrLK(
                previous, gray, corners, None,
                winSize=(21, 21), maxLevel=3
            )
            if nxt is None or status is None:
                self.features, self.inliers, self.last_good_t = 0, 0, 0.0
                self.session += 1
                return
            ok = status.reshape(-1) == 1
            p0, p1 = corners.reshape(-1, 2)[ok], nxt.reshape(-1, 2)[ok]
            self.features = int(len(p0))
            if len(p0) < 22:
                self.inliers, self.last_good_t = 0, 0.0
                self.session += 1
                return

            mat, inlier_mask = cv2.estimateAffinePartial2D(
                p0, p1, method=cv2.RANSAC,
                ransacReprojThreshold=2.5, maxIters=800, confidence=0.99
            )
            if mat is None or inlier_mask is None:
                self.inliers, self.last_good_t = 0, 0.0
                self.session += 1
                return
            self.inliers = int(inlier_mask.sum())
            if self.inliers < 18 or self.inliers / self.features < 0.60:
                self.last_good_t = 0.0
                self.session += 1
                return

            delta = math.degrees(math.atan2(mat[1, 0], mat[0, 0]))
            cx, cy = gray.shape[1] / 2, gray.shape[0] / 2
            # Tok pozadí ve středu obrazu, po odstranění rotace kolem středu.
            flow_x = mat[0, 0] * cx + mat[0, 1] * cy + mat[0, 2] - cx
            flow_y = mat[1, 0] * cx + mat[1, 1] * cy + mat[1, 2] - cy
            flow = math.hypot(flow_x, flow_y)
            self.last_flow = float(flow)
            if abs(delta) > 16.0 or flow > 130.0:
                self.last_good_t = 0.0
                self.session += 1
                return

            self.good_frames += 1
            self.last_good_t = now
            if self.downward:
                # Platí pouze pro CSI kameru namířenou kolmo dolů.
                # Znaménko yaw může vyžadovat kalibraci montáže.
                angle_before = math.radians(self.heading_deg)
                self.heading_deg += delta
                # Pixely dostupné i bez měřítka: užitečné pro budoucí
                # kalibraci na známé délce. Zatím nezasahují do SLAM XY.
                dx_px = -flow_x
                dy_px = flow_y
                self.x_px += math.cos(angle_before) * dx_px - math.sin(angle_before) * dy_px
                self.y_px += math.sin(angle_before) * dx_px + math.cos(angle_before) * dy_px
                if self.metres_per_pixel > 0:
                    self.x_m = self.x_px * self.metres_per_pixel
                    self.y_m = self.y_px * self.metres_per_pixel

    def snapshot(self, max_age=0.4):
        with self.lock:
            ready = self.downward and self.opened and bool(self.last_good_t) and (
                time.monotonic() - self.last_good_t <= max_age
            )
            return {
                "opened": self.opened,
                "ready": bool(ready),
                "mode": "downward" if self.downward else "monitor",
                "scale_calibrated": self.metres_per_pixel > 0,
                "heading_deg": round(self.heading_deg, 3) if ready else None,
                "x_m": round(self.x_m, 5) if ready and self.metres_per_pixel else None,
                "y_m": round(self.y_m, 5) if ready and self.metres_per_pixel else None,
                "x_px": round(self.x_px, 2) if ready else None,
                "y_px": round(self.y_px, 2) if ready else None,
                "session": self.session,
                "frames": self.frames,
                "good_frames": self.good_frames,
                "features": self.features,
                "inliers": self.inliers,
                "flow_px": round(self.last_flow, 2),
                "error": self.error or (
                    "Nedostatek konzistentních bodů / snímků." if not ready and self.downward else ""
                ),
            }


def camera_worker(tracker, camera_num=0, fps=10, preview=None):
    """Jednorázový opt-in pokus o otevření kamery bez přebírání cizích služeb."""
    camera = None
    try:
        from picamera2 import Picamera2
        import cv2  # noqa: F401 – kontrola závislosti
        camera = Picamera2(camera_num)
        cfg = camera.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            display=None, encode=None, buffer_count=3,
            controls={"FrameRate": float(fps)},
        )
        camera.configure(cfg)
        camera.start()
        tracker.set_opened()
        print("[CAM] Kamera připravena. Optický tok aktivní; "
              + ("JPEG náhled povolen pouze v RAM pro lokální síť."
                 if preview is not None else "bez přenosu obrazu do webu."),
              flush=True)
        interval = 1.0 / fps
        while True:
            frame = camera.capture_array()
            stamp = time.monotonic()
            tracker.process_frame(frame, timestamp=stamp)
            if preview is not None:
                preview.update(frame, timestamp=stamp)
            time.sleep(max(0.0, interval - (time.monotonic() - stamp)))
    except Exception as exc:
        # Při 'device busy' je kamera nejspíš využívaná ProjectionLabem:
        # nikdy ho neukončujeme ani se nepokoušíme kameru vyvlastnit.
        message = f"CSI kamera není dostupná: {exc}"
        tracker.set_error(message)
        print(f"[CAM] {message}. Kamera ponechána ostatním službám.", flush=True)
    finally:
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
            try:
                camera.close()
            except Exception:
                pass
