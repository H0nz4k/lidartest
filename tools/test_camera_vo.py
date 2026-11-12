#!/usr/bin/env python3
"""Offline testy CSI optického toku a bezpečné tří-senzorové integrace."""
import math
import time
import unittest

import cv2
import numpy as np

from camera_vo import VisualOdometry
from slam_core import Slam2D
from test_slam2d import scan_in_room


def make_scene():
    rng = np.random.default_rng(6050)
    image = np.zeros((480, 640), dtype=np.uint8)
    for _ in range(210):
        x = int(rng.integers(15, 625))
        y = int(rng.integers(15, 465))
        cv2.circle(image, (x, y), 3, 255, -1)
    return image


class VisualOdometryTest(unittest.TestCase):
    def test_monitor_never_supplies_slam_prior(self):
        tracker = VisualOdometry()
        tracker.set_opened()
        image = make_scene()
        base = time.monotonic()
        tracker.process_frame(image, base)
        shifted = cv2.warpAffine(image, np.float32([[1, 0, 4], [0, 1, -3]]),
                                 (640, 480))
        tracker.process_frame(shifted, base + .1)
        result = tracker.snapshot()
        self.assertTrue(result["opened"])
        self.assertGreater(result["inliers"], 25)
        self.assertFalse(result["ready"])
        self.assertIsNone(result["heading_deg"])
        self.assertEqual(result["mode"], "monitor")

    def test_downward_optical_flow_yaw_and_optional_metric_scale(self):
        image = make_scene()
        base = time.monotonic()
        center = (320, 240)
        matrix = cv2.getRotationMatrix2D(center, 3.0, 1.0)
        matrix[0, 2] += 4
        matrix[1, 2] += 2
        shifted = cv2.warpAffine(image, matrix, (640, 480))
        tracker = VisualOdometry(downward=True, metres_per_pixel=0.002)
        tracker.set_opened()
        tracker.process_frame(image, base)
        tracker.process_frame(shifted, base + .1)
        result = tracker.snapshot()
        self.assertTrue(result["ready"])
        self.assertGreater(result["inliers"], 25)
        self.assertAlmostEqual(result["heading_deg"], -3, delta=1.5)
        self.assertTrue(result["scale_calibrated"])
        self.assertAlmostEqual(result["x_m"], -0.008, delta=0.012)

    def test_camera_error_resets_session(self):
        tracker = VisualOdometry(downward=True)
        tracker.set_opened()
        tracker.process_frame(make_scene())
        tracker.set_error("Camera busy")
        result = tracker.snapshot()
        self.assertFalse(result["ready"])
        self.assertFalse(result["opened"])
        self.assertIn("busy", result["error"])
        self.assertEqual(result["session"], 1)

    def test_camera_yaw_used_only_when_explicitly_enabled(self):
        slam = Slam2D(size=160)
        image = scan_in_room()
        slam.process_scan(image, imu_heading_deg=0.0,
                          visual={"ready": False, "mode": "monitor"})
        self.assertTrue(slam.process_scan(image, imu_heading_deg=0.0,
                                          visual={"ready": False, "mode": "monitor"}))
        self.assertFalse(slam.snapshot()["visual_used"])
        v0 = {"ready": True, "mode": "downward", "heading_deg": 0.0,
              "session": 1, "scale_calibrated": False}
        v1 = {**v0, "heading_deg": 3.0}
        # Připojení kamery u již rozběhnutého SLAMu zakládá novou referenci.
        self.assertTrue(slam.process_scan(image, imu_heading_deg=0.0, visual=v0))
        self.assertFalse(slam.snapshot()["visual_used"])
        self.assertTrue(slam.process_scan(
            scan_in_room(yaw=math.radians(3)), imu_heading_deg=3.0,
            visual=v1
        ))
        result = slam.snapshot()
        self.assertTrue(result["visual_used"])
        self.assertFalse(result["visual_translation_used"])

    def test_unavailable_visual_session_never_extrapolates(self):
        slam = Slam2D(size=160)
        base = scan_in_room()
        a = {"ready": True, "mode": "downward", "heading_deg": 0.0,
             "session": 1, "scale_calibrated": False}
        slam.process_scan(base, visual=a)
        slam.process_scan(base, visual={**a, "heading_deg": 1.0, "session": 2})
        self.assertFalse(slam.snapshot()["visual_used"])


if __name__ == "__main__":
    unittest.main()
