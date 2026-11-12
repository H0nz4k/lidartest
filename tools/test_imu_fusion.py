#!/usr/bin/env python3
"""Offline test časovaného MPU-6050 gyra a jeho použití v LiDAR scan matchingu."""
import math
import time
import unittest

from imu_fusion import ImuAngleTracker
from slam_core import Slam2D
from test_slam2d import scan_in_room


class ImuHeadingTest(unittest.TestCase):
    def test_relative_gyro_integrates_rotation(self):
        imu = ImuAngleTracker()
        start = time.monotonic()
        imu.calibrate(0.25, 0.04, now=start)
        imu.add_sample(90.25, now=start + 0.10)
        status = imu.snapshot(max_age=100)
        self.assertTrue(status["ready"])
        self.assertAlmostEqual(status["heading_deg"], 9.0, delta=0.01)
        self.assertAlmostEqual(status["rate_dps"], 90.0, delta=0.01)
        self.assertEqual(status["samples"], 1)

    def test_imu_fails_closed_on_error(self):
        imu = ImuAngleTracker()
        imu.calibrate(-0.1, 0.05)
        imu.set_error("I2C EIO")
        status = imu.snapshot()
        self.assertFalse(status["ready"])
        self.assertIsNone(status["heading_deg"])
        self.assertIn("EIO", status["error"])

    def test_imu_delta_prior_and_fallback(self):
        slam = Slam2D(size=160)
        base = scan_in_room()
        self.assertTrue(slam.process_scan(base, imu_heading_deg=0.0))
        self.assertTrue(slam.process_scan(base, imu_heading_deg=0.0))
        moved = scan_in_room(yaw=math.radians(3))
        self.assertTrue(slam.process_scan(moved, imu_heading_deg=3.0))
        result = slam.snapshot()
        self.assertTrue(result["imu_used"])
        self.assertAlmostEqual(result["imu_delta_deg"], 3.0, places=1)
        self.assertAlmostEqual(result["pose"][2], 3.0, delta=4.0)

        # Přechod do fallbacku nepřenáší starý IMU offset.
        self.assertTrue(slam.process_scan(moved, imu_heading_deg=None))
        result = slam.snapshot()
        self.assertFalse(result["imu_used"])


if __name__ == "__main__":
    unittest.main()
