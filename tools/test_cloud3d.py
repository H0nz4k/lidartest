#!/usr/bin/env python3
"""Offline matematika 3D naklápění, limit paměti, PLY a MPU roll/pitch."""
import math
import time
import unittest

from cloud3d import StationaryCloud3D, point_xyz
from imu_fusion import ImuAngleTracker
from test_slam2d import scan_in_room


class Stationary3DTests(unittest.TestCase):
    def test_horizontal_2d_scan_stays_at_zero_height(self):
        for a in (0, 45, 90, 180, 315):
            p = point_xyz(a, 1000, 0, axis="x")
            self.assertAlmostEqual(p[2], 0.0, places=6)

    def test_roll_90_turns_forward_ray_vertical(self):
        x, y, z = point_xyz(0, 1000, 90, "x")
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 1.0, places=6)

    def test_pitch_90_turns_right_ray_down(self):
        x, y, z = point_xyz(90, 1000, 90, "y")
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, -1.0, places=6)

    def test_rotation_about_offset_pivot(self):
        # Střed LiDARu je o 10 cm před fyzickou osou naklápění.
        x, y, z = point_xyz(0, 1000, 90, "x", lever=(0, 0.1, 0))
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, -0.1, places=6)
        self.assertAlmostEqual(z, 1.1, places=6)

    def test_collect_steps_and_export_real_ascii_ply(self):
        cloud = StationaryCloud3D(axis="x", tilt_step_deg=2, max_points=1000)
        room = scan_in_room()
        imu0 = {"roll_deg": 0, "pitch_deg": 0, "session": 1}
        self.assertFalse(cloud.add_scan(room, None))
        self.assertTrue(cloud.add_scan(room, imu0))
        self.assertFalse(cloud.add_scan(room, {"roll_deg": 1, "pitch_deg": 0, "session": 1}))
        self.assertTrue(cloud.add_scan(room, {"roll_deg": 8, "pitch_deg": 0, "session": 1}))
        snapshot = cloud.snapshot(preview_limit=100)
        self.assertEqual(snapshot["scans_saved"], 2)
        self.assertEqual(snapshot["scans_skipped"], 1)
        self.assertEqual(snapshot["scans_bad_pose"], 1)
        self.assertGreater(snapshot["point_count"], 200)
        self.assertLessEqual(snapshot["preview_count"], 100)
        self.assertEqual(snapshot["tilt_min_deg"], 0)
        self.assertEqual(snapshot["tilt_max_deg"], 8)
        ply = cloud.ply()
        self.assertTrue(ply.startswith(b"ply\nformat ascii 1.0\n"))
        header, data = ply.split(b"end_header\n", 1)
        self.assertIn(
            ("element vertex " + str(snapshot["point_count"])).encode(), header
        )
        self.assertEqual(len(data.splitlines()), snapshot["point_count"])
        self.assertEqual(len(data.splitlines()[0].split()), 4)
        cloud.reset()
        self.assertEqual(cloud.snapshot()["point_count"], 0)

    def test_guard_secondary_axis_and_calibration_session(self):
        cloud = StationaryCloud3D()
        room = scan_in_room()
        imu = {"roll_deg": 0, "pitch_deg": 0, "session": 2}
        self.assertTrue(cloud.add_scan(room, imu))
        self.assertFalse(cloud.add_scan(room, {"roll_deg": 3, "pitch_deg": 20, "session": 2}))
        self.assertIn("vedlejší", cloud.snapshot()["error"])
        self.assertFalse(cloud.add_scan(room, {"roll_deg": 3, "pitch_deg": 0, "session": 3}))
        self.assertIn("znovu", cloud.snapshot()["error"])
        self.assertEqual(cloud.snapshot()["scans_saved"], 1)

    def test_memory_limit_prevents_unbounded_map(self):
        cloud = StationaryCloud3D(max_points=100)
        room = scan_in_room()
        self.assertTrue(cloud.add_scan(room, {"roll_deg": 0, "pitch_deg": 0, "session": 1}))
        self.assertTrue(cloud.snapshot()["full"])
        self.assertEqual(cloud.snapshot()["point_count"], 100)
        self.assertFalse(cloud.add_scan(room, {"roll_deg": 5, "pitch_deg": 0, "session": 1}))

    def test_imu_tilt_from_acc_and_gyro(self):
        tracker = ImuAngleTracker()
        now = time.monotonic()
        tracker.calibrate(0.0, 0.02, now=now,
                          accel_g=(0.0, 0.0, 1.0),
                          gyro_xy_bias_dps=(0.0, 0.0))
        self.assertTrue(tracker.snapshot(max_age=10)["tilt_ready"])
        rad = math.radians(6)
        tracker.add_sample(0.0, now=now+0.10,
                           gyro_xy_dps=(60.0, 0.0),
                           accel_g=(0.0, math.sin(rad), math.cos(rad)))
        data = tracker.sample_at(now + 0.10)
        self.assertIsNotNone(data)
        self.assertAlmostEqual(data["roll_deg"], 6, delta=1.0)
        self.assertAlmostEqual(data["pitch_deg"], 0, delta=0.5)
        self.assertIn("session", data)

    def test_imu_without_accel_cannot_fake_3d_orientation(self):
        tracker = ImuAngleTracker()
        now = time.monotonic()
        tracker.calibrate(0.0, 0.04, now=now)
        tracker.add_sample(0.0, now=now+0.04)
        self.assertFalse(tracker.snapshot(max_age=10)["tilt_ready"])
        self.assertIsNone(tracker.sample_at(now+0.04)["roll_deg"])


if __name__ == "__main__":
    unittest.main()
