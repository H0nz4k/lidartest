#!/usr/bin/env python3
"""Regrese: 10+ LiDAR otáček/s, časové přiřazení senzorů, žádné skryté ztráty."""
import math
from queue import Queue
import time
import unittest

from imu_fusion import ImuAngleTracker
from scan_pipeline import PipelineStats, queue_scan
from sensor_sync import TimedHistory
from slam_core import Slam2D
from slam_server import RevolutionAssembler
from test_slam2d import frame_at, scan_in_room


class TimelineTests(unittest.TestCase):
    def test_interpolate_only_within_session(self):
        history = TimedHistory()
        history.add(10.0, {"heading_deg": 10.0, "session": 2})
        history.add(10.1, {"heading_deg": 20.0, "session": 2})
        middle = history.at(10.05, max_age=0.15)
        self.assertAlmostEqual(middle["heading_deg"], 15, places=3)
        self.assertAlmostEqual(middle["sync_ms"], 50, places=1)
        self.assertIsNone(history.at(10.26, max_age=0.15))
        self.assertIsNone(history.at(9.99))
        history.add(10.2, {"heading_deg": 0.0, "session": 3})
        gap = history.at(10.15, max_age=0.15)
        self.assertEqual(gap["session"], 2)
        self.assertEqual(gap["heading_deg"], 20.0)
        history.clear()
        self.assertIsNone(history.at(10.22))

    def test_imu_heading_at_scan_end(self):
        sensor = ImuAngleTracker()
        base = time.monotonic()
        sensor.calibrate(0.0, 0.02, now=base)
        sensor.add_sample(90.0, now=base + 0.10)
        sensor.add_sample(90.0, now=base + 0.20)
        result = sensor.sample_at(base + 0.15)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result["heading_deg"], 13.5, places=2)
        self.assertIsNone(sensor.sample_at(base + 0.40))
        session = result["session"]
        sensor.calibrate(0.0, 0.02, now=base + 0.50)
        sensor.add_sample(0.0, now=base + 0.51)
        self.assertNotEqual(sensor.sample_at(base + 0.51)["session"], session)
        self.assertIsNone(sensor.sample_at(base + 0.15))

    def test_revolution_event_preserves_reception_time(self):
        asm = RevolutionAssembler()
        first = b"".join(frame_at(32000 + i*400) for i in range(9))
        self.assertEqual(asm.feed_events(first, received_at=10.0), [])
        events = asm.feed_events(frame_at(200), received_at=10.03)
        self.assertEqual(len(events), 1)
        points, stamp = events[0]
        self.assertEqual(len(points), 108)
        self.assertEqual(stamp, 10.03)

    def test_ten_scans_per_second_are_all_queued_without_gate(self):
        q = Queue(maxsize=16)
        stats = PipelineStats()
        for i in range(11):
            stats.received_scan(time.monotonic())
            self.assertTrue(queue_scan(q, i, stats))
        snap = stats.snapshot(q.qsize())
        self.assertEqual(q.qsize(), 11)
        self.assertEqual(snap["received"], 11)
        self.assertEqual(snap["enqueued"], 11)
        self.assertEqual(snap["dropped_overload"], 0)
        self.assertEqual([q.get_nowait() for _ in range(11)], list(range(11)))

    def test_overload_drops_oldest_and_counts_loss(self):
        q = Queue(maxsize=2)
        stats = PipelineStats()
        for i in (0, 1, 2):
            stats.received_scan()
            self.assertTrue(queue_scan(q, i, stats))
        self.assertEqual([q.get_nowait(), q.get_nowait()], [1, 2])
        self.assertEqual(stats.snapshot(0)["dropped_overload"], 1)

    def test_slam_does_not_mix_imu_recalibrations(self):
        slam = Slam2D(size=160)
        scan = scan_in_room()
        self.assertTrue(slam.process_scan(scan, imu_heading_deg=90, imu_session=1))
        self.assertTrue(slam.process_scan(scan, imu_heading_deg=92, imu_session=1))
        self.assertTrue(slam.snapshot()["imu_used"])
        # Kalibrace přepne session a vynuluje relativní yaw.
        self.assertTrue(slam.process_scan(scan, imu_heading_deg=0, imu_session=2))
        self.assertFalse(slam.snapshot()["imu_used"])
        self.assertTrue(slam.process_scan(scan, imu_heading_deg=1, imu_session=2))
        self.assertTrue(slam.snapshot()["imu_used"])


if __name__ == "__main__":
    unittest.main()
