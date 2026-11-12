#!/usr/bin/env python3
"""Offline testy 2D SLAM a skládání celých otáček LiDARu."""
import math
import struct
import unittest

from decode_lidar import crc8
from slam_core import Slam2D
from slam_server import RevolutionAssembler
from test_decoder import FRAME


def scan_in_room(x=0.0, y=0.0, yaw=0.0):
    """Simulovaná obdélníková místnost 4x3 m + kulatá překážka."""
    scan = []
    for degrees in range(0, 360, 2):
        a = math.radians(degrees) + yaw
        # Lokální paprsek je [sin,cos]. Rotace ve Slam2D je yaw v matematických souřadnicích.
        # Transformace: x'=-sin(yaw)*cos(a_local)+cos(yaw)*sin(a_local)
        dx, dy = math.sin(math.radians(degrees) - yaw), math.cos(math.radians(degrees) - yaw)
        distances = []
        for wallx in (-2.0, 2.0):
            if abs(dx) > 1e-6:
                t = (wallx - x) / dx
                if t > 0 and -1.5 <= y + t * dy <= 1.5:
                    distances.append(t)
        for wally in (-1.5, 1.5):
            if abs(dy) > 1e-6:
                t = (wally - y) / dy
                if t > 0 and -2 <= x + t * dx <= 2:
                    distances.append(t)
        # Kruh: střed (0.8,0.3), poloměr 0.18m.
        cx, cy, radius = 0.8 - x, 0.3 - y, 0.18
        proj = cx*dx + cy*dy
        disc = proj*proj - (cx*cx+cy*cy-radius*radius)
        if disc >= 0:
            t = proj - math.sqrt(disc)
            if t > 0:
                distances.append(t)
        distance = min(distances) if distances else 0
        scan.append((float(degrees), int(round(distance*1000)), 230))
    return scan


def frame_at(start):
    frame = bytearray(FRAME)
    struct.pack_into("<H", frame, 4, start)
    struct.pack_into("<H", frame, 42, (start + 200) % 36000)
    frame[-1] = crc8(frame[:-1])
    return bytes(frame)


class SlamTest(unittest.TestCase):
    def test_occupancy_and_pgm(self):
        slam = Slam2D(size=160)
        self.assertTrue(slam.process_scan(scan_in_room()))
        self.assertTrue(slam.process_scan(scan_in_room()))
        state = slam.snapshot()
        self.assertEqual(state["scans"], 2)
        self.assertEqual(state["rejected"], 0)
        self.assertTrue(state["initialized"])
        self.assertTrue(any(cell[2] >= 3 for cell in state["cells"]))
        self.assertTrue(any(cell[2] <= -2 for cell in state["cells"]))
        pgm = slam.pgm()
        self.assertTrue(pgm.startswith(b"P5\n"))
        self.assertEqual(len(pgm.split(b"\n", 4)[-1]), 160*160)
        self.assertIn("x_m,y_m", slam.trajectory_csv())

    def test_small_translation_tracks_without_odometry(self):
        slam = Slam2D(size=160)
        self.assertTrue(slam.process_scan(scan_in_room()))
        self.assertTrue(slam.process_scan(scan_in_room(x=.09, y=.05, yaw=.02)))
        x, y, angle = slam.pose
        self.assertLess(abs(x - .09), .17)
        self.assertLess(abs(y - .05), .17)
        self.assertLess(abs(math.degrees(angle) - math.degrees(.02)), 7.0)

    def test_reset_clears_trajectory(self):
        slam = Slam2D(size=160)
        slam.process_scan(scan_in_room())
        slam.reset()
        self.assertEqual(slam.snapshot()["scans"], 0)
        self.assertFalse(slam.snapshot()["cells"])

    def test_revolution_assembler(self):
        asm = RevolutionAssembler()
        first = b"".join(frame_at(32000 + i*400) for i in range(9))
        wrapped = frame_at(200)
        # Complete 108 points only after angle wrap.
        self.assertEqual(asm.feed(first), [])
        result = asm.feed(wrapped[:10]) + asm.feed(wrapped[10:])
        self.assertEqual(len(result), 1)
        self.assertEqual(len(result[0]), 108)
        self.assertEqual(asm.bad_crc, 0)


if __name__ == "__main__":
    unittest.main()
