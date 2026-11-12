#!/usr/bin/env python3
"""Test dekodéru na reálném 47b rámci z ETA Raggio, bez HW."""
import csv
import io
import unittest

from decode_lidar import Decoder, crc8

# První úplný rámec z odposlechu 2026-11-12, 115200 baud.
FRAME = bytes.fromhex(
    "54 2c 78 0e c2 47 62 02 e5 63 02 e3 68 02 e5 6e "
    "02 e3 74 02 e2 7b 02 e6 84 02 e2 8d 02 e4 97 02 "
    "e3 e4 01 e3 d2 01 e8 c1 01 e8 c9 4e 52 11 0e"
)


class DecodeTest(unittest.TestCase):
    def test_crc_and_points(self):
        self.assertEqual(len(FRAME), 47)
        self.assertEqual(crc8(FRAME[:-1]), FRAME[-1])
        out = io.StringIO()
        writer = csv.writer(out)
        d = Decoder()
        for chunk in (b"\x12\x34" + FRAME[:7], FRAME[7:28], FRAME[28:]):
            d.feed(chunk, writer)
        self.assertEqual((d.frames, d.bad_crc, d.points, d.nonzero), (1, 0, 12, 12))
        rows = list(csv.reader(io.StringIO(out.getvalue())))
        self.assertEqual(rows[0][:3], ["183.70", "610", "229"])
        self.assertAlmostEqual(float(rows[-1][0]), 201.69)
        self.assertAlmostEqual(float(rows[0][3]), 617.33, delta=0.02)

    def test_crc_error_is_rejected(self):
        damaged = bytearray(FRAME)
        damaged[12] ^= 1
        d = Decoder()
        d.feed(damaged)
        self.assertEqual(d.frames, 0)
        self.assertEqual(d.bad_crc, 1)


if __name__ == "__main__":
    unittest.main()
