#!/usr/bin/env python3
"""Lokální testy parseru živé mapy s reálným rámcem ETA Raggio."""
import unittest

from live_map import ScanState, BIN_COUNT
from test_decoder import FRAME


class LiveMapTest(unittest.TestCase):
    def test_real_frame_incremental(self):
        state = ScanState()
        state.set_connection(True, "")
        state.feed(b"\x00\x12" + FRAME[:8])
        state.feed(FRAME[8:20])
        state.feed(FRAME[20:])
        result = state.snapshot()
        self.assertEqual(result["frames"], 1)
        self.assertEqual(result["bad_crc"], 0)
        self.assertEqual(result["bytes"], len(FRAME) + 2)
        self.assertTrue(result["streaming"])
        self.assertEqual(len(state.bins), BIN_COUNT)
        self.assertTrue(result["points"])
        self.assertGreater(result["nearest_mm"], 0)
        point = min(result["points"], key=lambda row: abs(row[0] - 183.7))
        self.assertAlmostEqual(point[0], 183.5, delta=0.5)
        self.assertEqual(point[1], 610)
        self.assertEqual(point[2], 229)

    def test_bad_crc_discarded_then_recovers(self):
        state = ScanState()
        bad = bytearray(FRAME)
        bad[10] ^= 0x02
        state.feed(bad + FRAME)
        result = state.snapshot()
        self.assertEqual(result["frames"], 1)
        self.assertGreaterEqual(result["bad_crc"], 1)

    def test_zero_distance_clears_existing_bin(self):
        state = ScanState()
        state.feed(FRAME)
        self.assertTrue(state.snapshot()["points"])
        zero = bytearray(FRAME)
        zero[6:8] = b"\x00\x00"
        from decode_lidar import crc8
        zero[-1] = crc8(zero[:-1])
        state.feed(zero)
        index = round((183.7 * 2)) % BIN_COUNT
        self.assertEqual(state.bins[index][0], 0)


if __name__ == "__main__":
    unittest.main()
