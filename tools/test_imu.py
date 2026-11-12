#!/usr/bin/env python3
"""Regrese: Debian python3-smbus SMBus není context manager."""
import contextlib
import importlib.util
import io
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch


class FakeSMBus:
    instance = None

    def __init__(self, bus_id):
        self.bus_id = bus_id
        self.closed = False
        self.writes = []
        self.__class__.instance = self

    def read_byte_data(self, address, register):
        if (address, register) == (0x68, 0x75):
            return 0x68
        raise OSError("No response")

    def write_byte_data(self, address, register, value):
        self.writes.append((address, register, value))

    def read_i2c_block_data(self, address, register, length):
        assert address == 0x68 and register == 0x3B and length == 14
        # 1g v ose Z, nulová úhlová rychlost, zbytek nula.
        return list(struct.pack(">hhhhhhh", 0, 0, 16384, 0, 0, 0, 0))

    def close(self):
        self.closed = True


class TestImuSMBus(unittest.TestCase):
    def test_plain_smbus_without_context_manager(self):
        fake_package = types.ModuleType("smbus")
        fake_package.SMBus = FakeSMBus
        with patch.dict(sys.modules, {"smbus": fake_package}):
            spec = importlib.util.spec_from_file_location(
                "imu_test_regression", Path(__file__).with_name("imu_test.py")
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

        tick = [0.0]

        def clock():
            tick[0] += 0.03
            return tick[0]

        output = io.StringIO()
        with patch.object(sys, "argv", ["imu_test.py", "--seconds", "0.25", "--calibration", "1"]), \
                patch.object(module.time, "monotonic", clock), \
                patch.object(module.time, "sleep", lambda _: None), \
                contextlib.redirect_stdout(output):
            result = module.main()

        self.assertEqual(result, 0, output.getvalue())
        self.assertIn("MPU-6050 nalezen", output.getvalue())
        self.assertIn("Konec: relativní otočení", output.getvalue())
        self.assertEqual(FakeSMBus.instance.bus_id, 1)
        self.assertTrue(FakeSMBus.instance.closed)
        self.assertGreaterEqual(len(FakeSMBus.instance.writes), 5)


if __name__ == "__main__":
    unittest.main()
