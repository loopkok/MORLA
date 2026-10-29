import math
import struct
import unittest
from unittest.mock import Mock

from vader5_sdk import Vader5, _state
from vader5_sdk._vendor import protocol


class ProtocolTests(unittest.TestCase):
    def test_known_report_and_units(self):
        data = bytearray(32)
        data[:3] = b'\x5a\xa5\xef'
        struct.pack_into('<4h', data, 3, -32768, 32767, 0, 16384)
        data[11] = 0x10  # A
        data[13] = 0x04  # M1
        data[15:17] = bytes([255, 0])
        struct.pack_into('<3h', data, 17, 16384, 0, -16384)
        struct.pack_into('<3h', data, 23, 0, 0, 4096)
        state = _state(protocol.parse_input(bytes(data)))
        self.assertEqual(state.left_stick, (-1, 1))
        self.assertEqual(state.triggers, (1, 0))
        self.assertEqual(state.buttons, frozenset(('A', 'M1')))
        self.assertTrue(state.pressed('m1'))
        self.assertFalse(state.pressed('M2'))
        self.assertAlmostEqual(state.gyro_rad_s[0], math.radians(1000))
        self.assertAlmostEqual(state.accel_m_s2[2], 9.80665)
        with self.assertRaises(ValueError):
            state.pressed('typo')

    def test_invalid_reports_and_report_id(self):
        for packet in (b'', b'\x5a\xa5\xef', bytes(32), b'\x5a\xa5\x01' + bytes(29)):
            self.assertIsNone(protocol.parse_input(packet))
        packet = b'\x5a\xa5\xef' + bytes(29)
        self.assertEqual(protocol.strip_report_id(b'\x00' + packet), packet)

    def test_every_known_button_is_independent(self):
        for offset, mask, name in protocol.BUTTON_BITS:
            with self.subTest(name=name):
                packet = bytearray(b'\x5a\xa5\xef' + bytes(29))
                packet[offset] = mask
                self.assertEqual(protocol.parse_input(packet).buttons, frozenset((name,)))

    def test_no_cached_state_on_timeout(self):
        pad = Vader5()
        pad._controller = Mock(info=None)
        pad._controller.poll.return_value = []
        with self.assertRaises(TimeoutError):
            pad.read(timeout=0.002)
        pad.close()
        pad.close()
        with self.assertRaises(RuntimeError):
            pad.read()

    def test_wrong_model_rejected(self):
        pad = Vader5()
        pad._controller = Mock()
        pad._controller.info.device_id = 128
        with self.assertRaises(RuntimeError):
            pad.read()


if __name__ == '__main__':
    unittest.main()
