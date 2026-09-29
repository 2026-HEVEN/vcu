import struct
import unittest
from calibrate import Parser, decode, packet, validate


class ProtocolTests(unittest.TestCase):
    def test_wire_commands(self):
        self.assertEqual(packet(0xA4, b'\0\3').hex(), 'faffa402000358')
        self.assertEqual(packet(0xA4, b'\0\0').hex(), 'faffa40200005b')

    def test_partial_and_boot_noise(self):
        p = Parser()
        msg = packet(1, bytes.fromhex('02d01b9c'))
        self.assertEqual(list(p.feed(b'boot\r\n' + msg[:4])), [])
        self.assertEqual(list(p.feed(msg[4:])), [(1, bytes.fromhex('02d01b9c'))])

    def test_checksum_resync(self):
        p = Parser()
        bad = bytearray(packet(0xA5))
        bad[-1] ^= 1
        self.assertEqual(list(p.feed(bad + packet(0xA5))), [(0xA5, b'')])
        self.assertEqual(p.errors, 1)

    def test_decode_and_format_rejection(self):
        payload = b'\x40\x20\x0c' + struct.pack('>fff', 0, 0, 9.81)
        self.assertAlmostEqual(decode(payload)['acc_mps2'][2], 9.81, places=4)
        with self.assertRaises(ValueError):
            decode(b'\x40\x24' + payload[2:])
        with self.assertRaises(ValueError):
            decode(payload[:-1])

    def test_missing_and_moving_fail_closed(self):
        with self.assertRaises(ValueError):
            validate({})
        s = {key+'_count': 150 for key in ('rpy_deg', 'acc_mps2', 'gyro_rads')}
        s.update(rpy_deg=[0, 0, 0], acc_mps2=[0, 0, 9.81],
                 gyro_rads=[0, 0, 0], acc_mps2_std=[0, 0, 0])
        validate(s, zero=True)
        s['gyro_rads'] = [0, 0, 0.2]
        with self.assertRaises(ValueError):
            validate(s)


if __name__ == '__main__':
    unittest.main()
