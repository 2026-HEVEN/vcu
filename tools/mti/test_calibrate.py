import struct
import io
from contextlib import redirect_stdout
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from calibrate import Parser, decode, packet, validate, select_port, confirm


class ProtocolTests(unittest.TestCase):
    def test_prompt_has_newline_before_input(self):
        output = io.StringIO()
        def answer():
            self.assertEqual(output.getvalue(), 'Confirm (q = cancel):\n')
            return 'zero'
        with redirect_stdout(output), patch('builtins.input', side_effect=answer):
            confirm('Confirm', 'ZERO')

    def test_confirmation_case_and_retry(self):
        for value in ('zero', 'ZERO', ' Zero '):
            with patch('builtins.input', return_value=value):
                confirm('Confirm', 'ZERO')
        with patch('builtins.input', side_effect=['', 'wrong', 'zero']):
            confirm('Confirm', 'ZERO')

    def test_confirmation_cancel_and_eof(self):
        for values in (['q'], ['', '', '']):
            with patch('builtins.input', side_effect=values), self.assertRaises(ValueError):
                confirm('Confirm', 'ZERO')
        with patch('builtins.input', side_effect=EOFError), self.assertRaises(ValueError):
            confirm('Confirm', 'ZERO')

    def test_port_selection(self):
        usb = SimpleNamespace(device='COM13', description='CP210x', vid=0x10C4, pid=0xEA60)
        bt = SimpleNamespace(device='COM3', description='Bluetooth', vid=None, pid=None)
        self.assertEqual(select_port([bt, usb]), 'COM13')
        self.assertEqual(select_port([], 'COM99'), 'COM99')
        with self.assertRaises(ValueError):
            select_port([bt])
        with self.assertRaises(ValueError):
            select_port([usb, SimpleNamespace(device='COM14', description='CH340', vid=0x1A86, pid=0x7523)])

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
