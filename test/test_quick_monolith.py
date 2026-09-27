"""Host-only regression tests for the raw Monolith reader."""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from quick_monolith import BLOCK, DRIVE, L1, R1, RECORD, SPEED, summarize


def record(ms, can_id, payload):
    return RECORD.pack(0xAE, 2, ms, can_id, 8, payload)


def feedback(voltage_tenths, bus_tenths, phase_tenths=0):
    return (voltage_tenths.to_bytes(2, 'little') +
            (32000 + bus_tenths).to_bytes(2, 'little') +
            (32000 + phase_tenths).to_bytes(2, 'little') +
            (32000).to_bytes(2, 'little'))


class QuickMonolithTest(unittest.TestCase):
    def test_summary_and_timed_pair(self):
        frames = [
            record(0, L1, feedback(500, 100)),
            record(20, R1, feedback(500, 200)),
            record(50, SPEED, bytes([250, 0, 1, 0, 0, 0, 0, 0])),
            record(70, DRIVE, bytes([100, 0, 200, 0, 90, 0, 180, 0])),
            record(100, BLOCK, bytes([8, 0, 8, 0, 1, 0, 0, 0])),
            record(200, L1, feedback(350, 0)),
            record(210, R1, feedback(350, 0)),
        ]
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / 'sample.log'
            log.write_bytes(b''.join(frames))
            result = summarize(log)
        self.assertEqual(result['peak_speed']['kph'], 25.0)
        self.assertEqual(result['peak_controller_bus_sum']['sum_ibus_a'], 30.0)
        self.assertEqual(result['peak_command_phase_sum_abs']['sum_abs_a'], 30.0)
        self.assertEqual(result['peak_reported_phase_sum_abs']['sum_abs_a'], 27.0)
        self.assertEqual(result['voltage_drop_count'], 2)
        self.assertEqual(result['block_events'][0]['current_names'], ['feedback_stale'])

    def test_rejects_truncated_file(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / 'bad.log'
            log.write_bytes(b'bad')
            with self.assertRaisesRegex(ValueError, '24바이트'):
                summarize(log)


if __name__ == '__main__':
    unittest.main()
