#!/usr/bin/env python3
"""Fast, read-only summary of native Monolith .log files (24-byte records).

Uses the VCU telemetry decoder in decode_log_frames.py. No third-party packages.
This is diagnostic evidence, not a BMS/contactor fault verdict.
"""

import argparse
import json
import mmap
import struct
import sys
from pathlib import Path

from decode_log_frames import DRIVE, MOTOR, TV_YAW, TV_LOAD, decode

RECORD = struct.Struct('<BB2xII2xB1x8s')
L1, R1 = 0x1801D0EF, 0x1801D0F0
L2, R2 = 0x1802D0EF, 0x1802D0F0
STATUS, CONTROL, SPEED = 0x1801C0D0, 0x1807C0D0, 0x1803C0D0
BLOCK, FAULT, THROTTLE = 0x1C07C0D0, 0x1C08C0D0, 0x1C06C0D0
BMS, EM = 0x18F3FFC0, 0x1CF5FFC1
IDS = {L1, R1, L2, R2, STATUS, CONTROL, SPEED, BLOCK, FAULT,
       THROTTLE, BMS, EM, DRIVE, MOTOR, TV_YAW, TV_LOAD}
BLOCK_NAMES = ('throttle_invalid', 'safety_fsm', 'direction_gear',
               'feedback_stale', 'fault_latch', 'speed_mode', 'snapshot_stale',
               'scheduler_heartbeat', 'reconnect_inhibit', 'component_test',
               'nonfinite_command', 'thermal_zero', 'paddock_sensor_invalid')


def u16(data, offset=0):
    return struct.unpack_from('<H', data, offset)[0]


def names(mask):
    return [name for bit, name in enumerate(BLOCK_NAMES) if mask & (1 << bit)]


def fresh(value, now, max_age_ms=150):
    return value is not None and 0 <= now - value['t_ms'] <= max_age_ms


def summarize(path, around=None):
    path = Path(path)
    if not path.is_file():
        raise ValueError(f'파일 없음: {path}')
    size = path.stat().st_size
    if not size or size % RECORD.size:
        raise ValueError(f'24바이트 레코드 크기와 맞지 않음: {size} bytes')

    count = 0
    can_count = 0
    first_ms = last_ms = None
    counts = {}
    last_seen = {}
    feedback = {L1: None, R1: None}
    status = control = motor = None
    speed_peak = None
    bus_peak = None
    power_peak = None
    phase_peak = None
    command_peak = None
    loaded_min_v = None
    bms_peak_discharge = None
    bms_min_v = None
    em_count = 0
    probes = {L1: 0, R1: 0}
    gaps = {L1: [], R1: [], L2: [], R2: []}
    tv_active_frames = tv_request_frames = regen_active_frames = 0
    block_events = []
    block_previous = block_previous_count = None
    last_block = last_fault = last_throttle = None
    voltage_drops = []
    timeline = []
    fault_events = []
    last_fault_state = None

    with path.open('rb') as fp, mmap.mmap(fp.fileno(), 0, access=mmap.ACCESS_READ) as data:
        for offset in range(0, size, RECORD.size):
            magic, record_type, t, cid, dlc, payload = RECORD.unpack_from(data, offset)
            count += 1
            if magic != 0xAE:
                raise ValueError(f'잘못된 Monolith magic: record {count}, 0x{magic:02X}')
            if first_ms is None:
                first_ms = t
            last_ms = t
            if record_type != 2 or dlc != 8:
                continue
            can_count += 1
            if cid not in IDS:
                continue
            counts[cid] = counts.get(cid, 0) + 1
            if cid in (L1, R1) and payload == b'\x55' * 8:
                probes[cid] += 1
                continue
            if cid in (L1, R1, L2, R2):
                prior = last_seen.get(cid)
                if prior is not None and t - prior > 250:
                    gaps[cid].append({'from_s': prior / 1000, 'to_s': t / 1000,
                                      'gap_s': round((t - prior) / 1000, 3)})
                last_seen[cid] = t
            elif cid != EM:
                last_seen[cid] = t

            if cid in (L1, R1):
                v = round(u16(payload) / 10, 1)
                ibus = round(u16(payload, 2) / 10 - 3200, 1)
                phase = round(u16(payload, 4) / 10 - 3200, 1)
                rpm = u16(payload, 6) - 32000
                previous = feedback[cid]
                feedback[cid] = {'t_ms': t, 'v': v, 'ibus': ibus,
                                 'phase': phase, 'rpm': rpm}
                if previous and 0 < t - previous['t_ms'] <= 500 and previous['v'] - v >= 8:
                    voltage_drops.append({'t_s': t / 1000, 'side': 'L' if cid == L1 else 'R',
                                          'from_v': previous['v'], 'to_v': v,
                                          'prior_ibus_a': previous['ibus']})
                other = feedback[R1 if cid == L1 else L1]
                if fresh(other, t):
                    left, right = feedback[L1], feedback[R1]
                    pair = {'t_s': t / 1000, 'left_v': left['v'], 'right_v': right['v'],
                            'left_ibus_a': left['ibus'], 'right_ibus_a': right['ibus'],
                            'sum_ibus_a': round(left['ibus'] + right['ibus'], 1)}
                    if left['v'] >= 30 and right['v'] >= 30:
                        if bus_peak is None or pair['sum_ibus_a'] > bus_peak['sum_ibus_a']:
                            bus_peak = pair
                        power = round(left['v'] * left['ibus'] + right['v'] * right['ibus'])
                        if power_peak is None or power > power_peak['dc_power_w']:
                            power_peak = {**pair, 'dc_power_w': power}
                        if abs(pair['sum_ibus_a']) >= 20:
                            mv = min(left['v'], right['v'])
                            if loaded_min_v is None or mv < loaded_min_v['min_v']:
                                loaded_min_v = {**pair, 'min_v': mv}
            elif cid == STATUS:
                status = {'t_ms': t, 'throttle_pct': payload[3],
                          'throttle_valid': bool(payload[1] & 8),
                          'brake': bool(payload[1] & 1), 'paddock': bool(payload[1] & 16)}
            elif cid == CONTROL:
                control = {'t_ms': t, 'allow': bool(payload[2] & 16),
                           'tv_active': bool(payload[2] & 1),
                           'regen_active': bool(payload[2] & 4),
                           'tv_request': bool(payload[1] & 1),
                           'regen_request': bool(payload[1] & 2),
                           'tv_block': payload[3], 'regen_block': payload[4]}
                tv_active_frames += control['tv_active']
                tv_request_frames += control['tv_request']
                regen_active_frames += control['regen_active']
            elif cid == SPEED and payload[2]:
                kph = round(u16(payload) / 10, 1)
                if speed_peak is None or kph > speed_peak['kph']:
                    speed_peak = {'t_s': t / 1000, 'kph': kph}
            elif cid == DRIVE:
                drive = decode(cid, payload)
                command_sum = abs(drive['cmd_L']) + abs(drive['cmd_R'])
                phase_sum = abs(drive['iph_L']) + abs(drive['iph_R'])
                if command_peak is None or command_sum > command_peak['sum_abs_a']:
                    command_peak = {'t_s': t / 1000, 'sum_abs_a': round(command_sum, 1),
                                    'left_a': drive['cmd_L'], 'right_a': drive['cmd_R']}
                if phase_peak is None or phase_sum > phase_peak['sum_abs_a']:
                    phase_peak = {'t_s': t / 1000, 'sum_abs_a': round(phase_sum, 1),
                                  'left_a': drive['iph_L'], 'right_a': drive['iph_R']}
                if around is not None and abs(t / 1000 - around) <= 2:
                    timeline.append({'t_s': t / 1000, 'frame': hex(cid), **drive})
            elif cid == MOTOR:
                motor = {'t_ms': t, **decode(cid, payload)}
            elif cid in (DRIVE, TV_YAW, TV_LOAD):
                # Keep the canonical VCU decoder exercised on real binary frames.
                if around is not None and abs(t / 1000 - around) <= 2:
                    timeline.append({'t_s': t / 1000, 'frame': hex(cid), **decode(cid, payload)})
            elif cid == BLOCK:
                current, first, events, flags = struct.unpack('<4H', payload)
                last_block = {'t_s': t / 1000, 'current': current, 'first': first,
                              'events': events, 'limiter_flags': flags}
                if current != block_previous or (block_previous_count is not None and events != block_previous_count):
                    if current or (block_previous_count is not None and events != block_previous_count):
                        block_events.append({**last_block, 'current_names': names(current),
                                             'first_names': names(first)})
                block_previous, block_previous_count = current, events
            elif cid == FAULT:
                current_fault = (payload[0:6].hex(), payload[6], payload[7] & 1)
                last_fault = {'t_s': t / 1000, 'first_error_hex': payload[0:6].hex(),
                              'origin': payload[6], 'latched': bool(payload[7] & 1),
                              'rearm_ready': bool(payload[7] & 2)}
                if current_fault != last_fault_state and (payload[6] or payload[7] & 1):
                    fault_events.append(last_fault.copy())
                last_fault_state = current_fault
            elif cid == THROTTLE:
                raw, window_min, invalid_raw, invalid_count = struct.unpack('<4H', payload)
                last_throttle = {'t_s': t / 1000, 'raw': raw, 'window_min': window_min,
                                 'last_invalid_raw': None if invalid_raw == 65535 else invalid_raw,
                                 'invalid_count': invalid_count}
            elif cid == BMS:
                valid = payload[0] & 3 == 3
                if valid:
                    v = round(u16(payload, 2) / 10, 1)
                    i = round(u16(payload, 4) / 10 - 3200, 1)
                    if bms_min_v is None or v < bms_min_v['v']:
                        bms_min_v = {'t_s': t / 1000, 'v': v, 'current_a': i}
                    if bms_peak_discharge is None or i < bms_peak_discharge['current_a']:
                        bms_peak_discharge = {'t_s': t / 1000, 'v': v, 'current_a': i}
            elif cid == EM:
                em_count += 1

    return {
        'file': str(path.resolve()), 'size_bytes': size, 'records': count,
        'can_records': can_count, 'duration_s': round((last_ms - first_ms) / 1000, 3),
        'counts': {f'0x{k:08X}': v for k, v in sorted(counts.items())},
        'peak_speed': speed_peak, 'peak_controller_bus_sum': bus_peak,
        'peak_command_phase_sum_abs': command_peak,
        'peak_reported_phase_sum_abs': phase_peak,
        'peak_controller_dc_power': power_peak, 'loaded_min_controller_v': loaded_min_v,
        'bms_peak_discharge': bms_peak_discharge, 'bms_min_pack_v': bms_min_v,
        'energy_meter_frames': em_count,
        'tv_requested_frames': tv_request_frames, 'tv_active_frames': tv_active_frames,
        'regen_active_frames': regen_active_frames,
        'last_seen_s': {f'0x{k:08X}': round(v / 1000, 3) for k, v in sorted(last_seen.items())},
        'controller_gaps_over_250ms': {side: {'count': len(gaps[cid]),
                                               'longest_s': max((g['gap_s'] for g in gaps[cid]), default=0),
                                               'first': gaps[cid][:5]}
                                       for side, cid in [('L1', L1), ('R1', R1), ('L2', L2), ('R2', R2)]},
        'handshake_probes': {'L': probes[L1], 'R': probes[R1]},
        'voltage_drops_over_8v': voltage_drops[:20],
        'voltage_drop_count': len(voltage_drops),
        'block_events': block_events[:40], 'block_event_count_observed': len(block_events),
        'fault_events': fault_events[:20], 'last_block': last_block,
        'last_fault': last_fault, 'last_throttle': last_throttle,
        'last_status': status, 'last_control': control, 'last_motor': motor,
        'around_s': around, 'around_frames': timeline[:300],
    }


def render(result):
    def line(label, value):
        print(f'{label}: {value}')

    line('파일', result['file'])
    line('길이/레코드', f"{result['duration_s']:.1f}s / {result['records']:,}개 (CAN {result['can_records']:,})")
    for label, key, field, unit in [
        ('최고 차속', 'peak_speed', 'kph', 'km/h'),
        ('최대 컨트롤러 버스전류 합', 'peak_controller_bus_sum', 'sum_ibus_a', 'A'),
        ('최대 목표 상전류 절댓값 합', 'peak_command_phase_sum_abs', 'sum_abs_a', 'A'),
        ('최대 보고 상전류 절댓값 합', 'peak_reported_phase_sum_abs', 'sum_abs_a', 'A'),
        ('최대 컨트롤러 보고 DC 전력', 'peak_controller_dc_power', 'dc_power_w', 'W'),
        ('부하 중 최저 컨트롤러 전압', 'loaded_min_controller_v', 'min_v', 'V'),
        ('BMS 최대 방전 보고값', 'bms_peak_discharge', 'current_a', 'A'),
        ('BMS 최저 팩 전압', 'bms_min_pack_v', 'v', 'V')]:
        item = result[key]
        line(label, f"{item[field]} {unit} @ {item['t_s']:.2f}s" if item else '기록 없음')
    line('TV 요청/적용·회생 적용 프레임',
         f"{result['tv_requested_frames']}/{result['tv_active_frames']} · {result['regen_active_frames']}")
    line('에너지미터 프레임', result['energy_meter_frames'])
    line('컨트롤러 핸드셰이크 프로브 L/R',
         f"{result['handshake_probes']['L']}/{result['handshake_probes']['R']}")
    line('컨트롤러 피드백 250ms 초과 gap L1/R1/L2/R2',
         '/'.join(str(result['controller_gaps_over_250ms'][s]['count']) for s in ('L1','R1','L2','R2')))
    line('8V 이상 급락 관측', result['voltage_drop_count'])
    for drop in result['voltage_drops_over_8v'][:5]:
        print(f"  {drop['t_s']:.2f}s {drop['side']} {drop['from_v']}→{drop['to_v']}V, 직전 Ibus={drop['prior_ibus_a']}A")
    line('차단 이벤트 관측', result['block_event_count_observed'])
    for event in result['block_events'][:8]:
        print(f"  {event['t_s']:.2f}s count={event['events']} current={event['current_names']} first={event['first_names']}")
    line('마지막 차단 상태', result['last_block'])
    line('마지막 fault 상태', result['last_fault'])
    line('마지막 스로틀 진단', result['last_throttle'])
    if result['around_s'] is not None:
        print(f"±2초 상세 프레임: {len(result['around_frames'])}개 (JSON에서 확인)")
    print('주의: CAN 송신/보고값이며 실제 접촉기·BMS 차단 원인은 단독 확정 불가. BMS 보고값은 갱신 지연 가능.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', help='원본 Monolith .log 파일')
    parser.add_argument('--json', action='store_true', help='기계가 읽을 수 있는 JSON 출력')
    parser.add_argument('--around', type=float, help='해당 로그 시각(초) ±2초 VCU 프레임 JSON에 포함')
    args = parser.parse_args()
    try:
        results = [summarize(p, args.around) for p in args.logs]
    except (OSError, ValueError) as exc:
        parser.exit(2, f'오류: {exc}\n')
    if args.json:
        print(json.dumps(results if len(results) > 1 else results[0], ensure_ascii=False, indent=2))
    else:
        for index, result in enumerate(results):
            if index:
                print()
            render(result)


if __name__ == '__main__':
    main()
