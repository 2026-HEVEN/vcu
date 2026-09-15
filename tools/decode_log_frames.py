#!/usr/bin/env python3
"""Monolith SD 로그에서 VCU 고속 로깅 프레임을 CSV로 푼다.

Monolith는 수신 CAN 프레임을 필터 없이 raw로 남긴다. 이 스크립트는 그중
0x1C01~0x1C04C0D0 네 프레임만 골라 타임스탬프 기준으로 정렬한 CSV를 만든다.

사용:
    uv run python tools/decode_log_frames.py <csv_or_jsonl> [-o out.csv]

입력은 (timestamp, can_id, data_hex) 세 칼럼을 가진 CSV 또는 JSONL을 가정한다.
Monolith 뷰어의 내보내기 포맷이 다르면 read_records()만 고치면 된다.
"""
import argparse, csv, json, struct, sys
from pathlib import Path

DRIVE, MOTOR, TV_YAW, TV_LOAD = 0x1C01C0D0, 0x1C02C0D0, 0x1C03C0D0, 0x1C04C0D0

def i16(b, o): return struct.unpack_from('<h', b, o)[0]

def decode(can_id, d):
    """프레임 하나를 {필드: 물리값} 으로."""
    if can_id == DRIVE:
        return {'cmd_L': i16(d,0)/10, 'cmd_R': i16(d,2)/10,
                'iph_L': i16(d,4)/10, 'iph_R': i16(d,6)/10}
    if can_id == MOTOR:
        return {'rpm_L': i16(d,0), 'rpm_R': i16(d,2),
                'ibus_L': i16(d,4)/10, 'ibus_R': i16(d,6)/10}
    if can_id == TV_YAW:
        return {'desired_yaw': i16(d,0)/100, 'Mz': i16(d,2)/100,
                'req_L': i16(d,4)/10, 'req_R': i16(d,6)/10}
    if can_id == TV_LOAD:
        g = d[6]
        return {'fz_L': i16(d,0), 'fz_R': i16(d,2),
                'max_L': d[4]*4, 'max_R': d[5]*4,
                'tv_active': g & 1, 'g_switch': (g>>1)&1, 'g_gains': (g>>2)&1,
                'g_imu': (g>>3)&1, 'g_spd_valid': (g>>4)&1, 'g_spd_min': (g>>5)&1}
    return None

FIELDS = ['t_ms','cmd_L','cmd_R','iph_L','iph_R','rpm_L','rpm_R','ibus_L','ibus_R',
          'desired_yaw','Mz','req_L','req_R','fz_L','fz_R','max_L','max_R',
          'tv_active','g_switch','g_gains','g_imu','g_spd_valid','g_spd_min']

def read_records(path):
    """(t_ms, can_id, bytes) 스트림. 입력 포맷이 다르면 여기만 고친다."""
    text = Path(path).read_text(encoding='utf-8', errors='replace')
    if text.lstrip().startswith('{'):
        for line in text.splitlines():
            if not line.strip(): continue
            r = json.loads(line)
            yield int(r['t']), int(str(r['id']), 0), bytes.fromhex(r['data'])
    else:
        for r in csv.DictReader(text.splitlines()):
            k = {c.lower(): c for c in r}
            yield (int(float(r[k.get('t_ms', k.get('t','timestamp'))])),
                   int(r[k['can_id'] if 'can_id' in k else k['id']], 0),
                   bytes.fromhex(r[k['data']].replace(' ', '')))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input'); ap.add_argument('-o', '--out', default='vcu_log.csv')
    a = ap.parse_args()

    rows, cur, n = [], {}, 0
    for t, cid, data in read_records(a.input):
        f = decode(cid, data)
        if f is None: continue
        n += 1
        # 100Hz 프레임(DRIVE)이 올 때마다 한 행을 확정한다. 50Hz 프레임은
        # 직전 값을 그대로 이어 쓴다(zero-order hold).
        cur.update(f)
        if cid == DRIVE:
            rows.append({'t_ms': t, **cur})

    if not rows:
        sys.exit('VCU 로깅 프레임을 찾지 못했다. ID 또는 입력 포맷을 확인할 것.')

    with open(a.out, 'w', newline='', encoding='utf-8') as fp:
        w = csv.DictWriter(fp, fieldnames=FIELDS, extrasaction='ignore')
        w.writeheader(); w.writerows(rows)
    print(f'프레임 {n}개 → 행 {len(rows)}개 → {a.out}')

    # 바로 쓸 수 있는 요약: 명령 대비 실측 비율을 rpm 구간별로
    print('\nrpm 구간별 실측/명령 상전류 비율 (|명령| >= 5 A 인 표본만)')
    bins = [(0,200),(200,500),(500,1000),(1000,2000),(2000,9999)]
    for lo, hi in bins:
        rs = [abs(r['iph_L'])/abs(r['cmd_L']) for r in rows
              if abs(r.get('cmd_L',0)) >= 5 and lo <= abs(r.get('rpm_L',0)) < hi]
        if rs:
            rs.sort()
            print(f'  {lo:5d}~{hi:<5d} rpm  n={len(rs):5d}  '
                  f'중앙값 {rs[len(rs)//2]:.3f}')
        else:
            print(f'  {lo:5d}~{hi:<5d} rpm  표본 없음')

if __name__ == '__main__':
    main()
