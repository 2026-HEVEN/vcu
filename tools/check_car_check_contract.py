"""Cross-repository contract test. No firmware upload or source-file mutation.

Usage: python tools/check_car_check_contract.py --cluster ../HEVEN-cluster
Requires a C++17 host compiler on PATH (or --cxx).
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cluster', required=True, type=Path)
    p.add_argument('--cxx', default=shutil.which('g++') or shutil.which('clang++'))
    args = p.parse_args()
    vcu = Path(__file__).resolve().parents[1]
    cluster = args.cluster.resolve()
    if not args.cxx:
        p.error('C++17 compiler missing; pass --cxx or add g++ to PATH')
    for name in ('include/car_check_protocol.h', 'docs/CAR_CHECK_CAN.md'):
        assert (vcu/name).read_text(encoding='utf-8') == (cluster/name).read_text(encoding='utf-8'), name+' differs'
    with tempfile.TemporaryDirectory(prefix='heven-can-contract-') as tmp:
        exe = Path(tmp)/'roundtrip.exe'
        subprocess.run([args.cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror',
            '-I'+str(vcu/'include'), '-I'+str(cluster/'include'),
            str(vcu/'tools/car_check_roundtrip.cpp'),
            str(vcu/'src/logic/car_check_protocol.cpp'),
            str(cluster/'src/logic/car_check_protocol.cpp'),
            str(cluster/'src/logic/car_check_receiver.cpp'), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print('PASS: VCU/Cluster shared header and CAN handoff document match')

if __name__ == '__main__':
    main()
