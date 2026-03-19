#!/usr/bin/env python3
"""
从 RTT BSP 编译得到的 rtthread.bin 生成 PX4/AP bootloader 可烧录的 .apj 文件，
并可选择直接调用 uploader.py 上传（等同 waf upload）。

用法:
  python3 rtt_bin_to_apj.py rtthread.bin [--board rtt_cuav_v5] [-o firmware.apj] [--upload] [--port /dev/ttyACM0]

示例（在 pogo-apm 根目录）:
  python3 Tools/scripts/rtt_bin_to_apj.py modules/rt-thread/bsp/stm32/stm32f765-cuav-v5/rtthread.bin --board rtt_cuav_v5 -o arducopter.apj --upload
"""
import argparse
import base64
import json
import os
import subprocess
import sys
import zlib

# Board name -> (APJ board_id, flash_total_bytes). 与 uploader.py / waf rtt 一致
BOARD_APJ = {
    'rtt_cuav_v5': (50, 2048 * 1024),   # TARGET_HW_PX4_FMU_V5, 2MB
    'rtt_pixhawk6c_mini': (56, 2048 * 1024),
    'rtt_fmuv2': (9, 1024 * 1024),
}


def main():
    parser = argparse.ArgumentParser(description='RTT .bin to .apj and optional upload')
    parser.add_argument('bin_file', help='rtthread.bin (or any firmware.bin)')
    parser.add_argument('--board', default='rtt_cuav_v5', choices=list(BOARD_APJ),
                        help='Board name for board_id')
    parser.add_argument('-o', '--output', default=None,
                        help='Output .apj path (default: same base name as bin)')
    parser.add_argument('--upload', action='store_true', help='Run uploader.py after generating .apj')
    parser.add_argument('--port', default=None, help='Serial port for upload (e.g. /dev/ttyACM0)')
    parser.add_argument('--force', action='store_true', help='Pass --force to uploader')
    args = parser.parse_args()

    bin_path = os.path.abspath(args.bin_file)
    if not os.path.isfile(bin_path):
        sys.stderr.write('error: not a file: %s\n' % bin_path)
        sys.exit(1)

    board_id, flash_total = BOARD_APJ[args.board]
    with open(bin_path, 'rb') as f:
        bin_data = f.read()

    d = {
        'board_id': board_id,
        'magic': 'APJFWv1',
        'description': 'RTT firmware for %s' % args.board,
        'image': base64.b64encode(zlib.compress(bin_data, 9)).decode('utf-8'),
        'image_size': len(bin_data),
        'summary': args.board,
        'version': '0.1',
        'board_revision': 0,
        'flash_total': flash_total,
        'image_maxsize': flash_total,
        'flash_free': flash_total - len(bin_data),
    }

    if args.output:
        apj_path = os.path.abspath(args.output)
    else:
        base = os.path.splitext(bin_path)[0]
        apj_path = base + '.apj'

    with open(apj_path, 'w') as f:
        f.write(json.dumps(d, indent=4))
    print('Wrote %s (board_id=%u, image_size=%u)' % (apj_path, board_id, len(bin_data)))

    if not args.upload:
        return 0

    script_dir = os.path.dirname(os.path.abspath(__file__))
    uploader_py = os.path.join(script_dir, 'uploader.py')
    if not os.path.isfile(uploader_py):
        sys.stderr.write('error: uploader.py not found at %s\n' % uploader_py)
        sys.exit(1)

    cmd = [sys.executable, uploader_py, apj_path]
    if args.port:
        cmd += ['--port', args.port]
    if args.force:
        cmd += ['--force']
    print('Running: %s' % ' '.join(cmd))
    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
