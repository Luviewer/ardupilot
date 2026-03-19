#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Generate MAVLink C headers for scons RTT build (waf normally does this).
# Output: <build_dir>/libraries/GCS_MAVLink/include/mavlink/v2.0/
# Usage: rtt_mavgen.py <ap_root> <board>
#   e.g. rtt_mavgen.py /path/to/pogo-apm rtt_cuav_v5

import os
import sys


def main():
    if len(sys.argv) < 3:
        print('Usage: %s <ap_root> <board>' % sys.argv[0], file=sys.stderr)
        return 1
    ap_root = os.path.abspath(sys.argv[1])
    board = sys.argv[2]
    mavlink_dir = os.path.join(ap_root, 'modules', 'mavlink')
    xml_path = os.path.join(ap_root, 'modules', 'mavlink', 'message_definitions', 'v1.0', 'all.xml')
    output_dir = os.path.join(ap_root, 'build', board, 'libraries', 'GCS_MAVLink', 'include', 'mavlink', 'v2.0')
    if not os.path.isfile(xml_path):
        print('mavlink xml not found: %s' % xml_path, file=sys.stderr)
        return 1
    sys.path.insert(0, mavlink_dir)
    try:
        from pymavlink.generator import mavgen
    except ImportError as e:
        print('pymavlink not available (need modules/mavlink): %s' % e, file=sys.stderr)
        return 1
    os.makedirs(output_dir, exist_ok=True)
    class opts:
        language = 'C'
        wire_protocol = '2.0'
        validate = False
        output = output_dir
    if mavgen.mavgen(opts(), [xml_path]):
        return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())
