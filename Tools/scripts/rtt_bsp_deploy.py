#!/usr/bin/env python3
# encoding: utf-8
"""
Deploy RTT BSP from ArduPilot repo to build/rtt_deploy/<target>.
Used by root SConstruct when building with --target=<pixhawk6c_mini|cuav_v5>.
Usage: python3 rtt_bsp_deploy.py <AP_ROOT> <TARGET>
  On success prints BSP deploy absolute path (bsp_deploy_abspath) to stdout.
  Exit 0 on success, non-zero on error.
"""

import argparse
import os
import shutil
import sys


RTT_TARGETS = {
    'pixhawk6c_mini': {
        'bsp_src_rel': 'libraries/AP_HAL_RTT/rtt_bsp_pixhawk6c_mini',
        'board': 'rtt_pixhawk6c_mini',
    },
    'cuav_v5': {
        'bsp_src_rel': 'libraries/AP_HAL_RTT/rtt_bsp_cuav_v5',
        'board': 'rtt_cuav_v5',
    },
}

RTT_TARGET_ALIASES = {
    'pixhawk6c_mini': 'pixhawk6c_mini',
    'pixhawk6c-mini': 'pixhawk6c_mini',
    'pixhawk6c mini': 'pixhawk6c_mini',
    'rtt_pixhawk6c_mini': 'pixhawk6c_mini',
    'rtt-pixhawk6c-mini': 'pixhawk6c_mini',
    'cuav_v5': 'cuav_v5',
    'cuav-v5': 'cuav_v5',
    'cuav v5': 'cuav_v5',
    'rtt_cuav_v5': 'cuav_v5',
    'rtt-cuav-v5': 'cuav_v5',
}


def _norm(s):
    return s.replace('/', os.sep)


def normalize_target(target):
    if not target:
        return ''
    key = str(target).strip().lower().replace('\\', '/')
    key = key.replace('/', ' ')
    key = ' '.join(key.split())
    variants = {
        key,
        key.replace('-', '_'),
        key.replace('-', ' '),
        key.replace('_', ' '),
        key.replace(' ', '_'),
    }
    for variant in variants:
        if variant in RTT_TARGET_ALIASES:
            return RTT_TARGET_ALIASES[variant]
    return ''


def deploy(ap_root, target):
    """
    Deploy BSP for target to build/rtt_deploy/<canonical_target>.
    Returns (bsp_deploy_abspath, None) on success, (None, error_msg) on failure.
    """
    ap_root = os.path.abspath(ap_root)
    canonical = normalize_target(target)
    if canonical not in RTT_TARGETS:
        return None, "Unknown target: %s (supported: %s)" % (target, ', '.join(sorted(RTT_TARGETS.keys())))

    src_dir = os.path.join(ap_root, _norm(RTT_TARGETS[canonical]['bsp_src_rel']))
    deploy_dir = os.path.join(ap_root, 'build', 'rtt_deploy', canonical)
    rtt_root = os.path.join(ap_root, 'modules', 'rt-thread')

    if not os.path.isdir(ap_root):
        return None, "AP_ROOT not a directory: %s" % ap_root
    if not os.path.isdir(rtt_root):
        return None, "RTT_ROOT not a directory: %s" % rtt_root
    if not os.path.isdir(src_dir):
        return None, "BSP source not found: %s" % src_dir

    try:
        if os.path.isdir(deploy_dir):
            shutil.rmtree(deploy_dir)
        os.makedirs(os.path.dirname(deploy_dir), exist_ok=True)
        shutil.copytree(src_dir, deploy_dir)
    except Exception as e:
        return None, "Deploy copytree failed: %s" % e

    return deploy_dir, None


def main():
    parser = argparse.ArgumentParser(description='Deploy RTT BSP for ArduPilot scons build')
    parser.add_argument('ap_root', help='ArduPilot repository root')
    parser.add_argument('target', help='Target board name or alias')
    args = parser.parse_args()

    bsp_path, err = deploy(args.ap_root, args.target)
    if err:
        print(err, file=sys.stderr)
        return 1
    print(bsp_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
