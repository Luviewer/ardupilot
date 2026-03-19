# -*- coding: utf-8 -*-
# Root SConstruct for pogo-apm: RTT BSP staging + scons full build by --target.
# Usage: scons --v=ArduCopter --target=pixhawk6c-mini | scons --v=ArduCopter --target=cuav-v5 [scons options]
#        aliases like pixhawk6c_mini, cuav_v5, and quoted forms like "cuav v5" are accepted.
#        scons -c --target=cuav-v5  => clean staged BSP build.
# Product: build/rtt_deploy/<target>/rtthread.bin, rt-thread.elf; copied to build/rtt_<target>/rtthread.bin

import os
import subprocess
import sys
import shutil

from SCons.Script import AddOption, GetOption, ARGUMENTS

# --- Options (Agent 2 style) ---
AddOption('--target',
          dest='target',
          type='string',
          default='',
          help='RTT BSP target alias: pixhawk6c-mini/pixhawk6c_mini, cuav-v5/cuav_v5')
AddOption('--v',
          dest='vehicle',
          type='string',
          default='ArduCopter',
          help='Vehicle (e.g. ArduCopter), for future use')
AddOption('--upload',
          dest='upload',
          action='store_true',
          default=False,
          help='After build, upload firmware (optional)')
AddOption('--port',
          dest='upload_port',
          type='string',
          default=None,
          help='Serial port for upload (e.g. /dev/ttyACM0)')

RTT_TARGETS = {
    'pixhawk6c_mini': {'board': 'rtt_pixhawk6c_mini'},
    'cuav_v5': {'board': 'rtt_cuav_v5'},
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


def _normalize_target(target):
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


def _rtt_detect_exec_path():
    """If arm-none-eabi-gcc is on PATH, return its directory for RTT_EXEC_PATH."""
    try:
        import shutil
        gcc = shutil.which('arm-none-eabi-gcc')
        if gcc:
            return os.path.dirname(gcc)
    except Exception:
        pass
    return None


def _run_rtt_build(ap_root, target, bsp_deploy_abspath, scons_args):
    """Run scons (or scons -c) in bsp_deploy_abspath with ARDUPILOT_FULL=1, RTT_ROOT, RTT_EXEC_PATH."""
    rtt_root = os.path.join(ap_root, 'modules', 'rt-thread')
    env = os.environ.copy()
    env['AP_ROOT'] = ap_root
    env['ARDUPILOT_FULL'] = '1'
    env['RTT_ROOT'] = rtt_root
    rtt_exec = _rtt_detect_exec_path()
    if rtt_exec:
        env['RTT_EXEC_PATH'] = rtt_exec
    cmd = [sys.executable, '-m', 'SCons'] + scons_args
    ret = subprocess.call(cmd, cwd=bsp_deploy_abspath, env=env)
    return ret


def _copy_bin_to_build(ap_root, target, bsp_deploy_abspath):
    """Copy rtthread.bin to build/rtt_<target>/ for upload convenience."""
    src = os.path.join(bsp_deploy_abspath, 'rtthread.bin')
    if not os.path.isfile(src):
        return
    out_dir = os.path.join(ap_root, 'build', 'rtt_%s' % target)
    try:
        os.makedirs(out_dir, exist_ok=True)
        dst = os.path.join(out_dir, 'rtthread.bin')
        import shutil
        shutil.copy2(src, dst)
        print('Copied %s -> %s' % (src, dst))
    except Exception as e:
        print('Copy rtthread.bin to build: %s' % e, file=sys.stderr)


# --- When valid --target: deploy then scons in BSP (or scons -c for clean) ---
target = GetOption('target')
canonical_target = _normalize_target(target)
if target and not canonical_target:
    print('Unknown RTT target: %s (supported: pixhawk6c_mini, pixhawk6c-mini, cuav_v5, cuav-v5)' % target, file=sys.stderr)
    Exit(1)

if canonical_target:
    ap_root = Dir('#').abspath
    deploy_script = os.path.join(ap_root, 'Tools', 'scripts', 'rtt_bsp_deploy.py')
    if not os.path.isfile(deploy_script):
        print('rtt_bsp_deploy.py not found: %s' % deploy_script, file=sys.stderr)
        Exit(1)
    # 2) Build scons args: pass through command line but drop --target, --upload, --port (skip argv[0])
    scons_args = []
    i = 1
    while i < len(sys.argv):
        a = sys.argv[i]
        if a in ('--target', '--upload', '--port', '--v'):
            i += 1
            if i < len(sys.argv) and not sys.argv[i].startswith('-'):
                i += 1
            continue
        if a.startswith('--target=') or a.startswith('--upload=') or a.startswith('--port=') or a.startswith('--v='):
            i += 1
            continue
        scons_args.append(a)
        i += 1
    bsp_deploy_abspath = os.path.join(ap_root, 'build', 'rtt_deploy', canonical_target)
    is_clean = '-c' in scons_args or '--clean' in scons_args
    # 1) Deploy BSP only for build; clean should not recreate staging area.
    if not is_clean:
        try:
            bsp_deploy_abspath = subprocess.check_output(
                [sys.executable, deploy_script, ap_root, canonical_target],
                cwd=ap_root,
                text=True
            ).strip()
        except subprocess.CalledProcessError as e:
            Exit(e.returncode)
        if not os.path.isdir(bsp_deploy_abspath):
            print('Deploy path not a directory: %s' % bsp_deploy_abspath, file=sys.stderr)
            Exit(1)
    # 2.5) Generate mavlink headers for scons (waf does this for waf build)
    if not is_clean:
        board = RTT_TARGETS[canonical_target]['board']
        mavgen_script = os.path.join(ap_root, 'Tools', 'scripts', 'rtt_mavgen.py')
        if os.path.isfile(mavgen_script):
            ret_mav = subprocess.call([sys.executable, mavgen_script, ap_root, board], cwd=ap_root)
            if ret_mav != 0:
                print('rtt_mavgen.py failed', file=sys.stderr)
                Exit(ret_mav)
    # 3) Run scons in BSP (build or clean)
    if is_clean and not os.path.isdir(bsp_deploy_abspath):
        shutil.rmtree(os.path.join(ap_root, 'build', 'rtt_%s' % canonical_target), ignore_errors=True)
        Exit(0)
    ret = _run_rtt_build(ap_root, canonical_target, bsp_deploy_abspath, scons_args)
    if ret != 0:
        Exit(ret)
    # 4) If build (not clean), optionally copy rtthread.bin to build/rtt_<target>/
    if not is_clean:
        _copy_bin_to_build(ap_root, canonical_target, bsp_deploy_abspath)
        # 5) If --upload: rtthread.bin -> rtt_bin_to_apj.py -> uploader.py
        if GetOption('upload'):
            rtthread_bin = os.path.join(bsp_deploy_abspath, 'rtthread.bin')
            if not os.path.isfile(rtthread_bin):
                print('error: rtthread.bin not found at %s' % rtthread_bin, file=sys.stderr)
                Exit(1)
            board_apj = RTT_TARGETS[canonical_target]['board']
            rtt_bin_to_apj = os.path.join(ap_root, 'Tools', 'scripts', 'rtt_bin_to_apj.py')
            apj_path = os.path.join(bsp_deploy_abspath, 'arducopter.apj')
            cmd = [sys.executable, rtt_bin_to_apj, rtthread_bin, '--board', board_apj, '-o', apj_path, '--upload']
            if GetOption('upload_port'):
                cmd += ['--port', GetOption('upload_port')]
            ret = subprocess.call(cmd)
            if ret != 0:
                Exit(ret)
    Exit(0)

# No --target or unknown target: no-op (root does not define default targets)
# Allow other uses of this SConstruct without --target by not calling Exit(0) when target is empty.
