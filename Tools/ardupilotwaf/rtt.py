# encoding: utf-8
"""
Waf tool for RT-Thread (RTT) build.

Loaded when building an RTT-based board (e.g. from boards.py: cfg.load('rtt')).
Provides: configure(cfg), build(bld), pre_build(bld); generates hwdef.h/ldscript.ld
from hwdef.dat via rtt_hwdef.py; when BOARD=fmuv2 and RTT_ROOT=modules/rt-thread,
ensures librtthread.a is available (by scons+ar or manual) and adds its path to LIBPATH.

See docstring of build() and RTT_BUILD_FMUV2.md for minimal steps from zero to linkable.
"""

from waflib import Task, Utils
from waflib.TaskGen import after_method, before_method, feature
import os
import sys
import subprocess

# BOARD name (ArduPilot) -> RTT BSP path relative to RTT_ROOT/bsp/
# fmuv2 uses a dedicated BSP maintained in pogo-apm (rtt_bsp_fmuv2), deployed to bsp/stm32/stm32f427-fmuv2.
RTT_BSP_MAP = {
    'fmuv2': 'stm32/stm32f427-fmuv2',
    'rtt_fmuv2': 'stm32/stm32f427-fmuv2',
    'pixhawk6c_mini': 'stm32/stm32h743-pixhawk6c-mini',
    'rtt_pixhawk6c_mini': 'stm32/stm32h743-pixhawk6c-mini',
    'rtt_cuav_v5': 'stm32/stm32f765-cuav-v5',
}

# Board -> APJ board_id for PX4/AP bootloader (numeric)
RTT_APJ_BOARD_IDS = {
    'pixhawk6c_mini': 56,       # TARGET_HW_PX4_FMU_V6C
    'rtt_pixhawk6c_mini': 56,
    'fmuv2': 9,                 # TARGET_HW_PX4_FMU_V2 / TARGET_HW_CUBE_F4
    'rtt_fmuv2': 9,
    'rtt_cuav_v5': 50,          # TARGET_HW_PX4_FMU_V5 (CUAV V5)
}

# Board -> flash size in bytes (for apj image_maxsize)
RTT_FLASH_TOTAL = {
    'pixhawk6c_mini': 2048 * 1024,
    'rtt_pixhawk6c_mini': 2048 * 1024,
    'fmuv2': 1024 * 1024,
    'rtt_fmuv2': 1024 * 1024,
    'rtt_cuav_v5': 2048 * 1024,
}

# BSP source in pogo-apm for fmuv2; deployed to RTT_ROOT/bsp/stm32/stm32f427-fmuv2 when missing
RTT_BSP_FMUV2_SRC = 'libraries/AP_HAL_RTT/rtt_bsp_fmuv2'
# BSP source for Pixhawk6C-Mini; deployed to RTT_ROOT/bsp/stm32/stm32h743-pixhawk6c-mini when missing
RTT_BSP_PIXHAWK6C_MINI_SRC = 'libraries/AP_HAL_RTT/rtt_bsp_pixhawk6c_mini'


def configure(cfg):
    """Set BUILDROOT, RTT_ROOT (or RTT_SOURCE), AP_HAL_ROOT, toolchain; run hwdef generation."""
    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()

    env.AP_PROGRAM_FEATURES += ['rtt_ap_program']
    kw = env.AP_LIBRARIES_OBJECTS_KW
    kw['features'] = Utils.to_list(kw.get('features', [])) + ['rtt_ap_library']

    env.AP_HAL_ROOT = srcpath('libraries/AP_HAL_RTT')
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.RTT_SCRIPTS = srcpath('libraries/AP_HAL_RTT/hwdef/scripts')

    # RTT source root: env var RTT_ROOT or RTT_SOURCE, else modules/rt-thread submodule, else placeholder
    env.RTT_ROOT = os.environ.get('RTT_ROOT', os.environ.get('RTT_SOURCE', ''))
    if getattr(cfg.env, 'RTT_ROOT', None):
        env.RTT_ROOT = cfg.env.RTT_ROOT
    if not env.RTT_ROOT:
        default_rtt = srcpath('modules/rt-thread')
        env.RTT_ROOT = default_rtt if os.path.isdir(default_rtt) else bldpath('rtt_src_placeholder')

    # Optional: help RTT BSP scons find arm-none-eabi-gcc (rtconfig.py often expects RTT_EXEC_PATH)
    if getattr(env, 'RTT_EXEC_PATH', None) is None and env.get_flat('CC'):
        cc = env.get_flat('CC')
        if (('arm-none-eabi' in cc) or ('gcc' in cc)) and os.path.isabs(cc):
            env.RTT_EXEC_PATH = os.path.dirname(cc)

    if cfg.options.default_parameters:
        cfg.msg('Default parameters', cfg.options.default_parameters, color='YELLOW')
        env.DEFAULT_PARAMETERS = cfg.options.default_parameters
    else:
        env.DEFAULT_PARAMETERS = ''

    env.default_parameters = env.DEFAULT_PARAMETERS

    hwdef = getattr(env, 'HWDEF', None)
    if not hwdef or (isinstance(hwdef, (list, tuple)) and len(hwdef) == 0):
        env.HWDEF = os.path.join(env.SRCROOT, 'libraries/AP_HAL_RTT/hwdef/%s/hwdef.dat' % env.BOARD)

    try:
        ret = generate_hwdef_h(env)
    except Exception as e:
        cfg.fatal("Failed to process hwdef.dat: %s" % e)
    if ret != 0:
        cfg.fatal("Failed to process hwdef.dat ret=%d" % ret)

    # Match RTT BSP ABI: cortex-m4 (F4) or cortex-m7 (H7), hard float, so linking with librtthread.a works
    if env.get_flat('CC') and 'arm-none-eabi' in env.get_flat('CC'):
        mcu = _rtt_mcu_family(env)
        if mcu == 'h7':
            cpu_flags = ['-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard']
        else:
            cpu_flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard']
        env.append_value('CFLAGS', cpu_flags)
        env.append_value('CXXFLAGS', cpu_flags)
        env.append_value('LINKFLAGS', cpu_flags)
        # -Os for H7 to reduce code size (LTO incompatible with librtthread.a from RTT scons)
        if mcu == 'h7':
            env.append_value('CFLAGS', ['-Os'])
            env.append_value('CXXFLAGS', ['-Os'])

    # Upload support: OBJCOPY for bin, UPLOAD_TOOLS for uploader.py, APJ_BOARD_ID for apj
    cfg.find_program('arm-none-eabi-objcopy', var='OBJCOPY')
    env.UPLOAD_TOOLS = srcpath('Tools/scripts')
    env.APJ_BOARD_ID = RTT_APJ_BOARD_IDS.get(env.BOARD, 0)
    env.RTT_FLASH_TOTAL = RTT_FLASH_TOTAL.get(env.BOARD, 0)


def generate_hwdef_h(env):
    """Run rtt_hwdef.py to generate hwdef.h and ldscript.ld."""
    import subprocess
    hwdef_script = os.path.join(env.SRCROOT, 'libraries/AP_HAL_RTT/hwdef/scripts/rtt_hwdef.py')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.makedirs(hwdef_out)
    python = sys.executable
    params = getattr(env, 'DEFAULT_PARAMETERS', '') or ''
    hwdef_arg = env.HWDEF if isinstance(env.HWDEF, str) else env.HWDEF[0]
    cmd = [python, hwdef_script, '-D', hwdef_out, '--params', params, hwdef_arg]
    if getattr(env, 'HWDEF_EXTRA', None):
        cmd.append(env.HWDEF_EXTRA)
    return subprocess.call(cmd)


def _deploy_fmuv2_bsp_if_needed(env):
    """
    If BOARD uses the fmuv2 dedicated BSP (stm32/stm32f427-fmuv2) and the deploy target
    does not exist, copy from SRCROOT/libraries/AP_HAL_RTT/rtt_bsp_fmuv2 to
    RTT_ROOT/bsp/stm32/stm32f427-fmuv2 so scons can run there. Does nothing if target exists.
    """
    bsp_rel = RTT_BSP_MAP.get(getattr(env, 'BOARD', None))
    if bsp_rel != 'stm32/stm32f427-fmuv2':
        return
    rtt_root = getattr(env, 'RTT_ROOT', None)
    srcroot = getattr(env, 'SRCROOT', None)
    if not rtt_root or not srcroot or not os.path.isdir(rtt_root):
        return
    deploy_dir = os.path.join(rtt_root, 'bsp', 'stm32', 'stm32f427-fmuv2')
    if os.path.isdir(deploy_dir):
        return
    src_dir = os.path.join(srcroot, RTT_BSP_FMUV2_SRC.replace('/', os.sep))
    if not os.path.isdir(src_dir):
        return
    try:
        import shutil
        os.makedirs(os.path.dirname(deploy_dir), exist_ok=True)
        shutil.copytree(src_dir, deploy_dir)
    except Exception:
        pass


def _deploy_pixhawk6c_mini_bsp_if_needed(env):
    """
    If BOARD uses the Pixhawk6C-Mini BSP (stm32/stm32h743-pixhawk6c-mini) and the deploy
    target does not exist, copy from RTT_BSP_PIXHAWK6C_MINI_SRC.
    """
    bsp_rel = RTT_BSP_MAP.get(getattr(env, 'BOARD', None))
    if bsp_rel != 'stm32/stm32h743-pixhawk6c-mini':
        return
    rtt_root = getattr(env, 'RTT_ROOT', None)
    srcroot = getattr(env, 'SRCROOT', None)
    if not rtt_root or not srcroot or not os.path.isdir(rtt_root):
        return
    deploy_dir = os.path.join(rtt_root, 'bsp', 'stm32', 'stm32h743-pixhawk6c-mini')
    if os.path.isdir(deploy_dir):
        return
    src_dir = os.path.join(srcroot, RTT_BSP_PIXHAWK6C_MINI_SRC.replace('/', os.sep))
    if not os.path.isdir(src_dir):
        return
    try:
        import shutil
        os.makedirs(os.path.dirname(deploy_dir), exist_ok=True)
        shutil.copytree(src_dir, deploy_dir)
    except Exception:
        pass


def _ensure_pixhawk6c_mini_packages(env):
    """
    For Pixhawk6C-Mini BSP: (1) sync rtconfig.py from source (fix EXEC_PATH for Linux),
    (2) if packages are missing, run pkgs_update_manual.sh. No-op for other boards.
    """
    bsp_rel = RTT_BSP_MAP.get(getattr(env, 'BOARD', None))
    if bsp_rel != 'stm32/stm32h743-pixhawk6c-mini':
        return
    rtt_root = getattr(env, 'RTT_ROOT', None)
    srcroot = getattr(env, 'SRCROOT', None)
    if not rtt_root or not os.path.isdir(rtt_root):
        return
    deploy_dir = os.path.join(rtt_root, 'bsp', 'stm32', 'stm32h743-pixhawk6c-mini')
    if not os.path.isdir(deploy_dir):
        return
    # Sync key files from source (rtconfig, SConscript, board/*, packages, linker_scripts, USB cherryusb port)
    if srcroot:
        src_bsp = os.path.join(srcroot, RTT_BSP_PIXHAWK6C_MINI_SRC.replace('/', os.sep))
        sync_list = [
            'rtconfig.py', 'rtconfig.h', 'SConscript', '.config',
            os.path.join('board', 'board.c'),
            os.path.join('board', 'rt_board_init.c'),
            os.path.join('board', 'SConscript'), os.path.join('board', 'Kconfig'),
            os.path.join('board', 'linker_scripts', 'link.lds'),
            os.path.join('board', 'ports', 'cherryusb', 'cherryusb.c'),
            os.path.join('board', 'ports', 'cherryusb', 'usb_irq.c'),
            os.path.join('board', 'ports', 'cherryusb', 'SConscript'),
            os.path.join('board', 'ports', 'cherryusb', 'usb_config.h'),
            os.path.join('board', 'CubeMX_Config', 'Inc', 'stm32h7xx_hal_conf.h'),
            os.path.join('board', 'CubeMX_Config', 'Src', 'stm32h7xx_hal_msp.c'),
            os.path.join('packages', 'SConscript'),
        ]
        for rel in sync_list:
            src_f = os.path.join(src_bsp, rel)
            if os.path.isfile(src_f):
                dst = os.path.join(deploy_dir, rel)
                try:
                    import shutil
                    os.makedirs(os.path.dirname(dst) if os.path.dirname(dst) else '.', exist_ok=True)
                    shutil.copy2(src_f, dst)
                except Exception:
                    pass
    need_pkgs = False
    for pkg in ('stm32h7_cmsis_driver-latest', 'stm32h7_hal_driver-latest'):
        if not os.path.isdir(os.path.join(deploy_dir, 'packages', pkg)):
            need_pkgs = True
            break
    if not need_pkgs:
        return
    script = os.path.join(deploy_dir, 'pkgs_update_manual.sh')
    if not os.path.isfile(script) and srcroot:
        src_script = os.path.join(srcroot, RTT_BSP_PIXHAWK6C_MINI_SRC.replace('/', os.sep), 'pkgs_update_manual.sh')
        if os.path.isfile(src_script):
            try:
                import shutil
                shutil.copy2(src_script, script)
            except Exception:
                return
    if not os.path.isfile(script):
        return
    try:
        subprocess.check_call(['bash', script], cwd=deploy_dir, timeout=180)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
        pass


def _rtt_bsp_dir(env):
    """Return absolute path to RTT BSP directory for current BOARD, or None if not mapped."""
    board = getattr(env, 'BOARD', None)
    rtt_root = getattr(env, 'RTT_ROOT', None)
    if not board or not rtt_root or not os.path.isdir(rtt_root):
        return None
    bsp_rel = RTT_BSP_MAP.get(board)
    if not bsp_rel:
        return None
    return os.path.join(rtt_root, 'bsp', bsp_rel)


def _rtt_mcu_family(env):
    """Return 'f4' or 'h7' based on RTT_BSP_MAP for current BOARD."""
    bsp_rel = RTT_BSP_MAP.get(getattr(env, 'BOARD', None), '')
    if 'h743' in bsp_rel or 'h7' in bsp_rel:
        return 'h7'
    return 'f4'


# HAL/CMSIS package paths; selected by _rtt_mcu_family
_RTT_HAL_PKG = {'f4': 'packages/stm32f4_hal_driver-latest', 'h7': 'packages/stm32h7_hal_driver-latest'}
_RTT_CMSIS_PKG = {'f4': 'packages/stm32f4_cmsis_driver-latest', 'h7': 'packages/stm32h7_cmsis_driver-latest'}
_RTT_SYSTEM_C = {'f4': 'system_stm32f4xx.c', 'h7': 'system_stm32h7xx.c'}
_RTT_STARTUP_O = {'f4': 'startup_stm32f427xx.o', 'h7': 'startup_stm32h743xx.o'}

# STM32 common HAL_Drivers (drv_gpio, drv_usart) - scons may not build them into librtthread.a; waf compiles them for rt_hw_pin_init/rt_hw_usart_init
RTT_STM32_HAL_DRIVERS = 'modules/rt-thread/bsp/stm32/libraries/HAL_Drivers'


def _rtt_hal_drv_nodes(bld, env):
    """Return waf Nodes for drv_gpio.c and drv_usart.c so rt_hw_pin_init/rt_hw_usart_init are linked."""
    bsp_dir = _rtt_bsp_dir(env)
    if not bsp_dir:
        return []
    # stm32 common drivers: bsp/stm32/libraries/HAL_Drivers/drivers/
    stm32_lib = os.path.join(os.path.dirname(bsp_dir), 'libraries', 'HAL_Drivers', 'drivers')
    if not os.path.isdir(stm32_lib):
        return []
    root_abs = bld.root.abspath()
    stm32_lib_abs = os.path.abspath(stm32_lib)
    if not stm32_lib_abs.startswith(root_abs):
        return []
    rel = os.path.relpath(stm32_lib_abs, root_abs)
    drv_node = bld.root.find_node(rel)
    if not drv_node or not drv_node.is_child_of(bld.root):
        return []
    out = []
    for name in ('drv_gpio.c', 'drv_usart.c'):
        n = drv_node.find_node(name)
        if n and os.path.isfile(n.abspath()):
            out.append(n)
    return out


def _rtt_hal_src_nodes(bld, env):
    """
    Return list of waf Node for STM32 HAL Src/*.c to be compiled by waf.
    Excludes *_template.c (user copy) and Legacy/ (duplicate/legacy impl).
    Used so that librtthread.a's board.o/drv_*.o get HAL_Init, HAL_GPIO_*, HAL_UART_* etc.
    """
    bsp_dir = _rtt_bsp_dir(env)
    if not bsp_dir:
        return []
    mcu = _rtt_mcu_family(env)
    hal_pkg = _RTT_HAL_PKG.get(mcu, _RTT_HAL_PKG['f4'])
    hal_src_dir = os.path.join(bsp_dir, hal_pkg, 'Src')
    if not os.path.isdir(hal_src_dir):
        return []
    root_abs = bld.root.abspath()
    hal_abs = os.path.abspath(hal_src_dir)
    if not hal_abs.startswith(root_abs):
        return []
    rel = os.path.relpath(hal_abs, root_abs)
    hal_node = bld.root.find_node(rel)
    if not hal_node or not hal_node.is_child_of(bld.root):
        return []
    nodes = hal_node.ant_glob('*.c')
    return [n for n in nodes if '_template' not in n.name and not n.path_from(hal_node).startswith('Legacy')]


def _ensure_librtthread_a(env):
    """
    Ensure librtthread.a exists for the current BOARD (e.g. fmuv2).
    If not present: try to build via scons in RTT BSP then pack .o into librtthread.a.
    RTT default scons produces rt-thread.elf, not .a; we run 'ar rcs librtthread.a' on
    build/*.o to produce the library in the BSP directory.
    Returns (lib_dir, True) if librtthread.a is available; (None, False) on failure.
    """
    bsp_dir = _rtt_bsp_dir(env)
    if not bsp_dir or not os.path.isdir(bsp_dir):
        return None, False
    lib_path = os.path.join(bsp_dir, 'librtthread.a')
    rtconfig_h = os.path.join(bsp_dir, 'rtconfig.h')
    # If rtconfig.h is newer than librtthread.a, config changed (e.g. CherryUSB added) - force rebuild
    if os.path.isfile(lib_path) and os.path.isfile(rtconfig_h):
        if os.path.getmtime(rtconfig_h) > os.path.getmtime(lib_path):
            try:
                os.remove(lib_path)
            except OSError:
                pass
    if os.path.isfile(lib_path):
        return bsp_dir, True

    rtt_root = env.RTT_ROOT
    env_add = os.environ.copy()
    env_add['RTT_ROOT'] = rtt_root
    rtt_exec = getattr(env, 'RTT_EXEC_PATH', None)
    if not rtt_exec and env.get_flat('CC'):
        cc = env.get_flat('CC')
        if os.path.isabs(cc) and ('arm-none-eabi' in cc or 'gcc' in cc):
            rtt_exec = os.path.dirname(cc)
    if rtt_exec:
        env_add['RTT_EXEC_PATH'] = rtt_exec
    # 1) scons in BSP (RTT uses scons; run from BSP dir with RTT_ROOT set)
    try:
        ret = subprocess.call(['scons'], env=env_add, cwd=bsp_dir)
        if ret != 0:
            return None, False
    except OSError:
        return None, False

    # 2) collect build/*.o and create librtthread.a in BSP dir (exclude RTT applications/main so ArduPilot main is used)
    build_dir = os.path.join(bsp_dir, 'build')
    if not os.path.isdir(build_dir):
        return None, False
    objs = []
    for root, _dirs, files in os.walk(build_dir):
        for f in files:
            if not f.endswith('.o'):
                continue
            path = os.path.join(root, f)
            rel = os.path.relpath(path, build_dir).replace(os.sep, '/')
            if 'applications/' in rel and (f == 'main.o' or f == 'arduino_main.o'):
                continue  # skip RTT app main so ArduPilot vehicle main is the single definition
            if f == 'stm32h7xx_it.o':
                continue  # skip CubeMX it.c; USB IRQ from cherryusb usb_irq.c
            objs.append(path)
    # include CMSIS startup (Reset_Handler) if built under packages (scons may output there)
    mcu = _rtt_mcu_family(env)
    startup_name = _RTT_STARTUP_O.get(mcu, _RTT_STARTUP_O['f4'])
    cmsis_pkg = _RTT_CMSIS_PKG.get(mcu, _RTT_CMSIS_PKG['f4'])
    startup_o = os.path.join(bsp_dir, cmsis_pkg, 'Source', 'Templates', 'gcc', startup_name)
    if os.path.isfile(startup_o) and startup_o not in objs:
        objs.append(startup_o)
    if not objs:
        return None, False
    ar = env.get_flat('AR') or 'arm-none-eabi-ar'
    try:
        subprocess.check_call([ar, 'rcs', lib_path] + objs, cwd=bsp_dir)
    except (subprocess.CalledProcessError, OSError):
        return None, False
    return bsp_dir, os.path.isfile(lib_path)


def _manual_rtt_build_instructions(env):
    """Return a short string with manual steps to build librtthread.a."""
    bsp_dir = _rtt_bsp_dir(env)
    if not bsp_dir:
        return "No RTT BSP mapping for BOARD=%s or RTT_ROOT not set." % getattr(env, 'BOARD', '')
    rtt_root = getattr(env, 'RTT_ROOT', '')
    return (
        "Please build RT-Thread and create librtthread.a manually:\n"
        "  1. cd %s\n"
        "  2. export RTT_ROOT=%s\n"
        "  3. (Optional) Install/update packages: pkgs --update (if scons fails for missing packages)\n"
        "  4. scons\n"
        "  5. ar rcs librtthread.a $(find build -name '*.o')\n"
        "Then re-run waf build; LIBPATH will point to the BSP dir."
        % (bsp_dir, rtt_root)
    )


def pre_build(bld):
    """Pre-build hook: ensure hwdef.h exists (e.g. after waf clean)."""
    hwdef_h = os.path.join(bld.env.BUILDROOT, 'hwdef.h')
    if not os.path.exists(hwdef_h):
        try:
            ret = generate_hwdef_h(bld.env)
        except Exception as e:
            bld.fatal("Failed to process hwdef.dat: %s" % e)
        if ret != 0:
            bld.fatal("Failed to process hwdef.dat ret=%d" % ret)


class generate_rtt_bin(Task.Task):
    """Generate .bin from elf for RTT (no external flash)."""
    color = 'CYAN'
    always_run = True

    def keyword(self):
        return "Generating"
    def run(self):
        cmd = [self.env.get_flat('OBJCOPY'), '-O', 'binary',
               self.inputs[0].abspath(), self.outputs[0].abspath()]
        return self.exec_command(cmd)


class generate_rtt_apj(Task.Task):
    """Generate PX4/AP bootloader .apj from .bin."""
    color = 'CYAN'
    always_run = True

    def keyword(self):
        return "apj_gen"
    def run(self):
        import base64
        import json
        import zlib
        bin_data = open(self.inputs[0].abspath(), 'rb').read()
        flash_total = int(getattr(self.env, 'RTT_FLASH_TOTAL', 0) or 2 * 1024 * 1024)
        d = {
            "board_id": int(self.env.APJ_BOARD_ID),
            "magic": "APJFWv1",
            "description": "RTT firmware for %s" % self.env.BOARD,
            "image": base64.b64encode(zlib.compress(bin_data, 9)).decode('utf-8'),
            "image_size": len(bin_data),
            "summary": self.env.BOARD,
            "version": "0.1",
            "board_revision": 0,
            "flash_total": flash_total,
            "image_maxsize": flash_total,
            "flash_free": flash_total - len(bin_data),
        }
        apj_file = self.outputs[0].abspath()
        with open(apj_file, 'w') as f:
            f.write(json.dumps(d, indent=4))


class upload_fw(Task.Task):
    """Upload .apj via Tools/scripts/uploader.py (supports WSL2)."""
    color = 'BLUE'
    always_run = True

    def run(self):
        import platform
        upload_tools = self.env.get_flat('UPLOAD_TOOLS')
        upload_port = self.generator.bld.options.upload_port
        src = self.inputs[0]
        if 'AP_OVERRIDE_UPLOAD_CMD' in os.environ:
            cmd = "{} '{}'".format(os.environ['AP_OVERRIDE_UPLOAD_CMD'], src.abspath())
        elif "microsoft-standard-WSL2" in platform.release():
            if not self._wsl2_prereq_checks():
                return -1
            print("If this takes too long, try power-cycling your hardware\n")
            cmd = "{} -u '{}/uploader.py' '{}'".format('python.exe', upload_tools, src.abspath())
        else:
            cmd = "{} '{}/uploader.py' '{}'".format(
                self.env.get_flat('PYTHON'), upload_tools, src.abspath())
        if upload_port is not None:
            cmd += " '--port' '%s'" % upload_port
        if self.generator.bld.options.upload_force:
            cmd += " '--force'"
        return self.exec_command(cmd)

    def _wsl2_prereq_checks(self):
        try:
            where_python = subprocess.check_output('where.exe python.exe', shell=True, text=True)
        except subprocess.CalledProcessError:
            where_python = ""
        if "python.exe" not in str(where_python):
            print("WSL2: Windows python.exe not found. Install Python 3.9 and add to PATH.")
            return False
        return True

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(upload_fw, self).exec_command(cmd, **kw)

    def keyword(self):
        return "Uploading"


@feature('rtt_ap_library', 'rtt_ap_program')
@before_method('process_source')
def rtt_dynamic_includes(self):
    """Add RTT/AP_HAL_RTT include paths for targets with rtt_ap_library / rtt_ap_program.
    For rtt_ap_program, also add STM32F4 HAL Src/*.c so HAL_Init, HAL_GPIO_*, HAL_UART_* etc. are linked."""
    if self.bld.cmd == 'list':
        return
    self.use += ' rtt'
    self.env.append_value('INCLUDES', [
        self.env.BUILDROOT,
        os.path.join(self.env.AP_HAL_ROOT, 'include'),
    ])
    rtt_root = getattr(self.env, 'RTT_ROOT', None)
    if rtt_root and os.path.isdir(rtt_root):
        mcu = _rtt_mcu_family(self.env)
        cpu_subdir = 'cortex-m7' if mcu == 'h7' else 'cortex-m4'
        libcpu_inc = os.path.join(rtt_root, 'libcpu', 'arm', cpu_subdir)
        incs = [
            os.path.join(rtt_root, 'include'),
            os.path.join(rtt_root, 'bsp'),
            libcpu_inc,  # cpuport.h for rthw.h
        ]
        finsh = os.path.join(rtt_root, 'components', 'finsh')
        if os.path.isdir(finsh):
            incs.append(finsh)
        rtdevice_inc = os.path.join(rtt_root, 'components', 'drivers', 'include')
        if os.path.isdir(rtdevice_inc):
            incs.append(rtdevice_inc)
        self.env.append_value('INCLUDES', incs)
    # BSP directory first so rtconfig.h (from BSP) is found
    bsp_dir = _rtt_bsp_dir(self.env)
    if bsp_dir and os.path.isdir(bsp_dir):
        self.env.append_value('INCLUDES', [bsp_dir])
        mcu = _rtt_mcu_family(self.env)
        hal_pkg = _RTT_HAL_PKG.get(mcu, _RTT_HAL_PKG['f4'])
        cmsis_pkg = _RTT_CMSIS_PKG.get(mcu, _RTT_CMSIS_PKG['f4'])
        hal_inc = os.path.join(bsp_dir, hal_pkg, 'Inc')
        hal_conf_inc = os.path.join(bsp_dir, 'board', 'CubeMX_Config', 'Inc')
        cmsis_inc = os.path.join(bsp_dir, cmsis_pkg, 'Include')
        for d in (hal_inc, hal_conf_inc, cmsis_inc):
            if os.path.isdir(d):
                self.env.append_value('INCLUDES', [d])
    # For the main program: add STM32F4 HAL Src/*.c and stm32 HAL_Drivers (drv_gpio, drv_usart) so undefined refs are resolved
    if 'rtt_ap_program' in self.features:
        hal_sources = _rtt_hal_src_nodes(self.bld, self.env)
        if hal_sources:
            self.source = Utils.to_list(getattr(self, 'source', [])) + hal_sources
        # drv_gpio.c / drv_usart.c provide rt_hw_pin_init, rt_hw_usart_init for drv_common.o (in librtthread.a)
        hal_drv = _rtt_hal_drv_nodes(self.bld, self.env)
        if hal_drv:
            self.source = Utils.to_list(getattr(self, 'source', [])) + hal_drv
            bsp_dir = _rtt_bsp_dir(self.env)
            rtt_root = getattr(self.env, 'RTT_ROOT', None)
            stm32_lib = os.path.join(os.path.dirname(bsp_dir), 'libraries', 'HAL_Drivers')
            board_inc = os.path.join(bsp_dir, 'board') if bsp_dir else ''
            for inc in ([board_inc] if board_inc and os.path.isdir(board_inc) else []):
                self.env.append_value('INCLUDES', [inc])
            for sub in ('', 'drivers', 'config', 'drivers/config'):
                inc = os.path.join(stm32_lib, sub) if sub else stm32_lib
                if os.path.isdir(inc):
                    self.env.append_value('INCLUDES', [inc])
            # drv_*.h include rtdevice.h from RTT components
            if rtt_root:
                rtdevice_inc = os.path.join(rtt_root, 'components', 'drivers', 'include')
                if os.path.isdir(rtdevice_inc):
                    self.env.append_value('INCLUDES', [rtdevice_inc])
        # CMSIS device include for system_stm32xx.c (SystemInit)
        mcu = _rtt_mcu_family(self.env)
        cmsis_pkg = _RTT_CMSIS_PKG.get(mcu, _RTT_CMSIS_PKG['f4'])
        for inc in [
            os.path.join(bsp_dir, 'packages', 'CMSIS-Core-latest', 'Include'),
            os.path.join(bsp_dir, 'packages', cmsis_pkg.replace('packages/', ''), 'Include'),
        ]:
            if os.path.isdir(inc):
                self.env.append_value('INCLUDES', [inc])
    # rt_hw_board_init: strong symbol in BSP board/rt_board_init.c, built by scons and packed into librtthread.a
    # Add CMSIS system_stm32xx.c so startup can call SystemInit (not built by BSP scons for this board)
    if 'rtt_ap_program' in self.features and bsp_dir:
        mcu = _rtt_mcu_family(self.env)
        cmsis_pkg = _RTT_CMSIS_PKG.get(mcu, _RTT_CMSIS_PKG['f4'])
        system_name = _RTT_SYSTEM_C.get(mcu, _RTT_SYSTEM_C['f4'])
        system_c = os.path.join(bsp_dir, cmsis_pkg, 'Source', 'Templates', system_name)
        if os.path.isfile(system_c):
            self.source = Utils.to_list(self.source)
            # Use waf Node so path is resolved from bld.root (relative path as string gets misinterpreted)
            root_abs = self.bld.root.abspath()
            system_c_rel = os.path.relpath(os.path.abspath(system_c), root_abs)
            system_node = self.bld.root.find_node(system_c_rel)
            if system_node:
                self.source.append(system_node)
            else:
                self.source.append(system_c_rel)


@feature('rtt_ap_program')
@after_method('process_source')
def rtt_firmware(self):
    """Add bin/apj generation and upload for RTT builds (like chibios_firmware)."""
    if self.bld.cmd == 'list':
        return
    if not hasattr(self, 'link_task') or self.link_task is None:
        return
    self.link_task.always_run = True

    link_output = self.link_task.outputs[0]
    bin_target = [self.bld.bldnode.find_or_declare(
        'bin/' + link_output.change_ext('.bin').name)]
    apj_target = self.bld.bldnode.find_or_declare(
        'bin/' + link_output.change_ext('.apj').name)

    gen_bin_task = self.create_task('generate_rtt_bin', src=link_output, tgt=bin_target)
    gen_bin_task.set_run_after(self.link_task)

    gen_apj_task = self.create_task('generate_rtt_apj', src=bin_target, tgt=apj_target)
    gen_apj_task.set_run_after(gen_bin_task)

    if getattr(self.bld.options, 'upload', False) and getattr(
            self.env, 'APJ_BOARD_ID', 0):
        upload_task = self.create_task('upload_fw', src=apj_target)
        upload_task.set_run_after(gen_apj_task)


def build(bld):
    """Add hwdef.dat -> hwdef.h/ldscript.ld rule; set LIB/LIBPATH/INCLUDES/LINKFLAGS.
    When BOARD is in RTT_BSP_MAP (e.g. fmuv2) and RTT_ROOT points to modules/rt-thread,
    ensure librtthread.a is available: try auto build (scons + ar) or print manual steps and set LIBPATH."""
    env = bld.env
    _deploy_fmuv2_bsp_if_needed(env)
    _deploy_pixhawk6c_mini_bsp_if_needed(env)
    _ensure_pixhawk6c_mini_packages(env)
    py = env.get_flat('PYTHON')
    script = os.path.join(env.AP_HAL_ROOT, 'hwdef/scripts/rtt_hwdef.py')
    outdir = env.BUILDROOT
    params = getattr(env, 'default_parameters', '') or ''
    hwdef_path = env.HWDEF if isinstance(env.HWDEF, str) else env.HWDEF[0]
    hwdef_rule = "%s '%s' -D '%s' --params '%s' '%s'" % (py, script, outdir, params, hwdef_path)
    if getattr(env, 'HWDEF_EXTRA', None):
        hwdef_rule += " '%s'" % env.HWDEF_EXTRA

    hwdef_rel = hwdef_path if not os.path.isabs(hwdef_path) else os.path.relpath(hwdef_path, env.SRCROOT)
    src_node = bld.srcnode.find_node(hwdef_rel)
    if not src_node:
        src_node = bld.root.find_node(hwdef_path)
    if not src_node:
        bld.fatal("rtt: hwdef not found: %s" % hwdef_path)
    bld(
        source=src_node,
        rule=hwdef_rule,
        group='dynamic_sources',
        target=[
            bld.bldnode.find_or_declare('hwdef.h'),
            bld.bldnode.find_or_declare('ldscript.ld'),
        ],
    )

    env.LIB = Utils.to_list(getattr(env, 'LIB', []))
    env.LIBPATH = Utils.to_list(getattr(env, 'LIBPATH', []))
    env.INCLUDES = Utils.to_list(getattr(env, 'INCLUDES', [])) + [env.BUILDROOT, os.path.join(env.AP_HAL_ROOT, 'include')]
    env.LINKFLAGS = Utils.to_list(getattr(env, 'LINKFLAGS', []))

    # When BOARD has an RTT BSP: use BSP's full link.lds (defines _estack, _sdata, _edata, etc.);
    # otherwise use hwdef-generated ldscript.ld. Linker runs from BUILDROOT so -T needs absolute path for BSP.
    bsp_dir = _rtt_bsp_dir(env)
    if bsp_dir:
        link_lds = os.path.join(bsp_dir, 'board', 'linker_scripts', 'link.lds')
        if os.path.isfile(link_lds):
            env.append_value('LINKFLAGS', ['-T', os.path.abspath(link_lds)])
        else:
            env.append_value('LINKFLAGS', ['-T', 'ldscript.ld'])
    else:
        env.append_value('LINKFLAGS', ['-T', 'ldscript.ld'])

    # When BOARD has an RTT BSP: use librtthread.a from BSP (waf calls scons there if missing).
    # Must set LIBPATH and whole-archive link flags so that -L<bsp_dir> comes before -lrtthread.
    if bsp_dir:
        lib_path = os.path.join(bsp_dir, 'librtthread.a')
        if not os.path.isfile(lib_path):
            lib_dir, ok = _ensure_librtthread_a(env)
            if ok and lib_dir:
                bsp_dir = lib_dir
            else:
                bld.to_log("rtt: librtthread.a not found; link will fail unless you build RTT manually.\n")
                bld.to_log(_manual_rtt_build_instructions(env) + "\n")
        env.append_value('LIBPATH', [bsp_dir])
        # Link librtthread.a in static mode; with STATIC_LINKING, -lrtthread after -Bdynamic
        # would be looked up as shared lib. Add .a path in LINKFLAGS so it is linked statically
        # and before the vehicle lib (whole-archive so BSP/startup objects are included).
        lib_path = os.path.abspath(os.path.join(bsp_dir, 'librtthread.a'))
        env.append_value('LINKFLAGS', '-Wl,--whole-archive')
        env.append_value('LINKFLAGS', lib_path)
        env.append_value('LINKFLAGS', '-Wl,--no-whole-archive')
        # Force strong OTG_FS_IRQHandler from BSP usb_irq.o; startup .s has weak->Default_Handler
        env.append_value('LINKFLAGS', '-Wl,-u,OTG_FS_IRQHandler')
