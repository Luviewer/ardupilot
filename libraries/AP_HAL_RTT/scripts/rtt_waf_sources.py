# -*- coding: utf-8 -*-
"""
RT-Thread source lists for waf build (no scons).
Used by Tools/ardupilotwaf/rtt.py to compile RTT kernel, BSP, HAL_Drivers, components directly.
Each function returns (source_paths, cpppath, cppdefines).
"""

import os
import re


def _parse_rtconfig_h(rtconfig_path):
    """Parse rtconfig.h and return a dict of macro name -> value (1 if no value)."""
    opts = {}
    if not os.path.isfile(rtconfig_path):
        return opts
    with open(rtconfig_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line.startswith('#define'):
                continue
            parts = line.split(None, 2)
            if len(parts) < 2:
                continue
            name = parts[1]
            if name.startswith('_'):
                continue
            value = parts[2] if len(parts) > 2 else '1'
            value = value.strip()
            if value and value[0] in '0123456789':
                try:
                    opts[name] = int(value, 0)
                except ValueError:
                    opts[name] = value
            else:
                opts[name] = 1
    return opts


def _get_depend(opts, depend):
    """Simulate GetDepend: if depend is str, return True if defined and non-0."""
    if isinstance(depend, str):
        return opts.get(depend, 0) not in (0, None, '')
    for item in depend:
        if item and opts.get(item, 0) in (0, None, ''):
            return False
    return True


def get_rtt_kernel_sources(rtt_root, bsp_dir):
    """
    Return (source_paths, cpppath, cppdefines) for RTT kernel under rtt_root/src.
    Matches src/SConscript logic using rtconfig.h in bsp_dir.
    """
    rtconfig_path = os.path.join(bsp_dir, 'rtconfig.h')
    opts = _parse_rtconfig_h(rtconfig_path)

    src_dir = os.path.join(rtt_root, 'src')
    if not os.path.isdir(src_dir):
        return [], [], []

    # Collect all .c under src (including klibc), exclude utest entirely
    sources = []
    for root, dirs, files in os.walk(src_dir):
        rel = os.path.relpath(root, src_dir)
        if 'utest' in rel.split(os.sep):
            continue
        for f in files:
            if f.endswith('.c'):
                sources.append(os.path.join(root, f))

    # Exclude by RT_USING_* (same as SConscript SrcRemove)
    exclude = set()
    if not _get_depend(opts, 'RT_USING_SMALL_MEM'):
        exclude.add('mem.c')
    if not _get_depend(opts, 'RT_USING_SLAB'):
        exclude.add('slab.c')
    if not _get_depend(opts, 'RT_USING_MEMPOOL'):
        exclude.add('mempool.c')
    if not _get_depend(opts, 'RT_USING_MEMHEAP'):
        exclude.add('memheap.c')
    if not _get_depend(opts, 'RT_USING_SIGNALS'):
        exclude.add('signal.c')
    if not _get_depend(opts, 'RT_USING_DEVICE'):
        exclude.add('device.c')
    if _get_depend(opts, 'RT_USING_SMP'):
        exclude.add('cpu_up.c')
        exclude.add('scheduler_up.c')
    else:
        exclude.add('cpu_mp.c')
        exclude.add('scheduler_mp.c')

    source_paths = [p for p in sources if os.path.basename(p) not in exclude]

    cpppath = [
        os.path.join(rtt_root, 'include'),
        src_dir,
    ]
    cppdefines = ['__RTTHREAD__', '__RT_KERNEL_SOURCE__']
    return source_paths, cpppath, cppdefines


def get_rtt_libcpu_sources(rtt_root, bsp_dir):
    """
    Return (source_paths, cpppath, cppdefines) for libcpu (e.g. arm/cortex-m4 + common).
    Derives ARCH/CPU from rtconfig.h (ARCH_ARM, ARCH_ARM_CORTEX_M4 -> cortex-m4).
    """
    rtconfig_path = os.path.join(bsp_dir, 'rtconfig.h')
    opts = _parse_rtconfig_h(rtconfig_path)

    libcpu_root = os.path.join(rtt_root, 'libcpu')
    if not os.path.isdir(libcpu_root):
        return [], [], []

    arch = None
    if opts.get('ARCH_ARM'):
        arch = 'arm'
    if not arch:
        return [], [], []

    cpu = None
    if opts.get('ARCH_ARM_CORTEX_M4'):
        cpu = 'cortex-m4'
    elif opts.get('ARCH_ARM_CORTEX_M7'):
        cpu = 'cortex-m7'
    elif opts.get('ARCH_ARM_CORTEX_M3'):
        cpu = 'cortex-m3'
    elif opts.get('ARCH_ARM_CORTEX_M0'):
        cpu = 'cortex-m0'
    elif opts.get('ARCH_ARM_CORTEX_M33'):
        cpu = 'cortex-m33'
    else:
        cpu = 'cortex-m4'  # default for stm32f4

    arm_dir = os.path.join(libcpu_root, arch)
    common_dir = os.path.join(arm_dir, 'common')
    cpu_dir = os.path.join(arm_dir, cpu)
    source_paths = []
    cpppath = []

    # common: *.c and for gcc *_init.S, *_gcc.S; exclude atomic_arm.c if no RT_USING_HW_ATOMIC
    if os.path.isdir(common_dir):
        for f in os.listdir(common_dir):
            p = os.path.join(common_dir, f)
            if f.endswith('.c'):
                if f == 'atomic_arm.c' and not _get_depend(opts, 'RT_USING_HW_ATOMIC'):
                    continue
                source_paths.append(p)
            elif f.endswith('.S') and ('_gcc.S' in f or '_init.S' in f):
                source_paths.append(p)
        cpppath.append(common_dir)

    # cpu port: cpuport.c, context_gcc.S (for gcc)
    if os.path.isdir(cpu_dir):
        for f in os.listdir(cpu_dir):
            p = os.path.join(cpu_dir, f)
            if f.endswith('.c'):
                source_paths.append(p)
            elif f.endswith('.S') and 'gcc' in f.lower():
                source_paths.append(p)
        cpppath.append(cpu_dir)

    return source_paths, cpppath, []


def get_rtt_bsp_sources(bsp_dir):
    """
    Return (source_paths, cpppath, cppdefines) for BSP board + ports.
    Matches board/SConscript and board/ports/SConscript for stm32f427-robomaster-a.
    """
    board_dir = os.path.join(bsp_dir, 'board')
    if not os.path.isdir(board_dir):
        return [], [], []

    source_paths = []
    # board.c, CubeMX_Config/Src/stm32f4xx_hal_msp.c, stm32f4xx_it.c
    for name in ['board.c',
                 'CubeMX_Config/Src/stm32f4xx_hal_msp.c',
                 'CubeMX_Config/Src/stm32f4xx_it.c']:
        p = os.path.join(board_dir, name)
        if os.path.isfile(p):
            source_paths.append(p)

    # ports: sdcard_port.c only if BSP_USING_SDCARD
    opts = _parse_rtconfig_h(os.path.join(bsp_dir, 'rtconfig.h'))
    if _get_depend(opts, 'BSP_USING_SDCARD'):
        sp = os.path.join(board_dir, 'ports', 'sdcard_port.c')
        if os.path.isfile(sp):
            source_paths.append(sp)

    cpppath = [
        board_dir,
        os.path.join(board_dir, 'CubeMX_Config', 'Inc'),
        os.path.join(board_dir, 'ports'),
    ]
    cppdefines = ['STM32F427xx']
    return source_paths, cpppath, cppdefines


def get_rtt_hal_drivers_sources(rtt_root, bsp_dir):
    """
    Return (source_paths, cpppath, cppdefines) for HAL_Drivers.
    Path: os.path.dirname(bsp_dir)/libraries/HAL_Drivers (stm32/libraries/HAL_Drivers).
    """
    libraries_dir = os.path.join(os.path.dirname(bsp_dir), 'libraries')
    hal_drivers_dir = os.path.join(libraries_dir, 'HAL_Drivers')
    if not os.path.isdir(hal_drivers_dir):
        return [], [], []

    rtconfig_path = os.path.join(bsp_dir, 'rtconfig.h')
    opts = _parse_rtconfig_h(rtconfig_path)

    source_paths = []
    # drv_common.c always
    p_common = os.path.join(hal_drivers_dir, 'drv_common.c')
    if os.path.isfile(p_common):
        source_paths.append(p_common)

    drivers_dir = os.path.join(hal_drivers_dir, 'drivers')
    config_dir = os.path.join(hal_drivers_dir, 'drivers', 'config')
    if os.path.isdir(drivers_dir):
        # RT_USING_PIN -> drv_gpio.c (skip: drv_gpio.c can conflict with BSP/packages macro layout)
        # if _get_depend(opts, 'RT_USING_PIN'):
        #     p = os.path.join(drivers_dir, 'drv_gpio.c')
        #     if os.path.isfile(p):
        #         source_paths.append(p)
        # RT_USING_SERIAL -> drv_usart
        if _get_depend(opts, 'RT_USING_SERIAL') and not _get_depend(opts, 'RT_USING_SERIAL_V2'):
            p = os.path.join(drivers_dir, 'drv_usart.c')
            if os.path.isfile(p):
                source_paths.append(p)
        elif _get_depend(opts, 'RT_USING_SERIAL') and _get_depend(opts, 'RT_USING_SERIAL_V2'):
            p = os.path.join(drivers_dir, 'drv_usart_v2.c')
            if os.path.isfile(p):
                source_paths.append(p)

    cpppath = [hal_drivers_dir, drivers_dir]
    if os.path.isdir(config_dir):
        cpppath.append(config_dir)
    # CMSIS if not PKG_USING_CMSIS_CORE (BSP may use pkg)
    cmsis_include = os.path.join(hal_drivers_dir, 'CMSIS', 'Include')
    if os.path.isdir(cmsis_include):
        cpppath.append(cmsis_include)

    return source_paths, cpppath, []


def get_rtt_components_sources(rtt_root, bsp_dir):
    """
    Return (source_paths, cpppath, cppdefines) for components: finsh (+ optional libc).
    Matches RT_USING_FINSH -> finsh; RT_USING_NANO false -> libc subdirs (minimal set).
    """
    comp_dir = os.path.join(rtt_root, 'components')
    if not os.path.isdir(comp_dir):
        return [], [], []

    rtconfig_path = os.path.join(bsp_dir, 'rtconfig.h')
    opts = _parse_rtconfig_h(rtconfig_path)
    source_paths = []
    cpppath = []

    # Finsh: shell.c, msh.c, msh_parse.c; + cmd.c if MSH_USING_BUILT_IN_COMMANDS
    if _get_depend(opts, 'RT_USING_FINSH'):
        finsh_dir = os.path.join(comp_dir, 'finsh')
        if os.path.isdir(finsh_dir):
            for f in ['shell.c', 'msh.c', 'msh_parse.c']:
                p = os.path.join(finsh_dir, f)
                if os.path.isfile(p):
                    source_paths.append(p)
            if _get_depend(opts, 'MSH_USING_BUILT_IN_COMMANDS'):
                p = os.path.join(finsh_dir, 'cmd.c')
                if os.path.isfile(p):
                    source_paths.append(p)
            cpppath.append(finsh_dir)

    # components/drivers/core/device.c when RT_USING_DEVICE (used by many BSPs)
    if _get_depend(opts, 'RT_USING_DEVICE'):
        core_dir = os.path.join(comp_dir, 'drivers', 'core')
        device_c = os.path.join(core_dir, 'device.c')
        if os.path.isfile(device_c):
            source_paths.append(device_c)
            cpppath.append(os.path.join(comp_dir, 'drivers', 'include'))

    return source_paths, cpppath, []
