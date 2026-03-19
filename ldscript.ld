/*
 * Linker script for RT-Thread + AP_HAL_RTT build.
 * Actual memory layout may be provided by RTT BSP (e.g. modules/rt-thread/bsp/.../link.lds / *.ld).
 * 由 BSP 提供时请使用 BSP 的链接脚本。
 */

ENTRY(Reset_Handler)

SECTIONS
{
    .text : { *(.text*) }
    .data : { *(.data*) }
    .bss  : { *(.bss*) *(COMMON) }
}

