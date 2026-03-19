/*
 * Strong symbol for rt_hw_board_init so the linker resolves the reference
 * from HAL_RTT_Class (drv_common.o in librtthread.a only provides a weak symbol).
 * Minimal init: pin and usart so RTT board bring-up works.
 */
#include <rtthread.h>

extern int rt_hw_pin_init(void);
extern int rt_hw_usart_init(void);

void rt_hw_board_init(void)
{
    rt_hw_pin_init();
    rt_hw_usart_init();
}
