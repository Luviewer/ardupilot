/*
 * Strong symbol for rt_hw_board_init so the linker resolves the reference.
 * Minimal init: pin and usart (drv_common in HAL_Drivers provides weak symbol;
 * waf links this from librtthread.a so the ref from HAL_RTT_Class is satisfied).
 */
#include <rtthread.h>

extern int rt_hw_pin_init(void);
extern int rt_hw_usart_init(void);

void rt_hw_board_init(void)
{
    rt_hw_pin_init();
    rt_hw_usart_init();
}
