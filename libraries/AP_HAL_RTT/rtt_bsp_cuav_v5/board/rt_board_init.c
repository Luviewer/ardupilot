/*
 * Strong symbol for rt_hw_board_init. Set VTOR first for ArduPilot bootloader
 * (app at 0x08008000), then clock/pin/usart init.
 */
#include <rtthread.h>
#include "board.h"

extern int rt_hw_pin_init(void);
extern int rt_hw_usart_init(void);

void rt_hw_board_init(void)
{
    /* ArduPilot bootloader leaves 32KB at 0x08000000; app runs at 0x08008000 */
    SCB->VTOR = 0x08008000U;

    SystemClock_Config();
    rt_hw_pin_init();
    rt_hw_usart_init();
}
