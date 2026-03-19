/*
 * Strong symbol for rt_hw_board_init so the linker resolves the reference.
 * Minimal init: clock (HSI48 for USB), pin and usart (drv_common in HAL_Drivers
 * provides weak symbol; waf links this from librtthread.a so the ref from HAL_RTT_Class is satisfied).
 *
 * For Pixhawk6C Mini we also start a small LED blink thread on PD10/PD11 so
 * ACT/B/E LEDs give a visible heartbeat independent of ArduPilot.
 */
#include <rtthread.h>
#include "board.h"
#include "stm32h7xx.h"

extern int rt_hw_pin_init(void);
extern int rt_hw_usart_init(void);

/* Pixhawk6C LEDs from ChibiOS hwdef: PD10 RED (B/E), PD11 BLUE (ACT) */
#define LED_RED_PIN   GET_PIN(D, 10)
#define LED_BLUE_PIN  GET_PIN(D, 11)

/* Flash address where our application starts (must match link.lds ROM ORIGIN) */
#define APP_FLASH_BASE  0x08020000UL

static void _rtt_led_blink_entry(void *parameter)
{
    (void)parameter;
    int state = 0;
    while (1) {
        rt_pin_write(LED_RED_PIN, state ? PIN_LOW : PIN_HIGH);
        rt_pin_write(LED_BLUE_PIN, state ? PIN_HIGH : PIN_LOW);
        state = !state;
        rt_thread_mdelay(250);
    }
}

void rt_hw_board_init(void)
{
    /*
     * Relocate VTOR to our app's vector table.
     * After a jump from the bootloader, SCB->VTOR still points to 0x08000000
     * (bootloader's vector table). Every interrupt would go there instead of
     * here, breaking SysTick, UART, USB and everything else.
     * This MUST be the very first thing before any other init or interrupt.
     */
    SCB->VTOR = APP_FLASH_BASE;
    __DSB();
    __ISB();

    /* HAL init: configure HAL tick source, NVIC priority grouping, etc. */
    HAL_Init();

    SystemClock_Config(); /* HSE/PLL and HSI48 for USB OTG_FS; must run before any peripheral */
    rt_hw_pin_init();
    rt_hw_usart_init();

    /* Configure LEDs and start blink thread for basic alive indication */
    rt_pin_mode(LED_RED_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_BLUE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED_RED_PIN, PIN_HIGH);
    rt_pin_write(LED_BLUE_PIN, PIN_LOW);

    rt_thread_t led_thread = rt_thread_create(
        "led_blink",
        _rtt_led_blink_entry,
        RT_NULL,
        512,
        RT_THREAD_PRIORITY_MAX - 1,
        20);
    if (led_thread) {
        rt_thread_startup(led_thread);
    }
}
