/*
 * ArduPilot + RT-Thread HAL - RCInput
 * Delegates to AP_RCProtocol; UART bytes fed via AP::RC().update() (from Scheduler).
 * SerialManager binds RC UART (e.g. telem2) to AP::RC().add_uart().
 */

#include "RCInput.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#include <AP_RCProtocol/AP_RCProtocol_config.h>

#if AP_RCPROTOCOL_ENABLED
#include <AP_RCProtocol/AP_RCProtocol.h>
#endif

namespace RTT
{

void RCInput::init()
{
#if AP_RCPROTOCOL_ENABLED
    AP::RC().init();
#endif
}

bool RCInput::new_input()
{
#if AP_RCPROTOCOL_ENABLED
    return AP::RC().new_input();
#else
    return false;
#endif
}

uint8_t RCInput::num_channels()
{
#if AP_RCPROTOCOL_ENABLED
    return AP::RC().num_channels();
#else
    return 0;
#endif
}

uint16_t RCInput::read(uint8_t ch)
{
#if AP_RCPROTOCOL_ENABLED
    return AP::RC().read(ch);
#else
    (void)ch;
    return 0;
#endif
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
#if AP_RCPROTOCOL_ENABLED
    AP::RC().read(periods, len);
    return MIN(len, AP::RC().num_channels());
#else
    (void)periods;
    (void)len;
    return 0;
#endif
}

} // namespace RTT
