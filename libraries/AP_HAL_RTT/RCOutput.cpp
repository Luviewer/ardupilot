/*
 * ArduPilot + RT-Thread HAL - RCOutput
 * Pixhawk6C-Mini: pwm1 ch1-4, pwm4 ch3-4, pwm5 ch1-2 -> chan 0-7.
 * Uses rt_pwm_set(device, channel, period_ns, pulse_ns); 50Hz default.
 */

#include "RCOutput.h"
#include <rtthread.h>
#include <rtdevice.h>

#define RTT_PWM_MAX_CHANNELS 8
#define PWM_PERIOD_50HZ_NS  20000000U  // 20ms in ns

namespace RTT
{

#if defined(RT_USING_PWM)
struct PwmChan {
    struct rt_device_pwm *dev;
    int ch;
};
static struct PwmChan _chan_map[RTT_PWM_MAX_CHANNELS];
static uint16_t _last_write[RTT_PWM_MAX_CHANNELS];
static bool _initialized;
static bool _corked;
#endif

void RCOutput::init()
{
#if defined(RT_USING_PWM)
    if (_initialized) return;
    for (unsigned i = 0; i < RTT_PWM_MAX_CHANNELS; i++) {
        _chan_map[i].dev = nullptr;
        _last_write[i] = 0;
    }
    // Chan 0-3: pwm1 ch1-4, 4-5: pwm4 ch3-4, 6-7: pwm5 ch1-2
    static const struct { const char *name; int ch; } map[] = {
        {"pwm1", 1}, {"pwm1", 2}, {"pwm1", 3}, {"pwm1", 4},
        {"pwm4", 3}, {"pwm4", 4},
        {"pwm5", 1}, {"pwm5", 2},
    };
    struct rt_device_pwm *p1 = (struct rt_device_pwm *)rt_device_find("pwm1");
    struct rt_device_pwm *p4 = (struct rt_device_pwm *)rt_device_find("pwm4");
    struct rt_device_pwm *p5 = (struct rt_device_pwm *)rt_device_find("pwm5");
    if (!p1 || !p4 || !p5) return;
    for (unsigned i = 0; i < RTT_PWM_MAX_CHANNELS; i++) {
        if (i < 4) _chan_map[i].dev = p1;
        else if (i < 6) _chan_map[i].dev = p4;
        else _chan_map[i].dev = p5;
        _chan_map[i].ch = map[i].ch;
        rt_device_open(&_chan_map[i].dev->parent, RT_DEVICE_OFLAG_RDWR);
    }
    _initialized = true;
    _corked = false;
#endif
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    (void)chmask;
    (void)freq_hz;
    // Phase1: fixed 50Hz; period_ns = 20e6
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    (void)chan;
    return 50;
}

void RCOutput::enable_ch(uint8_t chan)
{
#if defined(RT_USING_PWM)
    if (!_initialized || chan >= RTT_PWM_MAX_CHANNELS) return;
    struct rt_device_pwm *dev = _chan_map[chan].dev;
    if (dev) rt_pwm_enable(dev, _chan_map[chan].ch);
#endif
    (void)chan;
}

void RCOutput::disable_ch(uint8_t chan)
{
#if defined(RT_USING_PWM)
    if (!_initialized || chan >= RTT_PWM_MAX_CHANNELS) return;
    struct rt_device_pwm *dev = _chan_map[chan].dev;
    if (dev) rt_pwm_disable(dev, _chan_map[chan].ch);
#endif
    (void)chan;
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
#if defined(RT_USING_PWM)
    if (!_initialized || chan >= RTT_PWM_MAX_CHANNELS) return;
    _last_write[chan] = period_us;
    uint32_t pulse_ns = (uint32_t)period_us * 1000U;
    struct rt_device_pwm *dev = _chan_map[chan].dev;
    int ch = _chan_map[chan].ch;
    if (!dev) return;
    if (!_corked) {
        rt_pwm_set(dev, ch, PWM_PERIOD_50HZ_NS, pulse_ns);
    }
#else
    (void)chan;
    (void)period_us;
#endif
}

void RCOutput::cork()
{
#if defined(RT_USING_PWM)
    _corked = true;
#endif
}

void RCOutput::push()
{
#if defined(RT_USING_PWM)
    _corked = false;
    for (unsigned i = 0; i < RTT_PWM_MAX_CHANNELS; i++) {
        if (_chan_map[i].dev) {
            rt_pwm_set(_chan_map[i].dev, _chan_map[i].ch,
                       PWM_PERIOD_50HZ_NS, (uint32_t)_last_write[i] * 1000U);
        }
    }
#endif
}

uint16_t RCOutput::read(uint8_t chan)
{
#if defined(RT_USING_PWM)
    if (chan < RTT_PWM_MAX_CHANNELS) return _last_write[chan];
#endif
    return 0;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
#if defined(RT_USING_PWM)
    for (uint8_t i = 0; i < len && i < RTT_PWM_MAX_CHANNELS; i++) {
        period_us[i] = _last_write[i];
    }
#endif
    (void)period_us;
    (void)len;
}

} // namespace RTT
