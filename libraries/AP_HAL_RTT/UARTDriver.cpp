/*
 * ArduPilot + RT-Thread HAL - UARTDriver (T2-1, T2-2: _begin, wait_timeout, rx_indicate)
 */

#include "UARTDriver.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

using namespace RTT;

/* 板级设备名表：未定义 HAL_RTT_UART_DEVICE_LIST 时使用占位 uart1, uart2, ... */
#ifndef HAL_RTT_UART_DEVICE_LIST
#define HAL_RTT_UART_DEVICE_LIST \
    "uart1", "uart2", "uart3", "uart4", "uart5", "uart6", "uart7", "uart8"
#endif

static const char *const _device_names[] = { HAL_RTT_UART_DEVICE_LIST };

UARTDriver *UARTDriver::_drivers[RTT_UART_MAX_DRIVERS] = {};

UARTDriver::UARTDriver(uint8_t port_num)
    : AP_HAL::UARTDriver()
    , _port_num(port_num)
    , _dev(nullptr)
    , _rx_sem(nullptr)
    , _baudrate(0)
    , _initialized(false)
{
    if (_port_num < RTT_UART_MAX_DRIVERS) {
        _drivers[_port_num] = this;
    }
}

rt_err_t UARTDriver::_rx_indicate_cb(rt_device_t dev, rt_size_t size)
{
    (void)size;
    for (uint8_t i = 0; i < RTT_UART_MAX_DRIVERS; i++) {
        if (_drivers[i] != nullptr && _drivers[i]->_dev == dev && _drivers[i]->_rx_sem != nullptr) {
            rt_sem_release(_drivers[i]->_rx_sem);
            break;
        }
    }
    return RT_EOK;
}

void UARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (baud == 0 && rxSpace == 0 && txSpace == 0 && _initialized) {
        return;
    }

    if (_port_num >= RTT_UART_MAX_DRIVERS ||
        _port_num >= ARRAY_SIZE(_device_names)) {
        return;
    }

    const char *name = _device_names[_port_num];
    rt_device_t dev = rt_device_find(name);
    if (dev == nullptr) {
        return;
    }

    /* INT_RX 或 DMA_RX 以便 rx_indicate 被驱动调用 */
    rt_err_t err = rt_device_open(dev, RT_DEVICE_FLAG_INT_RX);
    if (err != RT_EOK) {
        err = rt_device_open(dev, RT_DEVICE_FLAG_DMA_RX);
    }
    if (err != RT_EOK) {
        return;
    }

    _dev = dev;
    _baudrate = baud;
#if defined(RT_DEVICE_CTRL_SET_BAUD_RATE)
    if (_baudrate != 0) {
        rt_device_control(_dev, RT_DEVICE_CTRL_SET_BAUD_RATE, &_baudrate);
    }
#endif

    /* 每个端口独立 rx_sem */
    if (_rx_sem == nullptr) {
        char sem_name[RT_NAME_MAX];
        rt_snprintf(sem_name, sizeof(sem_name), "urx%u", (unsigned)_port_num);
        _rx_sem = rt_sem_create(sem_name, 0, RT_IPC_FLAG_FIFO);
        if (_rx_sem == nullptr) {
            rt_device_close(_dev);
            _dev = nullptr;
            return;
        }
    }

    rt_device_set_rx_indicate(_dev, _rx_indicate_cb);

    uint16_t rxS = rxSpace < 512 ? 512 : rxSpace;
    uint16_t txS = txSpace < 512 ? 512 : txSpace;
    if (_readbuf.get_size() != rxS) {
        _readbuf.set_size(rxS);
    }
    if (_writebuf.get_size() != txS) {
        _writebuf.set_size(txS);
    }

    _initialized = true;
}

void UARTDriver::_end()
{
    if (!_initialized) {
        return;
    }
    _initialized = false;
    if (_dev != nullptr) {
        rt_device_set_rx_indicate(_dev, nullptr);
        rt_device_close(_dev);
        _dev = nullptr;
    }
    if (_rx_sem != nullptr) {
        rt_sem_delete(_rx_sem);
        _rx_sem = nullptr;
    }
    _readbuf.set_size(0);
    _writebuf.set_size(0);
    if (_port_num < RTT_UART_MAX_DRIVERS) {
        _drivers[_port_num] = nullptr;
    }
}

void UARTDriver::_flush()
{
    _drain_writebuf_to_dev();
}

uint32_t UARTDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    return _readbuf.available();
}

void UARTDriver::_drain_rx_to_readbuf()
{
    if (_dev == nullptr) {
        return;
    }
    rt_size_t n = rt_device_read(_dev, 0, _rx_bounce, sizeof(_rx_bounce));
    if (n > 0) {
        _readbuf.write(_rx_bounce, n);
    }
}

void UARTDriver::_drain_writebuf_to_dev()
{
    if (_dev == nullptr) {
        return;
    }
    uint32_t n = _writebuf.peekbytes(_tx_bounce, sizeof(_tx_bounce));
    if (n > 0) {
        rt_size_t w = rt_device_write(_dev, 0, _tx_bounce, n);
        if (w > 0) {
            _writebuf.advance(w);
        }
    }
}

bool UARTDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    if (!_initialized || _rx_sem == nullptr) {
        return false;
    }
    rt_tick_t tick = rt_tick_from_millisecond(timeout_ms);
    uint32_t t0 = AP_HAL::millis();
    while (_readbuf.available() < n) {
        _drain_rx_to_readbuf();
        if (_readbuf.available() >= n) {
            return true;
        }
        rt_err_t ret = rt_sem_take(_rx_sem, tick);
        if (ret == RT_EOK) {
            _drain_rx_to_readbuf();
        } else {
            /* 超时：按 AP_HAL 语义返回 false */
            return false;
        }
        uint32_t elapsed = AP_HAL::millis() - t0;
        if (elapsed >= timeout_ms) {
            return false;
        }
        tick = rt_tick_from_millisecond(timeout_ms - elapsed);
    }
    return true;
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }
    _drain_rx_to_readbuf();
    uint32_t n = _readbuf.read(buffer, count);
    return n > 0 ? (ssize_t)n : 0;
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }
    return _writebuf.write(buffer, size);
}

bool UARTDriver::_discard_input()
{
    if (!_initialized) {
        return false;
    }
    _readbuf.clear();
    return true;
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return _writebuf.available() > 0;
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    return _writebuf.space();
}

void UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    _drain_rx_to_readbuf();
    _drain_writebuf_to_dev();
}

#if HAL_UART_STATS_ENABLED
void UARTDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
{
    (void)stats;
    (void)dt_ms;
    str.printf("RTT_UART%u baud=%lu\n", (unsigned)_port_num, (unsigned long)_baudrate);
}
#endif
