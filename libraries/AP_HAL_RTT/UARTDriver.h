/*
 * ArduPilot + RT-Thread HAL - UARTDriver (T2-1, T2-2: _begin, wait_timeout, rx_indicate)
 */

#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "HAL_RTT_Namespace.h"

#include <rtthread.h>

#define RTT_UART_MAX_DRIVERS 8
#define RTT_UART_RX_BOUNCE_SIZE 64
#define RTT_UART_TX_BOUNCE_SIZE 64

namespace RTT
{

class UARTDriver : public AP_HAL::UARTDriver
{
public:
    UARTDriver(uint8_t port_num);

    CLASS_NO_COPY(UARTDriver);

    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;
    uint32_t get_baud_rate() const override { return _baudrate; }

    /* 带超时阻塞等数据：rx_indicate 唤醒后从 dev 读到 _readbuf，满足 n 字节或超时返回 */
    bool wait_timeout(uint16_t n, uint32_t timeout_ms) override;

    void _timer_tick(void) override;

#if HAL_UART_STATS_ENABLED
    void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) override;
#endif

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override WARN_IF_UNUSED;
    size_t _write(const uint8_t *buffer, size_t size) override;
    bool _discard_input() override;

private:
    uint8_t _port_num;
    rt_device_t _dev;
    rt_sem_t _rx_sem;
    uint32_t _baudrate;
    bool _initialized;

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    uint8_t _rx_bounce[RTT_UART_RX_BOUNCE_SIZE];
    uint8_t _tx_bounce[RTT_UART_TX_BOUNCE_SIZE];

    void _drain_rx_to_readbuf();
    void _drain_writebuf_to_dev();

    /* 静态回调：仅 rt_sem_release(rx_sem)，供 rt_device_set_rx_indicate 使用 */
    static rt_err_t _rx_indicate_cb(rt_device_t dev, rt_size_t size);

    static UARTDriver *_drivers[RTT_UART_MAX_DRIVERS];
};

} // namespace RTT
