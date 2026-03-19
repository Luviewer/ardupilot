/*
 * ArduPilot + RT-Thread HAL - SPIDevice implementation
 */

#include "SPIDevice.h"
#include <AP_HAL/AP_HAL.h>
#include <cstring>
#include <rtthread.h>
#include <drivers/dev_spi.h>

using namespace RTT;

SPIDevice::SPIDevice(const char *name, uint8_t bus_id)
    : AP_HAL::SPIDevice()
    , _dev(nullptr)
    , _bus_id(bus_id)
{
    set_device_bus(bus_id);
    _dev = (struct rt_spi_device *)rt_device_find(name);
}

SPIDevice::~SPIDevice()
{
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    if (_dev == nullptr) return false;
    struct rt_spi_configuration cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = RT_SPI_MODE_3;
    cfg.data_width = 8;
    cfg.max_hz = (speed == AP_HAL::Device::SPEED_HIGH) ? 10000000 : 1000000;
    return rt_spi_configure(_dev, &cfg) == RT_EOK;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                        uint8_t *recv, uint32_t recv_len)
{
    if (_dev == nullptr) return false;
    bool need_sem = !_cs_held;
    if (need_sem && !_sem.take_nonblocking()) return false;
    rt_err_t err = RT_EOK;
    bool need_bus = !_cs_held;
    if (need_bus && rt_spi_take_bus(_dev) != RT_EOK) {
        if (need_sem) _sem.give();
        return false;
    }
    if (need_bus && rt_spi_take(_dev) != RT_EOK) {
        rt_spi_release_bus(_dev);
        if (need_sem) _sem.give();
        return false;
    }
    if (send_len > 0 && recv_len > 0) {
        err = rt_spi_send_then_recv(_dev, send, send_len, recv, recv_len);
    } else if (send_len > 0) {
        err = rt_spi_send(_dev, send, send_len);
    } else if (recv_len > 0) {
        err = rt_spi_recv(_dev, recv, recv_len);
    }
    if (need_bus) {
        rt_spi_release(_dev);
        rt_spi_release_bus(_dev);
    }
    if (need_sem) _sem.give();
    return err == RT_EOK;
}

bool SPIDevice::set_chip_select(bool set)
{
    if (_dev == nullptr) return false;
    if (set) {
        if (_cs_held) return true;
        if (rt_spi_take_bus(_dev) != RT_EOK || rt_spi_take(_dev) != RT_EOK) return false;
        _cs_held = true;
    } else {
        if (!_cs_held) return true;
        rt_spi_release(_dev);
        rt_spi_release_bus(_dev);
        _cs_held = false;
    }
    return true;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    if (_dev == nullptr) return false;
    if (!_sem.take_nonblocking()) return false;
    struct rt_spi_message msg;
    msg.send_buf = send;
    msg.recv_buf = recv;
    msg.length = len;
    msg.cs_take = 1;
    msg.cs_release = 1;
    msg.next = RT_NULL;
    bool ok = false;
    if (rt_spi_take_bus(_dev) == RT_EOK && rt_spi_take(_dev) == RT_EOK) {
        struct rt_spi_message *ret = rt_spi_transfer_message(_dev, &msg);
        rt_spi_release(_dev);
        rt_spi_release_bus(_dev);
        ok = (ret == RT_NULL);
    }
    _sem.give();
    return ok;
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &_sem;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(
    uint32_t, AP_HAL::Device::PeriodicCb)
{
    return nullptr;
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle, uint32_t)
{
    return false;
}
