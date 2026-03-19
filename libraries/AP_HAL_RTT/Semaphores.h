/*
 * ArduPilot + RT-Thread HAL - Semaphores (rt_sem wrappers).
 * 需链接 librtthread。
 */

#pragma once

/* Do not include AP_HAL.h here: rtt.h defines HAL_Semaphore after this include,
 * and AP_HAL.h (and its deps like RingBuffer.h) use HAL_Semaphore. */
#include <AP_HAL/Semaphores.h>
#include "HAL_RTT_Namespace.h"
#include <rtthread.h>

namespace RTT
{

class Semaphore : public AP_HAL::Semaphore
{
public:
    Semaphore();
    ~Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override WARN_IF_UNUSED;
    bool take_nonblocking() override WARN_IF_UNUSED;
    void take_blocking() override;
private:
    rt_sem_t _sem = nullptr;
};

class BinarySemaphore : public AP_HAL::BinarySemaphore
{
public:
    BinarySemaphore(bool initial_state = false);
    ~BinarySemaphore();
    bool wait(uint32_t timeout_us) override WARN_IF_UNUSED;
    bool wait_blocking() override;
    void signal() override;
private:
    rt_sem_t _sem = nullptr;
};

} // namespace RTT
