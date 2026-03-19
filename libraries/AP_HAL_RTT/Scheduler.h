/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_RTT_Namespace.h"
#include "Semaphores.h"
#include <rtthread.h>

#define RTT_SCHEDULER_MAX_TIMER_PROCS 10
#define RTT_SCHEDULER_MAX_IO_PROCS 10

class RTT::Scheduler : public AP_HAL::Scheduler
{
public:
    Scheduler();
    void init() override;
    void set_callbacks(AP_HAL::HAL::Callbacks* cb) { callbacks = cb; }
    void delay(uint16_t ms) override;
    void delay_microseconds(uint16_t us) override;
    void register_timer_process(AP_HAL::MemberProc proc) override;
    void register_io_process(AP_HAL::MemberProc proc) override;
    void register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) override;
    void reboot(bool hold_in_bootloader) override;
    bool in_main_thread() const override;
    void set_system_initialized() override;
    bool is_system_initialized() override;
    bool thread_create(AP_HAL::MemberProc proc, const char* name, uint32_t stack_size,
                       priority_base base, int8_t priority) override;

    /** Called from main loop entry to record main thread id (RTT port) */
    void set_main_thread_id(rt_thread_t id) { _main_thread_id = id; }

private:
    AP_HAL::HAL::Callbacks* callbacks = nullptr;
    AP_HAL::Proc _failsafe = nullptr;

    AP_HAL::MemberProc _timer_proc[RTT_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs = 0;
    AP_HAL::MemberProc _io_proc[RTT_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs = 0;

    static bool _system_initialized;
    rt_thread_t _main_thread_id = nullptr;

    Semaphore _timer_sem;
    Semaphore _io_sem;

    bool _in_timer_proc = false;
    bool _in_io_proc = false;

    void _run_timers();
    void _run_io();

    static void _storage_thread_entry(void *arg);
    rt_thread_t _storage_thread_ctx = nullptr;
};
