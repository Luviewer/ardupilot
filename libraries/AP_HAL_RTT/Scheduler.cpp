/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "AP_HAL_RTT/Scheduler.h"
#include <AP_HAL/AP_HAL.h>
#include <rtthread.h>

#include <AP_RCProtocol/AP_RCProtocol_config.h>
#if AP_RCPROTOCOL_ENABLED
#include <AP_RCProtocol/AP_RCProtocol.h>
#endif

using namespace RTT;

extern const AP_HAL::HAL& hal;

bool Scheduler::_system_initialized = false;

Scheduler::Scheduler()
{
}

void Scheduler::_storage_thread_entry(void *arg)
{
    (void)arg;
    while (1) {
        rt_thread_mdelay(1);
        if (hal.storage != nullptr) {
            hal.storage->_timer_tick();
        }
    }
}

void Scheduler::init()
{
    // _main_thread_id is set later in _main_loop_entry when we are in the main thread.
    // _timer_sem and _io_sem are already created in their constructors (rt_sem_create in Semaphore ctor).

#if HAL_WITH_RAMTRON
    _storage_thread_ctx = rt_thread_create("storage",
                                          _storage_thread_entry,
                                          nullptr,
                                          512,
                                          RT_THREAD_PRIORITY_MAX - 4,
                                          20);
    if (_storage_thread_ctx != nullptr) {
        rt_thread_startup(_storage_thread_ctx);
    }
#endif
}

void Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start) / 1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

void Scheduler::delay_microseconds(uint16_t us)
{
    if (in_main_thread() && us < 100) {
        // Busy wait for short delays in main thread
        uint64_t end = AP_HAL::micros64() + us;
        while (AP_HAL::micros64() < end) {
            // spin
        }
    } else {
        // rt_thread_delay tick; minimum delay is 1 tick (typically 1 ms)
        rt_tick_t tick = rt_tick_from_millisecond((us + 999) / 1000);
        if (tick > 0) {
            rt_thread_delay(tick);
        }
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    _timer_sem.take_blocking();
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            _timer_sem.give();
            return;
        }
    }
    if (_num_timer_procs < RTT_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    }
    _timer_sem.give();
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    _io_sem.take_blocking();
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            _io_sem.give();
            return;
        }
    }
    if (_num_io_procs < RTT_SCHEDULER_MAX_IO_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    }
    _io_sem.give();
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    (void)period_us;
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    (void)hold_in_bootloader;
    rt_thread_mdelay(100);
    // If RTT provides reboot: rt_hw_cpu_reset(); else infinite loop
    for (;;) {
        rt_thread_mdelay(1000);
    }
}

bool Scheduler::in_main_thread() const
{
    return _main_thread_id != nullptr && rt_thread_self() == _main_thread_id;
}

void Scheduler::set_system_initialized()
{
    if (_system_initialized) {
        AP_HAL::panic("PANIC: Scheduler::set_system_initialized called more than once");
    }
    _system_initialized = true;
}

bool Scheduler::is_system_initialized()
{
    return _system_initialized;
}

bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char* name, uint32_t stack_size,
                              priority_base base, int8_t priority)
{
    (void)base;
    (void)priority;
    // Stub: create RTT thread that runs proc. Requires copying MemberProc and passing to rt_thread_create.
    // TODO: implement with rt_thread_create(..., _thread_entry, proc, ...) and _thread_entry calls (*proc)() then deletes.
    return false;
}

void Scheduler::_run_timers()
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    uint8_t num_procs = 0;
    _timer_sem.take_blocking();
    num_procs = _num_timer_procs;
    _timer_sem.give();

    for (uint8_t i = 0; i < num_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_run_io()
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    uint8_t num_procs = 0;
    _io_sem.take_blocking();
    num_procs = _num_io_procs;
    _io_sem.give();

    for (uint8_t i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }

#if AP_RCPROTOCOL_ENABLED
    AP::RC().update();
#endif

    _in_io_proc = false;
}
