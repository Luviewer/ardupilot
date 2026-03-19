/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * micros64/millis 基于 rt_tick_get()；若无硬件微秒子计数则用 rt_tick_get()*1000 近似。
 * 工程需链接 librtthread。
 */

#include "AP_HAL_RTT/Util.h"
#include <AP_Common/ExpandingString.h>
#include <rtthread.h>

using namespace RTT;

uint32_t Util::available_memory(void)
{
    // Stub: 可后续用 RTT 的 rt_memory_info 或 heap 接口
    return 4096;
}

void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    (void)time_utc_usec;
}

uint64_t Util::get_hw_rtc() const
{
    return get_micros64();
}

uint32_t Util::get_millis() const
{
    return (uint32_t)rt_tick_get();
}

uint64_t Util::get_micros64() const
{
    // 无硬件子计数时用 tick*1000 近似（假设 1 tick = 1 ms）
    return (uint64_t)rt_tick_get() * 1000ULL;
}

void Util::thread_info(ExpandingString& str)
{
    str.printf("ThreadsV1\n");
    // Stub: 可后续用 rt_thread_list 等 RTT 接口
}
