/*
 * ArduPilot + RT-Thread HAL - Storage (FRAM via AP_RAMTRON)
 */

#include "Storage.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <cstring>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace RTT
{

void Storage::_storage_open(void)
{
    if (_initialisedType != StorageBackend::None) {
        return;
    }

    _dirty_mask.clearall();

#if HAL_WITH_RAMTRON
    if (_fram.init() && _fram.read(0, _buffer, RTT_STORAGE_SIZE)) {
        _initialisedType = StorageBackend::FRAM;
        ::printf("Initialised Storage type=FRAM\n");
        return;
    }
    AP_HAL::panic("Unable to init RAMTRON storage");
#else
    // No FRAM: use stub (params in RAM only, lost on reboot)
    memset(_buffer, 0xff, RTT_STORAGE_SIZE);
    _initialisedType = StorageBackend::Stub;
    ::printf("Initialised Storage type=Stub (non-persistent)\n");
#endif
}

void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    if (length == 0) return;
    uint16_t end = loc + length - 1;
    for (uint16_t line = loc >> RTT_STORAGE_LINE_SHIFT;
         line <= (end >> RTT_STORAGE_LINE_SHIFT);
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t src, size_t n)
{
    if (dst == nullptr || n > RTT_STORAGE_SIZE || src > (RTT_STORAGE_SIZE - n)) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[src], n);
}

void Storage::write_block(uint16_t dst, const void* src, size_t n)
{
    if (src == nullptr || n > RTT_STORAGE_SIZE || dst > (RTT_STORAGE_SIZE - n)) {
        return;
    }
    if (memcmp(src, &_buffer[dst], n) != 0) {
        _storage_open();
        if (_initialisedType == StorageBackend::Stub) {
            memcpy(&_buffer[dst], src, n);
            return;
        }
        _sem.take_blocking();
        memcpy(&_buffer[dst], src, n);
        _mark_dirty(dst, n);
        _sem.give();
    }
}

void Storage::_timer_tick(void)
{
    if (_initialisedType == StorageBackend::None || _initialisedType == StorageBackend::Stub) {
        return;
    }
    if (_dirty_mask.empty()) {
        return;
    }

    uint16_t i;
    for (i = 0; i < RTT_STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == RTT_STORAGE_NUM_LINES) {
        return;
    }

    _sem.take_blocking();
    memcpy(_tmpline, &_buffer[RTT_STORAGE_LINE_SIZE * i], RTT_STORAGE_LINE_SIZE);
    _sem.give();

    bool write_ok = false;

#if HAL_WITH_RAMTRON
    if (_initialisedType == StorageBackend::FRAM) {
        if (_fram.write(RTT_STORAGE_LINE_SIZE * i, _tmpline, RTT_STORAGE_LINE_SIZE)) {
            write_ok = true;
        }
    }
#endif

    if (write_ok) {
        _sem.take_blocking();
        if (memcmp(_tmpline, &_buffer[RTT_STORAGE_LINE_SIZE * i], RTT_STORAGE_LINE_SIZE) == 0) {
            _dirty_mask.clear(i);
        }
        _sem.give();
    }
}

} // namespace RTT
