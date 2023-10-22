#include "AP_BalanceCAN.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include <stdio.h>

extern const AP_HAL::HAL &hal;


void AP_BalanceCAN::init(uint8_t driver_index, bool enable_filters) {
  _driver_index = driver_index;

  if (_initialized) {
    return;
  }

  // start calls to loop in separate thread
  if (!hal.scheduler->thread_create(
          FUNCTOR_BIND_MEMBER(&AP_BalanceCAN::update, void), _thread_name, 4096,
          AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
    return;
  }
  _initialized = true;

  snprintf(_thread_name, sizeof(_thread_name), "BalanceCAN_%u", driver_index);
}

bool AP_BalanceCAN::add_interface(AP_HAL::CANIface *can_iface) {
  if (_can_iface != nullptr) {
    return false;
  }

  _can_iface = can_iface;

  if (_can_iface == nullptr) {
    return false;
  }

  if (!_can_iface->is_initialized()) {
    return false;
  }

  return true;
}

AP_BalanceCAN *AP_BalanceCAN::get_balancecan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::BalanceCAN) {
        return nullptr;
    }

    return static_cast<AP_BalanceCAN*>(AP::can().get_driver(driver_index));
}

void AP_BalanceCAN::update() {
  AP_HAL::CANFrame txFrame{};
  AP_HAL::CANFrame rxFrame{};

  while (true) {
    if (!_initialized) {
      hal.scheduler->delay_microseconds(10000);
      continue;
    }

    send_current(target_current[0], target_current[1], target_current[2],
                 target_current[3]);

    hal.scheduler->delay_microseconds(2500);

    uint64_t timeout = AP_HAL::micros64() + 250ULL;
    // Look for any message responses on the CAN bus
    while (read_frame(rxFrame, timeout)) {

      switch (rxFrame.id) {
      case CAN_2006Moto1_ID:
      case CAN_2006Moto2_ID:
      case CAN_2006Moto3_ID:
      case CAN_2006Moto4_ID:
        uint8_t i = rxFrame.id - CAN_2006Moto1_ID;
        handle_moto_measure(rxFrame, i);
      }
    }
  }
}

// read frame on CAN bus, returns true on succses
bool AP_BalanceCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) {
  if (!_initialized) {
    return false;
  }
  bool read_select = true;
  bool write_select = false;
  bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);

  if (!ret || !read_select) {
    // No frame available
    return false;
  }

  uint64_t time;
  AP_HAL::CANIface::CanIOFlags flags{};

  return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// write frame on CAN bus, returns true on success
bool AP_BalanceCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) {
  if (!_initialized) {
    return false;
  }

  bool read_select = false;
  bool write_select = true;

  bool ret = _can_iface->select(read_select, write_select, &out_frame, timeout);

  if (!ret || !write_select) {
    return false;
  }

  return (_can_iface->send(out_frame, timeout,
                           AP_HAL::CANIface::AbortOnError) == 1);
}

// parse inbound frames
void AP_BalanceCAN::handle_moto_measure(AP_HAL::CANFrame &frame, uint8_t id) {
  real_speed[id] = (frame.data[2] << 8 | frame.data[3]);
}

bool AP_BalanceCAN::send_current(const int16_t d1, const int16_t d2,
                                 const int16_t d3, const int16_t d4) {
  AP_HAL::CANFrame frame =
      AP_HAL::CANFrame(0x200, send_current_buffer, 8, false);

  frame.data[0] = HIGHBYTE(d1);
  frame.data[1] = LOWBYTE(d1);
  frame.data[2] = HIGHBYTE(d2);
  frame.data[3] = LOWBYTE(d2);
  frame.data[4] = HIGHBYTE(d3);
  frame.data[5] = LOWBYTE(d3);
  frame.data[6] = HIGHBYTE(d4);
  frame.data[7] = LOWBYTE(d4);

  uint64_t timeout_us = 10000UL;

  return write_frame(frame, timeout_us);
}

AP_BalanceCAN *AP_BalanceCAN::_singleton;

namespace AP {
AP_BalanceCAN *balanceCAN() { return AP_BalanceCAN::get_singleton(); }
}; // namespace AP