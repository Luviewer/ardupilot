#include "AP_Hiwonder.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

#define AP_SERIALMANAGER_Hiwonder_BAUD       115200
#define AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX 64
#define AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX 64

int8_t SERVO_DIR[4] = { -1, 1, -1, 1 };

AP_Hiwonder::AP_Hiwonder()
{
    if (_singleton != nullptr) {
        // it's an error to get here.  But I don't want to include
        // AP_HAL here
        return;
    }
    _singleton = this;
    _port      = NULL;
}

void AP_Hiwonder::init(void)
{
    AP_SerialManager& serial_manager = AP::serialmanager();
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hiwonder, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                     AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
    }
}

uint8_t AP_Hiwonder::serial_servo_checksum(const uint8_t buf[])
{
    uint16_t temp = 0;
    for (int i = 2; i < buf[3] + 2; ++i) {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

void AP_Hiwonder::set_position(uint32_t servo_id, int position, uint32_t duration)
{
    if (_port == NULL)
        return;

    Hiwonder_MOVE_WRITE_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command  = SERIAL_SERVO_MOVE_TIME_WRITE;

    position = constrain_int32(position, 1000, 2000);

    if (SERVO_DIR[servo_id - 1] == 1) {
        position = 500 + (position - 1500);
    } else if (SERVO_DIR[servo_id - 1] == -1) {
        position = 500 - (position - 1500);
    }

    frame.args[0] = LOWBYTE(position);
    frame.args[1] = HIGHBYTE(position);
    frame.args[2] = LOWBYTE(duration);
    frame.args[3] = HIGHBYTE(duration);

    frame.length = sizeof(frame.args) + 3;
    frame.crc    = serial_servo_checksum((uint8_t*)&frame);

    _port->write((uint8_t*)&frame, sizeof(frame));
}

void AP_Hiwonder::adjust_offset(uint32_t servo_id, int8_t adjust)
{
    if (_port == NULL)
        return;

    Hiwonder_OFFSET_ADJUST_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command  = SERVO_ANGLE_OFFSET_ADJUST;

    adjust     = constrain_int32(adjust, -125, 125);
    frame.args = (uint8_t)adjust;

    frame.length = sizeof(frame.args) + 3;
    frame.crc    = serial_servo_checksum((uint8_t*)&frame);

    _port->write((uint8_t*)&frame, sizeof(frame));
}

void AP_Hiwonder::write_offset(uint32_t servo_id)
{
    if (_port == NULL)
        return;

    Hiwonder_WRITE_ADJUST_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command  = SERVO_ANGLE_OFFSET_WRITE;

    frame.length = 3;
    frame.crc    = serial_servo_checksum((uint8_t*)&frame);

    _port->write((uint8_t*)&frame, sizeof(frame));
}

// singleton instance
AP_Hiwonder* AP_Hiwonder::_singleton;

namespace AP {
AP_Hiwonder& hiwonder()
{
    return *AP_Hiwonder::get_singleton();
}

}