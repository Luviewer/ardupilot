#include "AP_Hiwonder/AP_Hiwonder.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

#define AP_SERIALMANAGER_Hiwonder_BAUD       115200
#define AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX 256
#define AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX 256

uint8_t AP_Hiwonder::serial_servo_checksum(const uint8_t buf[])
{
    uint16_t temp = 0;
    for (int i = 2; i < buf[3] + 2; ++i) {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

bool AP_Hiwonder::is_ready_for_send() const
{
    return _port != NULL && _port->txspace() >= sizeof(Hiwonder_MOVE_WRITE_TypeDef);
}

void AP_Hiwonder::set_position(uint8_t servo_id, uint16_t position, uint16_t duration)
{
    if (_port == NULL) return;
    
    // 如果正在读取角度，禁止发送数据防止冲突
    // if (read_status.state != READ_IDLE) {
    //     return;
    // }
    
    if (_port->txspace() < sizeof(Hiwonder_MOVE_WRITE_TypeDef)) {
        return;
    }

    is_sending = true;

    Hiwonder_MOVE_WRITE_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command = SERIAL_SERVO_MOVE_TIME_WRITE;
    
    position = constrain_int32(position, 100, 900);
    frame.args[0] = LOWBYTE(position);
    frame.args[1] = HIGHBYTE(position);
    frame.args[2] = LOWBYTE(duration);
    frame.args[3] = HIGHBYTE(duration);
    
    frame.length = sizeof(frame.args) + 3;
    frame.crc = serial_servo_checksum((uint8_t*)&frame);
    
    _port->write((uint8_t*)&frame, sizeof(frame));
    
    is_sending = false;
}

void AP_Hiwonder::adjust_offset(uint32_t servo_id, int8_t adjust)
{
    if (_port == NULL) return;

    Hiwonder_OFFSET_ADJUST_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command = SERVO_ANGLE_OFFSET_ADJUST;
    
    adjust = constrain_int32(adjust, -125, 125);
    frame.args = (uint8_t)adjust;
    
    frame.length = sizeof(frame.args) + 3;
    frame.crc = serial_servo_checksum((uint8_t*)&frame);
    
    _port->write((uint8_t*)&frame, sizeof(frame));
}

void AP_Hiwonder::write_offset(uint32_t servo_id)
{
    if (_port == NULL) return;

    Hiwonder_WRITE_ADJUST_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command = SERVO_ANGLE_OFFSET_WRITE;
    
    frame.length = 3;
    frame.crc = serial_servo_checksum((uint8_t*)&frame);
    
    _port->write((uint8_t*)&frame, sizeof(frame));
}

void AP_Hiwonder::init(void)
{
    _port = hal.serial(serial_num);
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _port->begin(AP_SERIALMANAGER_Hiwonder_BAUD,
                 AP_SERIALMANAGER_Hiwonder_BUFSIZE_RX,
                 AP_SERIALMANAGER_Hiwonder_BUFSIZE_TX);
}

// bool AP_Hiwonder::start_position_read(uint8_t servo_id)
// {
//     if (_port == NULL || read_status.state != READ_IDLE || is_sending) {
//         return false;
//     }
    
//     if (_port->txspace() < sizeof(Hiwonder_SERVO_POS_READ_CMD_TypeDef)) {
//         return false;
//     }
    
//     Hiwonder_SERVO_POS_READ_CMD_TypeDef frame;
//     frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
//     frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
//     frame.servo_id = servo_id;
//     frame.command = SERVO_POS_READ;
//     frame.length = 3;
//     frame.crc = serial_servo_checksum((uint8_t*)&frame);
    
//     _port->write((uint8_t*)&frame, sizeof(frame));
    
//     read_status.state = READ_WAITING;
//     read_status.servo_id = servo_id;
//     read_status.start_time = AP_HAL::millis();
    
//     return true;
// }

// bool AP_Hiwonder::update_position_read(uint16_t& position)
// {
//     if (_port == NULL || read_status.state != READ_WAITING) {
//         return false;
//     }
    
//     if (AP_HAL::millis() - read_status.start_time > 100) {
//         read_status.state = READ_FAILED;
//         return false;
//     }
    
//     const uint8_t expected_size = sizeof(Hiwonder_SERVO_POS_READ_Response_TypeDef);
//     if (_port->available() < expected_size) {
//         return false;
//     }
    
//     uint8_t buffer[expected_size];
//     if (_port->read(buffer, expected_size) != expected_size) {
//         read_status.state = READ_FAILED;
//         return false;
//     }
    
//     Hiwonder_SERVO_POS_READ_Response_TypeDef* response = (Hiwonder_SERVO_POS_READ_Response_TypeDef*)buffer;
//     if (response->header_1 != SERIAL_SERVO_FRAME_HEADER || 
//         response->header_2 != SERIAL_SERVO_FRAME_HEADER || 
//         response->servo_id != read_status.servo_id || 
//         response->crc != serial_servo_checksum(buffer)) {
//         read_status.state = READ_FAILED;
//         return false;
//     }
    
//     position = (response->args[1] << 8) | response->args[0];
//     read_status.state = READ_COMPLETE;
//     return true;
// }

// bool AP_Hiwonder::is_read_complete() const
// {
//     return (read_status.state == READ_COMPLETE || read_status.state == READ_FAILED);
// }

// void AP_Hiwonder::reset_read_state()
// {
//     read_status.state = READ_IDLE;
//     read_status.servo_id = 0;
//     read_status.start_time = 0;
// }