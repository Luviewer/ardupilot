#include "AP_Hiwonder.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

void AP_Hiwonder::init(void)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
    if (port)
    {
        baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Hiwonder, 0);
    }
}

uint8_t AP_Hiwonder::serial_servo_checksum(const uint8_t buf[])
{
    uint16_t temp = 0;
    for (int i = 2; i < buf[3] + 2; ++i)
    {
        temp += buf[i];
    }
    return (uint8_t)(~temp);
}

void AP_Hiwonder::serial_servo_set_position(uint32_t servo_id, int position, uint32_t duration)
{
    Hiwonder_MOVE_WRITE_TypeDef frame;
    frame.header_1 = SERIAL_SERVO_FRAME_HEADER;
    frame.header_2 = SERIAL_SERVO_FRAME_HEADER;
    frame.servo_id = servo_id;
    frame.command = SERIAL_SERVO_MOVE_TIME_WRITE;

    position = position > 1000 ? 1000 : position;
    frame.args[0] = LOWBYTE(position);
    frame.args[1] = HIGHBYTE(position);
    frame.args[2] = LOWBYTE(duration);
    frame.args[3] = HIGHBYTE(duration);

    frame.length = 4 + 3;
    frame.crc = serial_servo_checksum((uint8_t *)&frame);

    port->write((uint8_t *)&frame, sizeof(frame));
}
