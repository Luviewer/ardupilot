#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#define SERIAL_SERVO_FRAME_HEADER 0x55
#define SERIAL_SERVO_MOVE_TIME_WRITE 1

#pragma pack(1)
typedef struct
{
    uint8_t header_1;
    uint8_t header_2;
    uint8_t servo_id;
    uint8_t length;
    uint8_t command;
    uint8_t args[4];
    uint8_t crc;
} Hiwonder_MOVE_WRITE_TypeDef;
#pragma pack()

typedef enum
{
    SERIAL_SERVO_RECV_STARTBYTE_1,
    SERIAL_SERVO_RECV_STARTBYTE_2,
    SERIAL_SERVO_RECV_SERVO_ID,
    SERIAL_SERVO_RECV_LENGTH,
    SERIAL_SERVO_RECV_COMMAND,
    SERIAL_SERVO_RECV_ARGUMENTS,
    SERIAL_SERVO_RECV_CHECKSUM,
} SerialServoRecvState;

class AP_Hiwonder
{
public:
    AP_Hiwonder();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Hiwonder);

    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_HAL::UARTDriver *port;
    uint32_t baudrate;

    void init(void);

    uint8_t serial_servo_checksum(const uint8_t buf[]);

    void serial_servo_set_position(uint32_t servo_id, int position, uint32_t duration);
};
