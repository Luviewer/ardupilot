#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

#define SERIAL_SERVO_FRAME_HEADER    0x55
#define SERIAL_SERVO_MOVE_TIME_WRITE 1
#define SERVO_ANGLE_OFFSET_ADJUST    17
#define SERVO_ANGLE_OFFSET_WRITE     18

enum {
    SERVO_1 = 1,
    SERVO_2 = 2,
    SERVO_3 = 3,
    SERVO_4 = 4,
};

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

typedef struct
{
    uint8_t header_1;
    uint8_t header_2;
    uint8_t servo_id;
    uint8_t length;
    uint8_t command;
    uint8_t args;
    uint8_t crc;
} Hiwonder_OFFSET_ADJUST_TypeDef;

typedef struct
{
    uint8_t header_1;
    uint8_t header_2;
    uint8_t servo_id;
    uint8_t length;
    uint8_t command;
    uint8_t crc;
} Hiwonder_WRITE_ADJUST_TypeDef;
#pragma pack()

class AP_Hiwonder {
public:
    AP_Hiwonder();

    virtual void init(void) = 0;
    void         set_position(uint32_t servo_id, int position, uint32_t duration);
    void         adjust_offset(uint32_t servo_id, int8_t adjust);
    void         write_offset(uint32_t servo_id);
    uint8_t      serial_servo_checksum(const uint8_t buf[]);

    AP_HAL::UARTDriver* _port;
};

class AP_Hiwonder_L : public AP_Hiwonder {
public:
    AP_Hiwonder_L();

    void init(void) override;

    // get singleton instance
    static AP_Hiwonder_L* get_singleton()
    {
        return _singleton;
    }

private:
    static AP_Hiwonder_L* _singleton;
};

class AP_Hiwonder_R : public AP_Hiwonder {
public:
    AP_Hiwonder_R();

    void init(void) override;

    // get singleton instance
    static AP_Hiwonder_R* get_singleton()
    {
        return _singleton;
    }

private:
    static AP_Hiwonder_R* _singleton;
};

namespace AP {
AP_Hiwonder_L& hiwonder_l();
AP_Hiwonder_R& hiwonder_r();

};