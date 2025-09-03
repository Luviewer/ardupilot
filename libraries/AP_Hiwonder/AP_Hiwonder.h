#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

#define SERIAL_SERVO_FRAME_HEADER    0x55
#define SERIAL_SERVO_MOVE_TIME_WRITE 1
#define SERVO_ANGLE_OFFSET_ADJUST    17
#define SERVO_ANGLE_OFFSET_WRITE     18
#define SERVO_POS_READ               20

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

typedef struct
{
    uint8_t header_1;
    uint8_t header_2;
    uint8_t servo_id;
    uint8_t length;
    uint8_t command;
    uint8_t crc;
} Hiwonder_SERVO_POS_READ_CMD_TypeDef;

typedef struct
{
    uint8_t header_1;
    uint8_t header_2;
    uint8_t servo_id;
    uint8_t length;
    uint8_t command;
    uint8_t args[2];
    uint8_t crc;
} Hiwonder_SERVO_POS_READ_Response_TypeDef;

#pragma pack()

class AP_Hiwonder {
public:
    AP_Hiwonder(int8_t _num)
        : serial_num(_num)
    {
        _port                  = NULL;
        read_status.state      = READ_IDLE;
        read_status.servo_id   = 0;
        read_status.start_time = 0;
        is_sending             = false;

        // AP_Param::setup_object_defaults(this, var_info);
    }

    enum {
        SERVO_RF = 0,
        SERVO_RB = 1,
        SERVO_LB = 2,
        SERVO_LF = 3,
        SERVO_Total,
    };

    void init(void);

    void set_position(uint8_t servo_id, uint16_t position, uint16_t duration);

    void    adjust_offset(uint32_t servo_id, int8_t adjust);
    void    write_offset(uint32_t servo_id);
    uint8_t serial_servo_checksum(const uint8_t buf[]);

    // 检查串口是否准备好
    bool is_ready_for_send() const;

    // 舵机角度读取方法（非阻塞）
    bool start_position_read(uint8_t servo_id);
    bool update_position_read(uint16_t& position);
    bool is_read_complete() const;
    void reset_read_state();

    // static const struct AP_Param::GroupInfo var_info[];

private:
    int8_t serial_num;

    AP_HAL::UARTDriver* _port;

    // 读取状态管理
    enum ReadState {
        READ_IDLE,
        READ_WAITING,
        READ_COMPLETE,
        READ_FAILED
    };

    struct ReadStatus {
        ReadState state;
        uint8_t   servo_id;
        uint32_t  start_time;
    } read_status;

    // 发送状态管理
    bool is_sending;
};
