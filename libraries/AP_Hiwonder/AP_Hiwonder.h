#pragma once

#include <AP_HAL/AP_HAL.h>

// 幻尔串行总线舵机的非阻塞协议驱动。
// 每个实例独占一个半双工UART；上层负责调度多个舵机ID并保存遥测结果。
class AP_Hiwonder
{
public:
    static constexpr uint8_t FRAME_HEADER = 0x55;
    static constexpr uint8_t BROADCAST_ID = 0xFE;
    static constexpr uint16_t POSITION_MIN = 0;
    static constexpr uint16_t POSITION_MAX = 1000;
    static constexpr uint16_t DURATION_MAX_MS = 30000;

    enum Command : uint8_t {
        MOVE_TIME_WRITE = 1,
        MOVE_TIME_READ = 2,
        MOVE_TIME_WAIT_WRITE = 7,
        MOVE_TIME_WAIT_READ = 8,
        MOVE_START = 11,
        MOVE_STOP = 12,
        ID_WRITE = 13,
        ID_READ = 14,
        ANGLE_OFFSET_ADJUST = 17,
        ANGLE_OFFSET_WRITE = 18,
        ANGLE_OFFSET_READ = 19,
        ANGLE_LIMIT_WRITE = 20,
        ANGLE_LIMIT_READ = 21,
        VIN_LIMIT_WRITE = 22,
        VIN_LIMIT_READ = 23,
        TEMP_MAX_LIMIT_WRITE = 24,
        TEMP_MAX_LIMIT_READ = 25,
        TEMP_READ = 26,
        VIN_READ = 27,
        POS_READ = 28,
        OR_MOTOR_MODE_WRITE = 29,
        OR_MOTOR_MODE_READ = 30,
        LOAD_OR_UNLOAD_WRITE = 31,
        LOAD_OR_UNLOAD_READ = 32,
        LED_CTRL_WRITE = 33,
        LED_CTRL_READ = 34,
        LED_ERROR_WRITE = 35,
        LED_ERROR_READ = 36,
        DISTANCE_READ = 48,
    };

    struct Response {
        uint8_t servo_id {};
        Command command { POS_READ };
        uint8_t param_count {};
        uint8_t params[4] {};
    };

    explicit AP_Hiwonder(uint8_t serial_num) : _serial_num(serial_num) {}

    // 每个枚举值代表一条腿的独立UART总线；顺序必须与DroneCAN和PWR原理图一致。
    enum {
        BUS_RF = 0,
        BUS_RB,
        BUS_LB,
        BUS_LF,
        BUS_RM,
        BUS_LM,
        BUS_COUNT,
    };

    void init();

    bool set_position(uint8_t servo_id, uint16_t position, uint16_t duration_ms);
    bool set_position_wait(uint8_t servo_id, uint16_t position, uint16_t duration_ms);
    bool move_start(uint8_t servo_id = BROADCAST_ID);
    bool move_stop(uint8_t servo_id = BROADCAST_ID);
    bool set_id(uint8_t servo_id, uint8_t new_id);
    bool adjust_offset(uint8_t servo_id, int8_t adjust);
    bool write_offset(uint8_t servo_id);
    bool set_angle_limits(uint8_t servo_id, uint16_t minimum, uint16_t maximum);
    bool set_voltage_limits(uint8_t servo_id, uint16_t minimum_mv, uint16_t maximum_mv);
    bool set_temperature_limit(uint8_t servo_id, uint8_t maximum_celsius);
    bool set_motor_mode(uint8_t servo_id, bool motor_mode, bool fixed_speed, int16_t speed);
    bool set_load_enabled(uint8_t servo_id, bool enabled);
    bool set_led_enabled(uint8_t servo_id, bool enabled);
    bool set_led_error_mask(uint8_t servo_id, uint8_t error_mask);

    // 通用非阻塞读取接口。start_read() 发送请求，update_read() 应在主循环中轮询。
    bool start_read(uint8_t servo_id, Command command);
    bool update_read(Response &response);
    bool start_position_read(uint8_t servo_id);
    bool update_position_read(int16_t &position);
    bool start_temperature_read(uint8_t servo_id);
    bool update_temperature_read(uint8_t &temperature_c);
    bool start_voltage_read(uint8_t servo_id);
    bool update_voltage_read(uint16_t &voltage_mv);
    bool start_distance_read(uint8_t servo_id);
    bool update_distance_read(int32_t &distance_pulse);
    bool read_finished() const;
    void reset_read_state();

private:
    static constexpr uint8_t MAX_PARAMS = 4;
    static constexpr uint8_t MAX_PACKET_LENGTH = 6 + MAX_PARAMS;
    // 115200 baud下应答帧传输不足1 ms；短超时可避免离线舵机长期占用本路UART。
    static constexpr uint32_t READ_TIMEOUT_MS = 20;

    enum ReadState : uint8_t {
        READ_IDLE,
        READ_WAITING,
        READ_COMPLETE,
        READ_FAILED,
    };

    struct ReadStatus {
        ReadState state = READ_IDLE;
        uint8_t servo_id = 0;
        Command command = POS_READ;
        uint8_t expected_params = 0;
        uint32_t start_time_ms = 0;
    } read_status;

    bool send_packet(uint8_t servo_id, Command command, const uint8_t *params, uint8_t param_count);
    bool is_ready_for_send(uint8_t param_count) const;
    bool parse_byte(uint8_t byte, Response &response);
    static uint8_t checksum(const uint8_t *packet, uint8_t total_length);
    static uint8_t expected_response_params(Command command);
    static bool valid_servo_id(uint8_t servo_id);
    static bool broadcast_read_allowed(Command command);
    static void put_u16(uint8_t *dst, uint16_t value);
    static uint16_t get_u16(const uint8_t *src);
    static uint32_t get_u32(const uint8_t *src);

    uint8_t _serial_num;
    AP_HAL::UARTDriver *_port = nullptr;
    uint8_t rx_buffer[MAX_PACKET_LENGTH] {};
    uint8_t rx_length = 0;
};
