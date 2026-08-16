#include "AP_Hiwonder.h"

#include <AP_Math/AP_Math.h>

static constexpr uint32_t HIWONDER_BAUD = 115200;
static constexpr uint16_t HIWONDER_RX_BUFFER_SIZE = 256;
static constexpr uint16_t HIWONDER_TX_BUFFER_SIZE = 256;

extern const AP_HAL::HAL &hal;

bool AP_Hiwonder::valid_servo_id(uint8_t servo_id)
{
    return servo_id <= BROADCAST_ID;
}

bool AP_Hiwonder::broadcast_read_allowed(Command command)
{
    // 协议只允许用广播地址读取ID，并要求总线上仅连接一只舵机。
    return command == ID_READ;
}

void AP_Hiwonder::put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = uint8_t(value);
    dst[1] = uint8_t(value >> 8);
}

uint16_t AP_Hiwonder::get_u16(const uint8_t *src)
{
    return uint16_t(src[0]) | (uint16_t(src[1]) << 8);
}

uint32_t AP_Hiwonder::get_u32(const uint8_t *src)
{
    return uint32_t(src[0]) |
           (uint32_t(src[1]) << 8) |
           (uint32_t(src[2]) << 16) |
           (uint32_t(src[3]) << 24);
}

uint8_t AP_Hiwonder::checksum(const uint8_t *packet, uint8_t total_length)
{
    if (packet == nullptr || total_length < 6) {
        return 0;
    }

    uint8_t sum = 0;
    for (uint8_t i = 2; i < total_length - 1; i++) {
        sum += packet[i];
    }
    return uint8_t(~sum);
}

bool AP_Hiwonder::is_ready_for_send(uint8_t param_count) const
{
    return _port != nullptr &&
           param_count <= MAX_PARAMS &&
           read_status.state != READ_WAITING &&
           _port->txspace() >= uint32_t(6U + param_count);
}

bool AP_Hiwonder::send_packet(uint8_t servo_id, Command command, const uint8_t *params, uint8_t param_count)
{
    if (!valid_servo_id(servo_id) || !is_ready_for_send(param_count) ||
        (param_count > 0 && params == nullptr)) {
        return false;
    }

    const uint8_t total_length = 6U + param_count;
    uint8_t packet[MAX_PACKET_LENGTH] {};
    packet[0] = FRAME_HEADER;
    packet[1] = FRAME_HEADER;
    packet[2] = servo_id;
    packet[3] = param_count + 3U;
    packet[4] = uint8_t(command);
    if (param_count != 0) {
        memcpy(&packet[5], params, param_count);
    }
    packet[total_length - 1] = checksum(packet, total_length);

    return _port->write(packet, total_length) == total_length;
}

bool AP_Hiwonder::set_position(uint8_t servo_id, uint16_t position, uint16_t duration_ms)
{
    uint8_t params[4];
    put_u16(&params[0], constrain_uint16(position, POSITION_MIN, POSITION_MAX));
    put_u16(&params[2], constrain_uint16(duration_ms, 0, DURATION_MAX_MS));
    return send_packet(servo_id, MOVE_TIME_WRITE, params, sizeof(params));
}

bool AP_Hiwonder::set_position_wait(uint8_t servo_id, uint16_t position, uint16_t duration_ms)
{
    uint8_t params[4];
    put_u16(&params[0], constrain_uint16(position, POSITION_MIN, POSITION_MAX));
    put_u16(&params[2], constrain_uint16(duration_ms, 0, DURATION_MAX_MS));
    return send_packet(servo_id, MOVE_TIME_WAIT_WRITE, params, sizeof(params));
}

bool AP_Hiwonder::move_start(uint8_t servo_id)
{
    return send_packet(servo_id, MOVE_START, nullptr, 0);
}

bool AP_Hiwonder::move_stop(uint8_t servo_id)
{
    return send_packet(servo_id, MOVE_STOP, nullptr, 0);
}

bool AP_Hiwonder::set_id(uint8_t servo_id, uint8_t new_id)
{
    if (new_id > 253) {
        return false;
    }
    return send_packet(servo_id, ID_WRITE, &new_id, 1);
}

bool AP_Hiwonder::adjust_offset(uint8_t servo_id, int8_t adjust)
{
    const uint8_t value = uint8_t(constrain_int16(adjust, -125, 125));
    return send_packet(servo_id, ANGLE_OFFSET_ADJUST, &value, 1);
}

bool AP_Hiwonder::write_offset(uint8_t servo_id)
{
    return send_packet(servo_id, ANGLE_OFFSET_WRITE, nullptr, 0);
}

bool AP_Hiwonder::set_angle_limits(uint8_t servo_id, uint16_t minimum, uint16_t maximum)
{
    minimum = constrain_uint16(minimum, POSITION_MIN, POSITION_MAX);
    maximum = constrain_uint16(maximum, POSITION_MIN, POSITION_MAX);
    if (minimum >= maximum) {
        return false;
    }
    uint8_t params[4];
    put_u16(&params[0], minimum);
    put_u16(&params[2], maximum);
    return send_packet(servo_id, ANGLE_LIMIT_WRITE, params, sizeof(params));
}

bool AP_Hiwonder::set_voltage_limits(uint8_t servo_id, uint16_t minimum_mv, uint16_t maximum_mv)
{
    minimum_mv = constrain_uint16(minimum_mv, 4500, 14000);
    maximum_mv = constrain_uint16(maximum_mv, 4500, 14000);
    if (minimum_mv >= maximum_mv) {
        return false;
    }
    uint8_t params[4];
    put_u16(&params[0], minimum_mv);
    put_u16(&params[2], maximum_mv);
    return send_packet(servo_id, VIN_LIMIT_WRITE, params, sizeof(params));
}

bool AP_Hiwonder::set_temperature_limit(uint8_t servo_id, uint8_t maximum_celsius)
{
    maximum_celsius = constrain_uint16(maximum_celsius, 50, 100);
    return send_packet(servo_id, TEMP_MAX_LIMIT_WRITE, &maximum_celsius, 1);
}

bool AP_Hiwonder::set_motor_mode(uint8_t servo_id, bool motor_mode, bool fixed_speed, int16_t speed)
{
    const int16_t speed_limit = fixed_speed ? 50 : 1000;
    speed = constrain_int16(speed, -speed_limit, speed_limit);
    uint8_t params[4] {
        uint8_t(motor_mode),
        uint8_t(fixed_speed),
        uint8_t(speed),
        uint8_t(uint16_t(speed) >> 8),
    };
    return send_packet(servo_id, OR_MOTOR_MODE_WRITE, params, sizeof(params));
}

bool AP_Hiwonder::set_load_enabled(uint8_t servo_id, bool enabled)
{
    const uint8_t value = uint8_t(enabled);
    return send_packet(servo_id, LOAD_OR_UNLOAD_WRITE, &value, 1);
}

bool AP_Hiwonder::set_led_enabled(uint8_t servo_id, bool enabled)
{
    // 协议定义0为常亮、1为常灭，与布尔enabled语义相反。
    const uint8_t value = enabled ? 0 : 1;
    return send_packet(servo_id, LED_CTRL_WRITE, &value, 1);
}

bool AP_Hiwonder::set_led_error_mask(uint8_t servo_id, uint8_t error_mask)
{
    error_mask &= 0x07;
    return send_packet(servo_id, LED_ERROR_WRITE, &error_mask, 1);
}

uint8_t AP_Hiwonder::expected_response_params(Command command)
{
    switch (command) {
    case MOVE_TIME_READ:
    case MOVE_TIME_WAIT_READ:
    case ANGLE_LIMIT_READ:
    case VIN_LIMIT_READ:
    case OR_MOTOR_MODE_READ:
    case DISTANCE_READ:
        return 4;
    case ID_READ:
    case ANGLE_OFFSET_READ:
    case TEMP_MAX_LIMIT_READ:
    case TEMP_READ:
    case LOAD_OR_UNLOAD_READ:
    case LED_CTRL_READ:
    case LED_ERROR_READ:
        return 1;
    case VIN_READ:
    case POS_READ:
        return 2;
    default:
        return 0;
    }
}

bool AP_Hiwonder::start_read(uint8_t servo_id, Command command)
{
    const uint8_t expected_params = expected_response_params(command);
    if (_port == nullptr || !valid_servo_id(servo_id) || expected_params == 0 ||
        (servo_id == BROADCAST_ID && !broadcast_read_allowed(command)) ||
        read_status.state == READ_WAITING) {
        return false;
    }

    reset_read_state();
    _port->discard_input();
    if (!send_packet(servo_id, command, nullptr, 0)) {
        return false;
    }

    read_status.state = READ_WAITING;
    read_status.servo_id = servo_id;
    read_status.command = command;
    read_status.expected_params = expected_params;
    read_status.start_time_ms = AP_HAL::millis();
    return true;
}

bool AP_Hiwonder::parse_byte(uint8_t byte, Response &response)
{
    if (rx_length == 0) {
        if (byte == FRAME_HEADER) {
            rx_buffer[rx_length++] = byte;
        }
        return false;
    }

    if (rx_length == 1) {
        if (byte == FRAME_HEADER) {
            rx_buffer[rx_length++] = byte;
        } else {
            rx_length = 0;
        }
        return false;
    }

    if (rx_length >= MAX_PACKET_LENGTH) {
        rx_length = 0;
        return false;
    }
    rx_buffer[rx_length++] = byte;

    if (rx_length == 4 && (rx_buffer[3] < 3 || rx_buffer[3] > 7)) {
        rx_length = byte == FRAME_HEADER ? 1 : 0;
        if (rx_length == 1) {
            rx_buffer[0] = FRAME_HEADER;
        }
        return false;
    }
    if (rx_length < 4) {
        return false;
    }

    const uint8_t total_length = rx_buffer[3] + 3U;
    if (rx_length < total_length) {
        return false;
    }

    const bool valid = rx_buffer[total_length - 1] == checksum(rx_buffer, total_length);
    if (valid) {
        response.servo_id = rx_buffer[2];
        response.command = Command(rx_buffer[4]);
        response.param_count = rx_buffer[3] - 3U;
        memset(response.params, 0, sizeof(response.params));
        memcpy(response.params, &rx_buffer[5], response.param_count);
    }
    rx_length = 0;
    return valid;
}

bool AP_Hiwonder::update_read(Response &response)
{
    if (_port == nullptr || read_status.state != READ_WAITING) {
        return false;
    }

    // 先处理已进入UART缓冲区的数据，再判断超时，避免在边界时刻丢弃完整应答。
    while (_port->available() > 0) {
        const int16_t value = _port->read();
        if (value < 0) {
            break;
        }
        Response candidate {};
        if (!parse_byte(uint8_t(value), candidate)) {
            continue;
        }

        const bool id_matches = read_status.servo_id == BROADCAST_ID ||
                                candidate.servo_id == read_status.servo_id;
        if (id_matches && candidate.command == read_status.command &&
            candidate.param_count == read_status.expected_params) {
            response = candidate;
            read_status.state = READ_COMPLETE;
            return true;
        }
        // PWR半双工电路可能回显发送帧；同时忽略UART中与本次事务无关的合法帧。
    }

    if (AP_HAL::millis() - read_status.start_time_ms >= READ_TIMEOUT_MS) {
        read_status.state = READ_FAILED;
    }
    return false;
}

bool AP_Hiwonder::start_position_read(uint8_t servo_id)
{
    return start_read(servo_id, POS_READ);
}

bool AP_Hiwonder::update_position_read(int16_t &position)
{
    Response response {};
    if (!update_read(response)) {
        return false;
    }
    position = int16_t(get_u16(response.params));
    return true;
}

bool AP_Hiwonder::start_temperature_read(uint8_t servo_id)
{
    return start_read(servo_id, TEMP_READ);
}

bool AP_Hiwonder::update_temperature_read(uint8_t &temperature_c)
{
    Response response {};
    if (!update_read(response)) {
        return false;
    }
    temperature_c = response.params[0];
    return true;
}

bool AP_Hiwonder::start_voltage_read(uint8_t servo_id)
{
    return start_read(servo_id, VIN_READ);
}

bool AP_Hiwonder::update_voltage_read(uint16_t &voltage_mv)
{
    Response response {};
    if (!update_read(response)) {
        return false;
    }
    voltage_mv = get_u16(response.params);
    return true;
}

bool AP_Hiwonder::start_distance_read(uint8_t servo_id)
{
    return start_read(servo_id, DISTANCE_READ);
}

bool AP_Hiwonder::update_distance_read(int32_t &distance_pulse)
{
    Response response {};
    if (!update_read(response)) {
        return false;
    }
    distance_pulse = int32_t(get_u32(response.params));
    return true;
}

bool AP_Hiwonder::read_finished() const
{
    return read_status.state == READ_COMPLETE || read_status.state == READ_FAILED;
}

void AP_Hiwonder::reset_read_state()
{
    read_status = {};
    rx_length = 0;
}

void AP_Hiwonder::init()
{
    _port = hal.serial(_serial_num);
    if (_port == nullptr) {
        return;
    }
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _port->begin(HIWONDER_BAUD, HIWONDER_RX_BUFFER_SIZE, HIWONDER_TX_BUFFER_SIZE);
    _port->discard_input();
}
