#include "AP_ADM002.h"

#if AP_CONTACT_SENSOR_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <string.h>

bool AP_ADM002::init(const AP_SerialManager &serial_manager)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ADM002, 0);
    if (_uart == nullptr) {
        return false;
    }

    _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_ADM002, 0));
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->discard_input();
    reset_rx();
    send_enable_stream();
    return true;
}

void AP_ADM002::update()
{
    if (_uart == nullptr) {
        return;
    }

    read_from_uart();
    const uint32_t now_ms = AP_HAL::millis();
    if (_config_state == ConfigState::PENDING && now_ms - _config_start_ms >= CONFIG_TIMEOUT_MS) {
        _config_state = ConfigState::FAILED;
        _config_sequence++;
    }
    if (_last_update_ms == 0 && now_ms - _last_enable_ms >= ENABLE_RETRY_MS) {
        send_enable_stream();
    }
}

bool AP_ADM002::start_config_command(ConfigCommand command, uint16_t value_kg)
{
    if (_uart == nullptr || _config_state == ConfigState::PENDING || command == ConfigCommand::NONE) {
        return false;
    }
    if ((command == ConfigCommand::FULL_SCALE || command == ConfigCommand::CALIBRATE_POINT) && value_kg == 0) {
        return false;
    }

    _config_command = command;
    _config_value_kg = value_kg;
    _config_state = ConfigState::PENDING;
    _config_start_ms = AP_HAL::millis();
    reset_rx();
    send_config_command();
    return true;
}

bool AP_ADM002::get_force_sample(ForceSample &sample) const
{
    if (_last_update_ms == 0) {
        return false;
    }

    const int32_t corrected_weight_g = _weight_g - _tare_offset_g;
    sample.tool_force_n = float(corrected_weight_g) * GRAVITY_MSS * 0.001f;
    sample.raw_value = corrected_weight_g;
    sample.timestamp_ms = _last_update_ms;
    sample.sequence = _sample_sequence;
    return true;
}

bool AP_ADM002::healthy() const
{
    const bool status_healthy = (_status & (STATUS_OVERLOAD | STATUS_AD_FAULT)) == 0;
    return _last_update_ms != 0 &&
           AP_HAL::millis() - _last_update_ms < HEALTH_TIMEOUT_MS &&
           status_healthy;
}

bool AP_ADM002::tare()
{
    if (_last_update_ms == 0) {
        return false;
    }

    _tare_offset_g = _weight_g;
    _last_tare_ms = AP_HAL::millis();
    return true;
}

// Enable the unsolicited stream with the ADM002 additive checksum protocol.
void AP_ADM002::send_enable_stream()
{
    uint8_t frame[] {
        DEVICE_ADDRESS,
        ENABLE_STREAM_FUNCTION,
        WRITE_OPERATION,
        ENABLE_STREAM,
        0,
    };
    frame[ARRAY_SIZE(frame) - 1] = checksum(frame, ARRAY_SIZE(frame) - 1);
    _uart->write(frame, ARRAY_SIZE(frame));
    _last_enable_ms = AP_HAL::millis();
}

void AP_ADM002::send_config_command()
{
    uint8_t frame[6] { DEVICE_ADDRESS, 0, WRITE_OPERATION, 0, 0, 0 };
    uint8_t frame_len = 5;

    switch (_config_command) {
    case ConfigCommand::ZERO:
        frame[1] = 0x04;
        frame[3] = 0;
        _config_ack_function = 0x05;
        break;
    case ConfigCommand::DEFAULT_ZERO:
        frame[1] = 0x04;
        frame[3] = 1;
        _config_ack_function = 0x05;
        break;
    case ConfigCommand::FULL_SCALE:
        frame[1] = 0x16;
        frame[3] = uint8_t(_config_value_kg >> 8);
        frame[4] = uint8_t(_config_value_kg);
        frame_len = 6;
        _config_ack_function = 0x17;
        break;
    case ConfigCommand::CALIBRATE_POINT:
        frame[1] = 0x18;
        frame[3] = uint8_t(_config_value_kg >> 8);
        frame[4] = uint8_t(_config_value_kg);
        frame_len = 6;
        _config_ack_function = 0x19;
        break;
    case ConfigCommand::NONE:
        return;
    }

    frame[frame_len - 1] = checksum(frame, frame_len - 1);
    _uart->write(frame, frame_len);
}

void AP_ADM002::read_from_uart()
{
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        const int16_t byte = _uart->read();
        if (byte < 0) {
            return;
        }
        if (_rx_len >= ARRAY_SIZE(_rx_buf)) {
            consume_rx(1);
        }
        _rx_buf[_rx_len++] = uint8_t(byte);
        parse_buffer();
    }
}

bool AP_ADM002::parse_buffer()
{
    bool parsed = false;
    if (_config_state == ConfigState::PENDING && parse_config_ack()) {
        parsed = true;
    }
    while (_rx_len >= STREAM_FRAME_LEN) {
        if (_config_state == ConfigState::PENDING && parse_config_ack()) {
            parsed = true;
            continue;
        }
        if (parse_stream_frame()) {
            consume_rx(STREAM_FRAME_LEN);
            parsed = true;
        } else {
            _checksum_error_count++;
            consume_rx(1);
        }
    }
    if (_config_state == ConfigState::PENDING && parse_config_ack()) {
        parsed = true;
    }
    return parsed;
}

// Configuration acknowledgements are interleaved with unsolicited samples.
bool AP_ADM002::parse_config_ack()
{
    if (_rx_len < 3 || _config_state != ConfigState::PENDING) {
        return false;
    }

    const uint8_t expected_checksum = uint8_t(DEVICE_ADDRESS + _config_ack_function);
    if (_rx_buf[0] != DEVICE_ADDRESS ||
        _rx_buf[1] != _config_ack_function ||
        _rx_buf[2] != expected_checksum) {
        return false;
    }
    consume_rx(3);
    if (_config_command == ConfigCommand::ZERO || _config_command == ConfigCommand::DEFAULT_ZERO) {
        _tare_offset_g = 0;
        _last_tare_ms = AP_HAL::millis();
    }
    _config_state = ConfigState::SUCCESS;
    _config_sequence++;
    return true;
}

bool AP_ADM002::parse_stream_frame()
{
    if ((_rx_buf[0] & ~STATUS_VALID_MASK) != 0 ||
        checksum(_rx_buf, STREAM_FRAME_LEN - 1) != _rx_buf[STREAM_FRAME_LEN - 1]) {
        return false;
    }

    const uint32_t magnitude_g = (uint32_t(_rx_buf[1]) << 16) |
                                 (uint32_t(_rx_buf[2]) << 8) |
                                 uint32_t(_rx_buf[3]);
    _status = _rx_buf[0];
    _weight_g = (_status & STATUS_POSITIVE) != 0 ? int32_t(magnitude_g) : -int32_t(magnitude_g);
    _last_update_ms = AP_HAL::millis();
    _sample_sequence++;
    return true;
}

void AP_ADM002::consume_rx(uint8_t nbytes)
{
    if (nbytes >= _rx_len) {
        reset_rx();
        return;
    }
    memmove(_rx_buf, &_rx_buf[nbytes], _rx_len - nbytes);
    _rx_len -= nbytes;
}

void AP_ADM002::reset_rx()
{
    _rx_len = 0;
}

uint8_t AP_ADM002::checksum(const uint8_t *buf, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

#endif // AP_CONTACT_SENSOR_ENABLED
