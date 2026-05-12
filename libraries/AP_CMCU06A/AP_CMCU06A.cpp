#include "AP_CMCU06A.h"

#if AP_CMCU06A_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <string.h>

bool AP_CMCU06A::init(const AP_SerialManager &serial_manager)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CMCU06A, 0);
    if (_uart == nullptr) {
        return false;
    }

    _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_CMCU06A, 0));
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    reset_rx();
    _pending = Pending::NONE;
    _tare_queued = false;
    _last_request_ms = 0;
    _next_request_ms = 0;
    return true;
}

void AP_CMCU06A::update()
{
    if (_uart == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    read_from_uart();

    if (_pending != Pending::NONE) {
        if (now_ms - _last_request_ms > RESPONSE_TIMEOUT_MS) {
            _pending = Pending::NONE;
            reset_rx();
        } else {
            return;
        }
    }

    if (_tare_queued) {
        send_tare_request(now_ms);
        _tare_queued = false;
        return;
    }

    if (now_ms >= _next_request_ms) {
        send_read_request(now_ms);
    }
}

bool AP_CMCU06A::tare()
{
    if (_uart == nullptr || _tare_queued || _pending == Pending::TARE) {
        return false;
    }

    _last_tare_ms = 0;
    if (_pending == Pending::NONE) {
        send_tare_request(AP_HAL::millis());
    } else {
        _tare_queued = true;
    }
    return true;
}

bool AP_CMCU06A::healthy() const
{
    return _last_update_ms != 0 && AP_HAL::millis() - _last_update_ms < HEALTH_TIMEOUT_MS;
}

void AP_CMCU06A::send_read_request(uint32_t now_ms)
{
    uint8_t frame[8] {
        CMCU06A_ADDRESS,
        MODBUS_FUNC_READ_HOLDING_REGISTERS,
        uint8_t(REG_FINAL_DATA >> 8),
        uint8_t(REG_FINAL_DATA & 0xFF),
        0,
        2,
        0,
        0,
    };
    append_crc(frame, ARRAY_SIZE(frame) - 2);
    send_frame(frame, ARRAY_SIZE(frame));
    _pending = Pending::READ;
    _last_request_ms = now_ms;
    _next_request_ms = now_ms + POLL_INTERVAL_MS;
}

void AP_CMCU06A::send_tare_request(uint32_t now_ms)
{
    uint8_t frame[8] {
        CMCU06A_ADDRESS,
        MODBUS_FUNC_WRITE_SINGLE_REGISTER,
        uint8_t(REG_TRIGGER >> 8),
        uint8_t(REG_TRIGGER & 0xFF),
        uint8_t(TRIGGER_TARE >> 8),
        uint8_t(TRIGGER_TARE & 0xFF),
        0,
        0,
    };
    append_crc(frame, ARRAY_SIZE(frame) - 2);
    send_frame(frame, ARRAY_SIZE(frame));
    _pending = Pending::TARE;
    _last_request_ms = now_ms;
    _next_request_ms = now_ms + TARE_SETTLE_MS;
}

void AP_CMCU06A::send_frame(const uint8_t *frame, uint8_t len)
{
    reset_rx();
    _uart->write(frame, len);
}

void AP_CMCU06A::read_from_uart()
{
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        const int16_t b = _uart->read();
        if (b < 0) {
            return;
        }

        if (_rx_len >= ARRAY_SIZE(_rx_buf)) {
            reset_rx();
        }
        _rx_buf[_rx_len++] = uint8_t(b);

        parse_buffer();
    }
}

bool AP_CMCU06A::parse_buffer()
{
    if (_pending == Pending::NONE) {
        reset_rx();
        return false;
    }

    const uint8_t expected_len = (_pending == Pending::READ) ? READ_RESPONSE_LEN : WRITE_RESPONSE_LEN;
    while (_rx_len >= expected_len) {
        bool parsed = false;
        switch (_pending) {
        case Pending::READ:
            parsed = parse_read_response();
            break;
        case Pending::TARE:
            parsed = parse_tare_response();
            break;
        case Pending::NONE:
            break;
        }

        if (parsed) {
            reset_rx();
            _pending = Pending::NONE;
            return true;
        }

        // Drop one byte and keep scanning. This handles RS485 adapters that
        // echo the request before the transmitter's response.
        consume_rx(1);
    }

    return false;
}

void AP_CMCU06A::consume_rx(uint8_t nbytes)
{
    if (nbytes >= _rx_len) {
        reset_rx();
        return;
    }

    memmove(_rx_buf, &_rx_buf[nbytes], _rx_len - nbytes);
    _rx_len -= nbytes;
}

bool AP_CMCU06A::parse_read_response()
{
    if (_rx_buf[0] != CMCU06A_ADDRESS ||
        _rx_buf[1] != MODBUS_FUNC_READ_HOLDING_REGISTERS ||
        _rx_buf[2] != 4) {
        return false;
    }

    const uint16_t expected_crc = crc16_modbus(_rx_buf, READ_RESPONSE_LEN - 2);
    const uint16_t received_crc = uint16_t(_rx_buf[READ_RESPONSE_LEN - 2]) | (uint16_t(_rx_buf[READ_RESPONSE_LEN - 1]) << 8);
    if (expected_crc != received_crc) {
        return false;
    }

    const uint16_t low_word = get_be16(&_rx_buf[3]);
    const uint16_t high_word = get_be16(&_rx_buf[5]);
    _value = int32_t((uint32_t(high_word) << 16) | low_word);
    _last_update_ms = AP_HAL::millis();
    return true;
}

bool AP_CMCU06A::parse_tare_response()
{
    if (_rx_buf[0] != CMCU06A_ADDRESS ||
        _rx_buf[1] != MODBUS_FUNC_WRITE_SINGLE_REGISTER ||
        get_be16(&_rx_buf[2]) != REG_TRIGGER ||
        get_be16(&_rx_buf[4]) != TRIGGER_TARE) {
        return false;
    }

    const uint16_t expected_crc = crc16_modbus(_rx_buf, WRITE_RESPONSE_LEN - 2);
    const uint16_t received_crc = uint16_t(_rx_buf[WRITE_RESPONSE_LEN - 2]) | (uint16_t(_rx_buf[WRITE_RESPONSE_LEN - 1]) << 8);
    if (expected_crc != received_crc) {
        return false;
    }

    _last_tare_ms = AP_HAL::millis();
    return true;
}

void AP_CMCU06A::reset_rx()
{
    _rx_len = 0;
}

uint16_t AP_CMCU06A::crc16_modbus(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void AP_CMCU06A::append_crc(uint8_t *buf, uint8_t len_without_crc)
{
    const uint16_t crc = crc16_modbus(buf, len_without_crc);
    buf[len_without_crc] = uint8_t(crc & 0xFF);
    buf[len_without_crc + 1] = uint8_t(crc >> 8);
}

uint16_t AP_CMCU06A::get_be16(const uint8_t *buf)
{
    return (uint16_t(buf[0]) << 8) | buf[1];
}

#endif  // AP_CMCU06A_ENABLED
