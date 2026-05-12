#pragma once

#include "AP_CMCU06A_config.h"

#if AP_CMCU06A_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

class AP_SerialManager;

class AP_CMCU06A {
public:
    AP_CMCU06A() = default;

    CLASS_NO_COPY(AP_CMCU06A);

    // initialise the serial port selected by SERIALn_PROTOCOL
    bool init(const AP_SerialManager &serial_manager);

    // poll serial input and send read requests when due
    void update();

    // request a tare operation
    bool tare();

    // true if valid data has been received recently
    bool healthy() const;
    bool tare_complete() const { return _last_tare_ms != 0; }
    uint32_t last_tare_ms() const { return _last_tare_ms; }

    int32_t get_value() const { return _value; }
    uint32_t last_update_ms() const { return _last_update_ms; }

private:
    static constexpr uint8_t CMCU06A_ADDRESS = 1;
    static constexpr uint8_t MODBUS_FUNC_READ_HOLDING_REGISTERS = 0x03;
    static constexpr uint8_t MODBUS_FUNC_WRITE_SINGLE_REGISTER = 0x06;
    static constexpr uint16_t REG_FINAL_DATA = 0;
    static constexpr uint16_t REG_TRIGGER = 21;
    static constexpr uint16_t TRIGGER_TARE = 1;
    static constexpr uint32_t POLL_INTERVAL_MS = 50;
    static constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;
    static constexpr uint32_t TARE_SETTLE_MS = 600;
    static constexpr uint32_t HEALTH_TIMEOUT_MS = 1000;
    static constexpr uint8_t READ_RESPONSE_LEN = 9;
    static constexpr uint8_t WRITE_RESPONSE_LEN = 8;
    static constexpr uint8_t RX_BUF_LEN = 32;

    enum class Pending : uint8_t {
        NONE,
        READ,
        TARE,
    };

    void send_read_request(uint32_t now_ms);
    void send_tare_request(uint32_t now_ms);
    void send_frame(const uint8_t *frame, uint8_t len);
    void read_from_uart();
    bool parse_buffer();
    void consume_rx(uint8_t nbytes);
    bool parse_read_response();
    bool parse_tare_response();
    void reset_rx();

    static uint16_t crc16_modbus(const uint8_t *buf, uint8_t len);
    static void append_crc(uint8_t *buf, uint8_t len_without_crc);
    static uint16_t get_be16(const uint8_t *buf);

    AP_HAL::UARTDriver *_uart = nullptr;
    int32_t _value = 0;
    uint32_t _last_update_ms = 0;
    uint32_t _last_tare_ms = 0;
    uint32_t _last_request_ms = 0;
    uint32_t _next_request_ms = 0;
    Pending _pending = Pending::NONE;
    bool _tare_queued = false;
    uint8_t _rx_buf[RX_BUF_LEN] {};
    uint8_t _rx_len = 0;
};

#endif  // AP_CMCU06A_ENABLED
