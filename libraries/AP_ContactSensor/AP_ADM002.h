#pragma once

#include "AP_ContactSensor_config.h"

#if AP_CONTACT_SENSOR_ENABLED

#include "AP_ContactSensor.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

class AP_SerialManager;

// ADM002 general-protocol backend using the high-speed unsolicited weight
// stream. The parser accepts any stream rate supported by the module firmware.
class AP_ADM002 : public AP_ContactSensor
{
public:
    enum class ConfigCommand : uint8_t {
        NONE = 0,
        ZERO,
        DEFAULT_ZERO,
        FULL_SCALE,
        CALIBRATE_POINT,
    };

    enum class ConfigState : uint8_t {
        IDLE = 0,
        PENDING,
        SUCCESS,
        FAILED,
    };

    AP_ADM002() = default;

    CLASS_NO_COPY(AP_ADM002);

    bool init(const AP_SerialManager &serial_manager);
    void update();

    bool get_force_sample(ForceSample &sample) const override;
    bool healthy() const override;
    bool tare() override;
    bool tare_complete() const override
    {
        return _last_tare_ms != 0;
    }

    bool configured() const
    {
        return _uart != nullptr;
    }
    uint8_t status() const
    {
        return _status;
    }
    uint32_t checksum_error_count() const
    {
        return _checksum_error_count;
    }
    bool start_config_command(ConfigCommand command, uint16_t value_kg = 0);
    ConfigCommand config_command() const
    {
        return _config_command;
    }
    ConfigState config_state() const
    {
        return _config_state;
    }
    uint32_t config_sequence() const
    {
        return _config_sequence;
    }

private:
    friend class AP_ADM002Test;

    static constexpr uint8_t DEVICE_ADDRESS = 1;
    static constexpr uint8_t ENABLE_STREAM_FUNCTION = 0x28;
    static constexpr uint8_t ENABLE_STREAM_ACK_FUNCTION = 0x29;
    static constexpr uint8_t WRITE_OPERATION = 1;
    static constexpr uint8_t ENABLE_STREAM = 1;
    static constexpr uint8_t STREAM_FRAME_LEN = 5;
    static constexpr uint8_t RX_BUF_LEN = 32;
    static constexpr uint8_t STATUS_VALID_MASK = 0x63;
    static constexpr uint8_t STATUS_POSITIVE = 1U << 0;
    static constexpr uint8_t STATUS_OVERLOAD = 1U << 5;
    static constexpr uint8_t STATUS_AD_FAULT = 1U << 6;
    static constexpr uint32_t HEALTH_TIMEOUT_MS = 100;
    static constexpr uint32_t ENABLE_RETRY_MS = 500;
    static constexpr uint32_t CONFIG_TIMEOUT_MS = 1000;

    void send_enable_stream();
    void send_config_command();
    void read_from_uart();
    bool parse_buffer();
    bool parse_config_ack();
    bool parse_enable_stream_ack();
    bool parse_stream_frame();
    void consume_rx(uint8_t nbytes);
    void reset_rx();
    static uint8_t checksum(const uint8_t *buf, uint8_t len);

    AP_HAL::UARTDriver *_uart = nullptr;
    int32_t _weight_g = 0;
    int32_t _tare_offset_g = 0;
    uint8_t _status = 0;
    uint8_t _rx_buf[RX_BUF_LEN] {};
    uint8_t _rx_len = 0;
    uint32_t _last_update_ms = 0;
    uint32_t _last_tare_ms = 0;
    uint32_t _last_enable_ms = 0;
    uint32_t _sample_sequence = 0;
    uint32_t _checksum_error_count = 0;
    ConfigCommand _config_command = ConfigCommand::NONE;
    ConfigState _config_state = ConfigState::IDLE;
    uint16_t _config_value_kg = 0;
    uint8_t _config_ack_function = 0;
    uint32_t _config_start_ms = 0;
    uint32_t _config_sequence = 0;
};

#endif // AP_CONTACT_SENSOR_ENABLED
