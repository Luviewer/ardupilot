#pragma once

#include "AP_ContactSensor_config.h"

#if AP_CONTACT_SENSOR_ENABLED

#include "AP_ADM002.h"
#include "AP_ContactSensor.h"

class AP_SerialManager;

class AP_ContactSensor_Manager : public AP_ContactSensor
{
public:
    enum class Type : uint8_t {
        NONE = 0,
        ADM002_STREAM,
    };

    enum class CalibrationAction : uint8_t {
        QUERY = 0,
        SOFTWARE_TARE,
        DEVICE_ZERO,
        DEVICE_DEFAULT_ZERO,
        SET_FULL_SCALE,
        CALIBRATE_POINT,
    };

    enum class CalibrationState : uint8_t {
        IDLE = 0,
        PENDING,
        SUCCESS,
        FAILED,
    };

    void init(const AP_SerialManager &serial_manager);
    void update();

    bool get_force_sample(ForceSample &sample) const override;
    bool healthy() const override;
    bool tare() override;
    bool tare_complete() const override;
    bool get_wrench_sample(WrenchSample &sample) const override;

    Type type() const
    {
        return _type;
    }
    const char *type_name() const;
    bool start_calibration(CalibrationAction action, uint16_t value_kg = 0);
    CalibrationAction calibration_action() const
    {
        return _calibration_action;
    }
    CalibrationState calibration_state() const;
    uint32_t calibration_sequence() const;

private:
    AP_ContactSensor *backend();
    const AP_ContactSensor *backend() const;

    AP_ADM002 _adm002;
    Type _type = Type::NONE;
    CalibrationAction _calibration_action = CalibrationAction::QUERY;
    CalibrationState _software_calibration_state = CalibrationState::IDLE;
    uint32_t _backend_calibration_sequence = 0;
    uint32_t _calibration_sequence = 0;
};

#endif // AP_CONTACT_SENSOR_ENABLED
