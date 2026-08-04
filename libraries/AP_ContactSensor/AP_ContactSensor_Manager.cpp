#include "AP_ContactSensor_Manager.h"

#if AP_CONTACT_SENSOR_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>

void AP_ContactSensor_Manager::init(const AP_SerialManager &serial_manager)
{
    if (_adm002.init(serial_manager)) {
        _type = Type::ADM002_STREAM;
    }
}

void AP_ContactSensor_Manager::update()
{
    switch (_type) {
    case Type::ADM002_STREAM:
        _adm002.update();
        if (_backend_calibration_sequence != _adm002.config_sequence()) {
            _backend_calibration_sequence = _adm002.config_sequence();
            _calibration_sequence++;
        }
        break;
    case Type::NONE:
        break;
    }
}

bool AP_ContactSensor_Manager::get_force_sample(ForceSample &sample) const
{
    const AP_ContactSensor *sensor = backend();
    return sensor != nullptr && sensor->get_force_sample(sample);
}

bool AP_ContactSensor_Manager::healthy() const
{
    const AP_ContactSensor *sensor = backend();
    return sensor != nullptr && sensor->healthy();
}

bool AP_ContactSensor_Manager::tare()
{
    AP_ContactSensor *sensor = backend();
    return sensor != nullptr && sensor->tare();
}

bool AP_ContactSensor_Manager::tare_complete() const
{
    const AP_ContactSensor *sensor = backend();
    return sensor != nullptr && sensor->tare_complete();
}

bool AP_ContactSensor_Manager::get_wrench_sample(WrenchSample &sample) const
{
    const AP_ContactSensor *sensor = backend();
    return sensor != nullptr && sensor->get_wrench_sample(sample);
}

bool AP_ContactSensor_Manager::start_calibration(CalibrationAction action, uint16_t value_kg)
{
    _calibration_action = action;
    if (action == CalibrationAction::QUERY) {
        return true;
    }
    if (_type != Type::ADM002_STREAM) {
        _software_calibration_state = CalibrationState::FAILED;
        _calibration_sequence++;
        return false;
    }
    if (_adm002.config_state() == AP_ADM002::ConfigState::PENDING) {
        return false;
    }
    if (action == CalibrationAction::SOFTWARE_TARE) {
        const bool success = _adm002.tare();
        _software_calibration_state = success ? CalibrationState::SUCCESS : CalibrationState::FAILED;
        _calibration_sequence++;
        return success;
    }

    AP_ADM002::ConfigCommand command = AP_ADM002::ConfigCommand::NONE;
    switch (action) {
    case CalibrationAction::DEVICE_ZERO:
        command = AP_ADM002::ConfigCommand::ZERO;
        break;
    case CalibrationAction::DEVICE_DEFAULT_ZERO:
        command = AP_ADM002::ConfigCommand::DEFAULT_ZERO;
        break;
    case CalibrationAction::SET_FULL_SCALE:
        command = AP_ADM002::ConfigCommand::FULL_SCALE;
        break;
    case CalibrationAction::CALIBRATE_POINT:
        command = AP_ADM002::ConfigCommand::CALIBRATE_POINT;
        break;
    case CalibrationAction::QUERY:
    case CalibrationAction::SOFTWARE_TARE:
        break;
    }
    return _adm002.start_config_command(command, value_kg);
}

AP_ContactSensor_Manager::CalibrationState AP_ContactSensor_Manager::calibration_state() const
{
    if (_calibration_action == CalibrationAction::SOFTWARE_TARE) {
        return _software_calibration_state;
    }
    return CalibrationState(uint8_t(_adm002.config_state()));
}

uint32_t AP_ContactSensor_Manager::calibration_sequence() const
{
    return _calibration_sequence;
}

const char *AP_ContactSensor_Manager::type_name() const
{
    switch (_type) {
    case Type::ADM002_STREAM:
        return "ADM002";
    case Type::NONE:
        return "None";
    }
    return "Unknown";
}

AP_ContactSensor *AP_ContactSensor_Manager::backend()
{
    switch (_type) {
    case Type::ADM002_STREAM:
        return &_adm002;
    case Type::NONE:
        return nullptr;
    }
    return nullptr;
}

const AP_ContactSensor *AP_ContactSensor_Manager::backend() const
{
    switch (_type) {
    case Type::ADM002_STREAM:
        return &_adm002;
    case Type::NONE:
        return nullptr;
    }
    return nullptr;
}

#endif // AP_CONTACT_SENSOR_ENABLED
