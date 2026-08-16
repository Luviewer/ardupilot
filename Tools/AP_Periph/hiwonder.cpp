#include "AP_Periph.h"

#ifdef HAL_USE_Hiwonder_Servo
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

static constexpr uint16_t POSITION_INTERVAL_MS = 20;       // 50 Hz
static constexpr uint16_t TEMPERATURE_INTERVAL_MS = 200;   // 5 Hz
static constexpr uint16_t VOLTAGE_INTERVAL_MS = 100;       // 10 Hz
static constexpr uint16_t DISTANCE_INTERVAL_MS = 500;      // 2 Hz
static constexpr uint16_t OFFLINE_PROBE_INTERVAL_MS = 1000;
static constexpr uint8_t POSITION_FAILURES_BEFORE_OFFLINE = 3;

static uint16_t read_interval_ms(AP_Periph_FW::HiwonderReadType type)
{
    switch (type) {
    case AP_Periph_FW::HiwonderReadType::POSITION:
        return POSITION_INTERVAL_MS;
    case AP_Periph_FW::HiwonderReadType::TEMPERATURE:
        return TEMPERATURE_INTERVAL_MS;
    case AP_Periph_FW::HiwonderReadType::VOLTAGE:
        return VOLTAGE_INTERVAL_MS;
    case AP_Periph_FW::HiwonderReadType::DISTANCE:
        return DISTANCE_INTERVAL_MS;
    case AP_Periph_FW::HiwonderReadType::COUNT:
        break;
    }
    return OFFLINE_PROBE_INTERVAL_MS;
}

static bool time_due(uint32_t now_ms, uint32_t due_ms)
{
    return int32_t(now_ms - due_ms) >= 0;
}

bool AP_Periph_FW::select_hiwonder_read(HiwonderPollState &poll, uint32_t now_ms)
{
    // 先按数据类型确定全局优先级，再从上次关节的下一个位置开始轮询。
    // 这样三只舵机全部在线时，低频遥测不会抢占后续关节的位置反馈。
    static constexpr HiwonderReadType priority[] {
        HiwonderReadType::POSITION,
        HiwonderReadType::VOLTAGE,
        HiwonderReadType::TEMPERATURE,
        HiwonderReadType::DISTANCE,
    };

    for (const HiwonderReadType type : priority) {
        for (uint8_t offset = 0; offset < HIWONDER_JOINTS_PER_LEG; offset++) {
            const uint8_t joint_index = (poll.next_joint_index + offset) % HIWONDER_JOINTS_PER_LEG;
            if (!poll.online[joint_index] && type != HiwonderReadType::POSITION) {
                continue;
            }
            if (!time_due(now_ms, poll.next_due_ms[joint_index][uint8_t(type)])) {
                continue;
            }

            poll.joint_index = joint_index;
            poll.next_joint_index = (joint_index + 1) % HIWONDER_JOINTS_PER_LEG;
            poll.read_type = type;
            return true;
        }
    }
    return false;
}

bool AP_Periph_FW::start_hiwonder_read(uint8_t leg_index, HiwonderPollState &poll, uint32_t now_ms)
{
    const uint8_t servo_id = poll.joint_index + 1;
    bool started = false;
    switch (poll.read_type) {
    case HiwonderReadType::POSITION:
        started = hiwonder[leg_index].start_position_read(servo_id);
        break;
    case HiwonderReadType::TEMPERATURE:
        started = hiwonder[leg_index].start_temperature_read(servo_id);
        break;
    case HiwonderReadType::VOLTAGE:
        started = hiwonder[leg_index].start_voltage_read(servo_id);
        break;
    case HiwonderReadType::DISTANCE:
        started = hiwonder[leg_index].start_distance_read(servo_id);
        break;
    case HiwonderReadType::COUNT:
        break;
    }

    if (!started) {
        return false;
    }

    poll.pending = true;
    uint16_t interval_ms = OFFLINE_PROBE_INTERVAL_MS;
    if (poll.online[poll.joint_index]) {
        interval_ms = read_interval_ms(poll.read_type);
    }
    poll.next_due_ms[poll.joint_index][uint8_t(poll.read_type)] = now_ms + interval_ms;
    return true;
}

void AP_Periph_FW::update_hiwonder_port(uint8_t leg_index, uint32_t now_ms)
{
    HiwonderPollState &poll = hiwonder_poll[leg_index];

    if (poll.pending) {
        const uint8_t channel = leg_index * HIWONDER_JOINTS_PER_LEG + poll.joint_index;
        const uint32_t channel_mask = 1UL << channel;
        bool read_ok = false;

        switch (poll.read_type) {
        case HiwonderReadType::POSITION: {
            int16_t position;
            if (hiwonder[leg_index].update_position_read(position)) {
                // DroneCAN沿用1000～2000命令域；协议反馈的0～1000在此统一偏移。
                const int16_t constrained_position = constrain_int16(position,
                                                     AP_Hiwonder::POSITION_MIN,
                                                     AP_Hiwonder::POSITION_MAX);
                hiwonder_positions[channel] = uint16_t(constrained_position + 1000);
                const bool newly_online = !poll.online[poll.joint_index];
                poll.online[poll.joint_index] = true;
                poll.position_failures[poll.joint_index] = 0;
                if (newly_online) {
                    // 探测周期为1秒；上线后立即切换到正常位置和低频遥测周期。
                    poll.next_due_ms[poll.joint_index][uint8_t(HiwonderReadType::POSITION)] =
                        now_ms + POSITION_INTERVAL_MS;
                    poll.next_due_ms[poll.joint_index][uint8_t(HiwonderReadType::TEMPERATURE)] = now_ms;
                    poll.next_due_ms[poll.joint_index][uint8_t(HiwonderReadType::VOLTAGE)] = now_ms;
                    poll.next_due_ms[poll.joint_index][uint8_t(HiwonderReadType::DISTANCE)] = now_ms;
                }
                read_ok = true;
            }
            break;
        }
        case HiwonderReadType::TEMPERATURE: {
            uint8_t temperature_c;
            if (hiwonder[leg_index].update_temperature_read(temperature_c)) {
                hiwonder_temperatures[channel] = temperature_c;
                hiwonder_temperature_valid_mask |= channel_mask;
                read_ok = true;
            }
            break;
        }
        case HiwonderReadType::VOLTAGE: {
            uint16_t voltage_mv;
            if (hiwonder[leg_index].update_voltage_read(voltage_mv)) {
                hiwonder_voltages[channel] = voltage_mv;
                hiwonder_voltage_valid_mask |= channel_mask;
                read_ok = true;
            }
            break;
        }
        case HiwonderReadType::DISTANCE: {
            int32_t distance_pulse;
            if (hiwonder[leg_index].update_distance_read(distance_pulse)) {
                hiwonder_distances[channel] = distance_pulse;
                hiwonder_distance_valid_mask |= channel_mask;
                read_ok = true;
            }
            break;
        }
        case HiwonderReadType::COUNT:
            break;
        }

        if (!read_ok && !hiwonder[leg_index].read_finished()) {
            return;
        }

        if (!read_ok) {
            switch (poll.read_type) {
            case HiwonderReadType::POSITION:
                hiwonder_positions[channel] = 0;
                poll.position_failures[poll.joint_index]++;
                if (poll.position_failures[poll.joint_index] >= POSITION_FAILURES_BEFORE_OFFLINE) {
                    poll.online[poll.joint_index] = false;
                    poll.position_failures[poll.joint_index] = 0;
                    poll.next_due_ms[poll.joint_index][uint8_t(HiwonderReadType::POSITION)] =
                        now_ms + OFFLINE_PROBE_INTERVAL_MS;
                    hiwonder_temperature_valid_mask &= ~channel_mask;
                    hiwonder_voltage_valid_mask &= ~channel_mask;
                    hiwonder_distance_valid_mask &= ~channel_mask;
                }
                break;
            case HiwonderReadType::TEMPERATURE:
                hiwonder_temperature_valid_mask &= ~channel_mask;
                break;
            case HiwonderReadType::VOLTAGE:
                hiwonder_voltage_valid_mask &= ~channel_mask;
                break;
            case HiwonderReadType::DISTANCE:
                hiwonder_distance_valid_mask &= ~channel_mask;
                break;
            case HiwonderReadType::COUNT:
                break;
            }
        }

        hiwonder[leg_index].reset_read_state();
        poll.pending = false;
    }

    if (select_hiwonder_read(poll, now_ms)) {
        start_hiwonder_read(leg_index, poll, now_ms);
    }
}

void AP_Periph_FW::publish_hiwonder_position(uint32_t now_ms)
{
    if (now_ms - hiwonder_position_last_send_ms < POSITION_INTERVAL_MS) {
        return;
    }

    com_usl_ServoInfo pkt {};
    pkt.pos.len = ARRAY_SIZE(hiwonder_positions);
    memcpy(pkt.pos.data, hiwonder_positions, sizeof(hiwonder_positions));
    uint8_t buffer[COM_USL_SERVOINFO_MAX_SIZE];
    const uint16_t size = com_usl_ServoInfo_encode(&pkt, buffer, !canfdout());
    canard_broadcast(COM_USL_SERVOINFO_SIGNATURE, COM_USL_SERVOINFO_ID,
                     CANARD_TRANSFER_PRIORITY_LOW, buffer, size);
    hiwonder_position_last_send_ms = now_ms;
}

void AP_Periph_FW::publish_hiwonder_temperature(uint32_t now_ms)
{
    if (now_ms - hiwonder_temperature_last_send_ms < TEMPERATURE_INTERVAL_MS) {
        return;
    }

    com_usl_ServoTemperature pkt {};
    pkt.valid_mask = hiwonder_temperature_valid_mask;
    memcpy(pkt.temperature_c, hiwonder_temperatures, sizeof(hiwonder_temperatures));
    uint8_t buffer[COM_USL_SERVOTEMPERATURE_MAX_SIZE];
    const uint16_t size = com_usl_ServoTemperature_encode(&pkt, buffer, !canfdout());
    canard_broadcast(COM_USL_SERVOTEMPERATURE_SIGNATURE, COM_USL_SERVOTEMPERATURE_ID,
                     CANARD_TRANSFER_PRIORITY_LOW, buffer, size);
    hiwonder_temperature_last_send_ms = now_ms;
}

void AP_Periph_FW::publish_hiwonder_voltage(uint32_t now_ms)
{
    if (now_ms - hiwonder_voltage_last_send_ms < VOLTAGE_INTERVAL_MS) {
        return;
    }

    com_usl_ServoVoltage pkt {};
    pkt.valid_mask = hiwonder_voltage_valid_mask;
    memcpy(pkt.voltage_mv, hiwonder_voltages, sizeof(hiwonder_voltages));
    uint8_t buffer[COM_USL_SERVOVOLTAGE_MAX_SIZE];
    const uint16_t size = com_usl_ServoVoltage_encode(&pkt, buffer, !canfdout());
    canard_broadcast(COM_USL_SERVOVOLTAGE_SIGNATURE, COM_USL_SERVOVOLTAGE_ID,
                     CANARD_TRANSFER_PRIORITY_LOW, buffer, size);
    hiwonder_voltage_last_send_ms = now_ms;
}

void AP_Periph_FW::publish_hiwonder_distance(uint32_t now_ms)
{
    if (now_ms - hiwonder_distance_last_send_ms < DISTANCE_INTERVAL_MS) {
        return;
    }

    com_usl_ServoDistance pkt {};
    pkt.valid_mask = hiwonder_distance_valid_mask;
    memcpy(pkt.distance_pulse, hiwonder_distances, sizeof(hiwonder_distances));
    uint8_t buffer[COM_USL_SERVODISTANCE_MAX_SIZE];
    const uint16_t size = com_usl_ServoDistance_encode(&pkt, buffer, !canfdout());
    canard_broadcast(COM_USL_SERVODISTANCE_SIGNATURE, COM_USL_SERVODISTANCE_ID,
                     CANARD_TRANSFER_PRIORITY_LOW, buffer, size);
    hiwonder_distance_last_send_ms = now_ms;
}

void AP_Periph_FW::update_hiwonder()
{
    const uint32_t now_ms = AP_HAL::millis();

    // 六条腿各占用一个UART，可并行保留六个读取事务；同一条腿内串行轮询ID 1～3。
    for (uint8_t leg_index = 0; leg_index < AP_Hiwonder::BUS_COUNT; leg_index++) {
        update_hiwonder_port(leg_index, now_ms);
    }

    publish_hiwonder_position(now_ms);
    publish_hiwonder_temperature(now_ms);
    publish_hiwonder_voltage(now_ms);
    publish_hiwonder_distance(now_ms);
}

void AP_Periph_FW::handle_hiwonder_cmd(CanardInstance *canard_ins, CanardRxTransfer *transfer)
{
    (void)canard_ins;
    com_usl_ServoCmd pkt {};
    if (com_usl_ServoCmd_decode(transfer, &pkt) ||
        pkt.cmd.len != HIWONDER_SERVO_COUNT) {
        return;
    }

    static constexpr uint16_t target_time_ms = 10;
    for (uint8_t leg_index = 0; leg_index < AP_Hiwonder::BUS_COUNT; leg_index++) {
        // 角度命令优先于遥测；被打断的读取事务由轮询器在下一周期重新发起。
        if (hiwonder_poll[leg_index].pending) {
            hiwonder[leg_index].reset_read_state();
            hiwonder_poll[leg_index].pending = false;
        }
        for (uint8_t joint_index = 0; joint_index < HIWONDER_JOINTS_PER_LEG; joint_index++) {
            const uint8_t cmd_index = leg_index * HIWONDER_JOINTS_PER_LEG + joint_index;
            const uint16_t command_position = constrain_uint16(pkt.cmd.data[cmd_index], 1000, 2000);
            const uint16_t protocol_position = command_position - 1000;
            hiwonder[leg_index].set_position(joint_index + 1,
                                             protocol_position,
                                             target_time_ms);
        }
    }
}

#endif // HAL_USE_Hiwonder_Servo
