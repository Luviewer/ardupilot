#include "AP_Periph.h"

#ifdef HAL_USE_Hiwonder_Servo

# include <dronecan_msgs.h>

// Send rpm message occasionally
void AP_Periph_FW::send_hiwonder_pos(void)
{

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - hiwonder_last_send_ms < (1000U / 10)) {
        return;
    }
    hiwonder_last_send_ms = now_ms;

    {
        // const uint8_t num_sensors = rpm_sensor.num_sensors();
        for (uint8_t i = 0; i < 16; i++) {
            // Send each sensor in turn

            com_usl_ServoInfo pkt {};
            // pkt.cmd = sensor_id;

            uint8_t        buffer[COM_USL_SERVOINFO_MAX_SIZE];
            const uint16_t total_size = com_usl_ServoInfo_encode(&pkt, buffer, !canfdout());

            canard_broadcast(COM_USL_SERVOINFO_SIGNATURE,
                             COM_USL_SERVOINFO_ID,
                             CANARD_TRANSFER_PRIORITY_LOW,
                             &buffer[0],
                             total_size);
        }
    }
}

void AP_Periph_FW::handle_hiwonder_cmd(CanardInstance* canard_ins, CanardRxTransfer* transfer)
{
    com_usl_ServoCmd pkt;
    if (com_usl_ServoCmd_decode(transfer, &pkt)) {
        return;
    }

    hiwonder_RF.set_position(1, pkt.cmd.data[0], 0);
    hiwonder_RF.set_position(2, pkt.cmd.data[1], 0);
    hiwonder_RF.set_position(3, pkt.cmd.data[2], 0);

    hiwonder_RB.set_position(1, pkt.cmd.data[3], 0);
    hiwonder_RB.set_position(2, pkt.cmd.data[4], 0);
    hiwonder_RB.set_position(3, pkt.cmd.data[5], 0);

    hiwonder_LB.set_position(1, pkt.cmd.data[6], 0);
    hiwonder_LB.set_position(2, pkt.cmd.data[7], 0);
    hiwonder_LB.set_position(3, pkt.cmd.data[8], 0);

    hiwonder_LF.set_position(1, pkt.cmd.data[9], 0);
    hiwonder_LF.set_position(2, pkt.cmd.data[10], 0);
    hiwonder_LF.set_position(3, pkt.cmd.data[11], 0);
}

#endif // HAL_PERIPH_ENABLE_RPM_STREAM
