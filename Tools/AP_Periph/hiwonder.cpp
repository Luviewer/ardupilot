#include "AP_Periph.h"

#ifdef HAL_USE_Hiwonder_Servo

# include <dronecan_msgs.h>

// Send rpm message occasionally
void AP_Periph_FW::send_hiwonder_pos(void)
{

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - hiwonder_last_send_ms < (1000U / 100)) {
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

    // for (uint8_t i = 0; i < 12; i++) {
    //     if (pkt.cmd.data[i] > 1750) {
    //         pkt.cmd.data[i] = 1750;
    //     } else if (pkt.cmd.data[i] < 1250) {
    //         pkt.cmd.data[i] = 1250;
    //     }
    // }

    AP_Hiwonder* hiwonder_ptr = nullptr;

    for (uint8_t index = 0; index < 4; index++) {
        switch (index) {
            case 0:
                hiwonder_ptr = &hiwonder_RF;
                break;

            case 1:
                hiwonder_ptr = &hiwonder_RB;
                break;

            case 2:
                hiwonder_ptr = &hiwonder_LB;
                break;

            case 3:
                hiwonder_ptr = &hiwonder_LF;
                break;
        }

        hiwonder_ptr->set_position(1, pkt.cmd.data[0 + index * 3] - 1000, 0);
        hiwonder_ptr->set_position(2, pkt.cmd.data[1 + index * 3] - 1000, 0);
        hiwonder_ptr->set_position(3, pkt.cmd.data[2 + index * 3] - 1000, 0);
    }

    // hiwonder_RB.set_position(0, pkt.cmd.data[3] - 1000, 0);
    // hiwonder_RB.set_position(1, pkt.cmd.data[4] - 1000, 0);
    // hiwonder_RB.set_position(2, pkt.cmd.data[5] - 1000, 0);

    // hiwonder_LB.set_position(0, pkt.cmd.data[6] - 1000, 0);
    // hiwonder_LB.set_position(1, pkt.cmd.data[7] - 1000, 0);
    // hiwonder_LB.set_position(2, pkt.cmd.data[8] - 1000, 0);

    // hiwonder_LF.set_position(0, pkt.cmd.data[9] - 1000, 0);
    // hiwonder_LF.set_position(1, pkt.cmd.data[10] - 1000, 0);
    // hiwonder_LF.set_position(2, pkt.cmd.data[11] - 1000, 0);
}

#endif // HAL_PERIPH_ENABLE_RPM_STREAM
