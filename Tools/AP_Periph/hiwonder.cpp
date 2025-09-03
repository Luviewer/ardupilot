#include "AP_Periph.h"
#ifdef HAL_USE_Hiwonder_Servo
# include <dronecan_msgs.h>

extern const AP_HAL::HAL& hal;

void AP_Periph_FW::send_hiwonder_pos(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - hiwonder_last_send_ms < (1000U / 100)) {
        return;
    }
    hiwonder_last_send_ms = now_ms;

    for (uint8_t i = 0; i < 16; i++) {
        com_usl_ServoInfo pkt {};
        uint8_t           buffer[COM_USL_SERVOINFO_MAX_SIZE];
        const uint16_t    total_size = com_usl_ServoInfo_encode(&pkt, buffer, !canfdout());

        canard_broadcast(COM_USL_SERVOINFO_SIGNATURE,
                         COM_USL_SERVOINFO_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);
    }
}

void AP_Periph_FW::handle_hiwonder_cmd(CanardInstance* canard_ins, CanardRxTransfer* transfer)
{
    com_usl_ServoCmd pkt;
    if (com_usl_ServoCmd_decode(transfer, &pkt)) {
        return;
    }

    const uint32_t target_time = 100;

    for (uint8_t index = 0; index < AP_Hiwonder::SERVO_Total; index++) {
        hiwonder[index].set_position(1, pkt.cmd.data[0 + index * 3] - 1000, target_time);
        hiwonder[index].set_position(2, pkt.cmd.data[1 + index * 3] - 1000, target_time);
        hiwonder[index].set_position(3, pkt.cmd.data[2 + index * 3] - 1000, target_time);
    }
}

// void AP_Periph_FW::test_read_servo_positions(void)
// {
//     static uint32_t last_test_time = 0;
//     static uint8_t  current_index  = 0;
//     static uint8_t  current_servo  = 1;
//     static enum { IDLE,
//                   STARTED,
//                   WAITING } test_state
//         = IDLE;

//     uint32_t now = AP_HAL::millis();

//     switch (test_state) {
//         case IDLE:
//             if (now - last_test_time >= 1000) {
//                 last_test_time = now;
//                 current_index  = 0;
//                 current_servo  = 1;
//                 test_state     = STARTED;
//             }
//             break;

//         case STARTED:
//             if (hiwonder[current_index].start_position_read(current_servo)) {
//                 test_state = WAITING;
//             } else {
//                 can_printf("Hiwonder[%d] Servo[%d] Start failed\n", current_index, current_servo);
//                 current_servo++;
//                 if (current_servo > 2) {
//                     current_servo = 1;
//                     current_index++;
//                     if (current_index >= AP_Hiwonder::SERVO_Total) {
//                         test_state = IDLE;
//                     }
//                 }
//                 hiwonder[current_index].reset_read_state();
//             }
//             break;

//         case WAITING: {
//             uint16_t position = 0;
//             if (hiwonder[current_index].update_position_read(position)) {
//                 can_printf("Hiwonder[%d] Servo[%d] Position: %d\n", current_index, current_servo, position);
//                 current_servo++;
//                 if (current_servo > 2) {
//                     current_servo = 1;
//                     current_index++;
//                     if (current_index >= AP_Hiwonder::SERVO_Total) {
//                         test_state = IDLE;
//                     }
//                 }
//                 hiwonder[current_index].reset_read_state();
//             } else if (hiwonder[current_index].is_read_complete()) {
//                 can_printf("Hiwonder[%d] Servo[%d] Read timeout\n", current_index, current_servo);
//                 current_servo++;
//                 if (current_servo > 2) {
//                     current_servo = 1;
//                     current_index++;
//                     if (current_index >= AP_Hiwonder::SERVO_Total) {
//                         test_state = IDLE;
//                     }
//                 }
//                 hiwonder[current_index].reset_read_state();
//             }
//         } break;
//     }
// }

#endif // HAL_PERIPH_ENABLE_RPM_STREAM