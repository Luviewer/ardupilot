/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_RMUART {
public:
    AP_RMUART();

    /* Do not allow copies */
    AP_RMUART(const AP_RMUART& other) = delete;
    AP_RMUART& operator=(const AP_RMUART&) = delete;

    // init - perform require initialisation including detecting which protocol
    // to use
    void init(const AP_SerialManager& serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    void update(void);

    void Receive(void);

    void Send(void);

    // 从电机转速映射到【-1~1】
    void getWheelSpeed(int16_t& wheelleft_f, int16_t& wheelright_f)
    {
        wheelleft_f = (stm32_2_apm.wheel_left_int);
        wheelright_f = (stm32_2_apm.wheel_right_int);
    }

    // 从【-1~1】映射到电机转速
    void setWheelSpeed(int16_t& wheelleft_f, int16_t& wheelright_f)
    {
        apm_2_stm32.wheel_left_int = (int16_t)(wheelleft_f);
        apm_2_stm32.wheel_right_int = (int16_t)(wheelright_f);
    }

    struct PACKED apm_2_stm32_struct {
        uint8_t header[2];
        uint8_t len;
        int16_t wheel_left_int;
        int16_t wheel_right_int; 
        uint8_t tail;
    };


    struct PACKED stm32_2_apm_struct {
        uint8_t header[2];
        uint8_t len;
        int16_t wheel_left_int;
        int16_t wheel_right_int;
        uint8_t tail;
    };

private:
    AP_HAL::UARTDriver* _port; // UART used to send data to receiver

    apm_2_stm32_struct apm_2_stm32;
    stm32_2_apm_struct stm32_2_apm;

    uint8_t _rx_step;

    uint8_t receive_buff[20];
};