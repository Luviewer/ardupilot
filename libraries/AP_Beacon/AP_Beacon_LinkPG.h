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
/*
 Original C Code by Marvelmind (https://github.com/MarvelmindRobotics/marvelmind.c)
 Adapted into Ardupilot by Karthik Desai, Amilcar Lucas
 April 2017
 */

#pragma once

#include "AP_Beacon_Backend.h"

#define AP_BEACON_LINKPG_BUF_SIZE 47

#define AP_BEACON_LINKPG_MAX_NUM 4

class AP_Beacon_LinkPG : public AP_Beacon_Backend {
public:
    // constructor
    AP_Beacon_LinkPG(AP_Beacon& frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update() override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum ParseState {
        ParseState_WaitingForModBusID = 0,
        ParseState_WaitingForModBusFunc = 1,
        ParseState_WaitingForLen = 2,
        ParseState_WaitingForFlag1 = 3,
        ParseState_WaitingForFlag2 = 4,
        ParseState_WaitingForInfoFlag1 = 5,
        ParseState_WaitingForInfoFlag2 = 6,
        ParseState_WaitingForContents = 7,
    } parse_state;

    // parse buffer
    void parse_buffer();

    uint8_t linebuf[AP_BEACON_LINKPG_BUF_SIZE];
    uint8_t linebuf_len = 0;
    uint32_t last_update_ms = 0;

    uint8_t parse_msg_id;
    uint8_t parse_msg_len;

    AP_Int16 beaconXYZ[AP_BEACON_LINKPG_MAX_NUM][3];

    AP_Int8 beaconID[AP_BEACON_LINKPG_MAX_NUM];

    AP_Float alpha;

    Vector3f last_veh_pos;

    float beaconDistance[AP_BEACON_LINKPG_MAX_NUM];

    float last_distance[AP_BEACON_LINKPG_MAX_NUM];
};
