#include "AP_HexRuped_Params.h"

#define COXA_LEN_DEFAULT    47.1f
#define FEMUR_LEN_DEFAULT   133.0f
#define TIBIA_LEN_DEFAULT   144.1f
#define FRAME_LEN_DEFAULT   185.0f
#define FRAME_WIDTH_DEFAULT 185.0f

const AP_Param::GroupInfo AP_HexRuped_Params::var_info[] = {
    AP_GROUPINFO("C_DIR", 1, AP_HexRuped_Params, COXA_DIR, 1),
    AP_GROUPINFO("F_DIR", 2, AP_HexRuped_Params, FEMU_DIR, 1),
    AP_GROUPINFO("T_DIR", 3, AP_HexRuped_Params, TIBI_DIR, 1),

    AP_GROUPINFO("C_OFS", 4, AP_HexRuped_Params, COXA_OFS, 0),
    AP_GROUPINFO("F_OFS", 5, AP_HexRuped_Params, FEMU_OFS, 0),
    AP_GROUPINFO("T_OFS", 6, AP_HexRuped_Params, TIBI_OFS, 0),

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_HexRuped_CHANNEL_Params::var_info[] = {
    AP_GROUPINFO("THR_X", 1, AP_HexRuped_CHANNEL_Params, throttle_x_channel, -1),
    AP_GROUPINFO("THR_Y", 2, AP_HexRuped_CHANNEL_Params, throttle_y_channel, -1),
    AP_GROUPINFO("HGH", 3, AP_HexRuped_CHANNEL_Params, body_height, -1),
    AP_GROUPINFO("LIFT", 4, AP_HexRuped_CHANNEL_Params, left_lift, -1),

    AP_GROUPINFO("ROL", 5, AP_HexRuped_CHANNEL_Params, roll_channel, -1),
    AP_GROUPINFO("PIT", 6, AP_HexRuped_CHANNEL_Params, pitch_channel, -1),
    AP_GROUPINFO("YAW", 7, AP_HexRuped_CHANNEL_Params, yaw_channel, -1),

    AP_GROUPINFO("CEN_X", 8, AP_HexRuped_CHANNEL_Params, centre_offset_x_channel, -1),
    AP_GROUPINFO("CEN_Y", 9, AP_HexRuped_CHANNEL_Params, centre_offset_y_channel, -1),

    AP_GROUPINFO("THR_XMX", 10, AP_HexRuped_CHANNEL_Params, throttle_x_max, 100),
    AP_GROUPINFO("THR_YMX", 11, AP_HexRuped_CHANNEL_Params, throttle_y_max, 100),

    AP_GROUPINFO("MODE", 12, AP_HexRuped_CHANNEL_Params, mode_channel, -1),
    AP_GROUPINFO("FLY", 13, AP_HexRuped_CHANNEL_Params, fly_mode_channel, -1),
    AP_GROUPINFO("WAK", 14, AP_HexRuped_CHANNEL_Params, walk_mode_channel, -1),
    AP_GROUPINFO("CLAW", 15, AP_HexRuped_CHANNEL_Params, claw_channel, -1),

    AP_GROUPINFO("RLL_MX", 16, AP_HexRuped_CHANNEL_Params, throttle_roll_max, 40),
    AP_GROUPINFO("PIT_MX", 17, AP_HexRuped_CHANNEL_Params, throttle_pitch_max, 40),
    AP_GROUPINFO("YAW_MX", 18, AP_HexRuped_CHANNEL_Params, throttle_yaw_max, 20),

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_HexRuped_SYS_Params::var_info[] = {
    // @Param: COXA
    // @DisplayName: 髋节长度
    // @Description: 髋关节(COXA)连杆长度,用于逆运动学
    // @Units: mm
    // @Range: 1 300
    // @User: Standard
    AP_GROUPINFO("COXA", 1, AP_HexRuped_SYS_Params, COXA_LEN, COXA_LEN_DEFAULT),

    // @Param: FEMUR
    // @DisplayName: 股节长度
    // @Description: 股关节(FEMUR)连杆长度,用于逆运动学
    // @Units: mm
    // @Range: 1 400
    // @User: Standard
    AP_GROUPINFO("FEMUR", 2, AP_HexRuped_SYS_Params, FEMUR_LEN, FEMUR_LEN_DEFAULT),

    // @Param: TIBIA
    // @DisplayName: 胫节长度
    // @Description: 胫关节(TIBIA)连杆长度,用于逆运动学,也决定站立初始高度
    // @Units: mm
    // @Range: 1 400
    // @User: Standard
    AP_GROUPINFO("TIBIA", 3, AP_HexRuped_SYS_Params, TIBIA_LEN, TIBIA_LEN_DEFAULT),

    // @Param: FLEN
    // @DisplayName: 机身前后长度
    // @Description: 机体坐标系X方向长度,前后髋关节间距
    // @Units: mm
    // @Range: 1 600
    // @User: Standard
    AP_GROUPINFO("FLEN", 4, AP_HexRuped_SYS_Params, FRAME_LEN, FRAME_LEN_DEFAULT),

    // @Param: FWID
    // @DisplayName: 机身左右宽度
    // @Description: 机体坐标系Y方向宽度,左右髋关节间距
    // @Units: mm
    // @Range: 1 600
    // @User: Standard
    AP_GROUPINFO("FWID", 5, AP_HexRuped_SYS_Params, FRAME_WIDTH, FRAME_WIDTH_DEFAULT),

    // @Param: ALPH_A
    // @DisplayName: USL_BV2股胫补偿边A
    // @Description: 仅 HEX_CLASS=USL_BV2 时生效。与 ALPH_B 一起计算股/胫安装补偿角, alpha=atan(A/B)
    // @Units: mm
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("ALPH_A", 6, AP_HexRuped_SYS_Params, Alpha_A, 55),

    // @Param: ALPH_B
    // @DisplayName: USL_BV2股胫补偿边B
    // @Description: 仅 HEX_CLASS=USL_BV2 时生效。与 ALPH_A 一起计算股/胫安装补偿角, alpha=atan(A/B)
    // @Units: mm
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("ALPH_B", 7, AP_HexRuped_SYS_Params, Alpha_B, 50),

    // @Param: F_YAW
    // @DisplayName: 右前腿安装偏航角
    // @Description: 右前腿髋关节安装偏航角。机体坐标:0°朝右(+Y),正值朝前(+X)。左前腿由镜像得到 180-F_YAW
    // @Units: deg
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("F_YAW", 8, AP_HexRuped_SYS_Params, FRONT_YAW, 45),

    // @Param: M_YAW
    // @DisplayName: 右中腿安装偏航角
    // @Description: 右中腿髋关节安装偏航角。默认0°为纯侧向。左中腿由镜像得到 180-M_YAW
    // @Units: deg
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("M_YAW", 9, AP_HexRuped_SYS_Params, MIDDLE_YAW, 0),

    // @Param: R_YAW
    // @DisplayName: 右后腿安装偏航角
    // @Description: 右后腿髋关节安装偏航角。默认-45°朝后右侧。左后腿由镜像得到 -180-R_YAW
    // @Units: deg
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("R_YAW", 10, AP_HexRuped_SYS_Params, REAR_YAW, -45),

    // @Param: MID_X
    // @DisplayName: 中腿髋关节前后位置
    // @Description: 左右中腿髋关节相对机体中心的X坐标,向前为正,左右中腿共用
    // @Units: mm
    // @Range: -300 300
    // @User: Standard
    AP_GROUPINFO("MID_X", 11, AP_HexRuped_SYS_Params, MIDDLE_X, 0),

    // @Param: CLAW_H
    // @DisplayName: 飞行收爪高度
    // @Description: 朝下测距高度门。低于或等于此高度(或无测距)强制默认爪展开以便降落;高于此高度才允许默认爪/横爪和合爪通道
    // @Units: cm
    // @Range: 5 200
    // @User: Standard
    AP_GROUPINFO("CLAW_H", 12, AP_HexRuped_SYS_Params, CLAW_H, 40),

    AP_GROUPEND
};

const AP_Param::GroupInfo AP_HexRuped_CTRL_Params::var_info[] = {
    AP_SUBGROUPINFO(pitch_pid, "PIT_", 1, AP_HexRuped_CTRL_Params, AC_PID),
    AP_SUBGROUPINFO(roll_pid, "ROL_", 2, AP_HexRuped_CTRL_Params, AC_PID),
    AP_SUBGROUPINFO(yaw_pid, "YAW_", 3, AP_HexRuped_CTRL_Params, AC_PID),
    AP_SUBGROUPINFO(pos_y_pid, "POY_", 4, AP_HexRuped_CTRL_Params, AC_PID),

    AP_GROUPEND
};

AP_HexRuped_Params::AP_HexRuped_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

AP_HexRuped_CHANNEL_Params::AP_HexRuped_CHANNEL_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

AP_HexRuped_SYS_Params::AP_HexRuped_SYS_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
