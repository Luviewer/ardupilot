# Scorpio 六足控制

`AP_HexRuped` 是天蝎座分支独立的六足地面控制模块，不再复用
`AP_QuadRuped` 文件夹。每条腿包含髋、股、胫三个关节，共输出 18 路舵机命令。

## 腿编号与步态

- 腿编号：`RF/RM/RB/LF/LM/LB`（右前、右中、右后、左前、左中、左后）。
- Tripod：`RF + LM + RB` 与 `LF + RM + LB` 两组三角支撑交替摆动，作为默认步态。
- Wave：`RF → RM → RB → LB → LM → LF`，每次仅摆动一条腿，用于低速行走。
- 步态选择通道低档选择 Tripod，高档选择 Wave。

六足结构在这两种步态下都有明确的静态支撑多边形，因此删除了四足代码中的
相位式重心迁移步态。通用 `center_offset` 只保留为静态标定和未来地形补偿接口，
当前两种步态均保持为零。姿态闭环产生的 roll/pitch 机体补偿仍然保留。

## 参数与输出

- 参数前缀：`HEX_`。
- 步态参数组：`HEX_TRI_`、`HEX_WAVE_`。
- 腿参数组包含新增的 `HEX_RM_` 和 `HEX_LM_`。
- 六足安装几何参数：`HEX_SYS_F_YAW`、`HEX_SYS_M_YAW`、
  `HEX_SYS_R_YAW`、`HEX_SYS_MID_X`。
- DroneCAN 与本地 PWM 均按六腿三关节输出 18 路命令。
