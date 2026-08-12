# Scorpio ALT_HOLD 与 LOITER 联合仿真报告

测试日期：2026-08-12

## 测试对象

- ArduPilot：`ArduCopter V4.7.0-dev`
- 机架：`FRAME_CLASS=18`，启动显示 `Frame: SCORPIO/tilt-tri`
- Gazebo：Classic 11，Scorpio六足倾转三旋翼模型
- 飞行执行器：16路Gazebo PWM中的第1～6路
- 六足执行器：`vcan0` 上18路DroneCAN关节指令

## 测试前修正

1. 六足运动轴由RC1～RC4迁移到RC9、RC13～RC16，避免RC3飞行油门同时触发行走。
2. 扩展RC通道尚未初始化而返回0时，六足控制器按1500中位处理。
3. 实测Gazebo模型总质量为5.816 kg，原三旋翼最大总推力约48 N，小于约57 N重力，无法起飞。
4. 将单旋翼力常数改为 `5.2e-5`，力矩常数同比改为 `1.4955e-6`，保持力矩/推力比不变。
5. 设置 `MOT_THST_HOVER=0.50`。
6. 删除物理反扭矩前馈参数。偏航由PID闭环和三个倾转舵机控制，旋翼反扭矩作为外部扰动自动抑制。

修改 `model.rsdf` 后必须运行：

```bash
cd /home/pix/uavros_ws/src/uav_simulator/uav_gazebo/models/Scorpio/models/scorpio
erb model.rsdf > model.sdf.tmp && mv model.sdf.tmp model.sdf
```

## ALT_HOLD结果

流程：冷启动、飞行腿姿态、切换 `ALT_HOLD`、解锁、RC3=1650起飞，约2 m时RC3回中。
因上升惯性，最终定高约3.02 m。

- 稳定段平均高度：3.0175 m
- 稳定段最低高度：3.0080 m
- 稳定段最高高度：3.0309 m
- 高度峰峰值：0.0229 m
- 稳态三电机转速约：550、707、551 rad/s
- 倾转舅机保持接近中位

结论：ALT_HOLD高度环通过。该模式不控制水平位置，长时间水平漂移属于预期行为；
切换LOITER后可消除漂移。

## LOITER结果

等待控制台显示两个EKF实例均 `is using GPS` 后切换LOITER。测试入口带有已有水平速度，
用于同时验证刹车和定点能力。

- 30秒观测窗口位置变化：X=-0.0184 m，Y=-0.0784 m
- 最终水平速度：X=0.0018 m/s，Y=-0.0038 m/s
- 高度峰峰值：0.0092 m
- 稳态横滚/俯仰约：0.24°/-0.23°
- 另一次大幅前向输入测试中，速度达到约1.67 m/s；摇杆回中后LOITER能够刹停并重新锁点

结论：LOITER水平速度闭环、位置闭环和高度闭环均通过。

## 注意事项

- 冷启动后GPS已有3D Fix不等于EKF已融合GPS。必须等待控制台出现
  `EKF3 IMU0 is using GPS` 和 `EKF3 IMU1 is using GPS`，否则LOITER会提示 `requires position`。
- 当前参数是Gazebo首轮稳定参数，不可直接视为实物最终参数。实物需要核对质量、推力曲线和
  转动惯量，并调节姿态PID；不要求标定旋翼反扭矩。
- 当前LAND流程尚未作为本次验收项目；ALT_HOLD和LOITER通过不代表自动降落已经完成验证。

## 测试记录

- ROS bag：`recordings/scorpio_alt_hold_loiter_final.bag`

ROS bag包含 `/gazebo/model_states`、`/gazebo/command/prop_speed`和
`/gazebo/command/tilt_pos`，总时长约129秒，共383146条消息。
