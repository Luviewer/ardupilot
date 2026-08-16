# Scorpio 联合仿真

Scorpio 使用两条独立执行器链路：

- Gazebo UDP 保持 ArduPilot 标准16路格式：当前预留第1–3路给三个旋翼、第4–6路给三个倾转舵机，其余通道仍可用于飞行系统。
- 六足18个关节使用 `vcan0` 上的 DroneCAN `com.usl.ServoCmd`。

腿部功能不再映射到 `SERVO1`～`SERVO32`，因此不会占用或挤压飞行侧的16路PWM。

飞行与行走遥控输入也已隔离：飞控保留RC1～RC4，六足使用RC9和RC13～RC16。
未收到扩展通道数据时，六足会把PWM零值安全解释为中位1500，避免冷启动自动行走。

## 三旋翼倾转分配器

Scorpio 使用独立的 `AP_MotorsScorpio`，对应 `FRAME_CLASS=18`。飞行输出顺序固定为：

1. 前右旋翼，俯视顺时针（CW）。
2. 后旋翼，俯视逆时针（CCW）。
3. 前左旋翼，俯视顺时针（CW）。
4. 前右倾转舵机。
5. 后倾转舵机。
6. 前左倾转舵机。

三路旋翼转速分配总升力、横滚力矩和俯仰力矩；三个倾转机构分配 X/Y
水平力和偏航力矩。分配器使用三个旋翼的平面 XY 坐标自动计算并归一化各轴力臂。

关键可调参数如下：

- `MOT_SC_F_ANG`、`MOT_SC_R_ANG`：前、后倾转轴安装角。
- `MOT_SC_TILT_MAX`：最大倾转角，默认45度。
- `MOT_SC_XY_GAIN`：X/Y水平力增益。
- `MOT_SC_FR_X/Y`：前右旋翼中心相对机体原点的平面位置。
- `MOT_SC_FL_X/Y`：前左旋翼中心相对机体原点的平面位置。
- `MOT_SC_R_X/Y`：后旋翼中心相对机体原点的平面位置。

三组位置使用 ArduPilot 机体 FRD 坐标系：X向前、Y向右，单位为米。
这些是机械测量参数，
不是试飞调节增益。混控器根据实际坐标自动计算悬停升力分配、横滚/俯仰/
偏航力臂并归一化，允许实物存在小的左右不对称。

旋翼反扭矩不设置用户标定参数，与ArduPilot其他电机类型一致：它作为偏航扰动，
由偏航PID和三个倾转舵机的闭环控制自动消除。偏航分配尺度由旋翼安装几何自动
归一化，也不需要标定等效力臂；实物只按常规方法调节 `ATC_RAT_YAW_*` 参数。

当前默认值来自已有 CAD/RSDF 尺寸；得到更准确的实物坐标后直接更新上述六个参数。

首次启动或重启系统后创建虚拟CAN接口：

```bash
sudo modprobe vcan
sudo modprobe can_raw
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

若接口已存在，`ip link add` 提示 `File exists` 可以忽略。
当前 WSL2 内核若提示 `Module vcan not found`，需要先为当前内核安装 `vcan.ko`；
WSL 内核升级后可能需要重新安装对应版本的模块。

## 启动顺序

必须先启动 Gazebo，确认 Scorpio 模型和 `ArduRotorScorpio` 插件加载完成，再启动
ArduPilot。这样 SITL 首次发送状态时不会因为 Gazebo 尚未监听而反复重连。

### 1. 启动 Gazebo

```bash
cd /home/pix/uavros_ws
source devel/setup.bash
# 修改model.rsdf后必须重新生成运行时model.sdf。
cd src/uav_simulator/uav_gazebo/models/Scorpio/models/scorpio
erb model.rsdf > model.sdf.tmp && mv model.sdf.tmp model.sdf
cd /home/pix/uavros_ws
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
roslaunch uav_gazebo spawn.launch world_name:=Scorpio
```

启动日志应出现以下内容，且不能出现 `failed to open SocketCAN interface`：

```text
motor_num:3
servo_num:3
coxa_num:6
femur_num:6
tibia_num:6
```

### 2. 启动 ArduPilot

```bash
cd /home/pix/firmare/llw-apm/Scorpio
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
  --console --add-param=Scorpio.param
```

等待控制台出现：

```text
AP: Hexapod tripod gait init
AP: ArduPilot Ready
```

Gazebo随后应出现：

```text
[scorpio] received DroneCAN com.usl.ServoCmd frames.
[scorpio] decoded all 18 SocketCAN leg commands.
```

### 3. 进入交替三角步态并前进

在MAVProxy控制台输入：

```text
rc 10 1000
rc 11 1000
rc 14 1500
rc 13 1500
rc 9 1600
```

- RC10低位：行走模式。
- RC11低位：交替三角步态。
- RC14中位：无偏航命令。
- RC13中位：无横移命令。
- RC9为1600：使用20%的前进输入，当前验证速度约0.1 m/s。

单独验证横移时保持 `RC9=1500`、`RC14=1500`，再用 `RC13>1550` 或
`RC13<1450` 分别给出两个方向的开环横移指令；回到 `RC13=1500` 后按相同的安全
收步逻辑停止。

停止行走：

```text
rc 9 1500
```

正常回中后，控制器会先让当前摆动腿落地，再用半个步态周期的 smoothstep 曲线将
六条腿收回对称站姿；不会保持悬空姿态，也不会立即阶跃归零。RC失联时为保证安全，
仍会立即归零。

`HEX_CH_LIFT=20` 是当前稳定步态的关键参数，表示摆线轨迹相对站姿的**实际最大抬腿
高度为20 mm**（不再产生旧实现的40 mm峰值）；旧默认值
`-1` 会令摆动腿向下压，造成拖脚、横漂和偏航。`HEX_CH_HGH=0` 表示不额外改变
机身高度。

## 录像、截图和测量

Gazebo右上角摄像机图标右侧的录像按钮可打开录像菜单：

1. 选择 `MP4` 开始录像。
2. 再次打开录像菜单，按 `Stop`。
3. 在保存窗口输入文件名并保存；必须按 `Stop`，直接关闭GUI会留下没有MP4索引的临时文件。

测量机体世界位姿：

```bash
rosservice call /gazebo/get_model_state scorpio world
```

检查18路腿部CAN流量：

```bash
ip -s link show vcan0
```

## 2026-08-12交替三角步态验证

使用 `RC9=1600`、`HEX_CH_LIFT=20`、`HEX_CH_HGH=0`：

- 冷启动复现实验约4.6秒仿真时间前进0.483 m。
- 横向偏移0.036 m，最终偏航约3.2度；偏航没有持续发散。
- 机体高度保持在0.141～0.142 m，波动小于1 mm。
- 第二段6.56秒实验前进0.265 m，横向偏移0.022 m，航向基本不变。
- 两次实验均无倾覆、支撑塌陷或CAN丢帧。

## 停止顺序

1. 在MAVProxy输入 `rc 9 1500`。
2. 若正在录像，先按录像菜单中的 `Stop` 并保存。
3. 使用 `Ctrl-C`停止ArduPilot。
4. 使用 `Ctrl-C`停止Gazebo。
