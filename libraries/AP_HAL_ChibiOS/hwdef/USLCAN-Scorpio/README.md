# USLCAN-Scorpio

`USLCAN-Scorpio` 是用于 Scorpio 六足机器人的六路幻尔串行总线舵机
DroneCAN 转接板。板载 MCU 为 STM32F405RGT6。

## 硬件依据

- 实物晶振为 16 MHz。`PWR.pdf` 第 2 页的 X1 标注为 8 MHz，与当前实物不符，
  因此 `hwdef.dat` 和 `hwdef-bl.dat` 均以实物频率为准。
- CAN1 使用 PA11/PA12，原理图中 R14 为板载 120 欧终端电阻。
- 默认使用 DroneCAN 节点 ID 20，无 DNA 服务器时也可直接启动。
- 红色状态灯连接 PA6，低电平点亮。
- 为兼容现有 Bootloader，暂时沿用 USLF4-CAN 的 `APJ_BOARD_ID=1014`。

## 舵机串口顺序

PWR 板在机身平面内旋转 180 度安装。串口枚举顺序经过置换，使 DroneCAN
18 路数组仍保持 `RF、RB、LB、LF、RM、LM` 的逻辑腿顺序。

| 逻辑腿序 | 原 PWR 连接器 | MCU 外设 |
|---|---|---|
| RF（右前） | SERVO4 | USART3 |
| RB（右后） | SERVO6 | UART4 |
| LB（左后） | SERVO1 | UART5 |
| LF（左前） | SERVO3 | USART6 |
| RM（右中） | SERVO5 | USART2 |
| LM（左中） | SERVO2 | USART1 |

每路串口内部的舵机 ID 顺序固定为 `1=Coxa`、`2=Femur`、`3=Tibia`。

## 构建

```bash
./waf configure --board USLCAN-Scorpio
./waf AP_Periph
```

Bootloader 与应用必须使用相同的晶振频率和板号。
