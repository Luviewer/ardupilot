# STM32F765-CUAV-V5 BSP (ArduPilot RTT)

**说明**：本目录为 pogo-apm 的 **RTT BSP 唯一源**（CUAV V5）。构建时会 deploy 到 `modules/rt-thread/bsp/stm32/stm32f765-cuav-v5`，请在此目录修改 BSP 配置与板级代码。

---

本 BSP 基于 STM32F767-Nucleo，用于 **CUAV V5** 与 ArduPilot RT-Thread 集成。使用 ArduPilot bootloader 时，固件从 **0x08008000** 启动（前 32KB 预留给 bootloader）。

- **链接**：`board/linker_scripts/link.lds` 已配置 `ORIGIN = 0x08008000`，`LENGTH = 2016k`。
- **VTOR**：`board/rt_board_init.c` 在 `rt_hw_board_init()` 开头设置 `SCB->VTOR = 0x08008000U`。
- **全量 ArduPilot 构建**：先执行 `./pkgs_update_manual.sh` 拉取依赖，再在 BSP 目录下执行  
  `ARDUPILOT_FULL=1 RTT_ROOT=<pogo-apm>/modules/rt-thread RTT_EXEC_PATH=<gcc-bin/> scons -j8`。  
  工具链路径若未安装到 `/opt/gcc-arm-none-eabi-7-2017-q4-major/bin/`，请设置 `RTT_EXEC_PATH`。
- **校验**：编译得到 `rt-thread.elf` 后可用 `arm-none-eabi-objdump -h rt-thread.elf` 确认 `.text` 的 VMA 为 `0x08008000`。

### 上传固件（类似 waf upload）

使用 **ArduPilot/PX4 串口 bootloader**（板子需已刷该 bootloader，且上电时进入 bootloader）：

1. 在 **pogo-apm 仓库根目录** 执行，将 `rtthread.bin` 打成 .apj 并上传：
   ```bash
   python3 Tools/scripts/rtt_bin_to_apj.py \
     modules/rt-thread/bsp/stm32/stm32f765-cuav-v5/rtthread.bin \
     --board rtt_cuav_v5 -o arducopter.apj --upload
   ```
2. 若需指定串口：加上 `--port /dev/ttyACM0`（或 Windows `COM3` 等）。
3. 仅生成 .apj 不烧录：去掉 `--upload`；之后可手动执行  
   `python3 Tools/scripts/uploader.py arducopter.apj [--port /dev/ttyACM0]`。

**通过 JTAG/SWD 直接烧录**（不依赖 bootloader，写 0x08008000）：

- OpenOCD 示例：  
  `openocd -f interface/stlink.cfg -f target/stm32f7x.cfg -c "program rt-thread.elf 0x08008000 verify reset"`
- pyocd 示例：  
  `pyocd flash rtthread.bin --target stm32f765xx --base-address 0x08008000`

[中文](README_zh.md)

## MCU: STM32F767ZI @216MHz, 2MB FLASH, 512KB RAM

The STM32F765xx, STM32F767xx, STM32F768Ax and STM32F769xx devices are based on the high-performance Arm® Cortex®-M7 32-bit RISC core operating at up to 216 MHz frequency. The Cortex®-M7 core features a floating point unit (FPU) which supports Arm® double-precision and single-precision data-processing instructions and data types. It also implements a full set of DSP instructions and a memory protection unit (MPU) which enhances the application security.

The STM32F765xx, STM32F767xx, STM32F768Ax and STM32F769xx devices incorporate high-speed embedded memories with a Flash memory up to 2 Mbytes, 512 Kbytes of SRAM (including 128 Kbytes of Data TCM RAM for critical real-time data), 16 Kbytes of instruction TCM RAM (for critical real-time routines), 4 Kbytes of backup SRAM available in the lowest power modes, and an extensive range of enhanced I/Os and peripherals connected to two APB buses, two AHB buses, a 32-bit multi-AHB bus matrix and a multi layer AXI interconnect supporting internal and external memories access.
All the devices offer three 12-bit ADCs, two DACs, a low-power RTC, twelve general-purpose 16-bit timers including two PWM timers for motor control, two general-purpose 32-bit timers, a true random number generator (RNG). They also feature standard and advanced communication interfaces.
Advanced peripherals include two SDMMC interfaces, a flexible memory control (FMC) interface, a Quad-SPI Flash memory interface, a camera interface for CMOS sensors.
The STM32F765xx, STM32F767xx, STM32F768Ax and STM32F769xx devices operate in the –40 to +105 °C temperature range from a 1.7 to 3.6 V power supply. Dedicated supply inputs for USB (OTG_FS and OTG_HS) and SDMMC2 (clock, command and 4-bit data) are available on all the packages except LQFP100 for a greater power supply choice.
The supply voltage can drop to 1.7 V with the use of an external power supply supervisor. A comprehensive set of power-saving mode allows the design of low-power applications.
The STM32F765xx, STM32F767xx, STM32F768Ax and STM32F769xx devices offer devices in 11 packages ranging from 100 pins to 216 pins. The set of included peripherals changes with the device chosen.
These features make the STM32F765xx, STM32F767xx, STM32F768Ax and STM32F769xx microcontrollers suitable for a wide range of applications.

#### KEY FEATURES

- Core: Arm® 32-bit Cortex®-M7 CPU with DPFPU, ART Accelerator™ and L1-cache: 16 Kbytes I/D cache, allowing 0-wait state execution from embedded Flash and external memories, up to 216 MHz, MPU, 462 DMIPS/2.14 DMIPS/MHz (Dhrystone 2.1), and DSP instructions.
- Memories
  - Up to 2 Mbytes of Flash memory organized into two banks allowing read-while-write
  - SRAM: 512 Kbytes (including 128 Kbytes of data TCM RAM for critical real-time data) + 16 Kbytes of instruction TCM RAM (for critical real-time routines) + 4 Kbytes of backup SRAM
  - Flexible external memory controller with up to 32-bit data bus: SRAM, PSRAM, SDRAM/LPSDR SDRAM, NOR/NAND memories
- Dual mode Quad-SPI
- Graphics
  - Chrom-ART Accelerator™ (DMA2D), graphical hardware accelerator enabling enhanced graphical user interface
  - Hardware JPEG codec
  - LCD-TFT controller supporting up to XGA resolution
  - MIPI® DSI host controller supporting up to 720p 30 Hz resolution
- Clock, reset and supply management
  - 1.7 V to 3.6 V application supply and I/Os
  - POR, PDR, PVD and BOR
  - Dedicated USB power
  - 4-to-26 MHz crystal oscillator
  - Internal 16 MHz factory-trimmed RC (1% accuracy)
  - 32 kHz oscillator for RTC with calibration
  - Internal 32 kHz RC with calibration
- Low-power
  - Sleep, Stop and Standby modes
  - VBAT supply for RTC, 32×32 bit backup registers + 4 Kbytes backup SRAM
- 3×12-bit, 2.4 MSPS ADC: up to 24 channels
- Digital filters for sigma delta modulator (DFSDM), 8 channels / 4 filters
- 2×12-bit D/A converters
- General-purpose DMA: 16-stream DMA controller with FIFOs and burst support

- Up to 18 timers: up to thirteen 16-bit (1x low- power 16-bit timer available in Stop mode) and two 32-bit timers, each with up to 4 IC/OC/PWM or pulse counter and quadrature (incremental) encoder input. All 15 timers running up to 216 MHz. 2x watchdogs, SysTick timer
- Debug mode
  - SWD & JTAG interfaces
  - Cortex®-M7 Trace Macrocell™
- Up to 168 I/O ports with interrupt capability
  - Up to 164 fast I/Os up to 108 MHz
  - Up to 166 5 V-tolerant I/Os
- Up to 28 communication interfaces
  - Up to 4 I2C interfaces (SMBus/PMBus)
  - Up to 4 USARTs/4 UARTs (12.5 Mbit/s, ISO7816 interface, LIN, IrDA, modem control)
  - Up to 6 SPIs (up to 54 Mbit/s), 3 with muxed simplex I2S for audio
  - 2 x SAIs (serial audio interface)
  - 3 x CANs (2.0B Active) and 2 x SDMMCs
  - SPDIFRX interface
  - HDMI-CEC
  - MDIO slave interface
- Advanced connectivity
  - USB 2.0 full-speed device/host/OTG controller with on-chip PHY
  - USB 2.0 high-speed/full-speed device/host/OTG controller with dedicated DMA, on-chip full-speed PHY and ULPI
  - 10/100 Ethernet MAC with dedicated DMA: supports IEEE 1588v2 hardware, MII/RMII
- 8- to 14-bit camera interface up to 54 Mbyte/s
- True random number generator
- CRC calculation unit
- RTC: subsecond accuracy, hardware calendar
- 96-bit unique ID



## Read more

|                          Documents                           |                         Description                          |
| :----------------------------------------------------------: | :----------------------------------------------------------: |
| [STM32_Nucleo-144_BSP_Introduction](../docs/STM32_Nucleo-144_BSP_Introduction.md) | How to run RT-Thread on STM32 Nucleo-144 boards (**Must-Read**) |
| [STM32F767ZI ST Official Website](https://www.st.com/en/microcontrollers-microprocessors/stm32f767zi.html#documentation) |          STM32F767ZI datasheet and other resources           |



## Maintained By

[e31207077](https://github.com/e31207077)  <e31207077@yahoo.com.tw>



## Translated By

Meco Man @ RT-Thread Community

> jiantingman@foxmail.com 
>
> https://github.com/mysterywolf
