#pragma once

/**
 * RT-Thread (RTT) board HAL header.
 * Selected when CONFIG_HAL_BOARD == HAL_BOARD_RTT.
 * HAL implementation is provided by AP_HAL_RTT (libraries/AP_HAL_RTT).
 */

#define HAL_BOARD_NAME "RTT"

#define HAL_BOARD_SUBTYPE_RTT_GENERIC 7000
#ifndef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_RTT_GENERIC
#endif

#define HAL_INS_DEFAULT HAL_INS_NONE

#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_MEM_CLASS HAL_MEM_CLASS_192

#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE 16384
#endif
#define HAL_STORAGE_SIZE_AVAILABLE HAL_STORAGE_SIZE

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 2048
#endif
/* No QuadSPI on generic RTT; disable scripting for first port (no filesystem yet) */
#ifndef HAL_USE_QUADSPI
#define HAL_USE_QUADSPI 0
#endif
#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED 0
#endif
/* Define early so any config that uses them sees them (include order) */
#ifndef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 0
#endif
#ifndef AP_SIM_ENABLED
#define AP_SIM_ENABLED 0
#endif
#ifndef AP_HAL_UARTDRIVER_ENABLED
#define AP_HAL_UARTDRIVER_ENABLED 1
#endif
#ifndef HAL_OS_LITTLEFS_IO
#define HAL_OS_LITTLEFS_IO 0
#endif
#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 1
#endif
#ifndef HAL_WITH_DSP
#define HAL_WITH_DSP 0
#endif
/* GyroFFT requires DSP; disable when HAL_WITH_DSP is 0 to avoid missing AP_HAL::DSP::FrequencyPeak */
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif
#ifndef HAL_CANFD_SUPPORTED
#define HAL_CANFD_SUPPORTED 0
#endif

#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE 0
#endif
#ifndef HAL_HAVE_SERVO_VOLTAGE
#define HAL_HAVE_SERVO_VOLTAGE 0
#endif
#ifndef HAL_HAVE_SAFETY_SWITCH
#define HAL_HAVE_SAFETY_SWITCH 0
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY "/APM"
#endif
#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#endif

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif
#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

#ifndef HAL_WITH_MCU_MONITORING
#define HAL_WITH_MCU_MONITORING 0
#endif

#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 0
#endif

/* Macros used in #if (not #ifdef) in HAL headers; must be defined before
 * rtt.h pulls in AP_HAL.h via Semaphores.h, since AP_HAL_Boards.h is
 * not yet fully processed at that point (-Werror=undef). */
#ifndef AP_CRASHDUMP_ENABLED
#define AP_CRASHDUMP_ENABLED 0
#endif
#ifndef HAL_ENABLE_DFU_BOOT
#define HAL_ENABLE_DFU_BOOT 0
#endif
#ifndef HAL_USE_WSPI_DEFAULT_CFG
#define HAL_USE_WSPI_DEFAULT_CFG 0
#endif

#ifdef __cplusplus
/* AP_Common.h defines WARN_IF_UNUSED etc.; include before HAL so HAL headers see it */
#include <AP_Common/AP_Common.h>
/* Requires AP_HAL_RTT (libraries/AP_HAL_RTT) to be present */
#include <AP_HAL_RTT/Semaphores.h>
#define HAL_Semaphore RTT::Semaphore
#define HAL_BinarySemaphore RTT::BinarySemaphore
#endif
