/*
 * CherryUSB usb_config.h for Pixhawk6C Mini (STM32H7 OTG_FS DWC2)
 */
#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

#include "rtthread.h"

#define CONFIG_USB_PRINTF(...) rt_kprintf(__VA_ARGS__)

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_INFO
#endif

#define CONFIG_USB_PRINTF_COLOR_ENABLE

#ifndef CONFIG_USB_ALIGN_SIZE
#define CONFIG_USB_ALIGN_SIZE 32
#endif

#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))
/* STM32H7 has cache; enable dcache ops for USB coherence */
#define CONFIG_USB_DCACHE_ENABLE

/* USB Device Stack */
#ifndef CONFIG_USBDEV_MAX_BUS
#define CONFIG_USBDEV_MAX_BUS 1
#endif

#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512
#endif

#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 6
#endif

/* DWC2 Configuration for STM32H7 OTG_FS */
#define CONFIG_USB_DWC2_TX0_FIFO_SIZE (64 / 4)
#define CONFIG_USB_DWC2_TX1_FIFO_SIZE (64 / 4)
#define CONFIG_USB_DWC2_TX2_FIFO_SIZE (64 / 4)
#define CONFIG_USB_DWC2_TX3_FIFO_SIZE (64 / 4)

/* USB Host (required by usbh_core.h included from glue) */
#ifndef CONFIG_USBHOST_DEV_NAMELEN
#define CONFIG_USBHOST_DEV_NAMELEN 16
#endif
#define CONFIG_USBHOST_MAX_RHPORTS          1
#define CONFIG_USBHOST_MAX_EXTHUBS          1
#define CONFIG_USBHOST_MAX_EHPORTS          4
#define CONFIG_USBHOST_MAX_INTERFACES       8
#define CONFIG_USBHOST_MAX_INTF_ALTSETTINGS 8
#define CONFIG_USBHOST_MAX_ENDPOINTS        4

#endif
