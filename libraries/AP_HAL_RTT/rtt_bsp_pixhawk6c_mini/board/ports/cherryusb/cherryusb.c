/*
 * CherryUSB CDC ACM board init for Pixhawk6C Mini (USB OTG_FS PA11/PA12)
 */
#include "board.h"
#include "rtthread.h"

#ifdef RT_CHERRYUSB_DEVICE_TEMPLATE_CDC_ACM_CHARDEV
static int rt_hw_cherryusb_cdc_init(void)
{
    extern void cdc_acm_chardev_init(uint8_t busid, uintptr_t reg_base);
    cdc_acm_chardev_init(0, USB_OTG_FS_PERIPH_BASE);
    return 0;
}
INIT_COMPONENT_EXPORT(rt_hw_cherryusb_cdc_init);
#endif
