/*
 * USB OTG_FS IRQ for Pixhawk6C Mini (CherryUSB DWC2).
 *
 * The upstream usb_glue_st.c OTG_FS_IRQHandler unconditionally calls
 * g_usb_dwc2_irq[0]() without a null check. If the VBUS/OTG_FS interrupt
 * fires before cdc_acm_chardev_init() has registered the handler (e.g. the
 * USB cable was already plugged in at power-on), it dereferences a null
 * function pointer and causes a HardFault — so the board boots but USB never
 * enumerates and the RTT shell is silent.
 *
 * This file provides a strong OTG_FS_IRQHandler that:
 *   1. Calls USBD_IRQHandler(0) — the real CherryUSB DWC2 ISR entry point
 *      (defined in usb_dc_dwc2.c, safe to call at any time).
 *   2. Does NOT touch g_usb_dwc2_irq[] which is private to usb_glue_st.c.
 */
#ifdef BSP_USING_USB_DEVICE

extern void USBD_IRQHandler(uint8_t busid);

void OTG_FS_IRQHandler(void)
{
    USBD_IRQHandler(0);
}

#endif /* BSP_USING_USB_DEVICE */
