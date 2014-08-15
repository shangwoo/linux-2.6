/*  linux/arch/arm/mach-as9260/usb.c
 *
 *  *  Copyright (C) 2006-2014 Alpscale
 *
 *
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/serial_8250.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/input.h>
#include <asm/hardware/iomd.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <linux/usb/musb.h>
#include <linux/usb/otg.h>

#ifdef CONFIG_USB_SUPPORT
extern struct musb_hdrc_platform_data usb_data;

/************************************* 
 *open USB Controller Power and CLK
 *
 */
void __init setup_usb(void)
{
    unsigned mA = 500;/*power supplied 500mA*/
    unsigned potpgt_msec = 8;/*ms VBUS on till power good*/

    as3310_writel((as3310_readl(HW_PDRUNCFG)&0xFFFFF8FF),HW_PDRUNCFG);//USB0,USB1 USBPLL Power open
    as3310_writel((as3310_readl(HW_PDRUNCFG)|0x100),HW_PDRUNCFG);//close USB 0 Power
        
    as3310_writel(1<<7,HW_AHBCLKCTRL0+8);//close usb0 clk

    as3310_writel(1<<8,HW_AHBCLKCTRL0+4);//open usb1 clk
 
    usb_data.power = mA ;
	usb_data.potpgt = potpgt_msec ;

  
    return;
}
#endif
