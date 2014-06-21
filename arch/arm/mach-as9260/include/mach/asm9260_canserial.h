/***********************************************
 *  linux/arch/arm/mach-as9260/asm9260_canserial.h
 *  Copyright (C) 2014 Alpscale
 *
 */
#include <linux/device.h>
#include <linux/platform_device.h>

# define HW_CAN0 0x8004C000
# define HW_CAN1 0x80050000



struct asm9260_data {
	u32 clock;	/* CAN bus oscillator frequency in Hz */

	u8 ocr;		/* output control register */
	u8 cdr;		/* clock divider register */
        void __iomem	*regs;
};
extern void __init asm9260_register_can(unsigned id, unsigned portnr, unsigned pins);
extern void __init asm9260_add_device_can(void);
