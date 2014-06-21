/* arch/arm/mach-s3c2410/include/mach/regs-watchdog.h
 *
 * Copyright (c) 2003 Simtec Electronics <linux@simtec.co.uk>
 *		      http://www.simtec.co.uk/products/SWLINUX/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2410 Watchdog timer control
*/


#ifndef __ASM_ARCH_REGS_WATCHDOG_H
#define __ASM_ARCH_REGS_WATCHDOG_H

#define S3C_WDOGREG(x) ((x) + S3C_VA_WATCHDOG)

#define S3C2410_WTCON	   S3C_WDOGREG(0x00)
#define S3C2410_WTDAT	   S3C_WDOGREG(0x04)
#define S3C2410_WTCNT	   S3C_WDOGREG(0x08)

/* the watchdog can either generate a reset pulse, or an
 * interrupt.
 */

#define S3C2410_WTCON_RSTEN   (0x01)
#define S3C2410_WTCON_INTEN   (1<<2)
#define S3C2410_WTCON_ENABLE  (1<<5)

#define S3C2410_WTCON_DIV16   (0<<3)
#define S3C2410_WTCON_DIV32   (1<<3)
#define S3C2410_WTCON_DIV64   (2<<3)
#define S3C2410_WTCON_DIV128  (3<<3)

#define S3C2410_WTCON_PRESCALE(x) ((x) << 8)
#define S3C2410_WTCON_PRESCALE_MASK (0xff00)

#endif /* __ASM_ARCH_REGS_WATCHDOG_H */

#define HW_PRESETCTRL0        0x80040000
#define HW_PRESETCTRL1        0x80040010
#define HW_AHBCLKCTRL0        0x80040020
#define HW_AHBCLKCTRL1        0x80040030
#define HW_WDTCLKSEL          0x80040160
#define HW_WDTCLKUEN          0x80040164
#define HW_WDTCLKDIV          0x800401EC
#define HW_WATCHDOG_WDMOD           0x80048000
#define HW_WATCHDOG_WDTC            0x80048004
#define HW_WATCHDOG_WDFEED          0x80048008
#define HW_WATCHDOG_WDTV            0x8004800C
#define HW_LCDCLKDIV 		    0x800401fc
