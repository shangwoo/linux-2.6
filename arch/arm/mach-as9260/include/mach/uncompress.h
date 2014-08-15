/*
 * linux/include/asm-arm/arch-omap/uncompress.h
 *
 * Serial port stubs for kernel decompress status messages
 *
 * Initially based on:
 * linux-2.4.15-rmk1-dsplinux1.6/include/asm-arm/arch-omap1510/uncompress.h
 * Copyright (C) 2000 RidgeRun, Inc.
 * Author: Greg Lonnon <glonnon@ridgerun.com>
 *
 * Rewritten by:
 * Author: <source@mvista.com>
 * 2004 (c) MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/serial_reg.h>
//#include <asm/arch/serial.h>



/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()


static inline void putc(int c)
{
}

static inline void flush(void)
{
}
