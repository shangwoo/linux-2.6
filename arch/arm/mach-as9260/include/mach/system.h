/*
 * Copied from linux/include/asm-arm/arch-sa1100/system.h
 * Copyright (c) 1999 Nicolas Pitre <nico@cam.org>
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H
#include <asm/mach-types.h>
#include <mach/hardware.h>
//#include <asm/cpu-single.h>
#include <mach/io.h>
#include <linux/delay.h>
#include <asm/proc-fns.h>

#ifndef CONFIG_MACH_VOICEBLUE
#define voiceblue_reset()		do {} while (0)
#endif

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode)
{
         if (machine_is_voiceblue())
		voiceblue_reset();
	as3310_writel(1<<26,HW_AHBCLKCTRL0+4);  
     as3310_writel(1<<26,HW_PRESETCTRL0+8);  //reset watchdog
	mdelay(1);
	as3310_writel(1<<26,HW_PRESETCTRL0+4);  //clear watchdog reset
       
	as3310_writel(1,HW_WDTCLKSEL);
	as3310_writel(0,HW_WDTCLKUEN);
	as3310_writel(1,HW_WDTCLKUEN);
	as3310_writel(480,HW_WDTCLKDIV);
	as3310_writel(0x3,HW_WATCHDOG_WDMOD);
	as3310_writel(0x1E8480,HW_WATCHDOG_WDTC);
	as3310_writel(0xaa,HW_WATCHDOG_WDFEED);
	as3310_writel(0x55,HW_WATCHDOG_WDFEED);
}

void as3310_dbg_hexdump(const void *ptr, int size);
#endif
