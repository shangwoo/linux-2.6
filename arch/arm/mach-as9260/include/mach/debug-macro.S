/* arch/arm/mach-sa1100/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
#include <mach/hardware.h>

#define ASM9260_DBG		UART4_BASEESS
#define ASM9260_UART_DATA	0x50
#define ASM9260_UART_STAT	0x60
#define ASM9260_UART_TXEMPTY	(1<<27)
#define ASM9260_UART_TXFULL	(1<<25)


		.macro	addruart, rp, rv, tmp
		ldr	\rp, =ASM9260_DBG			@ physical base address
		ldr	\rv, =ASM9260_DBG			@ virtual address
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #ASM9260_UART_DATA]
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #ASM9260_UART_STAT]
		tst	\rd, #ASM9260_UART_TXFULL
		beq	1001b
		.endm

		.macro	busyuart,rd,rx
1001:		ldr	\rd, [\rx, #ASM9260_UART_STAT]
		tst	\rd, #ASM9260_UART_TXEMPTY
		bne	1001b
		.endm
