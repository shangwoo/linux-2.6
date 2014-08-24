/*
 * include/asm-arm/arch-omap/entry-macro.S
 *
 * Low-level IRQ helper macros for OMAP-based platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */


@---------------------------------------------------------------------
@               AS3310 version           


		 .macro	disable_fiq
		 .endm

		 .macro  get_irqnr_preamble, base, tmp
		 .endm

	         .macro  arch_ret_to_user, tmp1, tmp2
		 .endm
		 
 #include <mach/hardware.h>

                 .macro get_irqnr_and_base, irqnr, irqstat, base, tmp

                 ldr     \base, =IO_ADDRESS(HW_ICOLL_STAT)
                 ldr    \base, [\base]
 		 mov    \irqnr, \base
 		 cmp     \base, #0x3f
                 bne       3310f
        	 ldr     \base, =IO_ADDRESS(HW_ICOLL_RAW1)
		 ldr    \base, [\base]
  		 and	 \base, \base, #0x80000000
  		 cmp     \base, #0x0
  		 moveq   \irqnr, #0x3f
 3310:
                 .endm


@---------------------------------------------------------------------

