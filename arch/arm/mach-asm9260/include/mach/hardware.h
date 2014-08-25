/*
 * linux/include/asm-arm/arch-as3310/hardware.h
 *
 * Hardware definitions for TI OMAP processors and boards
 *
 * NOTE: Please put device driver specific defines into a separate header
 *	 file for each driver.
 *
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: RidgeRun, Inc. Greg Lonnon <glonnon@ridgerun.com>
 *
 * Reorganized for Linux-2.6 by Tony Lindgren <tony@atomide.com>
 *                          and Dirk Behme <dirk.behme@de.bosch.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
///#include <linux/config.h>
#ifndef __ASSEMBLER__
#include <asm/types.h>
//#include <asm/arch/cpu.h>
#endif
#include <mach/io.h>
//#include <asm/arch/serial.h>

#if 0
#ifdef __ASSEMBLY__
#define IOMEM(x) x
#else
#define IOMEM(x) ((void __iomem *)(x))
#endif
#endif

/*  reg bit modification   */

#define set_bit_u32(reg,i)          do{ as3310_writel((as3310_readl(reg)|(1<<(i))),reg);}while(0)
#define set_bit_u16(reg,i)          do{ as3310_writew((as3310_readl(reg)|(1<<(i))),reg);}while(0)
#define set_bit_u8(reg,i)           do{ as3310_writeb((as3310_readl(reg)|(1<<(i))),reg);}while(0)

#define clr_bit_u32(reg,i)          do{ as3310_writel((as3310_readl(reg)&(~(1<<(i)))),reg);}while(0)
#define clr_bit_u16(reg,i)          do{ as3310_writew((as3310_readl(reg)&(~(1<<(i)))),reg);}while(0)
#define clr_bit_u8(reg,i)           do{ as3310_writeb((as3310_readl(reg)&(~(1<<(i)))),reg);}while(0)

#define     GENERAL_sftrst          31
#define     GENERAL_clkgate         30
/*  Device Discriptor   */

#define DEV_NAND   0
#define DEV_I2C    1
#define DEV_NOR    2
#define DEV_ROM    3
#define DEV_USB    4
#define DEV_UART   5
#define DEV_SDRAM  6

#define AS9260_SRAM_VIRT_BASE   0xd0000000
#define AS9260_SRAM_PHY_BASE    0x40000000
#define AS9260_SRAM_SIZE        0x000a0000




/*
 * ----------------------------------------------------------------------------
 * REGISTER OFFSET
 * ----------------------------------------------------------------------------
 */
#define SET_OFFSET  0x4
#define CLR_OFFSET  0x8
#define TOG_OFFSET  0xC

/*
 * ----------------------------------------------------------------------------
 * CLK CONTROL
 * ----------------------------------------------------------------------------
 */
 
#define  HW_CLK_BASE               (0x80040000)
#define  HW_CLK_PLLCTRL0           (HW_CLK_BASE + 0x0)
#define  HW_CLK_PLLCTRL1           (HW_CLK_BASE + 0x10)
#define  HW_CLK_CPUCLKCTRL         (HW_CLK_BASE + 0x20)
#define  HW_CLK_HBUSCLKCTRL        (HW_CLK_BASE + 0x30)
#define  HW_CLK_XBUSCLKCTRL        (HW_CLK_BASE + 0x40)
#define  HW_CLK_XTALCLKCTRL        (HW_CLK_BASE + 0x50)
#define  HW_CLK_OCRAMCLKCTRL       (HW_CLK_BASE + 0x60)
#define  HW_CLK_UTMICLKCTRL        (HW_CLK_BASE + 0x70)
#define  HW_CLK_SSPCLKCTRL         (HW_CLK_BASE + 0x80)
#define  HW_CLK_GPMICLKCTRL        (HW_CLK_BASE + 0x90)
#define  HW_CLK_SPDIFCLKCTRL       (HW_CLK_BASE + 0xA0)
#define  HW_CLK_EMICLKCTRL         (HW_CLK_BASE + 0xB0)
#define  HW_CLK_IRCLKCTRL          (HW_CLK_BASE + 0xC0)
#define  HW_CLK_CLKCTRL_PLL180     (HW_CLK_BASE + 0xD0)
#define  HW_CLK_I2SMCLK            (HW_CLK_BASE + 0xE0)

/*
 * ----------------------------------------------------------------------------
 * interrupts
 * ----------------------------------------------------------------------------
 */
#if 0
#define HW_ICOLL_BASE           (0x80000000)
#define HW_ICOLL_VECTOR		    (HW_ICOLL_BASE + 0x0)
#define HW_ICOLL_LEVELACK		(HW_ICOLL_BASE + 0x10)
#define HW_ICOLL_CTRL			(HW_ICOLL_BASE + 0x20)
#define HW_ICOLL_STAT			(HW_ICOLL_BASE + 0x30)
#define HW_ICOLL_RAW0	   	 	(HW_ICOLL_BASE + 0x40)
#define HW_ICOLL_RAW1	   	 	(HW_ICOLL_BASE + 0x50)
#define HW_ICOLL_PRIORITY0		(HW_ICOLL_BASE + 0x60)
#define HW_ICOLL_PRIORITY1		(HW_ICOLL_BASE + 0x70)
#define HW_ICOLL_PRIORITY2		(HW_ICOLL_BASE + 0x80)
#define HW_ICOLL_PRIORITY3		(HW_ICOLL_BASE + 0x90)
#define HW_ICOLL_PRIORITY4		(HW_ICOLL_BASE + 0xA0)
#define HW_ICOLL_PRIORITY5		(HW_ICOLL_BASE + 0xB0)
#define HW_ICOLL_PRIORITY6		(HW_ICOLL_BASE + 0xC0)
#define HW_ICOLL_PRIORITY7		(HW_ICOLL_BASE + 0xD0)
#define HW_ICOLL_PRIORITY8		(HW_ICOLL_BASE + 0xE0)
#define HW_ICOLL_PRIORITY9		(HW_ICOLL_BASE + 0xF0)
#define HW_ICOLL_PRIORITY10	    (HW_ICOLL_BASE + 0x100)
#define HW_ICOLL_PRIORITY11	    (HW_ICOLL_BASE + 0x110)
#define HW_ICOLL_PRIORITY12	    (HW_ICOLL_BASE + 0x120)
#define HW_ICOLL_PRIORITY13	    (HW_ICOLL_BASE + 0x130)
#define HW_ICOLL_PRIORITY14	    (HW_ICOLL_BASE + 0x140)
#define HW_ICOLL_PRIORITY15	    (HW_ICOLL_BASE + 0x150)
#define HW_ICOLL_VBASE			(HW_ICOLL_BASE + 0x160)
#define HW_ICOLL_DEBUG			(HW_ICOLL_BASE + 0x170)
#define HW_ICOLL_DEBUGRD0 		(HW_ICOLL_BASE + 0x180)	//	R	Debug Read Register 0
#define HW_ICOLL_DEBUGRD1 		(HW_ICOLL_BASE + 0x190)	//	R	Debug Read Register 1
#define HW_ICOLL_DEBUGFLAG 		(HW_ICOLL_BASE + 0x1A0)	//	R/W	Debug Flag Register
#define HW_ICOLL_DEBUGRDREQ0 	(HW_ICOLL_BASE + 0x1B0)	//	R	Debug Read Request Register 0
#define HW_ICOLL_DEBUGRDREQ1 	(HW_ICOLL_BASE + 0x1C0)	//	R	Debug Read Request Register 1
#define HW_ICOLL_IRQCLEAR0 	    (HW_ICOLL_BASE + 0x1D0)	//	IRQ Clear 0
#define HW_ICOLL_IRQCLEAR1 	    (HW_ICOLL_BASE + 0x1E0)	//	IRQ Clear 1
C#define HW_ICOLL_UNDEF_VECTOR   (HW_ICOLL_BASE + 0x1F0)	//	IRQ Clear 1
#endif

////////////////////////////////////////////////////
/*irq*/
#define IRQ_BASE_ADDRESS        0x80054000

#if 1
#define	HW_ICOLL_VECTOR         0x80054000                
#define	HW_ICOLL_VECTOR_SET     0x80054004 
#define	HW_ICOLL_VECTOR_CLR     0x80054008
#define	HW_ICOLL_VECTOR_TOG     0x8005400C

#define HW_ICOLL_LEVELACK       0x80054010
#define HW_ICOLL_LEVELACK_SET   0x80054014
#define HW_ICOLL_LEVELACK_CLR   0x80054018
#define HW_ICOLL_LEVELACK_TOG   0x8005401C

#define	HW_ICOLL_CTRL           0x80054020
#define	HW_ICOLL_CTRL_SET       0x80054024
#define	HW_ICOLL_CTRL_CLR       0x80054028
#define	HW_ICOLL_CTRL_TOG       0x8005402C

#define HW_ICOLL_STAT           0x80054030
#define HW_ICOLL_STAT_SET       0x80054034
#define HW_ICOLL_STAT_CLR       0x80054038
#define HW_ICOLL_STAT_TOG       0x8005403C

#define HW_ICOLL_RAW0           0x80054040
#define HW_ICOLL_RAW0_SET       0x80054044
#define HW_ICOLL_RAW0_CLR       0x80054048
#define HW_ICOLL_RAW0_TOG       0x8005404c

#define HW_ICOLL_RAW1           0x80054050
#define HW_ICOLL_RAW1_SET       0x80054054
#define HW_ICOLL_RAW1_CLR       0x80054058
#define HW_ICOLL_RAW1_TOG       0x8005405c

#define	HW_ICOLL_PRIORITY0      0x80054060
#define	HW_ICOLL_PRIORITY0_SET  0x80054064
#define	HW_ICOLL_PRIORITY0_CLR  0x80054068
#define	HW_ICOLL_PRIORITY0_TOG  0x8005406C

#define	HW_ICOLL_PRIORITY1      0x80054070
#define	HW_ICOLL_PRIORITY1_SET  0x80054074
#define	HW_ICOLL_PRIORITY1_CLR  0x80054078
#define	HW_ICOLL_PRIORITY1_TOG  0x8005407C

#define	HW_ICOLL_PRIORITY2      0x80054080
#define	HW_ICOLL_PRIORITY2_SET  0x80054084
#define	HW_ICOLL_PRIORITY2_CLR  0x80054088
#define	HW_ICOLL_PRIORITY2_TOG  0x8005408C

#define	HW_ICOLL_PRIORITY3      0x80054090
#define	HW_ICOLL_PRIORITY3_SET  0x80054094
#define	HW_ICOLL_PRIORITY3_CLR  0x80054098
#define	HW_ICOLL_PRIORITY3_TOG  0x8005409C

#define	HW_ICOLL_PRIORITY4      0x800540A0
#define	HW_ICOLL_PRIORITY4_SET  0x800540A4
#define	HW_ICOLL_PRIORITY4_CLR  0x800540A8
#define	HW_ICOLL_PRIORITY4_TOG  0x800540AC

#define	HW_ICOLL_PRIORITY5      0x800540B0
#define	HW_ICOLL_PRIORITY5_SET  0x800540B4
#define	HW_ICOLL_PRIORITY5_CLR  0x800540B8
#define	HW_ICOLL_PRIORITY5_TOG  0x800540BC

#define	HW_ICOLL_PRIORITY6      0x800540C0
#define	HW_ICOLL_PRIORITY6_SET  0x800540C4
#define	HW_ICOLL_PRIORITY6_CLR  0x800540C8
#define	HW_ICOLL_PRIORITY6_TOG  0x800540CC

#define	HW_ICOLL_PRIORITY7      0x800540D0
#define	HW_ICOLL_PRIORITY7_SET  0x800540D4
#define	HW_ICOLL_PRIORITY7_CLR  0x800540D8
#define	HW_ICOLL_PRIORITY7_TOG  0x800540DC

#define	HW_ICOLL_PRIORITY8      0x800540E0
#define	HW_ICOLL_PRIORITY8_SET  0x800540E4
#define	HW_ICOLL_PRIORITY8_CLR  0x800540E8
#define	HW_ICOLL_PRIORITY8_TOG  0x800540EC

#define	HW_ICOLL_PRIORITY9      0x800540F0
#define	HW_ICOLL_PRIORITY9_SET  0x800540F4
#define	HW_ICOLL_PRIORITY9_CLR  0x800540F8
#define	HW_ICOLL_PRIORITY9_TOG  0x800540FC

#define	HW_ICOLL_PRIORITY10     0x80054100
#define	HW_ICOLL_PRIORITY10_SET 0x80054104
#define	HW_ICOLL_PRIORITY10_CLR 0x80054108
#define	HW_ICOLL_PRIORITY10_TOG 0x8005410C

#define	HW_ICOLL_PRIORITY11     0x80054110
#define	HW_ICOLL_PRIORITY11_SET 0x80054114
#define	HW_ICOLL_PRIORITY11_CLR 0x80054118
#define	HW_ICOLL_PRIORITY11_TOG 0x8005411C

#define	HW_ICOLL_PRIORITY12     0x80054120
#define	HW_ICOLL_PRIORITY12_SET 0x80054124
#define	HW_ICOLL_PRIORITY12_CLR 0x80054128
#define	HW_ICOLL_PRIORITY12_TOG 0x8005412C

#define	HW_ICOLL_PRIORITY13     0x80054130
#define	HW_ICOLL_PRIORITY13_SET 0x80054134
#define	HW_ICOLL_PRIORITY13_CLR 0x80054138
#define	HW_ICOLL_PRIORITY13_TOG 0x8005413C

#define	HW_ICOLL_PRIORITY14     0x80054140
#define	HW_ICOLL_PRIORITY14_SET 0x80054144
#define	HW_ICOLL_PRIORITY14_CLR 0x80054148
#define	HW_ICOLL_PRIORITY14_TOG 0x8005414C

#define	HW_ICOLL_PRIORITY15     0x80054150
#define	HW_ICOLL_PRIORITY15_SET 0x80054154
#define	HW_ICOLL_PRIORITY15_CLR 0x80054158
#define	HW_ICOLL_PRIORITY15_TOG 0x8005415C

#define	HW_ICOLL_VBASE          0x80054160
#define	HW_ICOLL_VBASE_SET      0x80054164
#define	HW_ICOLL_VBASE_CLR      0x80054168
#define	HW_ICOLL_VBASE_TOG      0x8005416C

#define HW_ICOLL_DEBUG          0x80054170
#define HW_ICOLL_DEBUG_SET      0x80054174
#define HW_ICOLL_DEBUG_CLR      0x80054178
#define HW_ICOLL_DEBUG_TOG      0x8005417C

#define HW_ICOLL_DBGREAD0       0x80054180
#define HW_ICOLL_DBGREAD0_SET   0x80054184
#define HW_ICOLL_DBGREAD0_CLR   0x80054188
#define HW_ICOLL_DBGREAD0_TOG   0x8005418C

#define HW_ICOLL_DBGREAD1       0x80054190
#define HW_ICOLL_DBGREAD1_SET   0x80054194
#define HW_ICOLL_DBGREAD1_CLR   0x80054198
#define HW_ICOLL_DBGREAD1_TOG   0x8005419C

#define HW_ICOLL_DBGFLAG        0x800541A0
#define HW_ICOLL_DBGFLAG_SET    0x800541A4
#define HW_ICOLL_DBGFLAG_CLR    0x800541A8
#define HW_ICOLL_DBGFLAG_TOG    0x800541AC

#define HW_ICOLL_DBGREQUEST0     0x800541B0
#define HW_ICOLL_DBGREQUEST0_SET 0x800541B4
#define HW_ICOLL_DBGREQUEST0_CLR 0x800541B8
#define HW_ICOLL_DBGREQUEST0_TOG 0x800541BC

#define HW_ICOLL_DBGREQUEST1     0x800541C0
#define HW_ICOLL_DBGREQUEST1_SET 0x800541C4
#define HW_ICOLL_DBGREQUEST1_CLR 0x800541C8
#define HW_ICOLL_DBGREQUEST1_TOG 0x800541CC

#define	HW_ICOLL_CLEAR0         0x800541D0
#define	HW_ICOLL_CLEAR0_SET     0x800541D4
#define	HW_ICOLL_CLEAR0_CLR     0x800541D8
#define	HW_ICOLL_CLEAR0_TOG     0x800541DC

#define	HW_ICOLL_CLEAR1         0x800541E0
#define	HW_ICOLL_CLEAR1_SET     0x800541E4
#define	HW_ICOLL_CLEAR1_CLR     0x800541E8
#define	HW_ICOLL_CLEAR1_TOG     0x800541EC

#define HW_ICOLL_UNDEF_VECTOR     0x800541F0
#define HW_ICOLL_UNDEF_VECTOR_SET 0x800541F4
#define HW_ICOLL_UNDEF_VECTOR_CLR 0x800541F8
#define HW_ICOLL_UNDEF_VECTOR_TOG 0x800541FC  

#endif
/*
 * ----------------------------------------------------------------------------
 * DIGFILT register
 * ----------------------------------------------------------------------------
 */ 
#define HW_DIGCTL_CTRL         (0x8001c000)
#define HW_DIGCTL_STATUS       (0x8001c010)
#define HW_DIGCTL_HCLKCOUNT    (0x8001c020)
#define HW_DIGCTL_WRITEONCE    (0x8001c060)
#define HW_DIGCTL_AHBCYCLES    (0x8001c070)
#define HW_DIGCTL_AHBSTALLED   (0x8001c080)
#define HW_DIGCTL_ENTROPY      (0x8001c090)
#define HW_DIGCTL_MICROSECONDS (0x8001c0B0)
#define HW_DIGCTL_DBGRD        (0x8001c0C0)
#define HW_DIGCTL_DBG          (0x8001c0D0)
#define HW_POWER_PWR           (0x8001c320)

/*
 * ----------------------------------------------------------------------------
 * USB register
 * ----------------------------------------------------------------------------
 */ 
#define HW_USBPHY_PWD           (0x8007C000)
#define HW_USBPHY_TX            (0x8007C010)
#define HW_USBPHY_RX            (0x8007C020)
#define HW_USBPHY_CTRL          (0x8007C030)
#define HW_USBPHY_STATUS        (0x8007C040)
#define HW_USBPHY_DEBUG         (0x8007C050)
#define HW_USBPHY_DEBUG0_STATUS (0x8007C060)
#define HW_USBPHY_DEBUG1_STATUS (0x8007C070)
#define HW_USBPHY_DEBUG2_STATUS (0x8007C080)
#define HW_USBPHY_SYSCTRL       (0x8007C090)
#define HW_USBPHY_ANALOG        (0x8007C0A0)

#if 1 //We will delete the following forever when new one is finished for not interfering with other modules using 1826 DMA for the moment.
/*
 * ----------------------------------------------------------------------------
 * DMA_APBH
 * ----------------------------------------------------------------------------
 */

#define HW_APBH_BASE 			(0x80004000)

#define HW_APBH_CTRL0 			(0x80004000)
#define HW_APBH_CTRL0_SET 		(0x80004004)
#define HW_APBH_CTRL0_CLR 		(0x80004008)
#define HW_APBH_CTRL0_TOG 		(0x8000400C)

#define HW_APBH_CTRL1 			(0x80004010)
#define HW_APBH_CTRL1_SET 		(0x80004014)
#define HW_APBH_CTRL1_CLR 		(0x80004018)
#define HW_APBH_CTRL1_TOG 		(0x8000401C)

#define HW_APBH_DEVSEL          (0x80004020)

#define HW_APBH_CH0_CURCMDAR    (0x80004030)
#define HW_APBH_CH0_NXTCMDAR    (0x80004040)
#define HW_APBH_CH0_CMD         (0x80004050)
#define HW_APBH_CH0_BAR         (0x80004060)
#define HW_APBH_CH0_SEMA        (0x80004070)
#define HW_APBH_CH0_DEBUG1      (0x80004080)
#define HW_APBH_CH0_DEBUG2      (0x80004090)

#define HW_APBH_CH1_CURCMDAR    (0x800040A0)
#define HW_APBH_CH1_NXTCMDAR    (0x800040B0)
#define HW_APBH_CH1_CMD         (0x800040C0)
#define HW_APBH_CH1_BAR         (0x800040D0)
#define HW_APBH_CH1_SEMA        (0x800040E0)
#define HW_APBH_CH1_DEBUG1      (0x800040F0)
#define HW_APBH_CH1_DEBUG2      (0x80004100)

#define HW_APBH_CH2_CURCMDAR    (0x80004110)
#define HW_APBH_CH2_NXTCMDAR    (0x80004120)
#define HW_APBH_CH2_CMD         (0x80004130)
#define HW_APBH_CH2_BAR         (0x80004140)
#define HW_APBH_CH2_SEMA        (0x80004150)
#define HW_APBH_CH2_DEBUG1      (0x80004160)
#define HW_APBH_CH2_DEBUG2      (0x80004170)

#define HW_APBH_CH3_CURCMDAR    (0x80004180)
#define HW_APBH_CH3_NXTCMDAR    (0x80004190)
#define HW_APBH_CH3_CMD         (0x800041A0)
#define HW_APBH_CH3_BAR         (0x800041B0)
#define HW_APBH_CH3_SEMA        (0x800041C0)
#define HW_APBH_CH3_DEBUG1      (0x800041D0)
#define HW_APBH_CH3_DEBUG2      (0x800041E0)

#define HW_APBH_CH4_CURCMDAR    (0x800041F0)
#define HW_APBH_CH4_NXTCMDAR    (0x80004200)
#define HW_APBH_CH4_CMD         (0x80004210)
#define HW_APBH_CH4_BAR         (0x80004220)
#define HW_APBH_CH4_SEMA        (0x80004230)
#define HW_APBH_CH4_DEBUG1      (0x80004240)
#define HW_APBH_CH4_DEBUG2      (0x80004250)

#define HW_APBH_CH5_CURCMDAR    (0x80004260)
#define HW_APBH_CH5_NXTCMDAR    (0x80004270)
#define HW_APBH_CH5_CMD         (0x80004280)
#define HW_APBH_CH5_BAR         (0x80004290)
#define HW_APBH_CH5_SEMA        (0x800042A0)
                                            
#define HW_APBH_CH6_CURCMDAR    (0x800042D0)
#define HW_APBH_CH6_NXTCMDAR    (0x800042E0)
#define HW_APBH_CH6_CMD         (0x800042F0)
#define HW_APBH_CH6_BAR         (0x80004300)
#define HW_APBH_CH6_SEMA        (0x80004310)

#define HW_APBH_CH7_CURCMDAR    (0x80004340)
#define HW_APBH_CH7_NXTCMDAR    (0x80004350)
#define HW_APBH_CH7_CMD         (0x80004360)
#define HW_APBH_CH7_BAR         (0x80004370)
#define HW_APBH_CH7_SEMA        (0x80004380)

/*
 * ----------------------------------------------------------------------------
 * DMA_APBX
 * ----------------------------------------------------------------------------
 */
#define HW_APBX_BASE 			(0x80024000)

#define HW_APBX_CTRL0 			(0x80024000)
#define HW_APBX_CTRL0_SET 		(0x80024004)
#define HW_APBX_CTRL0_CLR 		(0x80024008)
#define HW_APBX_CTRL0_TOG 		(0x8002400C)

#define HW_APBX_CTRL1 			(0x80024010)
#define HW_APBX_CTRL1_SET 		(0x80024014)
#define HW_APBX_CTRL1_CLR 		(0x80024018)
#define HW_APBX_CTRL1_TOG 		(0x8002401C)

#define HW_APBX_DEVSEL          (0x80024020)

#define HW_APBX_CH0_CURCMDAR    (0x80024030)
#define HW_APBX_CH0_NXTCMDAR    (0x80024040)
#define HW_APBX_CH0_CMD         (0x80024050)
#define HW_APBX_CH0_BAR         (0x80024060)
#define HW_APBX_CH0_SEMA        (0x80024070)
#define HW_APBX_CH0_DEBUG1      (0x80024080)
#define HW_APBX_CH0_DEBUG2      (0x80024090)

#define HW_APBX_CH1_CURCMDAR    (0x800240A0)
#define HW_APBX_CH1_NXTCMDAR    (0x800240B0)
#define HW_APBX_CH1_CMD         (0x800240C0)
#define HW_APBX_CH1_BAR         (0x800240D0)
#define HW_APBX_CH1_SEMA        (0x800240E0)
#define HW_APBX_CH1_DEBUG1      (0x800240F0)
#define HW_APBX_CH1_DEBUG2      (0x80024100)

#define HW_APBX_CH2_CURCMDAR    (0x80024110)
#define HW_APBX_CH2_NXTCMDAR    (0x80024120)
#define HW_APBX_CH2_CMD         (0x80024130)
#define HW_APBX_CH2_BAR         (0x80024140)
#define HW_APBX_CH2_SEMA        (0x80024150)
#define HW_APBX_CH2_DEBUG1      (0x80024160)
#define HW_APBX_CH2_DEBUG2      (0x80024170)

#define HW_APBX_CH3_CURCMDAR    (0x80024180)
#define HW_APBX_CH3_NXTCMDAR    (0x80024190)
#define HW_APBX_CH3_CMD         (0x800241A0)
#define HW_APBX_CH3_BAR         (0x800241B0)
#define HW_APBX_CH3_SEMA        (0x800241C0)
#define HW_APBX_CH3_DEBUG1      (0x800241D0)
#define HW_APBX_CH3_DEBUG2      (0x800241E0)

#define HW_APBX_CH4_CURCMDAR    (0x800241F0)
#define HW_APBX_CH4_NXTCMDAR    (0x80024200)
#define HW_APBX_CH4_CMD         (0x80024210)
#define HW_APBX_CH4_BAR         (0x80024220)
#define HW_APBX_CH4_SEMA        (0x80024230)
#define HW_APBX_CH4_DEBUG1      (0x80024240)
#define HW_APBX_CH4_DEBUG2      (0x80024250)

#define HW_APBX_CH5_CURCMDAR    (0x80024260)
#define HW_APBX_CH5_NXTCMDAR    (0x80024270)
#define HW_APBX_CH5_CMD         (0x80024280)
#define HW_APBX_CH5_BAR         (0x80024290)
#define HW_APBX_CH5_SEMA        (0x800242A0)

#define HW_APBX_CH6_CURCMDAR    (0x800242D0)
#define HW_APBX_CH6_NXTCMDAR    (0x800242E0)
#define HW_APBX_CH6_CMD         (0x800242F0)
#define HW_APBX_CH6_BAR         (0x80024300)
#define HW_APBX_CH6_SEMA        (0x80024310)

#define HW_APBX_CH7_CURCMDAR    (0x80024340)
#define HW_APBX_CH7_NXTCMDAR    (0x80024350)
#define HW_APBX_CH7_CMD         (0x80024360)
#define HW_APBX_CH7_BAR         (0x80024370)
#define HW_APBX_CH7_SEMA        (0x80024380)
#endif

//DMA0
#define HW_DMA0_SAR0           0x80100000
#define HW_DMA0_DAR0           0x80100008
#define HW_DMA0_LLP0           0x80100010
#define HW_DMA0_CTL0           0x80100018
#define HW_DMA0_SSTAT0         0x80100020
#define HW_DMA0_DSTAT0         0x80100028
#define HW_DMA0_SSTATAR0       0x80100030      
#define HW_DMA0_DSTATAR0       0x80100038 
#define HW_DMA0_CFG0           0x80100040
#define HW_DMA0_SGR0           0x80100048
#define HW_DMA0_DSR0           0x80100050

#define HW_DMA0_SAR1           0x80100058
#define HW_DMA0_DAR1           0x80100060
#define HW_DMA0_LLP1           0x80100068
#define HW_DMA0_CTL1           0x80100070
#define HW_DMA0_SSTAT1         0x80100078
#define HW_DMA0_DSTAT1         0x80100080
#define HW_DMA0_SSTATAR1       0x80100088      
#define HW_DMA0_DSTATAR1       0x80100090 
#define HW_DMA0_CFG1           0x80100098
#define HW_DMA0_SGR1           0x801000a0
#define HW_DMA0_DSR1           0x801000a8

#define HW_DMA0_SAR2           0x801000b0
#define HW_DMA0_DAR2           0x801000b8
#define HW_DMA0_LLP2           0x801000c0
#define HW_DMA0_CTL2           0x801000c8
#define HW_DMA0_SSTAT2         0x801000d0
#define HW_DMA0_DSTAT2         0x801000d8
#define HW_DMA0_SSTATAR2       0x801000e0      
#define HW_DMA0_DSTATAR2       0x801000e8 
#define HW_DMA0_CFG2           0x801000f0
#define HW_DMA0_SGR2           0x801000f8
#define HW_DMA0_DSR2           0x80100100

#define HW_DMA0_SAR3           0x80100108
#define HW_DMA0_DAR3           0x80100110
#define HW_DMA0_LLP3           0x80100118
#define HW_DMA0_CTL3           0x80100120
#define HW_DMA0_SSTAT3         0x80100128
#define HW_DMA0_DSTAT3         0x80100130
#define HW_DMA0_SSTATAR3       0x80100138      
#define HW_DMA0_DSTATAR3       0x80100140 
#define HW_DMA0_CFG3           0x80100148
#define HW_DMA0_SGR3           0x80100150
#define HW_DMA0_DSR3           0x80100158

#define HW_DMA0_SAR4           0x80100160
#define HW_DMA0_DAR4           0x80100168
#define HW_DMA0_LLP4           0x80100170
#define HW_DMA0_CTL4           0x80100178
#define HW_DMA0_SSTAT4         0x80100180
#define HW_DMA0_DSTAT4         0x80100188
#define HW_DMA0_SSTATAR4       0x80100190      
#define HW_DMA0_DSTATAR4       0x80100198 
#define HW_DMA0_CFG4           0x801001a0
#define HW_DMA0_SGR4           0x801001a8
#define HW_DMA0_DSR4           0x801001b0

#define HW_DMA0_SAR5           0x801001b8
#define HW_DMA0_DAR5           0x801001c0
#define HW_DMA0_LLP5           0x801001c8
#define HW_DMA0_CTL5           0x801001d0
#define HW_DMA0_SSTAT5         0x801001d8
#define HW_DMA0_DSTAT5         0x801001e0
#define HW_DMA0_SSTATAR5       0x801001e8      
#define HW_DMA0_DSTATAR5       0x801001f0 
#define HW_DMA0_CFG5           0x801001f8
#define HW_DMA0_SGR5           0x80100200
#define HW_DMA0_DSR5           0x80100208

#define HW_DMA0_SAR6           0x80100210
#define HW_DMA0_DAR6           0x80100218
#define HW_DMA0_LLP6           0x80100220
#define HW_DMA0_CTL6           0x80100228
#define HW_DMA0_SSTAT6         0x80100230
#define HW_DMA0_DSTAT6         0x80100238
#define HW_DMA0_SSTATAR6       0x80100240      
#define HW_DMA0_DSTATAR6       0x80100248 
#define HW_DMA0_CFG6           0x80100250
#define HW_DMA0_SGR6           0x80100258
#define HW_DMA0_DSR6           0x80100260

#define HW_DMA0_SAR7           0x80100268
#define HW_DMA0_DAR7           0x80100270
#define HW_DMA0_LLP7           0x80100278
#define HW_DMA0_CTL7           0x80100280
#define HW_DMA0_SSTAT7         0x80100288
#define HW_DMA0_DSTAT7         0x80100290
#define HW_DMA0_SSTATAR7       0x80100298      
#define HW_DMA0_DSTATAR7       0x801002a0 
#define HW_DMA0_CFG7           0x801002a8
#define HW_DMA0_SGR7           0x801002b0
#define HW_DMA0_DSR7           0x801002b8

#define HW_DMA0_RawTFR         0x801002c0
#define HW_DMA0_RawBLOCK       0x801002c8
#define HW_DMA0_RawSRCTRAN     0x801002d0
#define HW_DMA0_RawDSTTRAN     0x801002d8
#define HW_DMA0_RawERR         0x801002e0

#define HW_DMA0_StatusTFR      0x801002e8
#define HW_DMA0_StatusBLOCK    0x801002f0
#define HW_DMA0_StatusSRCTRAN  0x801002f8
#define HW_DMA0_StatusDSTTRAN  0x80100300
#define HW_DMA0_StatusERR      0x80100308

#define HW_DMA0_MaskTFR        0x80100310
#define HW_DMA0_MaskBLOCK      0x80100318
#define HW_DMA0_MaskSRCTRAN    0x80100320
#define HW_DMA0_MaskDSTTRAN    0x80100328
#define HW_DMA0_MaskERR        0x80100330

#define HW_DMA0_ClearTFR       0x80100338
#define HW_DMA0_ClearBLOCK     0x80100340
#define HW_DMA0_ClearSRCTRAN   0x80100348
#define HW_DMA0_ClearDSTTRAN   0x80100350
#define HW_DMA0_ClearERR       0x80100358

#define HW_DMA0_STATUSINT      0x80100360

#define HW_DMA0_ReqSrcReg      0x80100368
#define HW_DMA0_ReqDstReg      0x80100370
#define HW_DMA0_SglReqSrcReg   0x80100378
#define HW_DMA0_SglReqDstReg   0x80100380
#define HW_DMA0_LstSrcReg      0x80100388
#define HW_DMA0_LstDstReg      0x80100390

#define HW_DMA0_DMACFGREG      0x80100398
#define HW_DMA0_CHENREG        0x801003a0


/////////////////////////////////////////////////////////
//DMA1
#define HW_DMA1_SAR0           0x80200000
#define HW_DMA1_DAR0           0x80200008
#define HW_DMA1_LLP0           0x80200010
#define HW_DMA1_CTL0           0x80200018
#define HW_DMA1_SSTAT0         0x80200020
#define HW_DMA1_DSTAT0         0x80200028
#define HW_DMA1_SSTATAR0       0x80200030      
#define HW_DMA1_DSTATAR0       0x80200038 
#define HW_DMA1_CFG0           0x80200040
#define HW_DMA1_SGR0           0x80200048
#define HW_DMA1_DSR0           0x80200050

#define HW_DMA1_SAR1           0x80200058
#define HW_DMA1_DAR1           0x80200060
#define HW_DMA1_LLP1           0x80200068
#define HW_DMA1_CTL1           0x80200070
#define HW_DMA1_SSTAT1         0x80200078
#define HW_DMA1_DSTAT1         0x80200080
#define HW_DMA1_SSTATAR1       0x80200088      
#define HW_DMA1_DSTATAR1       0x80200090 
#define HW_DMA1_CFG1           0x80200098
#define HW_DMA1_SGR1           0x802000a0
#define HW_DMA1_DSR1           0x802000a8

#define HW_DMA1_SAR2           0x802000b0
#define HW_DMA1_DAR2           0x802000b8
#define HW_DMA1_LLP2           0x802000c0
#define HW_DMA1_CTL2           0x802000c8
#define HW_DMA1_SSTAT2         0x802000d0
#define HW_DMA1_DSTAT2         0x802000d8
#define HW_DMA1_SSTATAR2       0x802000e0      
#define HW_DMA1_DSTATAR2       0x802000e8 
#define HW_DMA1_CFG2           0x802000f0
#define HW_DMA1_SGR2           0x802000f8
#define HW_DMA1_DSR2           0x80200100

#define HW_DMA1_SAR3           0x80200108
#define HW_DMA1_DAR3           0x80200110
#define HW_DMA1_LLP3           0x80200118
#define HW_DMA1_CTL3           0x80200120
#define HW_DMA1_SSTAT3         0x80200128
#define HW_DMA1_DSTAT3         0x80200130
#define HW_DMA1_SSTATAR3       0x80200138      
#define HW_DMA1_DSTATAR3       0x80200140 
#define HW_DMA1_CFG3           0x80200148
#define HW_DMA1_SGR3           0x80200150
#define HW_DMA1_DSR3           0x80200158

#define HW_DMA1_SAR4           0x80200160
#define HW_DMA1_DAR4           0x80200168
#define HW_DMA1_LLP4           0x80200170
#define HW_DMA1_CTL4           0x80200178
#define HW_DMA1_SSTAT4         0x80200180
#define HW_DMA1_DSTAT4         0x80200188
#define HW_DMA1_SSTATAR4       0x80200190      
#define HW_DMA1_DSTATAR4       0x80200198 
#define HW_DMA1_CFG4           0x802001a0
#define HW_DMA1_SGR4           0x802001a8
#define HW_DMA1_DSR4           0x802001b0

#define HW_DMA1_SAR5           0x802001b8
#define HW_DMA1_DAR5           0x802001c0
#define HW_DMA1_LLP5           0x802001c8
#define HW_DMA1_CTL5           0x802001d0
#define HW_DMA1_SSTAT5         0x802001d8
#define HW_DMA1_DSTAT5         0x802001e0
#define HW_DMA1_SSTATAR5       0x802001e8      
#define HW_DMA1_DSTATAR5       0x802001f0 
#define HW_DMA1_CFG5           0x802001f8
#define HW_DMA1_SGR5           0x80200200
#define HW_DMA1_DSR5           0x80200208

#define HW_DMA1_SAR6           0x80200210
#define HW_DMA1_DAR6           0x80200218
#define HW_DMA1_LLP6           0x80200220
#define HW_DMA1_CTL6           0x80200228
#define HW_DMA1_SSTAT6         0x80200230
#define HW_DMA1_DSTAT6         0x80200238
#define HW_DMA1_SSTATAR6       0x80200240      
#define HW_DMA1_DSTATAR6       0x80200248 
#define HW_DMA1_CFG6           0x80200250
#define HW_DMA1_SGR6           0x80200258
#define HW_DMA1_DSR6           0x80200260

#define HW_DMA1_SAR7           0x80200268
#define HW_DMA1_DAR7           0x80200270
#define HW_DMA1_LLP7           0x80200278
#define HW_DMA1_CTL7           0x80200280
#define HW_DMA1_SSTAT7         0x80200288
#define HW_DMA1_DSTAT7         0x80200290
#define HW_DMA1_SSTATAR7       0x80200298      
#define HW_DMA1_DSTATAR7       0x802002a0 
#define HW_DMA1_CFG7           0x802002a8
#define HW_DMA1_SGR7           0x802002b0
#define HW_DMA1_DSR7           0x802002b8

#define HW_DMA1_RawTFR         0x802002c0
#define HW_DMA1_RawBLOCK       0x802002c8
#define HW_DMA1_RawSRCTRAN     0x802002d0
#define HW_DMA1_RawDSTTRAN     0x802002d8
#define HW_DMA1_RawERR         0x802002e0

#define HW_DMA1_StatusTFR      0x802002e8
#define HW_DMA1_StatusBLOCK    0x802002f0
#define HW_DMA1_StatusSRCTRAN  0x802002f8
#define HW_DMA1_StatusDSTTRAN  0x80200300
#define HW_DMA1_StatusERR      0x80200308

#define HW_DMA1_MaskTFR        0x80200310
#define HW_DMA1_MaskBLOCK      0x80200318
#define HW_DMA1_MaskSRCTRAN    0x80200320
#define HW_DMA1_MaskDSTTRAN    0x80200328
#define HW_DMA1_MaskERR        0x80200330

#define HW_DMA1_ClearTFR       0x80200338
#define HW_DMA1_ClearBLOCK     0x80200340
#define HW_DMA1_ClearSRCTRAN   0x80200348
#define HW_DMA1_ClearDSTTRAN   0x80200350
#define HW_DMA1_ClearERR       0x80200358

#define HW_DMA1_STATUSINT      0x80200360

#define HW_DMA1_ReqSrcReg      0x80200368
#define HW_DMA1_ReqDstReg      0x80200370
#define HW_DMA1_SglReqSrcReg   0x80200378
#define HW_DMA1_SglReqDstReg   0x80200380
#define HW_DMA1_LstSrcReg      0x80200388
#define HW_DMA1_LstDstReg      0x80200390

#define HW_DMA1_DMACFGREG      0x80200398
#define HW_DMA1_CHENREG        0x802003a0


#if 0
/*
 * ----------------------------------------------------------------------------
 * EMI
 * ----------------------------------------------------------------------------
 */

#define HW_EMI_CTRL         (0x80020000)  
#define HW_EMI_STAT         (0x80020010)  
#define HW_EMI_DEBUG        (0x80020020)  
#define HW_EMI_DRAMSTAT     (0x80020080)  
#define HW_EMI_DRAMCRTL     (0x80020090)  
#define HW_EMI_DRAMADDR     (0x800200A0)  
#define HW_EMI_DRAMMODE     (0x800200B0)  
#define HW_EMI_DRAMTIME     (0x800200C0)  
#define HW_EMI_DRAMTIME2    (0x800200D0)  
#define HW_EMI_STATICCTRL   (0x80020100)  
#define HW_EMI_STATICTIME   (0x80020110)  
#endif

/*
 * ----------------------------------------------------------------------------
 * GPMI
 * ----------------------------------------------------------------------------
 */

#define HW_GPMI_CTRL0           (0x80020000)  
#define HW_GPMI_COMPARE         (0x80020010)  
#define HW_GPMI_CTRL1           (0x80020020)  
#define HW_GPMI_TIMING0         (0x80020080)  
#define HW_GPMI_TIMING1         (0x80020090)  
#define HW_GPMI_TIMING2         (0x800200A0)  
#define HW_GPMI_DATA            (0x800200B0)  
#define HW_GPMI_STAT            (0x800200C0)  
#define HW_GPMI_DEBUG           (0x800200D0)  

/*
 * ----------------------------------------------------------------------------
 * SSP -MMC -SD
 * ----------------------------------------------------------------------------
 */ 
#define ALPAS9260_SSP_BASE          (0X8006c000)
#if 0
#define HW_SSP_CTRL0         (ALPAS3310_SSP_BASE + 0)
#define HW_SSP_CMD0          (ALPAS3310_SSP_BASE + 0x10)
#define HW_SSP_CMD1          (ALPAS3310_SSP_BASE + 0x20)
#define HW_SSP_COMPREF       (ALPAS3310_SSP_BASE + 0x30)
#define HW_SSP_COMPMASK      (ALPAS3310_SSP_BASE + 0x40)
#define HW_SSP_TIMING        (ALPAS3310_SSP_BASE + 0x50)
#define HW_SSP_CTRL1         (ALPAS3310_SSP_BASE + 0x60)
#define HW_SSP_DATA          (ALPAS3310_SSP_BASE + 0x70)
#define HW_SSP_SDRESP0       (ALPAS3310_SSP_BASE + 0x80)
#define HW_SSP_SDRESP1       (ALPAS3310_SSP_BASE + 0x90)
#define HW_SSP_SDRESP2       (ALPAS3310_SSP_BASE + 0xa0)
#define HW_SSP_SDRESP3       (ALPAS3310_SSP_BASE + 0xb0)
#define HW_SSP_STATUS        (ALPAS3310_SSP_BASE + 0xc0)
#define HW_SSP_DEBUG         (ALPAS3310_SSP_BASE + 0x100)
#else
#define HW_SSP_CTRL0         0
#define HW_SSP_CMD0          0x10
#define HW_SSP_CMD1          0x20
#define HW_SSP_COMPREF       0x30
#define HW_SSP_COMPMASK      0x40
#define HW_SSP_TIMING        0x50
#define HW_SSP_CTRL1         0x60
#define HW_SSP_DATA          0x70
#define HW_SSP_SDRESP0       0x80
#define HW_SSP_SDRESP1       0x90
#define HW_SSP_SDRESP2       0xa0
#define HW_SSP_SDRESP3       0xb0
#define HW_SSP_STATUS        0xc0
#define HW_SSP_DEBUG         0x100
#define HW_SSP_XFER			 0x110
#endif



/*SPI0*/
#define ALPAS9260_SPI0_BASE      0x80060000

#define HW_SPI0_CTRL0         	 0x80060000
#define HW_SPI0_CTRL1         	 0x80060010
#define HW_SPI0_TIMING          	 0x80060020
#define HW_SPI0_DATA      		 0x80060030
#define HW_SPI0_STATUS     		 0x80060040
#define HW_SPI0_DEBUG0     		 0x80060050
#define HW_SPI0_XFER     		 0x80060060


/*SPI1*/
#define ALPAS9260_SPI1_BASE      0x80064000

#define HW_SPI1_CTRL0         	 0x80064000
#define HW_SPI1_CTRL1         	 0x80064010
#define HW_SPI1_TIMING          	 0x80064020
#define HW_SPI1_DATA      		 0x80064030
#define HW_SPI1_STATUS     		 0x80064040
#define HW_SPI1_DEBUG0     		 0x80064050
#define HW_SPI1_XFER     		 0x80064060


/*QUAD spi*/
#define ALPAS9260_QUAD_SPI_BASE  0x80068000 
  
#define HW_QUAD_SPI_CTRL0        0x80068000
#define HW_QUAD_SPI_CTRL1        0x80068010
#define HW_QUAD_SPI_CMD          0x80068020
#define HW_QUAD_SPI_TIMING       0x80068030
#define HW_QUAD_SPI_DATA         0x80068040
#define HW_QUAD_SPI_STATUS       0x80068050
#define HW_QUAD_SPI_DEBUG0       0x80068060
#define HW_QUAD_SPI_XFER         0x80068070

/*
 * ----------------------------------------------------------------------------
 * LCDIF_1820
 * ----------------------------------------------------------------------------
 */ 
#define HW_LCDIF_BASE 					(0x80808000)
#define	HW_LCDIF_YUV_CTRL 				(0x80808000)
#define	HW_LCDIF_YUV_CTRL1 				(0x80808010)
#define	HW_LCDIF_YUV_TIMING 			(0x80808020)
#define	HW_LCDIF_YUV_TIMING1        	(0x80808030)
#define	HW_LCDIF_YUV_DATA 		    	(0x80808040)
#define	HW_LCDIF_YRGB_DATA          	(0x80808050)
#define	HW_LCDIF_U_DATA             	(0x80808060)
#define HW_LCDIF_V_DATA             	(0x80808070)
#define HW_LCDIF_UV_XFER            	(0x80808080)

/*
 * ----------------------------------------------------------------------------
 * LCDIF_1810
 * ----------------------------------------------------------------------------
 */ 

#define HW_LCDIF_CTRL            0x80060000
#define HW_LCDIF_CTRL_SET        (0x80060000 + 0x4)
#define HW_LCDIF_CTRL_CLR        (0x80060000 + 0x8)
#define HW_LCDIF_CTRL_TOG        (0x80060000 + 0xc)

#define HW_LCDIF_TIMING          (0x80060010)
#define HW_LCDIF_DATA            (0x80060020)
#define HW_LCDIF_DEBUG           (0x80060030)

/*
 * ----------------------------------------------------------------------------
 * LCD
 * ----------------------------------------------------------------------------
 */
#define HW_APBH_LCD_CTRL0 			(0x80800000)
#define HW_APBH_LCD_CTRL0_SET 		(0x80800004)
#define HW_APBH_LCD_CTRL0_CLR 		(0x80800008)
#define HW_APBH_LCD_CTRL0_TOG 		(0x8080000C)

#define HW_APBH_LCD_CH0_NXTCMDAR    (0x80800040)
#define HW_APBH_LCD_CH0_SEMA        (0x80800070)

#define HW_APBH_LCD_CH1_NXTCMDAR    (0x808000B0)
#define HW_APBH_LCD_CH1_SEMA        (0x808000E0)

#define HW_APBH_LCD_CH2_NXTCMDAR    (0x80800120)
#define HW_APBH_LCD_CH2_SEMA        (0x80800150)


/*
 * ----------------------------------------------------------------------------
 * PIN CONTROL FOR ASM9260
 * ----------------------------------------------------------------------------
 */
/////////////////////////////////////////////////////////
//GPIO

#define GPIO_BASEESS       0x50000000
#define HW_GPIO_DATA_BASE 	0x50000000

#define HW_GPIO_DMA_CTRL        0x50000010
#define HW_GPIO_DMA_DATA        0x50000020
#define HW_GPIO_DMA_PADCTRL0    0x50000030
#define HW_GPIO_DMA_PADCTRL1    0x50000040
#define HW_GPIO_DMA_PADCTRL2    0x50000050


#define HW_GPIO_DATA0           0x50000000
#define HW_GPIO_DATA1           0x50010000
#define HW_GPIO_DATA2           0x50020000
#define HW_GPIO_DATA3           0x50030000
#define HW_GPIO_DIR0            0x50008000
#define HW_GPIO_DIR1            0x50018000
#define HW_GPIO_DIR2            0x50028000
#define HW_GPIO_DIR3            0x50038000
#define HW_GPIO_IS0             0x50008010
#define HW_GPIO_IS1             0x50018010
#define HW_GPIO_IS2             0x50028010
#define HW_GPIO_IS3             0x50038010
#define HW_GPIO_IBE0            0x50008020
#define HW_GPIO_IBE1            0x50018020
#define HW_GPIO_IBE2            0x50028020
#define HW_GPIO_IBE3            0x50038020
#define HW_GPIO_IEV0            0x50008030
#define HW_GPIO_IEV1            0x50018030
#define HW_GPIO_IEV2            0x50028030
#define HW_GPIO_IEV3            0x50038030
#define HW_GPIO_IE0             0x50008040
#define HW_GPIO_IE1             0x50018040
#define HW_GPIO_IE2             0x50028040
#define HW_GPIO_IE3             0x50038040
#define HW_GPIO_IRS0            0x50008050
#define HW_GPIO_IRS1            0x50018050
#define HW_GPIO_IRS2            0x50028050
#define HW_GPIO_IRS3            0x50038050
#define HW_GPIO_MIS0            0x50008060
#define HW_GPIO_MIS1            0x50018060
#define HW_GPIO_MIS2            0x50028060
#define HW_GPIO_MIS3            0x50038060
#define HW_GPIO_IC0             0x50008070
#define HW_GPIO_IC1             0x50018070
#define HW_GPIO_IC2             0x50028070
#define HW_GPIO_IC3             0x50038070 
#define HW_GPIO_DATAMASK0       0x50008080
#define HW_GPIO_DATAMASK1       0x50018080
#define HW_GPIO_DATAMASK2       0x50028080
#define HW_GPIO_DATAMASK3       0x50038080 

                                             
/////////////////////////////////////////////////////////
//IOCON FOR ASM9260
#define HW_IOCON_PIO_BASE	HW_IOCON_PIO0_0

#define HW_IOCON_PIO0_0        0x80044000
#define HW_IOCON_PIO0_1        0x80044004
#define HW_IOCON_PIO0_2        0x80044008
#define HW_IOCON_PIO0_3        0x8004400C
#define HW_IOCON_PIO0_4        0x80044010
#define HW_IOCON_PIO0_5        0x80044014
#define HW_IOCON_PIO0_6        0x80044018 
#define HW_IOCON_PIO0_7        0x8004401c
#define HW_IOCON_PIO1_0        0x80044020
#define HW_IOCON_PIO1_1        0x80044024
#define HW_IOCON_PIO1_2        0x80044028
#define HW_IOCON_PIO1_3        0x8004402c
#define HW_IOCON_PIO1_4        0x80044030
#define HW_IOCON_PIO1_5        0x80044034
#define HW_IOCON_PIO1_6        0x80044038
#define HW_IOCON_PIO1_7        0x8004403c
#define HW_IOCON_PIO2_0        0x80044040
#define HW_IOCON_PIO2_1        0x80044044
#define HW_IOCON_PIO2_2        0x80044048
#define HW_IOCON_PIO2_3        0x8004404c
#define HW_IOCON_PIO2_4        0x80044050
#define HW_IOCON_PIO2_5        0x80044054
#define HW_IOCON_PIO2_6        0x80044058
#define HW_IOCON_PIO2_7        0x8004405c
#define HW_IOCON_PIO3_0        0x80044060
#define HW_IOCON_PIO3_1        0x80044064
#define HW_IOCON_PIO3_2        0x80044068
#define HW_IOCON_PIO3_3        0x8004406c
#define HW_IOCON_PIO3_4        0x80044070
#define HW_IOCON_PIO3_5        0x80044074
#define HW_IOCON_PIO3_6        0x80044078
#define HW_IOCON_PIO3_7        0x8004407c
#define HW_IOCON_PIO4_0        0x80044080
#define HW_IOCON_PIO4_1        0x80044084
#define HW_IOCON_PIO4_2        0x80044088
#define HW_IOCON_PIO4_3        0x8004408c
#define HW_IOCON_PIO4_4        0x80044090
#define HW_IOCON_PIO4_5        0x80044094
#define HW_IOCON_PIO4_6        0x80044098
#define HW_IOCON_PIO4_7        0x8004409c
#define HW_IOCON_PIO5_0        0x800440a0
#define HW_IOCON_PIO5_1        0x800440a4
#define HW_IOCON_PIO5_2        0x800440a8
#define HW_IOCON_PIO5_3        0x800440ac
#define HW_IOCON_PIO5_4        0x800440b0
#define HW_IOCON_PIO5_5        0x800440b4
#define HW_IOCON_PIO5_6        0x800440b8
#define HW_IOCON_PIO5_7        0x800440bc
#define HW_IOCON_PIO6_0        0x800440c0
#define HW_IOCON_PIO6_1        0x800440c4
#define HW_IOCON_PIO6_2        0x800440c8
#define HW_IOCON_PIO6_3        0x800440cc
#define HW_IOCON_PIO6_4        0x800440d0
#define HW_IOCON_PIO6_5        0x800440d4
#define HW_IOCON_PIO6_6        0x800440d8
#define HW_IOCON_PIO6_7        0x800440dc
#define HW_IOCON_PIO7_0        0x800440e0
#define HW_IOCON_PIO7_1        0x800440e4
#define HW_IOCON_PIO7_2        0x800440e8
#define HW_IOCON_PIO7_3        0x800440ec
#define HW_IOCON_PIO7_4        0x800440f0
#define HW_IOCON_PIO7_5        0x800440f4
#define HW_IOCON_PIO7_6        0x800440f8
#define HW_IOCON_PIO7_7        0x800440fc
#define HW_IOCON_PIO8_0        0x80044100
#define HW_IOCON_PIO8_1        0x80044104
#define HW_IOCON_PIO8_2        0x80044108
#define HW_IOCON_PIO8_3        0x8004410c
#define HW_IOCON_PIO8_4        0x80044110
#define HW_IOCON_PIO8_5        0x80044114
#define HW_IOCON_PIO8_6        0x80044118
#define HW_IOCON_PIO8_7        0x8004411c
#define HW_IOCON_PIO9_0        0x80044120
#define HW_IOCON_PIO9_1        0x80044124
#define HW_IOCON_PIO9_2        0x80044128
#define HW_IOCON_PIO9_3        0x8004412c
#define HW_IOCON_PIO9_4        0x80044130
#define HW_IOCON_PIO9_5        0x80044134
#define HW_IOCON_PIO9_6        0x80044138
#define HW_IOCON_PIO9_7        0x8004413c
#define HW_IOCON_PIO10_0       0x80044140
#define HW_IOCON_PIO10_1       0x80044144
#define HW_IOCON_PIO10_2       0x80044148
#define HW_IOCON_PIO10_3       0x8004414c
#define HW_IOCON_PIO10_4       0x80044150
#define HW_IOCON_PIO10_5       0x80044154
#define HW_IOCON_PIO10_6       0x80044158
#define HW_IOCON_PIO10_7       0x8004415c

#define HW_IOCON_PIO11_0       0x80044160
#define HW_IOCON_PIO11_1       0x80044164
#define HW_IOCON_PIO11_2       0x80044168
#define HW_IOCON_PIO11_3       0x8004416c
#define HW_IOCON_PIO11_4       0x80044170
#define HW_IOCON_PIO11_5       0x80044174
#define HW_IOCON_PIO11_6       0x80044178
#define HW_IOCON_PIO11_7       0x8004417c

#define HW_IOCON_PIO12_0       0x80044180
#define HW_IOCON_PIO12_1       0x80044184
#define HW_IOCON_PIO12_2       0x80044188
#define HW_IOCON_PIO12_3       0x8004418c
#define HW_IOCON_PIO12_4       0x80044190
#define HW_IOCON_PIO12_5       0x80044194
#define HW_IOCON_PIO12_6       0x80044198
#define HW_IOCON_PIO12_7       0x8004419c

#define HW_IOCON_PIO13_0       0x800441a0
#define HW_IOCON_PIO13_1       0x800441a4
#define HW_IOCON_PIO13_2       0x800441a8
#define HW_IOCON_PIO13_3       0x800441ac
#define HW_IOCON_PIO13_4       0x800441b0
#define HW_IOCON_PIO13_5       0x800441b4
#define HW_IOCON_PIO13_6       0x800441b8
#define HW_IOCON_PIO13_7       0x800441bc

#define HW_IOCON_PIO14_0       0x800441c0
#define HW_IOCON_PIO14_1       0x800441c4
#define HW_IOCON_PIO14_2       0x800441c8
#define HW_IOCON_PIO14_3       0x800441cc
#define HW_IOCON_PIO14_4       0x800441d0
#define HW_IOCON_PIO14_5       0x800441d4
#define HW_IOCON_PIO14_6       0x800441d8
#define HW_IOCON_PIO14_7       0x800441dc

#define HW_IOCON_PIO15_0       0x800441e0
#define HW_IOCON_PIO15_1       0x800441e4
#define HW_IOCON_PIO15_2       0x800441e8
#define HW_IOCON_PIO15_3       0x800441ec
#define HW_IOCON_PIO15_4       0x800441f0
#define HW_IOCON_PIO15_5       0x800441f4
#define HW_IOCON_PIO15_6       0x800441f8
#define HW_IOCON_PIO15_7       0x800441fc

#define HW_IOCON_PIO16_0       0x80044200
#define HW_IOCON_PIO16_1       0x80044204
#define HW_IOCON_PIO16_2       0x80044208
#define HW_IOCON_PIO16_3       0x8004420c
#define HW_IOCON_PIO16_4       0x80044210
#define HW_IOCON_PIO16_5       0x80044214
#define HW_IOCON_PIO16_6       0x80044218
#define HW_IOCON_PIO16_7       0x8004421c

#define HW_IOCON_PIO17_0       0x80044220
#define HW_IOCON_PIO17_1       0x80044224
#define HW_IOCON_PIO17_2       0x80044228
#define HW_IOCON_PIO17_3       0x8004422c
#define HW_IOCON_PIO17_4       0x80044230
#define HW_IOCON_PIO17_5       0x80044234
#define HW_IOCON_PIO17_6       0x80044238
#define HW_IOCON_PIO17_7       0x8004423c

#define HW_IOCON_SCKLOC        0x800442c0                                             
                                             
/*
 * ----------------------------------------------------------------------------
 * TIMER
 * ----------------------------------------------------------------------------
 */
#if 0
#define HW_TIMER_BASE		            0x80068000
#define HW_TIMROT_ROTCTRL	            (HW_TIMER_BASE)	//	Timer CTRL register
#define HW_TIMROT_ROTCOUNT 	            (HW_TIMER_BASE + 0x10)	//	Timer COUNT register
#define HW_TIMROT_TIMCTRL0 	            (HW_TIMER_BASE + 0x20)	//	Timer 0 control register
#define HW_TIMROT_TIMCOUNTER0	        (HW_TIMER_BASE + 0x30)	//	Timer 0 counter register
#define HW_TIMROT_TIMCTRL1 	            (HW_TIMER_BASE + 0x40)	//	Timer 1 control register
#define HW_TIMROT_TIMCOUNTER1           (HW_TIMER_BASE + 0x50)	//	Timer 1 counter register
#define HW_TIMROT_TIMCTRL2 	            (HW_TIMER_BASE + 0x60)	//	Timer 2 control register
#define HW_TIMROT_TIMCOUNTER2	        (HW_TIMER_BASE + 0x70)	//	Timer 2 counter register
#define HW_TIMROT_TIMCTRL3 	            (HW_TIMER_BASE + 0x80)	//	Timer 3 control register
#define HW_TIMROT_TIMCOUNTER3	        (HW_TIMER_BASE + 0x90)	//	Timer 3 counter register
#endif
/////////////////////////////////////////////////////////
//CT32b0

#define TIMER0_BASE_ADDRESS         0x80088000
#define HW_TIMER0_IR                0x80088000
#define HW_TIMER0_TCR               0x80088010
#define HW_TIMER0_DIR               0x80088020
#define HW_TIMER0_TC0               0x80088030
#define HW_TIMER0_TC1               0x80088040
#define HW_TIMER0_TC2               0x80088050
#define HW_TIMER0_TC3               0x80088060
#define HW_TIMER0_PR                0x80088070
#define HW_TIMER0_PC                0x80088080
#define HW_TIMER0_MCR               0x80088090
#define HW_TIMER0_MR0               0x800880a0
#define HW_TIMER0_MR1               0x800880b0
#define HW_TIMER0_MR2               0x800880C0
#define HW_TIMER0_MR3               0x800880D0
#define HW_TIMER0_CCR               0x800880E0
#define HW_TIMER0_CR0               0x800880F0
#define HW_TIMER0_CR1               0x80088100
#define HW_TIMER0_CR2               0x80088110
#define HW_TIMER0_CR3               0x80088120
#define HW_TIMER0_EMR               0x80088130
#define HW_TIMER0_PWMTH0            0x80088140
#define HW_TIMER0_PWMTH1            0x80088150
#define HW_TIMER0_PWMTH2            0x80088160
#define HW_TIMER0_PWMTH3            0x80088170
#define HW_TIMER0_CTCR              0x80088180
#define HW_TIMER0_PWMC              0x80088190


/////////////////////////////////////////////////////////
//CT32b1

#define TIMER1_BASE_ADDRESS         0x8008C000
#define HW_TIMER1_IR                0x8008C000
#define HW_TIMER1_TCR               0x8008C010
#define HW_TIMER1_DIR               0x8008C020
#define HW_TIMER1_TC0               0x8008C030
#define HW_TIMER1_TC1               0x8008C040
#define HW_TIMER1_TC2               0x8008C050
#define HW_TIMER1_TC3               0x8008C060
#define HW_TIMER1_PR                0x8008C070
#define HW_TIMER1_PC                0x8008C080
#define HW_TIMER1_MCR               0x8008C090
#define HW_TIMER1_MR0               0x8008C0a0
#define HW_TIMER1_MR1               0x8008C0b0
#define HW_TIMER1_MR2               0x8008C0c0
#define HW_TIMER1_MR3               0x8008C0d0
#define HW_TIMER1_CCR               0x8008C0e0
#define HW_TIMER1_CR0               0x8008C0f0
#define HW_TIMER1_CR1               0x8008C100
#define HW_TIMER1_CR2               0x8008C110
#define HW_TIMER1_CR3               0x8008C120
#define HW_TIMER1_EMR               0x8008C130
#define HW_TIMER1_PWMTH0            0x8008C140
#define HW_TIMER1_PWMTH1            0x8008C150
#define HW_TIMER1_PWMTH2            0x8008C160
#define HW_TIMER1_PWMTH3            0x8008C170
#define HW_TIMER1_CTCR              0x8008C180
#define HW_TIMER1_PWMC              0x8008C190

/////////////////////////////////////////////////////////
//CT32b2

#define TIMER2_BASE_ADDRESS         0x80090000
#define HW_TIMER2_IR                0x80090000
#define HW_TIMER2_TCR               0x80090010
#define HW_TIMER2_DIR               0x80090020
#define HW_TIMER2_TC0               0x80090030
#define HW_TIMER2_TC1               0x80090040
#define HW_TIMER2_TC2               0x80090050
#define HW_TIMER2_TC3               0x80090060
#define HW_TIMER2_PR                0x80090070
#define HW_TIMER2_PC                0x80090080
#define HW_TIMER2_MCR               0x80090090
#define HW_TIMER2_MR0               0x800900a0
#define HW_TIMER2_MR1               0x800900b0
#define HW_TIMER2_MR2               0x800900c0
#define HW_TIMER2_MR3               0x800900d0
#define HW_TIMER2_CCR               0x800900e0
#define HW_TIMER2_CR0               0x800900f0
#define HW_TIMER2_CR1               0x80090100
#define HW_TIMER2_CR2               0x80090110
#define HW_TIMER2_CR3               0x80090120
#define HW_TIMER2_EMR               0x80090130
#define HW_TIMER2_PWMTH0            0x80090140
#define HW_TIMER2_PWMTH1            0x80090150
#define HW_TIMER2_PWMTH2            0x80090160
#define HW_TIMER2_PWMTH3            0x80090170
#define HW_TIMER2_CTCR              0x80090180
#define HW_TIMER2_PWMC              0x80090190


/////////////////////////////////////////////////////////
//CT32b3

#define TIMER3_BASE_ADDRESS         0x80094000
#define HW_TIMER3_IR                0x80094000
#define HW_TIMER3_TCR               0x80094010
#define HW_TIMER3_DIR               0x80094020
#define HW_TIMER3_TC0               0x80094030
#define HW_TIMER3_TC1               0x80094040
#define HW_TIMER3_TC2               0x80094050
#define HW_TIMER3_TC3               0x80094060
#define HW_TIMER3_PR                0x80094070
#define HW_TIMER3_PC                0x80094080
#define HW_TIMER3_MCR               0x80094090
#define HW_TIMER3_MR0               0x800940a0
#define HW_TIMER3_MR1               0x800940b0
#define HW_TIMER3_MR2               0x800940c0
#define HW_TIMER3_MR3               0x800940d0
#define HW_TIMER3_CCR               0x800940e0
#define HW_TIMER3_CR0               0x800940f0
#define HW_TIMER3_CR1               0x80094100
#define HW_TIMER3_CR2               0x80094110
#define HW_TIMER3_CR3               0x80094120
#define HW_TIMER3_EMR               0x80094130
#define HW_TIMER3_PWMTH0            0x80094140
#define HW_TIMER3_PWMTH1            0x80094150
#define HW_TIMER3_PWMTH2            0x80094160
#define HW_TIMER3_PWMTH3            0x80094170
#define HW_TIMER3_CTCR              0x80094180
#define HW_TIMER3_PWMC              0x80094190

///////////////////////////////////////////////////////
//MCPWM
#define MCPWM_BASE_ADDRESS        0x8005C000
#define HW_MCPWM_CON              0x8005C000
#define HW_MCPWM_CON_SET          0x8005C004
#define HW_MCPWM_CON_CLR          0x8005C008
#define HW_MCPWM_CAPCON           0x8005C00C
#define HW_MCPWM_CAPCON_SET       0x8005C010
#define HW_MCPWM_CAPCON_CLR       0x8005C014
#define HW_MCPWM_TC0              0x8005C018
#define HW_MCPWM_TC1              0x8005C01C
#define HW_MCPWM_TC2              0x8005C020
#define HW_MCPWM_LIM0             0x8005C024
#define HW_MCPWM_LIM1             0x8005C028
#define HW_MCPWM_LIM2             0x8005C02C
#define HW_MCPWM_MAT0             0x8005C030
#define HW_MCPWM_MAT1             0x8005C034
#define HW_MCPWM_MAT2             0x8005C038
#define HW_MCPWM_DT               0x8005C03C
#define HW_MCPWM_CCP              0x8005C040
#define HW_MCPWM_CAP0             0x8005C044
#define HW_MCPWM_CAP1             0x8005C048
#define HW_MCPWM_CAP2             0x8005C04C
#define HW_MCPWM_INTEN            0x8005C050
#define HW_MCPWM_INTEN_SET        0x8005C054
#define HW_MCPWM_INTEN_CLR        0x8005C058
#define HW_MCPWM_CNTCON           0x8005C05C
#define HW_MCPWM_CNTCON_SET       0x8005C060
#define HW_MCPWM_CNTCON_CLR       0x8005C064
#define HW_MCPWM_INTF             0x8005C068
#define HW_MCPWM_INTF_SET         0x8005C06C
#define HW_MCPWM_INTF_CLR         0x8005C070
#define HW_MCPWM_CAP_CLR          0x8005C074
#define HW_MCPWM_HALL             0x8005C078
#define HW_MCPWM_HALLS            0x8005C07C
#define HW_MCPWM_HALL_VEL_CMP     0x8005C080
#define HW_MCPWM_HALL_VEL_VAL     0x8005C084
#define HW_MCPWM_HALL_VEL_TH      0x8005C088
#define HW_MCPWM_HALL_MCIST       0x8005C08C

 
/*
 * ----------------------------------------------------------------------------
 * RTC
 * ----------------------------------------------------------------------------
 */

#define  HW_RTC_ILR        0x800A0000
#define  HW_RTC_CCR        0x800A0008
#define  HW_RTC_CIIR       0x800A000c
#define  HW_RTC_AMR        0x800A0010
#define  HW_RTC_CTIME0     0x800A0014
#define  HW_RTC_CTIME1     0x800A0018
#define  HW_RTC_CTIME2     0x800A001c
#define  HW_RTC_SEC        0x800A0020
#define  HW_RTC_MIN        0x800A0024
#define  HW_RTC_HOUR       0x800A0028
#define  HW_RTC_DOM        0x800A002c
#define  HW_RTC_DOW        0x800A0030
#define  HW_RTC_DOY        0x800A0034
#define  HW_RTC_MONTH      0x800A0038
#define  HW_RTC_YEAR       0x800A003c
#define  HW_RTC_CAL        0x800A0040
#define  HW_RTC_GP0        0x800A0044
#define  HW_RTC_GP1        0x800A0048
#define  HW_RTC_GP2        0x800A004c
#define  HW_RTC_GP3        0x800A0050
#define  HW_RTC_GP4        0x800A0054
#define  HW_RTC_ALSEC      0x800A0060
#define  HW_RTC_ALMIN      0x800A0064
#define  HW_RTC_ALHOUR     0x800A0068
#define  HW_RTC_ALDOM      0x800A006c
#define  HW_RTC_ALDOW      0x800A0070
#define  HW_RTC_ALDOY      0x800A0074
#define  HW_RTC_ALMONTH    0x800A0078
#define  HW_RTC_ALYEAR     0x800A007c
#define  HW_RTC_ALARM      0x800A0080

/*
 * ----------------------------------------------------------------------------
 *  Watchdog 
 * ----------------------------------------------------------------------------
 */
#define HW_WATCHDOG_WDMOD           0x80048000
#define HW_WATCHDOG_WDTC            0x80048004
#define HW_WATCHDOG_WDFEED          0x80048008
#define HW_WATCHDOG_WDTV            0x8004800C

  /*
 * ----------------------------------------------------------------------------
 * PWM
 * ----------------------------------------------------------------------------
 */ 
#define HW_PWM_CTRL (0x80064000)
#define HW_PWM_CTRL_SET (HW_PWM_CTRL + 0x4)
#define HW_PWM_CTRL_CLR (HW_PWM_CTRL + 0x8)
#define HW_PWM_CTRL_TOG (HW_PWM_CTRL + 0xc)

#define HW_PWM_ACTIVE(x) (HW_PWM_CTRL + 0x10 + 0x20*(x))
#define HW_PWM_PERIOD(x) (HW_PWM_CTRL + 0x20 + 0x20*(x))

//#define HW_PWM_ACTIVE_SET(x) (HW_PWM_ACTIVE(x) + 0x4)
//#define HW_PWM_ACTIVE_CLR(x) (HW_PWM_ACTIVE(x) + 0x8)
//#define HW_PWM_ACTIVE_TOG(x) (HW_PWM_ACTIVE(x) + 0xc)
//                                                   
//#define HW_PWM_PERIOD_SET(x) (HW_PWM_PERIOD(x) + 0x4)
//#define HW_PWM_PERIOD_CLR(x) (HW_PWM_PERIOD(x) + 0x8)
//#define HW_PWM_PERIOD_TOG(x) (HW_PWM_PERIOD(x) + 0xc) 

/*
 * ----------------------------------------------------------------------------
 * I2C 0
 * ----------------------------------------------------------------------------
 */ 
#define I2C0_BASE_ADDRESS         0x8002c000

#define HW_I2C0_CON                0x8002c000
#define HW_I2C0_TAR                0x8002c004
#define HW_I2C0_SAR                0x8002c008
#define HW_I2C0_HS_MADDR           0x8002c00C
#define HW_I2C0_DATA_CMD           0x8002c010
#define HW_I2C0_SS_SCL_HCNT        0x8002c014
#define HW_I2C0_SS_SCL_LCNT        0x8002c018
#define HW_I2C0_FS_SCL_HCNT        0x8002c01C
#define HW_I2C0_FS_SCL_LCNT        0x8002c020
#define HW_I2C0_HS_SCL_HCNT        0x8002c024
#define HW_I2C0_HS_SCL_LCNT        0x8002c028
#define HW_I2C0_INTR_STAT          0x8002c02C
#define HW_I2C0_INTR_MASK          0x8002c030
#define HW_I2C0_RAW_INTR_STAT      0x8002c034
#define HW_I2C0_RX_TL              0x8002c038
#define HW_I2C0_TX_TL              0x8002c03C
#define HW_I2C0_CLR_INTR           0x8002c040    
#define HW_I2C0_CLR_RX_UNDER       0x8002c044
#define HW_I2C0_CLR_RX_OVER        0x8002c048
#define HW_I2C0_CLR_TX_OVER        0x8002c04C
#define HW_I2C0_CLR_RD_REQ         0x8002c050
#define HW_I2C0_CLR_TX_ABRT        0x8002c054
#define HW_I2C0_CLR_RX_DONE        0x8002c058
#define HW_I2C0_CLR_ACTIVITY       0x8002c05C
#define HW_I2C0_CLR_STOP_DET       0x8002c060
#define HW_I2C0_CLR_START_DET      0x8002c064
#define HW_I2C0_CLR_GEN_CALL       0x8002c068
#define HW_I2C0_ENABLE             0x8002c06C
#define HW_I2C0_STAT               0x8002c070
#define HW_I2C0_TXFLR              0x8002c074
#define HW_I2C0_RXFLR              0x8002c078

#define HW_I2C0_TX_ABRT_SOURCE     0x8002c080
#define HW_I2C0_SLV_DATA_NACK_ONLY 0x8002c084
#define HW_I2C0_DMA_CR             0x8002c088
#define HW_I2C0_DMA_TDLR           0x8002c08C
#define HW_I2C0_DMA_RDLR           0x8002c090
#define HW_I2C0_SDA_SETUP          0x8002c094
#define HW_I2C0_ACK_GENERAL_CALL   0x8002c098
#define HW_I2C0_ENABLE_STATUS      0x8002c09C
#define HW_I2C0_COMP_PARAM_1       0x8002c0f4
#define HW_I2C0_COMP_VERSION       0x8002c0f8
#define HW_I2C0_COMP_TYPE          0x8002c0fc

/*
 * ----------------------------------------------------------------------------
 * I2C 1
 * ----------------------------------------------------------------------------
 */ 
#define I2C1_BASE_ADDRESS         0x80030000

#define HW_I2C1_CON                0x80030000
#define HW_I2C1_TAR                0x80030004
#define HW_I2C1_SAR                0x80030008
#define HW_I2C1_HS_MADDR           0x8003000C
#define HW_I2C1_DATA_CMD           0x80030010
#define HW_I2C1_SS_SCL_HCNT        0x80030014
#define HW_I2C1_SS_SCL_LCNT        0x80030018
#define HW_I2C1_FS_SCL_HCNT        0x8003001C
#define HW_I2C1_FS_SCL_LCNT        0x80030020
#define HW_I2C1_HS_SCL_HCNT        0x80030024
#define HW_I2C1_HS_SCL_LCNT        0x80030028
#define HW_I2C1_INTR_STAT          0x8003002C
#define HW_I2C1_INTR_MASK          0x80030030
#define HW_I2C1_RAW_INTR_STAT      0x80030034
#define HW_I2C1_RX_TL              0x80030038
#define HW_I2C1_TX_TL              0x8003003C
#define HW_I2C1_CLR_INTR           0x80030040    
#define HW_I2C1_CLR_RX_UNDER       0x80030044
#define HW_I2C1_CLR_RX_OVER        0x80030048
#define HW_I2C1_CLR_TX_OVER        0x8003004C
#define HW_I2C1_CLR_RD_REQ         0x80030050
#define HW_I2C1_CLR_TX_ABRT        0x80030054
#define HW_I2C1_CLR_RX_DONE        0x80030058
#define HW_I2C1_CLR_ACTIVITY       0x8003005C
#define HW_I2C1_CLR_STOP_DET       0x80030060
#define HW_I2C1_CLR_START_DET      0x80030064
#define HW_I2C1_CLR_GEN_CALL       0x80030068
#define HW_I2C1_ENABLE             0x8003006C
#define HW_I2C1_STAT               0x80030070
#define HW_I2C1_TXFLR              0x80030074
#define HW_I2C1_RXFLR              0x80030078

#define HW_I2C1_TX_ABRT_SOURCE     0x80030080
#define HW_I2C1_SLV_DATA_NACK_ONLY 0x80030084
#define HW_I2C1_DMA_CR             0x80030088
#define HW_I2C1_DMA_TDLR           0x8003008C
#define HW_I2C1_DMA_RDLR           0x80030090
#define HW_I2C1_SDA_SETUP          0x80030094
#define HW_I2C1_ACK_GENERAL_CALL   0x80030098
#define HW_I2C1_ENABLE_STATUS      0x8003009C
#define HW_I2C1_COMP_PARAM_1       0x800300f4
#define HW_I2C1_COMP_VERSION       0x800300f8
#define HW_I2C1_COMP_TYPE          0x800300fc



#if 0
/*
 * ----------------------------------------------------------------------------
 * APPUART
 * ----------------------------------------------------------------------------
 */ 
#ifndef __ALPAS3310_UART1_BASE_H
#define __ALPAS3310_UART1_BASE_H

#define ALPAS3310_UART1_BASE_PHY	  (0x8006C000)
#define ALPAS3310_UART1_BASE	      (0xf006C000)	 
#define HW_UARTAPP_CTRL0              ALPAS3310_UART1_BASE
#define HW_UARTAPP_CTRL1              (ALPAS3310_UART1_BASE + 0x10)
#define HW_UARTAPP_CTRL2              (ALPAS3310_UART1_BASE + 0x20)
#define HW_UARTAPP_LINE_CTRL          (ALPAS3310_UART1_BASE + 0x30)
#define HW_UARTAPP_INTR               (ALPAS3310_UART1_BASE + 0x40)
#define HW_UARTAPP_INTR_CLEAR         (ALPAS3310_UART1_BASE + 0x48)
#define HW_UARTAPP_DATA               (ALPAS3310_UART1_BASE + 0x50)
#define HW_UARTAPP_SATA               (ALPAS3310_UART1_BASE + 0x60)
#define HW_UARTAPP_DEBUG              (ALPAS3310_UART1_BASE + 0x70)
#endif 
#endif

#define ASM9260_MAX_UART	10

/////////////////////////////////////////////////////////
//UART0
#define UART0_BASEESS  0x80000000

#define HW_UART0_CTRL0          (UART0_BASEESS + 0x00)
#define HW_UART0_CTRL1          (UART0_BASEESS + 0x10)
#define HW_UART0_CTRL2          (UART0_BASEESS + 0x20)
#define HW_UART0_LINECTRL       (UART0_BASEESS + 0x30)
#define HW_UART0_INTR           (UART0_BASEESS + 0x40)
#define HW_UART0_DATA           (UART0_BASEESS + 0x50)
#define HW_UART0_STAT           (UART0_BASEESS + 0x60)
#define HW_UART0_DEBUG          (UART0_BASEESS + 0x70)
#define HW_UART0_ILPR           (UART0_BASEESS + 0x80)
#define HW_UART0_RS485CTRL      (UART0_BASEESS + 0x90)
#define HW_UART0_RS485ADRMATCH  (UART0_BASEESS + 0xa0)
#define HW_UART0_RS485DLY       (UART0_BASEESS + 0xb0)
#define HW_UART0_AUTOBAUD       (UART0_BASEESS + 0xc0)
#define HW_UART0_CTRL3          (UART0_BASEESS + 0xd0)
#define HW_UART0_ISO7816        (UART0_BASEESS + 0xe0)

#define HW_UART0_ISO7816_ERR_CNT (UART0_BASEESS + 0xf0)
#define HW_UART0_ISO7816_STATUS (UART0_BASEESS + 0x100)

/////////////////////////////////////////////////////////
//UART1
#define UART1_BASEESS  0x80004000

#define HW_UART1_CTRL0          (UART1_BASEESS + 0x00)
#define HW_UART1_CTRL1          (UART1_BASEESS + 0x10)
#define HW_UART1_CTRL2          (UART1_BASEESS + 0x20)
#define HW_UART1_LINECTRL       (UART1_BASEESS + 0x30)
#define HW_UART1_INTR           (UART1_BASEESS + 0x40)
#define HW_UART1_DATA           (UART1_BASEESS + 0x50)
#define HW_UART1_STAT           (UART1_BASEESS + 0x60)
#define HW_UART1_DEBUG          (UART1_BASEESS + 0x70)
#define HW_UART1_ILPR           (UART1_BASEESS + 0x80)
#define HW_UART1_RS485CTRL      (UART1_BASEESS + 0x90)
#define HW_UART1_RS485ADRMATCH  (UART1_BASEESS + 0xa0)
#define HW_UART1_RS485DLY       (UART1_BASEESS + 0xb0)
#define HW_UART1_AUTOBAUD       (UART1_BASEESS + 0xc0)
#define HW_UART1_CTRL3          (UART1_BASEESS + 0xd0)
#define HW_UART1_ISO7816        (UART1_BASEESS + 0xe0)

#define HW_UART1_ISO7816_ERR_CNT (UART1_BASEESS + 0xf0)
#define HW_UART1_ISO7816_STATUS (UART1_BASEESS + 0x100)

/////////////////////////////////////////////////////////
//UART2
#define UART2_BASEESS  0x80008000

#define HW_UART2_CTRL0          (UART2_BASEESS + 0x00)
#define HW_UART2_CTRL1          (UART2_BASEESS + 0x10)
#define HW_UART2_CTRL2          (UART2_BASEESS + 0x20)
#define HW_UART2_LINECTRL       (UART2_BASEESS + 0x30)
#define HW_UART2_INTR           (UART2_BASEESS + 0x40)
#define HW_UART2_DATA           (UART2_BASEESS + 0x50)
#define HW_UART2_STAT           (UART2_BASEESS + 0x60)
#define HW_UART2_DEBUG          (UART2_BASEESS + 0x70)
#define HW_UART2_ILPR           (UART2_BASEESS + 0x80)
#define HW_UART2_RS485CTRL      (UART2_BASEESS + 0x90)
#define HW_UART2_RS485ADRMATCH  (UART2_BASEESS + 0xa0)
#define HW_UART2_RS485DLY       (UART2_BASEESS + 0xb0)
#define HW_UART2_AUTOBAUD       (UART2_BASEESS + 0xc0)
#define HW_UART2_CTRL3          (UART2_BASEESS + 0xd0)
#define HW_UART2_ISO7816        (UART2_BASEESS + 0xe0)

#define HW_UART2_ISO7816_ERR_CNT (UART2_BASEESS + 0xf0)
#define HW_UART2_ISO7816_STATUS (UART2_BASEESS + 0x100)

/////////////////////////////////////////////////////////
//UART3
#define UART3_BASEESS  0x8000c000

#define HW_UART3_CTRL0          (UART3_BASEESS + 0x00)
#define HW_UART3_CTRL1          (UART3_BASEESS + 0x10)
#define HW_UART3_CTRL2          (UART3_BASEESS + 0x20)
#define HW_UART3_LINECTRL       (UART3_BASEESS + 0x30)
#define HW_UART3_INTR           (UART3_BASEESS + 0x40)
#define HW_UART3_DATA           (UART3_BASEESS + 0x50)
#define HW_UART3_STAT           (UART3_BASEESS + 0x60)
#define HW_UART3_DEBUG          (UART3_BASEESS + 0x70)
#define HW_UART3_ILPR           (UART3_BASEESS + 0x80)
#define HW_UART3_RS485CTRL      (UART3_BASEESS + 0x90)
#define HW_UART3_RS485ADRMATCH  (UART3_BASEESS + 0xa0)
#define HW_UART3_RS485DLY       (UART3_BASEESS + 0xb0)
#define HW_UART3_AUTOBAUD       (UART3_BASEESS + 0xc0)
#define HW_UART3_CTRL3          (UART3_BASEESS + 0xd0)
#define HW_UART3_ISO7816        (UART3_BASEESS + 0xe0)

#define HW_UART3_ISO7816_ERR_CNT (UART3_BASEESS + 0xf0)
#define HW_UART3_ISO7816_STATUS (UART3_BASEESS + 0x100)

/////////////////////////////////////////////////////////
//UART4
#define UART4_BASEESS  0x80010000

#define HW_UART4_CTRL0          (UART4_BASEESS + 0x00)
#define HW_UART4_CTRL1          (UART4_BASEESS + 0x10)
#define HW_UART4_CTRL2          (UART4_BASEESS + 0x20)
#define HW_UART4_LINECTRL       (UART4_BASEESS + 0x30)
#define HW_UART4_INTR           (UART4_BASEESS + 0x40)
#define HW_UART4_DATA           (UART4_BASEESS + 0x50)
#define HW_UART4_STAT           (UART4_BASEESS + 0x60)
#define HW_UART4_DEBUG          (UART4_BASEESS + 0x70)
#define HW_UART4_ILPR           (UART4_BASEESS + 0x80)
#define HW_UART4_RS485CTRL      (UART4_BASEESS + 0x90)
#define HW_UART4_RS485ADRMATCH  (UART4_BASEESS + 0xa0)
#define HW_UART4_RS485DLY       (UART4_BASEESS + 0xb0)
#define HW_UART4_AUTOBAUD       (UART4_BASEESS + 0xc0)
#define HW_UART4_CTRL3          (UART4_BASEESS + 0xd0)
#define HW_UART4_ISO7816        (UART4_BASEESS + 0xe0)

#define HW_UART4_ISO7816_ERR_CNT (UART4_BASEESS + 0xf0)
#define HW_UART4_ISO7816_STATUS (UART4_BASEESS + 0x100)
/////////////////////////////////////////////////////////
//UART5
#define UART5_BASEESS  0x80014000

#define HW_UART5_CTRL0          (UART5_BASEESS + 0x00)
#define HW_UART5_CTRL1          (UART5_BASEESS + 0x10)
#define HW_UART5_CTRL2          (UART5_BASEESS + 0x20)
#define HW_UART5_LINECTRL       (UART5_BASEESS + 0x30)
#define HW_UART5_INTR           (UART5_BASEESS + 0x40)
#define HW_UART5_DATA           (UART5_BASEESS + 0x50)
#define HW_UART5_STAT           (UART5_BASEESS + 0x60)
#define HW_UART5_DEBUG          (UART5_BASEESS + 0x70)
#define HW_UART5_ILPR           (UART5_BASEESS + 0x80)
#define HW_UART5_RS485CTRL      (UART5_BASEESS + 0x90)
#define HW_UART5_RS485ADRMATCH  (UART5_BASEESS + 0xa0)
#define HW_UART5_RS485DLY       (UART5_BASEESS + 0xb0)
#define HW_UART5_AUTOBAUD       (UART5_BASEESS + 0xc0)
#define HW_UART5_CTRL3          (UART5_BASEESS + 0xd0)
#define HW_UART5_ISO7816        (UART5_BASEESS + 0xe0)

#define HW_UART5_ISO7816_ERR_CNT (UART5_BASEESS + 0xf0)
#define HW_UART5_ISO7816_STATUS (UART5_BASEESS + 0x100)
/////////////////////////////////////////////////////////
//UART6
#define UART6_BASEESS  0x80018000

#define HW_UART6_CTRL0          (UART6_BASEESS + 0x00)
#define HW_UART6_CTRL1          (UART6_BASEESS + 0x10)
#define HW_UART6_CTRL2          (UART6_BASEESS + 0x20)
#define HW_UART6_LINECTRL       (UART6_BASEESS + 0x30)
#define HW_UART6_INTR           (UART6_BASEESS + 0x40)
#define HW_UART6_DATA           (UART6_BASEESS + 0x50)
#define HW_UART6_STAT           (UART6_BASEESS + 0x60)
#define HW_UART6_DEBUG          (UART6_BASEESS + 0x70)
#define HW_UART6_ILPR           (UART6_BASEESS + 0x80)
#define HW_UART6_RS485CTRL      (UART6_BASEESS + 0x90)
#define HW_UART6_RS485ADRMATCH  (UART6_BASEESS + 0xa0)
#define HW_UART6_RS485DLY       (UART6_BASEESS + 0xb0)
#define HW_UART6_AUTOBAUD       (UART6_BASEESS + 0xc0)
#define HW_UART6_CTRL3          (UART6_BASEESS + 0xd0)
#define HW_UART6_ISO7816        (UART6_BASEESS + 0xe0)

#define HW_UART6_ISO7816_ERR_CNT (UART6_BASEESS + 0xf0)
#define HW_UART6_ISO7816_STATUS (UART6_BASEESS + 0x100)
/////////////////////////////////////////////////////////
//UART7
#define UART7_BASEESS  0x8001c000

#define HW_UART7_CTRL0          (UART7_BASEESS + 0x00)
#define HW_UART7_CTRL1          (UART7_BASEESS + 0x10)
#define HW_UART7_CTRL2          (UART7_BASEESS + 0x20)
#define HW_UART7_LINECTRL       (UART7_BASEESS + 0x30)
#define HW_UART7_INTR           (UART7_BASEESS + 0x40)
#define HW_UART7_DATA           (UART7_BASEESS + 0x50)
#define HW_UART7_STAT           (UART7_BASEESS + 0x60)
#define HW_UART7_DEBUG          (UART7_BASEESS + 0x70)
#define HW_UART7_ILPR           (UART7_BASEESS + 0x80)
#define HW_UART7_RS485CTRL      (UART7_BASEESS + 0x90)
#define HW_UART7_RS485ADRMATCH  (UART7_BASEESS + 0xa0)
#define HW_UART7_RS485DLY       (UART7_BASEESS + 0xb0)
#define HW_UART7_AUTOBAUD       (UART7_BASEESS + 0xc0)
#define HW_UART7_CTRL3          (UART7_BASEESS + 0xd0)
#define HW_UART7_ISO7816        (UART7_BASEESS + 0xe0)

#define HW_UART7_ISO7816_ERR_CNT (UART7_BASEESS + 0xf0)
#define HW_UART7_ISO7816_STATUS (UART7_BASEESS + 0x100)
/////////////////////////////////////////////////////////
//UART8
#define UART8_BASEESS  0x80020000

#define HW_UART8_CTRL0          (UART8_BASEESS + 0x00)
#define HW_UART8_CTRL1          (UART8_BASEESS + 0x10)
#define HW_UART8_CTRL2          (UART8_BASEESS + 0x20)
#define HW_UART8_LINECTRL       (UART8_BASEESS + 0x30)
#define HW_UART8_INTR           (UART8_BASEESS + 0x40)
#define HW_UART8_DATA           (UART8_BASEESS + 0x50)
#define HW_UART8_STAT           (UART8_BASEESS + 0x60)
#define HW_UART8_DEBUG          (UART8_BASEESS + 0x70)
#define HW_UART8_ILPR           (UART8_BASEESS + 0x80)
#define HW_UART8_RS485CTRL      (UART8_BASEESS + 0x90)
#define HW_UART8_RS485ADRMATCH  (UART8_BASEESS + 0xa0)
#define HW_UART8_RS485DLY       (UART8_BASEESS + 0xb0)
#define HW_UART8_AUTOBAUD       (UART8_BASEESS + 0xc0)
#define HW_UART8_CTRL3          (UART8_BASEESS + 0xd0)
#define HW_UART8_ISO7816        (UART8_BASEESS + 0xe0)

#define HW_UART8_ISO7816_ERR_CNT (UART8_BASEESS + 0xf0)
#define HW_UART8_ISO7816_STATUS (UART8_BASEESS + 0x100)
/////////////////////////////////////////////////////////
//UART9
#define UART9_BASEESS  0x80024000

#define HW_UART9_CTRL0          (UART9_BASEESS + 0x00)
#define HW_UART9_CTRL1          (UART9_BASEESS + 0x10)
#define HW_UART9_CTRL2          (UART9_BASEESS + 0x20)
#define HW_UART9_LINECTRL       (UART9_BASEESS + 0x30)
#define HW_UART9_INTR           (UART9_BASEESS + 0x40)
#define HW_UART9_DATA           (UART9_BASEESS + 0x50)
#define HW_UART9_STAT           (UART9_BASEESS + 0x60)
#define HW_UART9_DEBUG          (UART9_BASEESS + 0x70)
#define HW_UART9_ILPR           (UART9_BASEESS + 0x80)
#define HW_UART9_RS485CTRL      (UART9_BASEESS + 0x90)
#define HW_UART9_RS485ADRMATCH  (UART9_BASEESS + 0xa0)
#define HW_UART9_RS485DLY       (UART9_BASEESS + 0xb0)
#define HW_UART9_AUTOBAUD       (UART9_BASEESS + 0xc0)
#define HW_UART9_CTRL3          (UART9_BASEESS + 0xd0)
#define HW_UART9_ISO7816        (UART9_BASEESS + 0xe0)

#define HW_UART9_ISO7816_ERR_CNT (UART9_BASEESS + 0xf0)
#define HW_UART9_ISO7816_STATUS (UART9_BASEESS + 0x100)


/////////////////////////////////////////////////////////
//SYSTEM CONFIG
#define HW_PRESETCTRL0        0x80040000
#define HW_PRESETCTRL1        0x80040010
#define HW_AHBCLKCTRL0        0x80040020
#define HW_AHBCLKCTRL1        0x80040030
#define HW_SYSTCKCAL          0x80040040
#define HW_SYSPLLCTRL         0x80040100
#define HW_SYSPLLSTAT         0x80040104
#define HW_SYSRSTSTAT         0x80040110
#define HW_MAINCLKSEL         0x80040120
#define HW_MAINCLKUEN         0x80040124
#define HW_UARTCLKSEL		0x80040128
#define HW_UARTCLKUEN		0x8004012C
#define HW_I2S0CLKSEL         0x80040130
#define HW_I2S0CLKUEN         0x80040134
#define HW_I2S1CLKSEL         0x80040138
#define HW_I2S1CLKUEN         0x8004013C
#define HW_USBCLKSEL          0x80040140
#define HW_USBCLKUEN          0x80040144
#define HW_WDTCLKSEL          0x80040160
#define HW_WDTCLKUEN          0x80040164
#define HW_OUTCLKSEL          0x80040170
#define HW_OUTCLKUEN          0x80040174
#define HW_CPUCLKDIV          0x8004017C
#define HW_SYSAHBCLKDIV       0x80040180
#define HW_I2S1_MCLKDIV       0x80040188
#define HW_I2S1_SCLKDIV		  0x8004018C	
#define HW_I2S0_MCLKDIV       0x80040190
#define HW_I2S0_SCLKDIV       0x80040194
#define HW_UART0CLKDIV        0x80040198
#define HW_UART1CLKDIV        0x8004019C
#define HW_UART2CLKDIV        0x800401A0
#define HW_UART3CLKDIV        0x800401A4
#define HW_UART4CLKDIV        0x800401A8
#define HW_UART5CLKDIV        0x800401AC
#define HW_UART6CLKDIV        0x800401B0
#define HW_UART7CLKDIV        0x800401B4
#define HW_UART8CLKDIV        0x800401B8
#define HW_UART9CLKDIV        0x800401BC
#define HW_SPI0CLKDIV         0x800401C0
#define HW_SPI1CLKDIV         0x800401C4
#define HW_QUADSPI0CLKDIV     0x800401C8
#define HW_SSP0CLKDIV         0x800401D0
#define HW_NANDCLKDIV         0x800401D4
#define HW_TRACECLKDIV        0x800401E0
#define HW_CAMMCLKDIV         0x800401E8
#define HW_WDTCLKDIV          0x800401EC
#define HW_USBCLKDIV          0x800401F0
#define HW_OUTCLKDIV          0x800401F4
#define HW_ADCCLKDIV          0x80040200
#define HW_PDRUNCFG           0x80040238
#define HW_MATRIXPRI0         0x80040300
#define HW_MATRIXPRI1         0x80040304
#define HW_MATRIXPRI2         0x80040308
#define HW_MATRIXPRI3         0x8004030C
#define HW_MATRIXPRI4         0x80040310
#define HW_MATRIXPRI5         0x80040314
#define HW_MATRIXPRI6         0x80040318
#define HW_MATRIXPRI7         0x8004031C
#define HW_MATRIXPRI8         0x80040320
#define HW_MATRIXPRI9         0x80040324
#define HW_MATRIXPRI10        0x80040328
#define HW_MATRIXPRI11        0x8004032C
#define HW_MATRIXPRI12        0x80040330
#define HW_MATRIXPRI13        0x80040334
#define HW_MATRIXPRI14        0x80040338
#define HW_MATRIXPRI15        0x8004033C
#define HW_EMI_CTRL           0x8004034C
#define HW_RISC_CTRL          0x80040350
#define HW_MACPHY_SEL         0x80040360
#define HW_USB_CTRL           0x80040368
#define HW_DEVICEID           0x80040400
#define HW_PCON_ADDR          0x80040500
#define HW_DMA_CTRL			  0x80040354

/////////////////////////////////////////////////////////
//I2S0
#define I2S0_BASE_ADDRESS       0x80028000
//#define I2S0_BASE_ADDRESS       0x80070000

#define HW_I2S0_IER             I2S0_BASE_ADDRESS 
#define HW_I2S0_IRER            (I2S0_BASE_ADDRESS + 0x4)
#define HW_I2S0_ITER            (I2S0_BASE_ADDRESS + 0x8)
#define HW_I2S0_CER             (I2S0_BASE_ADDRESS + 0xc)       
#define HW_I2S0_CCR             (I2S0_BASE_ADDRESS + 0x10)
#define HW_I2S0_RXFFR           (I2S0_BASE_ADDRESS + 0x14)
#define HW_I2S0_TXFFR           (I2S0_BASE_ADDRESS + 0x18)

#define HW_I2S0_LRBR0           (I2S0_BASE_ADDRESS + 0x20)
#define HW_I2S0_LTHR0           (I2S0_BASE_ADDRESS + 0x20)
#define HW_I2S0_RRBR0           (I2S0_BASE_ADDRESS + 0x24)
#define HW_I2S0_RTHR0           (I2S0_BASE_ADDRESS + 0x24)
#define HW_I2S0_RER0            (I2S0_BASE_ADDRESS + 0x28)
#define HW_I2S0_TER0            (I2S0_BASE_ADDRESS + 0x2c)
#define HW_I2S0_RCR0            (I2S0_BASE_ADDRESS + 0x30)
#define HW_I2S0_TCR0            (I2S0_BASE_ADDRESS + 0x34)
#define HW_I2S0_ISR0            (I2S0_BASE_ADDRESS + 0x38)
#define HW_I2S0_IMR0            (I2S0_BASE_ADDRESS + 0x3c)
#define HW_I2S0_ROR0            (I2S0_BASE_ADDRESS + 0x40)
#define HW_I2S0_TOR0            (I2S0_BASE_ADDRESS + 0x44)
#define HW_I2S0_RFCR0           (I2S0_BASE_ADDRESS + 0x48)
#define HW_I2S0_TFCR0           (I2S0_BASE_ADDRESS + 0x4c)
#define HW_I2S0_RFF0            (I2S0_BASE_ADDRESS + 0x50)
#define HW_I2S0_TFF0            (I2S0_BASE_ADDRESS + 0x54)

#define HW_I2S0_LRBR1           (I2S0_BASE_ADDRESS + 0x60)
#define HW_I2S0_LTHR1           (I2S0_BASE_ADDRESS + 0x60)
#define HW_I2S0_RRBR1           (I2S0_BASE_ADDRESS + 0x64)
#define HW_I2S0_RTHR1           (I2S0_BASE_ADDRESS + 0x64)
#define HW_I2S0_RER1            (I2S0_BASE_ADDRESS + 0x68)
#define HW_I2S0_TER1            (I2S0_BASE_ADDRESS + 0x6c)
#define HW_I2S0_RCR1            (I2S0_BASE_ADDRESS + 0x70)
#define HW_I2S0_TCR1            (I2S0_BASE_ADDRESS + 0x74)
#define HW_I2S0_ISR1            (I2S0_BASE_ADDRESS + 0x78)
#define HW_I2S0_IMR1            (I2S0_BASE_ADDRESS + 0x7c)
#define HW_I2S0_ROR1            (I2S0_BASE_ADDRESS + 0x80)
#define HW_I2S0_TOR1            (I2S0_BASE_ADDRESS + 0x84)
#define HW_I2S0_RFCR1           (I2S0_BASE_ADDRESS + 0x88)
#define HW_I2S0_TFCR1           (I2S0_BASE_ADDRESS + 0x8c)
#define HW_I2S0_RFF1            (I2S0_BASE_ADDRESS + 0x90)
#define HW_I2S0_TFF1            (I2S0_BASE_ADDRESS + 0x94)

#define HW_I2S0_LRBR2           (I2S0_BASE_ADDRESS + 0xa0)
#define HW_I2S0_LTHR2           (I2S0_BASE_ADDRESS + 0xa0)
#define HW_I2S0_RRBR2           (I2S0_BASE_ADDRESS + 0xa4)
#define HW_I2S0_RTHR2           (I2S0_BASE_ADDRESS + 0xa4)
#define HW_I2S0_RER2            (I2S0_BASE_ADDRESS + 0xa8)
#define HW_I2S0_TER2            (I2S0_BASE_ADDRESS + 0xac)
#define HW_I2S0_RCR2            (I2S0_BASE_ADDRESS + 0xb0)
#define HW_I2S0_TCR2            (I2S0_BASE_ADDRESS + 0xb4)
#define HW_I2S0_ISR2            (I2S0_BASE_ADDRESS + 0xb8)
#define HW_I2S0_IMR2            (I2S0_BASE_ADDRESS + 0xbc)
#define HW_I2S0_ROR2            (I2S0_BASE_ADDRESS + 0xc0)
#define HW_I2S0_TOR2            (I2S0_BASE_ADDRESS + 0xc4)
#define HW_I2S0_RFCR2           (I2S0_BASE_ADDRESS + 0xc8)
#define HW_I2S0_TFCR2           (I2S0_BASE_ADDRESS + 0xcc)
#define HW_I2S0_RFF2            (I2S0_BASE_ADDRESS + 0xd0)
#define HW_I2S0_TFF2            (I2S0_BASE_ADDRESS + 0xd4)

#define HW_I2S0_LRBR3           (I2S0_BASE_ADDRESS + 0xe0)
#define HW_I2S0_LTHR3           (I2S0_BASE_ADDRESS + 0xe0)
#define HW_I2S0_RRBR3           (I2S0_BASE_ADDRESS + 0xe4)
#define HW_I2S0_RTHR3           (I2S0_BASE_ADDRESS + 0xe4)
#define HW_I2S0_RER3            (I2S0_BASE_ADDRESS + 0xe8)
#define HW_I2S0_TER3            (I2S0_BASE_ADDRESS + 0xec)
#define HW_I2S0_RCR3            (I2S0_BASE_ADDRESS + 0xf0)
#define HW_I2S0_TCR3            (I2S0_BASE_ADDRESS + 0xf4)
#define HW_I2S0_ISR3            (I2S0_BASE_ADDRESS + 0xf8)
#define HW_I2S0_IMR3            (I2S0_BASE_ADDRESS + 0xfc)
#define HW_I2S0_ROR3            (I2S0_BASE_ADDRESS + 0x100)
#define HW_I2S0_TOR3            (I2S0_BASE_ADDRESS + 0x104)
#define HW_I2S0_RFCR3           (I2S0_BASE_ADDRESS + 0x108)
#define HW_I2S0_TFCR3           (I2S0_BASE_ADDRESS + 0x10c)
#define HW_I2S0_RFF3            (I2S0_BASE_ADDRESS + 0x110)
#define HW_I2S0_TFF3            (I2S0_BASE_ADDRESS + 0x114)

#define HW_I2S0_RXDMA           (I2S0_BASE_ADDRESS + 0x1c0)
#define HW_I2S0_RRXDMA          (I2S0_BASE_ADDRESS + 0x1c4)
#define HW_I2S0_TXDMA           (I2S0_BASE_ADDRESS + 0x1c8)
#define HW_I2S0_RTXDMA          (I2S0_BASE_ADDRESS + 0x1cc)

#define HW_I2S0_COMP_PARAM_2    (I2S0_BASE_ADDRESS + 0x1f0)
#define HW_I2S0_COMP_PARAM_1    (I2S0_BASE_ADDRESS + 0x1f4)
#define HW_I2S0_COMP_VERSION    (I2S0_BASE_ADDRESS + 0x1f8)
#define HW_I2S0_COMP_TYPE       (I2S0_BASE_ADDRESS + 0x1fc)



/////////////////////////////////////////////////////////
//I2S1
#define I2S1_BASE_ADDRESS       0x80070000

#define HW_I2S1_IER             I2S1_BASE_ADDRESS 
#define HW_I2S1_IRER            (I2S1_BASE_ADDRESS + 0x4)
#define HW_I2S1_ITER            (I2S1_BASE_ADDRESS + 0x8)
#define HW_I2S1_CER             (I2S1_BASE_ADDRESS + 0xc)       
#define HW_I2S1_CCR             (I2S1_BASE_ADDRESS + 0x10)
#define HW_I2S1_RXFFR           (I2S1_BASE_ADDRESS + 0x14)
#define HW_I2S1_TXFFR           (I2S1_BASE_ADDRESS + 0x18)

#define HW_I2S1_LRBR0           (I2S1_BASE_ADDRESS + 0x20)
#define HW_I2S1_LTHR0           (I2S1_BASE_ADDRESS + 0x20)
#define HW_I2S1_RRBR0           (I2S1_BASE_ADDRESS + 0x24)
#define HW_I2S1_RTHR0           (I2S1_BASE_ADDRESS + 0x24)
#define HW_I2S1_RER0            (I2S1_BASE_ADDRESS + 0x28)
#define HW_I2S1_TER0            (I2S1_BASE_ADDRESS + 0x2c)
#define HW_I2S1_RCR0            (I2S1_BASE_ADDRESS + 0x30)
#define HW_I2S1_TCR0            (I2S1_BASE_ADDRESS + 0x34)
#define HW_I2S1_ISR0            (I2S1_BASE_ADDRESS + 0x38)
#define HW_I2S1_IMR0            (I2S1_BASE_ADDRESS + 0x3c)
#define HW_I2S1_ROR0            (I2S1_BASE_ADDRESS + 0x40)
#define HW_I2S1_TOR0            (I2S1_BASE_ADDRESS + 0x44)
#define HW_I2S1_RFCR0           (I2S1_BASE_ADDRESS + 0x48)
#define HW_I2S1_TFCR0           (I2S1_BASE_ADDRESS + 0x4c)
#define HW_I2S1_RFF0            (I2S1_BASE_ADDRESS + 0x50)
#define HW_I2S1_TFF0            (I2S1_BASE_ADDRESS + 0x54)

#define HW_I2S1_LRBR1           (I2S1_BASE_ADDRESS + 0x60)
#define HW_I2S1_LTHR1           (I2S1_BASE_ADDRESS + 0x60)
#define HW_I2S1_RRBR1           (I2S1_BASE_ADDRESS + 0x64)
#define HW_I2S1_RTHR1           (I2S1_BASE_ADDRESS + 0x64)
#define HW_I2S1_RER1            (I2S1_BASE_ADDRESS + 0x68)
#define HW_I2S1_TER1            (I2S1_BASE_ADDRESS + 0x6c)
#define HW_I2S1_RCR1            (I2S1_BASE_ADDRESS + 0x70)
#define HW_I2S1_TCR1            (I2S1_BASE_ADDRESS + 0x74)
#define HW_I2S1_ISR1            (I2S1_BASE_ADDRESS + 0x78)
#define HW_I2S1_IMR1            (I2S1_BASE_ADDRESS + 0x7c)
#define HW_I2S1_ROR1            (I2S1_BASE_ADDRESS + 0x80)
#define HW_I2S1_TOR1            (I2S1_BASE_ADDRESS + 0x84)
#define HW_I2S1_RFCR1           (I2S1_BASE_ADDRESS + 0x88)
#define HW_I2S1_TFCR1           (I2S1_BASE_ADDRESS + 0x8c)
#define HW_I2S1_RFF1            (I2S1_BASE_ADDRESS + 0x90)
#define HW_I2S1_TFF1            (I2S1_BASE_ADDRESS + 0x94)

#define HW_I2S1_LRBR2           (I2S1_BASE_ADDRESS + 0xa0)
#define HW_I2S1_LTHR2           (I2S1_BASE_ADDRESS + 0xa0)
#define HW_I2S1_RRBR2           (I2S1_BASE_ADDRESS + 0xa4)
#define HW_I2S1_RTHR2           (I2S1_BASE_ADDRESS + 0xa4)
#define HW_I2S1_RER2            (I2S1_BASE_ADDRESS + 0xa8)
#define HW_I2S1_TER2            (I2S1_BASE_ADDRESS + 0xac)
#define HW_I2S1_RCR2            (I2S1_BASE_ADDRESS + 0xb0)
#define HW_I2S1_TCR2            (I2S1_BASE_ADDRESS + 0xb4)
#define HW_I2S1_ISR2            (I2S1_BASE_ADDRESS + 0xb8)
#define HW_I2S1_IMR2            (I2S1_BASE_ADDRESS + 0xbc)
#define HW_I2S1_ROR2            (I2S1_BASE_ADDRESS + 0xc0)
#define HW_I2S1_TOR2            (I2S1_BASE_ADDRESS + 0xc4)
#define HW_I2S1_RFCR2           (I2S1_BASE_ADDRESS + 0xc8)
#define HW_I2S1_TFCR2           (I2S1_BASE_ADDRESS + 0xcc)
#define HW_I2S1_RFF2            (I2S1_BASE_ADDRESS + 0xd0)
#define HW_I2S1_TFF2            (I2S1_BASE_ADDRESS + 0xd4)

#define HW_I2S1_LRBR3           (I2S1_BASE_ADDRESS + 0xe0)
#define HW_I2S1_LTHR3           (I2S1_BASE_ADDRESS + 0xe0)
#define HW_I2S1_RRBR3           (I2S1_BASE_ADDRESS + 0xe4)
#define HW_I2S1_RTHR3           (I2S1_BASE_ADDRESS + 0xe4)
#define HW_I2S1_RER3            (I2S1_BASE_ADDRESS + 0xe8)
#define HW_I2S1_TER3            (I2S1_BASE_ADDRESS + 0xec)
#define HW_I2S1_RCR3            (I2S1_BASE_ADDRESS + 0xf0)
#define HW_I2S1_TCR3            (I2S1_BASE_ADDRESS + 0xf4)
#define HW_I2S1_ISR3            (I2S1_BASE_ADDRESS + 0xf8)
#define HW_I2S1_IMR3            (I2S1_BASE_ADDRESS + 0xfc)
#define HW_I2S1_ROR3            (I2S1_BASE_ADDRESS + 0x100)
#define HW_I2S1_TOR3            (I2S1_BASE_ADDRESS + 0x104)
#define HW_I2S1_RFCR3           (I2S1_BASE_ADDRESS + 0x108)
#define HW_I2S1_TFCR3           (I2S1_BASE_ADDRESS + 0x10c)
#define HW_I2S1_RFF3            (I2S1_BASE_ADDRESS + 0x110)
#define HW_I2S1_TFF3            (I2S1_BASE_ADDRESS + 0x114)

#define HW_I2S1_RXDMA           (I2S1_BASE_ADDRESS + 0x1c0)
#define HW_I2S1_RRXDMA          (I2S1_BASE_ADDRESS + 0x1c4)
#define HW_I2S1_TXDMA           (I2S1_BASE_ADDRESS + 0x1c8)
#define HW_I2S1_RTXDMA          (I2S1_BASE_ADDRESS + 0x1cc)

#define HW_I2S1_COMP_PARAM_2    (I2S1_BASE_ADDRESS + 0x1f0)
#define HW_I2S1_COMP_PARAM_1    (I2S1_BASE_ADDRESS + 0x1f4)
#define HW_I2S1_COMP_VERSION    (I2S1_BASE_ADDRESS + 0x1f8)
#define HW_I2S1_COMP_TYPE       (I2S1_BASE_ADDRESS + 0x1fc)

/*
 * ----------------------------------------------------------------------------
 * memcpy
 * ----------------------------------------------------------------------------
 */
#define HW_MEMCPY_CTRL              (0X80014000) 
#define HW_MEMCPY_DATA              (0X80014010) 
#define HW_MEMCPY_DEBUG             (0x80014020)

/*
 * ----------------------------------------------------------------------------
 * DRI
 * ----------------------------------------------------------------------------
 */

#define HW_DRI_CTRL                 (0x80074000)
#define HW_DRI_TIMING               (0x80074010)
#define HW_DRI_STAT                 (0x80074020)
#define HW_DRI_DATA                 (0x80074030)
#define HW_DRI_DEBUG0               (0x80074040)
#define HW_DRI_DEBUG1               (0x80074050)


/*
 * ----------------------------------------------------------------------------
 * audio
 * ----------------------------------------------------------------------------
 */
//audioin adc register
#define HW_AUDIOIN_CTRL                (0x8004c000)
#define HW_AUDIOIN_STAT                (0x8004c010)
#define HW_AUDIOIN_ADCSRR              (0x8004c020)
#define HW_AUDIOIN_ADCVOLUME           (0x8004c030)
#define HW_AUDIOIN_ADCDEBUG            (0x8004c040)
#define HW_AUDIOIN_ADCVOL              (0x8004c050)
#define HW_AUDIOIN_MICLINE             (0x8004c060)
#define HW_AUDIOIN_ANACLKCTRL          (0x8004c070)

/*
 * ----------------------------------------------------------------------------
 * UART2
 * ----------------------------------------------------------------------------
 */

#define ALPAS3310_UARTDBG_BASE 0x80070000

#define HW_UARTDBG_DATA         ALPAS3310_UARTDBG_BASE
#define HW_UARTDBG_RSR_ECR      (ALPAS3310_UARTDBG_BASE + 0x4)
#define HW_UARTDBG_FR           (ALPAS3310_UARTDBG_BASE + 0x18)
#define HW_UARTDBG_IrDA_ILPR    (ALPAS3310_UARTDBG_BASE + 0x20)
#define HW_UARTDBG_IBRD         (ALPAS3310_UARTDBG_BASE + 0x24)
#define HW_UARTDBG_FBRD         (ALPAS3310_UARTDBG_BASE + 0x28)
#define HW_UARTDBG_LCR_H        (ALPAS3310_UARTDBG_BASE + 0x2C)
#define HW_UARTDBG_CTRL         (ALPAS3310_UARTDBG_BASE + 0x30)
#define HW_UARTDBG_IFLS         (ALPAS3310_UARTDBG_BASE + 0x34)
#define HW_UARTDBG_IMSC         (ALPAS3310_UARTDBG_BASE + 0x38)
#define HW_UARTDBG_RIS          (ALPAS3310_UARTDBG_BASE + 0x3C)
#define HW_UARTDBG_MIS          (ALPAS3310_UARTDBG_BASE + 0x40)
#define HW_UARTDBG_ICR          (ALPAS3310_UARTDBG_BASE + 0x44)

/*
 * ----------------------------------------------------------------------------
 * SPDIF
 * ----------------------------------------------------------------------------
 */

#define HW_SPDIF_CTRL                   (0x80054000)
#define HW_SPDIF_STAT                   (0x80054010)
#define HW_SPDIF_FRAMECTRL              (0x80054020)
#define HW_SPDIF_SRR                    (0x80054030)
#define HW_SPDIF_DEBUG                  (0x80054040)
#define HW_SPDIF_DATA                   (0x80054050)
/*
 * ----------------------------------------------------------------------------
 * HWECC
 * ----------------------------------------------------------------------------
 */

#define HW_HWECC_CTRL                   (0x80008000)
#define HW_HWECC_STAT                   (0x80008010)
#define HW_HWECC_DEBUG0                 (0x80008020)
#define HW_HWECC_DEBUG1                 (0x80008030)
#define HW_HWECC_DEBUG2                 (0x80008040)
#define HW_HWECC_DEBUG3                 (0x80008050)
#define HW_HWECC_DEBUG4                 (0x80008060)
#define HW_HWECC_DEBUG5                 (0x80008070)
#define HW_HWECC_DEBUG6                 (0x80008080)
#define HW_HWECC_DATA                   (0x80008090)


/*
 * ----------------------------------------------------------------------------
 * lradc
 * ----------------------------------------------------------------------------
 */

#define HW_LRADC_BASE       0x800A4000
                                              
#define HW_LRADC_CTRL0   HW_LRADC_BASE//0x800A4000
#define HW_LRADC_CTRL0_SET (HW_LRADC_CTRL0 + 0x4)
#define HW_LRADC_CTRL0_CLR (HW_LRADC_CTRL0 + 0x8)
#define HW_LRADC_CTRL0_TOG (HW_LRADC_CTRL0 + 0xc)

#define HW_LRADC_CTRL1   (HW_LRADC_BASE+0x10)//0x800A4010
#define HW_LRADC_CTRL1_SET (HW_LRADC_CTRL1 + 0x4)                                           // 
#define HW_LRADC_CTRL1_CLR (HW_LRADC_CTRL1 + 0x8)                                           // 
#define HW_LRADC_CTRL1_TOG (HW_LRADC_CTRL1 + 0xc)   
                                        // 
#define HW_LRADC_CTRL2   (HW_LRADC_BASE+0x20)//0x800A4020
#define HW_LRADC_CTRL2_SET (HW_LRADC_CTRL2 + 0x4)                                              
#define HW_LRADC_CTRL2_CLR (HW_LRADC_CTRL2 + 0x8) 
#define HW_LRADC_CTRL2_TOG (HW_LRADC_CTRL2 + 0xc) 
                                             
#define HW_LRADC_CTRL3   (HW_LRADC_BASE+0x30)//0x800A4030
#define HW_LRADC_CTRL3_SET (HW_LRADC_CTRL3 + 0x4)                                              
#define HW_LRADC_CTRL3_CLR (HW_LRADC_CTRL3 + 0x8) 
#define HW_LRADC_CTRL3_TOG (HW_LRADC_CTRL3 + 0xc)    
                      
#define HW_LRADC_CTRL4   (HW_LRADC_BASE+0x150)//0x800A4150
#define HW_LRADC_CTRL4_SET (HW_LRADC_CTRL4 + 0x4)                                              
#define HW_LRADC_CTRL4_CLR (HW_LRADC_CTRL4 + 0x8) 
#define HW_LRADC_CTRL4_TOG (HW_LRADC_CTRL4 + 0xc)           
       
                      
/*                        
 * Status register bits.  
 */                       
#define HW_LRADC_STATUS  (HW_LRADC_BASE+0x40)//0x800A4040                        
/*                       
 * Result register bits  
 */                      
#define HW_LRADC_CH0     (HW_LRADC_BASE+0x50)//0x800A4050
#define HW_LRADC_CH1     (HW_LRADC_BASE+0x60)//0x800A4060
#define HW_LRADC_CH2     (HW_LRADC_BASE+0x70)//0x800A4070
#define HW_LRADC_CH3     (HW_LRADC_BASE+0x80)//0x800A4080
#define HW_LRADC_CH4     (HW_LRADC_BASE+0x90)//0x800A4090
#define HW_LRADC_CH5     (HW_LRADC_BASE+0xA0)//0x800A40A0
#define HW_LRADC_CH6     (HW_LRADC_BASE+0xB0)//0x800A40B0
#define HW_LRADC_CH7     (HW_LRADC_BASE+0xC0)//0x800A40C0                        


/*                          
 * Debug register bits      
 */                         
#define HW_LRADC_DEBUG0     (HW_LRADC_BASE+0x110)//0x800A4110
#define HW_LRADC_DEBUG1     (HW_LRADC_BASE+0x120)//0x800A4120
#define HW_LRADC_CONVERSION (HW_LRADC_BASE+0x130)//0x800A4130
                                               
/*
 * ----------------------------------------------------------------------------
 * I2S
 * ----------------------------------------------------------------------------
 */

#define HW_I2SCTRL_CTRL0        (0x80056000)    
#define HW_I2SCTRL_SPCR         (0x80056010)        
#define HW_I2SCTRL_CFG          (0x80056020)        
#define HW_I2SCTRL_DATA         (0x80056030)
#define HW_I2SCTRL_STATUS       (0x80056040)
#define HW_I2SCTRL_EVENT        (0x80056050)

/*
 * ----------------------------------------------------------------------------
 * NOR Flash
 * ----------------------------------------------------------------------------
 */
#define ALPAS3310_NOR_BASE	 		0x00000000
#define ALPAS3310_NOR_SIZE	 		0x00200000

/*
 * ----------------------------------------------------------------------------
 * Camera
 * ----------------------------------------------------------------------------
 */

#define HW_DCMI_CR_ADDR          0x80080000
#define HW_DCMI_SR_ADDR          0x80080010
#define HW_DCMI_RIS_ADDR         0x80080020
#define HW_DCMI_IER_ADDR         0x80080030
#define HW_DCMI_MIS_ADDR         0x80080040
#define HW_DCMI_ICR_ADDR         0x80080050
#define HW_DCMI_ESCR_ADDR        0x80080060
#define HW_DCMI_ESUR_ADDR        0x80080070
#define HW_DCMI_CWSTRT_ADDR    0x80080080
#define HW_DCMI_CWSIZE_ADDR    0x80080090
#define HW_DCMI_DR_ADDR           0x800800A0


/* ----------------------------------------------------------------------------*/                                 
#ifndef __ASSEMBLY__                      


/*********************** Register in structure **************************/
                                       
typedef volatile u8		AS3310_REG8;      
typedef volatile u16	AS3310_REG16;     
typedef volatile u32	AS3310_REG32;

#define R_VAL 0
#define R_SET 1
#define R_CLR 2
#define R_TOG 3


/*  ==================  INT Reg defines  =============================*/

typedef struct {
		AS3310_REG32		VECTOR[4];	                //	0x00	R/W		
		AS3310_REG32		LEVELACK[4];                //	0x10	R/W 
		AS3310_REG32		CTRL[4];                    //	0x20	R/W 
		AS3310_REG32		STAT[4];                    //	0x30	R/W 
		AS3310_REG32		RAW0[4];                    //	0x40	R/W 
		AS3310_REG32		RAW1[4];                    //	0x50	R/W 
		AS3310_REG32		PRIORITY0[4];               //	0x60	R/W 
		AS3310_REG32		PRIORITY1[4];               //	0x70	R/W	
		AS3310_REG32		PRIORITY2[4];               //	0x80	R/W
		AS3310_REG32		PRIORITY3[4];               //	0x90	R/W 
		AS3310_REG32		PRIORITY4[4];               //	0xA0	R/W 
		AS3310_REG32		PRIORITY5[4];               //	0xB0	R/W	
		AS3310_REG32		PRIORITY6[4];               //	0xC0	R/W
		AS3310_REG32		PRIORITY7[4];               //	0xD0	R/W   
		AS3310_REG32		PRIORITY8[4];               //	0xE0	R/W
        AS3310_REG32		PRIORITY9[4];               //  0xF0
        AS3310_REG32		PRIORITY10[4];              //  0x100
        AS3310_REG32		PRIORITY11[4];              //  0x110
        AS3310_REG32		PRIORITY12[4];              //  0x120
        AS3310_REG32		PRIORITY13[4];              //  0x130
        AS3310_REG32		PRIORITY14[4];              //  0x140
        AS3310_REG32		PRIORITY15[4];              //  0x150
        AS3310_REG32		VBASE[4];                   //  0x160
        AS3310_REG32		RDEBUG[4];                   //  0x170
        AS3310_REG32		DBGREAD0[4];                //  0x180
        AS3310_REG32		DBGREAD1[4];                //  0x190
        AS3310_REG32		DBGFLAG[4];                 //  0x1A0
        AS3310_REG32		DBGREQUEST0[4];             //  0x1B0
        AS3310_REG32		DBGREQUEST1[4];             //  0x1C0
        AS3310_REG32		CLEAR0[4];                  //  0x1D0
        AS3310_REG32		CLEAR1[4];                  //  0x1E0
        AS3310_REG32		UNDEF_VECTOR[4];            //  0x1F0
} /*__attribute__((__packed__))*/ AS3310_INT_CTRL;

#define AS3310_INT_CTRL_BASE   0x80000000  /* for AS3310D */

static inline AS3310_INT_CTRL * AS3310_GetBase_INT_CTRL(void)
{
	return (AS3310_INT_CTRL *)(IO_ADDRESS(AS3310_INT_CTRL_BASE));
}

/*  ==================  PINCTRL Reg defines  =============================*/

typedef struct _asap18xx_pincontrol_bank{
		AS3310_REG32		CTRL[4]	        ;	//	0x00	R/W		
		AS3310_REG32		MUXSEL0[4]      ;  //	0x10	R/W 
		AS3310_REG32		MUXSEL1[4]      ;  //	0x20	R/W 
		AS3310_REG32		DRIVE[4]	    ;  //	0x30	R/W 
		AS3310_REG32		NOT_USED0[4]    ;  //	0x40	R/W 
		AS3310_REG32		DOUT[4]         ;  //	0x50	R/W 
		AS3310_REG32		DIN[4]          ;  //	0x60	R/W 
		AS3310_REG32		DOE[4]          ;  //	0x70	R/W	
		AS3310_REG32		PIN2IRQ[4]      ;  //	0x80	R/W
		AS3310_REG32		IRQEN[4]        ;  //	0x90	R/W 
		AS3310_REG32		IRQLEVEL[4]     ;  //	0xA0	R/W 
		AS3310_REG32		IRQPOL[4]       ;  //	0xB0	R/W	
		AS3310_REG32		IRQSTAT[4]      ;  //	0xC0	R/W
		AS3310_REG32		PU[4]           ;  //	0xD0	R/W   
		AS3310_REG32		PD[4]           ;  //	0xE0	R/W   
        AS3310_REG32		NOT_USED1[4]    ;  //0xF0
} /*__attribute__((__packed__))*/ ASAP18xx_PIN_CTRL_BANK;

typedef struct _asap18xx_pincontrol{
		ASAP18xx_PIN_CTRL_BANK		BANK[4]	 ;	//	0x100*4	R/W		
} /*__attribute__((__packed__))*/ ASAP18xx_PIN_CTRL;

#define PIN_CTRL_BASE       (0x80018000)

static inline ASAP18xx_PIN_CTRL *  ASAP18xx_GetBase_PinControl(void)
{
	return (ASAP18xx_PIN_CTRL *)(IO_ADDRESS(PIN_CTRL_BASE));
}

/*  ==================  LRADC Reg defines  =============================*/

typedef struct {
		AS3310_REG32		CTRL0[4];	        //	0x00	R/W		
		AS3310_REG32		CTRL1[4];           //	0x10	R/W 
		AS3310_REG32		CTRL2[4];           //	0x20	R/W 
		AS3310_REG32		CTRL3[4];           //	0x30	R/W 
		AS3310_REG32		STATUS[4];          //	0x40	R/W 
		AS3310_REG32		CH0[4];             //	0x50	R/W 
		AS3310_REG32		CH1[4];             //	0x60	R/W 
		AS3310_REG32		CH2[4];             //	0x70	R/W	
		AS3310_REG32		CH3[4];             //	0x80	R/W
		AS3310_REG32		CH4[4];             //	0x90	R/W 
		AS3310_REG32		CH5[4];             //	0xA0	R/W 
		AS3310_REG32		CH6[4];             //	0xB0	R/W	
		AS3310_REG32		CH7[4];             //	0xC0	R/W
		AS3310_REG32		DELAY0[4];          //	0xD0	R/W   
		AS3310_REG32		DELAY1[4];          //	0xE0	R/W
        AS3310_REG32		DELAY2[4];          //  0xF0
        AS3310_REG32		DELAY3[4];          //  0x100
        AS3310_REG32		DEBUG0[4];          //  0x110
        AS3310_REG32		DEBUG1[4];          //  0x120
        AS3310_REG32		CONVERSION[4];      //  0x130
} /*__attribute__((__packed__))*/ AS3310_LRADC_CTRL;

#define AS3310_LRADC_CTRL_BASE   0x80050000  /* for AS3310D */

static inline AS3310_LRADC_CTRL * AS3310_GetBase_LRADC_CTRL(void)
{
	return (AS3310_LRADC_CTRL *)(IO_ADDRESS(AS3310_LRADC_CTRL_BASE));
}

/*  ==================  UARTAPP Reg defines  =============================*/

typedef struct {
		AS3310_REG32		CTRL0[4];	                     //	0x00	R/W		
		AS3310_REG32		CTRL1[4];                        //	0x10	R/W 
		AS3310_REG32		CTRL2[4];                        //	0x20	R/W 
		AS3310_REG32		LINECTRL[4];                     //	0x30	R/W 
		AS3310_REG32		INTR[4];                         //	0x40	R/W 
		AS3310_REG32		DATA[4];  //NO SET CLR TOG       //	0x50	R/W 
		AS3310_REG32		STAT[4];  //NO SET CLR TOG       //	0x60	R/W 
		AS3310_REG32		RDEBUG[4]; //NO SET CLR TOG       //	0x70	R/W	
} /*__attribute__((__packed__))*/ AS3310_UARTAPP_CTRL;

#define AS3310_UARTAPP_CTRL_BASE               0x8006C000 

static inline  AS3310_UARTAPP_CTRL* AS3310_GetBase_UARTAPP_CTRL(void)
{
	return ( AS3310_UARTAPP_CTRL*)(IO_ADDRESS(AS3310_UARTAPP_CTRL_BASE));
}

/*  ==================  LCDIF Reg defines  =============================*/

typedef struct {
		AS3310_REG32		CTRL[4];	                    //	0x00	R/W		
		AS3310_REG32		TIMING[4]; //NO SET CLR TOG     //	0x10	R/W 
		AS3310_REG32		DATA[4];   //NO SET CLR TOG     //	0x20	R/W 
		AS3310_REG32		RDEBUG[4];  //NO SET CLR TOG     //	0x30	R/W 
} AS3310_LCDIF; /*__attribute__((__packed__))*/

#define AS3310_LCDIF_BASE	0x80060000

static inline AS3310_LCDIF * AS3310_GetBase_LCDIF(void)
{
	return (AS3310_LCDIF * ) IO_ADDRESS(AS3310_LCDIF_BASE);
}

/*  ==================  TIMER Reg defines  =============================*/

typedef struct {
		AS3310_REG32		ROTCTRL[4];	                        //	0x00	R/W		
		AS3310_REG32		ROTCOUNT[4];   //NO SET CLR TOG     //	0x10	R/W 
		AS3310_REG32		TIMCTRL0[4];                        //	0x20	R/W 
		AS3310_REG32		TIMCOUNT0[4];  //NO SET CLR TOG     //	0x30	R/W 
		AS3310_REG32		TIMCTRL1[4];                        //	0x40	R/W 
		AS3310_REG32		TIMCOUNT1[4];  //NO SET CLR TOG     //	0x50	R/W 
		AS3310_REG32		TIMCTRL2[4];                        //	0x60	R/W 
		AS3310_REG32		TIMCOUNT2[4];  //NO SET CLR TOG     //	0x70	R/W	
		AS3310_REG32		TIMCTRL3[4];                        //	0x80	R/W
        AS3310_REG32		TIMCOUNT3[4];  //NO SET CLR TOG     //	0x90	R/W 
} /*__attribute__((__packed__))*/ AS3310_TIMER_CTRL;

#define AS3310_TIMER_CTRL_BASE               0x80068000 

static inline  AS3310_TIMER_CTRL* AS3310_GetBase_TIMER_CTRL(void)
{
	return ( AS3310_TIMER_CTRL*)(IO_ADDRESS(AS3310_TIMER_CTRL_BASE));
}
/*  ==================  SSP Reg defines  =============================*/

typedef struct {
		AS3310_REG32		CTRL0[4];	                    //	0x00	R/W		
		AS3310_REG32		CMD0[4];                        //	0x10	R/W 
		AS3310_REG32		CMD1[4];                        //	0x20	R/W 
		AS3310_REG32		COMPREF[4];                     //	0x30	R/W 
		AS3310_REG32		COMPMASK[4];                    //	0x40	R/W 
		AS3310_REG32		TIMING[4];                      //	0x50	R/W 
		AS3310_REG32		CTRL1[4];                       //	0x60	R/W 
		AS3310_REG32		DATA[4];    //NO SET CLR TOG    //	0x70	R/W	
		AS3310_REG32		SDRESP0[4]; //NO SET CLR TOG    //	0x80	R/W
        AS3310_REG32		SDRESP1[4]; //NO SET CLR TOG    //	0x90	R/W 
        AS3310_REG32		SDRESP2[4]; //NO SET CLR TOG    //	0xA0	R/W 
        AS3310_REG32		SDRESP3[4]; //NO SET CLR TOG    //	0xB0	R/W	
        AS3310_REG32		STATUS[4];  //NO SET CLR TOG    //	0xC0	R/W
        AS3310_REG32		RDEBUG[4];   //NO SET CLR TOG    //	0xD0	R/W   
} /*__attribute__((__packed__))*/ AS3310_SSP_CTRL;

#define AS3310_SSP_CTRL_BASE               0x80010000 

static inline  AS3310_SSP_CTRL* AS3310_GetBase_SSP_CTRL(void)
{
	return ( AS3310_SSP_CTRL*)(IO_ADDRESS(AS3310_SSP_CTRL_BASE));
}


/*  ==================  LCD Reg defines  =============================*/

typedef struct {
		AS3310_REG32		CTRL0[4]	 ;	    //	0x00	R/W		
		AS3310_REG32		CTRL1[4]   ;        //	0x10	R/W 
		AS3310_REG32		DATA[4]     ;       //	0x20	R/W 
		AS3310_REG32		TIMING0[4]	 ;      //	0x30	R/W 
		AS3310_REG32		TIMING1[4]   ;      //	0x40	R/W 
		AS3310_REG32		TIMING2[4]   ;      //	0x50	R/W 
		AS3310_REG32		FIFO_STAT[4] ;      //	0x60	R/W 
		AS3310_REG32		STAT[4]      ;      //	0x70	R/W	
		AS3310_REG32		SUBPANEL[4]  ;      //	0x80	R/W
		AS3310_REG32		LINEINT[4]   ;      //	0x90	R/W 
		AS3310_REG32		DISPLAYSTATUS[4];   //	0xA0	R/W 
		AS3310_REG32		MONITOR[4]      ;   //	0xB0	R/W	
		AS3310_REG32		SWITCH[4]     ;     //	0xC0	R/W
		AS3310_REG32		CHECK[4]     ;      //	0xD0	R/W   
		AS3310_REG32		SECURE[4]     ;     //	0xE0	R/W
        AS3310_REG32		U_DATA[4]     ;     //  0xF0
        AS3310_REG32		V_DATA[4]     ;     //  0x100
        AS3310_REG32		UV_XFER[4]     ;    //  0x110
        AS3310_REG32		TIMING3[4]     ;    //  0x120
        
} /*__attribute__((__packed__))*/ AS3310_LCD_CTRL;

#define AS3310_LCD_CTRL_BASE   0x80084000  /* for AS3310D */

static inline AS3310_LCD_CTRL * AS3310_GetBase_LCD_CTRL(void)
{
	return (AS3310_LCD_CTRL *)(IO_ADDRESS(AS3310_LCD_CTRL_BASE));
}

/*  ==================  CLK Reg defines  =============================*/

typedef struct {
		AS3310_REG32		PLLCTRL0[4]	 ;	            //	0x00	R/W		
		AS3310_REG32		PLLCTRL1[4]   ;             //	0x10	R/W 
		AS3310_REG32		CPUCLKCTRL[4]     ;         //	0x20	R/W 
		AS3310_REG32		HBUSCLKCTRL[4]	 ;          //	0x30	R/W 
		AS3310_REG32		XBUSCLKCTRL[4]   ;          //	0x40	R/W 
		AS3310_REG32		XTALCLKCTRL[4]   ;          //	0x50	R/W 
		AS3310_REG32		OCRAMCLKCTRL[4]      ;      //	0x60	R/W 
		AS3310_REG32		UTMICLKCTRL[4]      ;       //	0x70	R/W	
		AS3310_REG32		SSPCLKCTRL[4]     ;         //	0x80	R/W
		AS3310_REG32		GPMICLKCTRL[4]   ;          //	0x90	R/W 
		AS3310_REG32		SPDIFCLKCTRL[4]      ;      //	0xA0	R/W 
		AS3310_REG32		EMICLKCTRL[4]      ;        //	0xB0	R/W	
		AS3310_REG32		IRCLKCTRL[4]     ;          //	0xC0	R/W
		AS3310_REG32		CLKCTRL_PLL180[4]     ;     //	0xD0	R/W   LCD
} /*__attribute__((__packed__))*/ AS3310_CLK;


/*  ==================  GPMI Reg defines  =============================*/
#if 0
typedef struct {
		AS3310_REG32		CTRL0[4];	            //	0x00	R/W		
		AS3310_REG32		COMP[4];                //	0x10	R/W 
		AS3310_REG32		CTRL1[4];               //	0x20	R/W 
		AS3310_REG32		TIMING0[4];             //	0x30	R/W 
		AS3310_REG32		TIMING1[4];             //	0x40	R/W 
		AS3310_REG32		TIMING2[4];             //	0x50	R/W 
		AS3310_REG32		DATA[4];                //	0x60	R/W 
		AS3310_REG32		STAT[4];                //	0x70	R	
		AS3310_REG32		R_DEBUG[4];             //	0x80	R
} AS3310_GPMI; /*__attribute__((__packed__))*/

#define AS3310_GPMI_BASE	0x8000C000

static inline AS3310_GPMI * AS3310_GetBase_GPMI(void)
{
	return (AS3310_GPMI * ) IO_ADDRESS(AS3310_GPMI_BASE);
}
#endif
/*  ==================  USB Reg defines  =============================*/

typedef struct {
		AS3310_REG32		USB_PHY_PWD[4]	 ;	        //	0x00	R/W		
		AS3310_REG32		USB_PHY_TX[4]   ;           //	0x10	R/W 
		AS3310_REG32		USB_PHY_RX[4]     ;         //	0x20	R/W 
		AS3310_REG32		USB_PHY_CTRL[4]	 ;          //	0x30	R/W 
		AS3310_REG32		USB_PHY_STATUS[4]   ;       //	0x40	R/W 
		AS3310_REG32		USB_PHY_DEBUG[4]   ;        //	0x50	R/W 
		AS3310_REG32		USB_PHY_DEBUG0_STATUS[4] ;  //	0x60	R/W 
		AS3310_REG32		USB_PHY_DEBUG1_STATUS[4] ;  //	0x70	R/W	
		AS3310_REG32		USB_PHY_DEBUG2_STATUS[4] ;  //	0x80	R/W
		AS3310_REG32		USB_PHY_SYS_CTRL[4]   ;     //	0x90	R/W 
		AS3310_REG32		USB_PHY_ANALOG[4]      ;    //	0xA0	R/W 
} /*__attribute__((__packed__))*/ AS3310_USB_PHY_CTRL;

#define AS3310_USB_PHY_CTRL_BASE    0x8007C000

static inline AS3310_USB_PHY_CTRL * AS3310_GetBase_USB_PHY_CTRL(void)
{
	return (AS3310_USB_PHY_CTRL * )IO_ADDRESS(AS3310_USB_PHY_CTRL_BASE);
}

typedef struct asm9260_dma_pkg_s {
		unsigned long		NEXT_PKG	;       //	0x00	R/W		
		unsigned long		CTRL        ;       //	0x04	R/W 
		unsigned long		BUFFER      ;       //	0x08	R/W 
		unsigned long		CMD0        ;       //	0x0c	R/W 
		unsigned long		CMD1        ;       //	0x10	R/W 
} /*__attribute__((__packed__))*/ ASM9260_DMA_PKG;

#endif //__ASSEMBLY__

/*===========================================================================*/
/* Bit Field in AS3310_CLK->XTALCLKCTRL 	0x50	R/W */
#define UART_Clock_Gate             (1<<31)
#define Digi_Filter_Clock_Gate      (1<<30)
#define PWM_Clock_Gate              (1<<29)
#define DRI_Clock_Gate              (1<<28)
#define DIG_Clock_Gate              (1<<27)
#define TIMROT_Clock_Gate           (1<<26)
#define EXRAM_Clock_Gate            (1<<25)
#define LRADC_Clock_Gate            (1<<24)
#define UTMI_120MHz_Clock_Gate      (1<<31)
#define UTMI_30MHz_Clock_Gate       (1<<30)





#endif	/* __ASM_ARCH_AS3310_HARDWARE_H */
