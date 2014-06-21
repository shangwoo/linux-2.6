/* linux/include/asm-arm/arch-omap/uart_regl.h
 *
 *  From linux/include/asm-arm/hardware/serial_s3c2410.h
 *
 *  Internal header file for Samsung S3C2410 serial ports (UART0-2)
 *
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *
 *  Additional defines, (c) 2003 Simtec Electronics (linux@simtec.co.uk)
 *
 *  Adapted from:
 *
 *  Internal header file for Alpscale 3310c serial ports (UART1)
 *
 *  Copyright (C) 2007 tony (tony@alpscale.com.cn)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __ASM_ARM_UART_REG_H
#define __ASM_ARM_UART_REG_H


#define AS3310_PA_UART0	(0x80010000)
#define AS3310_VA_UART0        IO_ADDRESS(AS3310_PA_UART0)


#define AS3310_CTRL0		(0x00)
#define AS3310_CTRL0_SET	(0x04)
#define AS3310_CTRL0_CLR	(0x08)
#define AS3310_CTRL0_TOG	(0x0c)

#define AS3310_CTRL1		(0x10)
#define AS3310_CTRL1_SET	(0x14)
#define AS3310_CTRL1_CLR	(0x18)
#define AS3310_CTRL1_TOG	(0x1c)


#define AS3310_CTRL2		(0x20)
#define AS3310_CTRL2_SET	(0x24)
#define AS3310_CTRL2_CLR	(0x28)
#define AS3310_CTRL2_TOG	(0x2c)

#define AS3310_LINECTRL	       	(0x30)
#define AS3310_LINECTRL_SET	(0x34)
#define AS3310_LINECTRL_CLR	(0x38)
#define AS3310_LINECTRL_TOG	(0x3c)

#define AS3310_INTR			(0x40)
#define AS3310_INTR_SET	(0x44)
#define AS3310_INTR_CLR		(0x48)
#define AS3310_INTR_TOG	(0x4c)

#define AS3310_DATA		(0x50)

#define AS3310_STAT			(0x60)

#define AS3310_DBG			(0x70)



//the control register 
#define AS3310_CTRL0_CFGMASK		(0x3<<30)
#define AS3310_CTRL0_SFTRST		(0x01<<31)
#define AS3310_CTRL0_CLKGATE		(0x01<<30)

//----------------------------------------------------//
//the line control register mask to reconfig the uart
#define AS3310_LINECTRL_CFGMASK  ((0xff<<16)|(0x3f<<8)|(0x7f<<1))
#define AS3310_LINECTRL_CS5  		(0x0)
#define AS3310_LINECTRL_CS6		(0x01<<5)
#define AS3310_LINECTRL_CS7		(0x02<<5)
#define AS3310_LINECTRL_CS8		(0x03<<5)
#define AS3310_LINECTRL_CSMASK	(0x03<<5)



#define AS3310_LINECTRL_PNONE	  (0x0)
#define AS3310_LINECTRL_PEVEN	  (0x3 << 1)
#define AS3310_LINECTRL_PODD	  (0x1 << 1)
#define AS3310_LINECTRL_PMASK	  (0x3 << 1)

#define AS3310_LINECTRL_STOP2	  (1<<3)

#define AS3310_LINECTRL_FEN        (1<<4)

//----------------------------------------------------------//



#define AS3310_CTRL2_RXE     (1<<9)
#define AS3310_CTRL2_TXE     (1<<8)




//FIFP STATUS
#define AS3310_FIFO_RMASK           (0x1<<26)
#define AS3310_FIFO_TMASK           (0x1<<25)
#define AS3310_STAT_TXFE           (1<<27)
#define AS3310_STAT_RXFF          (1<<26)
#define AS3310_STAT_TXFF           (1<<25)
#define AS3310_STAT_TXSHIFT        (25)
#define AS3310_STAT_RXFE           (1<<24)
#define AS3310_STAT_RXSHIFT        (26)


#define AS3310_STAT_OERR           (1<<19)
#define AS3310_STAT_BERR           (1<<18)
#define AS3310_STAT_PERR           (1<<17)
#define AS3310_STAT_FERR           (1<<16)

#define AS3310_STAT_ERRALL          (AS3310_STAT_OERR|AS3310_STAT_BERR|AS3310_STAT_PERR|AS3310_STAT_FERR)
//------------------------------------------------------------//



#ifndef __ASSEMBLY__

/* struct s3c24xx_uart_clksrc
 *
 * this structure defines a named clock source that can be used for the
 * uart, so that the best clock can be selected for the requested baud
 * rate.
 *
 * min_baud and max_baud define the range of baud-rates this clock is
 * acceptable for, if they are both zero, it is assumed any baud rate that
 * can be generated from this clock will be used.
 *
 * divisor gives the divisor from the clock to the one seen by the uart
*/

struct as3310_uart_clksrc {
	const char	*name;
	unsigned int	 divisor;
	unsigned int	 min_baud;
	unsigned int	 max_baud;
};

/* configuration structure for per-machine configurations for the
 * serial port
 *
 * the pointer is setup by the machine specific initialisation from the
 * arch/arm/mach-s3c2410/ directory.
*/

struct as3310_uartcfg {
	unsigned char	   hwport;	 /* hardware port number */
	unsigned char	   unused;
	unsigned short	   flags;
	unsigned long	   uart_flags;	 /* default uart flags */

	unsigned long	   ucon;	 /* value of ucon for port */
	unsigned long	   ulcon;	 /* value of ulcon for port */
	unsigned long	   ufcon;	 /* value of ufcon for port */

	struct as3310_uart_clksrc *clocks;
	unsigned int		    clocks_size;
};

/* s3c24xx_uart_devs
 *
 * this is exported from the core as we cannot use driver_register(),
 * or platform_add_device() before the console_initcall()
*/

extern struct platform_device *as3310_uart_dev;

#endif /* __ASSEMBLY__ */

#endif /* __ASM_ARM_REGS_SERIAL_H */


