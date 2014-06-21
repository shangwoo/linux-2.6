#ifndef __CONFIG_H
#define __CONFIG_H

//This segment should not be modified
#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef __KERNEL__
    #define __KERNEL__
#endif
#ifndef MODULE
    #define MODULE
#endif

#ifdef CONFIG_SMP
	#define __SMP__
#endif

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/fs.h>           /* everything... */
#include <linux/errno.h>        /* error codes */
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/sched.h>
#include <asm/irq.h>
#include <linux/irq.h>
#include <asm/system.h>         /* cli(), *_flags */
#include <linux/wait.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <asm/sizes.h>
//#include <asm/arch/irqs.h>
//#include  <arch/arm/mach-as9260/include/mach/io.h>
#include  <mach/io.h>

#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

//#include <asm/arch/hardware.h>
//#include <asm/arch/lpc32xx_emc.h>
//#include <asm/arch/lpc32xx_gpio.h>


typedef u8      uint8;          /* defined for unsigned 8-bits integer variable 	 ???8?????  */
typedef s8      int8;           /* defined for signed 8-bits integer variable		 ???8?????  */
typedef u16     uint16;         /* defined for unsigned 16-bits integer variable 	 ???16????? */
typedef s16     int16;          /* defined for signed 16-bits integer variable 		 ???16????? */
typedef u32     uint32;         /* defined for unsigned 32-bits integer variable 	 ???32????? */
typedef s32     int32;          /* defined for signed 32-bits integer variable 		 ???32????? */
typedef float   fp32;           /* single precision floating point variable (32bits) ???????32???? */
typedef double  fp64;           /* double precision floating point variable (64bits) ???????64???? */

/********************************/
/*Application Program Configurations*/
/*     ??????             */
/********************************/
//This segment could be modified as needed.
//????????

#include "asm9260.h"
#define DEV_NAME "ASM9260_CAN"                //CAN??????

#define GPIO_IOBASE io_p2v(GPIO_BASE)


#define CAN0    0x8004C000
#define CAN1    0x80050000


#define WAITWEN2        0x02     
#define WAITOEN2        0x02       
#define WAITRD2         0x1F       
#define WAITPAGE2       0x0F       
#define WAITWR2         0x1F     
#define WAITTURN2       0x0F       

#define LPC32XX_EMC_BASE io_p2v(EMC_BASE)

#define BCFG_16DEF      0x00000001   /* 16Bit Bus                  */
#define BCFG2           ((0x00<<3) | (0x00<<6) | (0x01<<07) | (0x00<<8) | (0x00 <<19) | (0x00<<20))
#define STATICCFG2      ( BCFG_16DEF | BCFG2 ) 


#endif
/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/

