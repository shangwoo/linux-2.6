/*

Alpha Scale AS3310X IronPalms Console
He Yong, AlpScale Software Engineering, heyong@alpscale.com

GPIO SPI Header file
 
Change log: 

------------------- Version 1.0  ----------------------
He Yong 2007-10-08
    -- Create file

*/

#ifndef __GPIO_SPI_H__
#define __GPIO_SPI_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/pincontrol.h>

//#include <asm/hardware.h>

#define PACKED __attribute__((packed))
//#define malloc asmalloc

#define	ENOMEM		12	/* Out of memory */

#define GPIO_SPI_BANDRATE 0x4E /*  24M / 4 / 38400 /2 = 0x4E  */
//#define GPIO_SPI_BANDRATE 0xffff//0x4E /*  24M / 4 / 38400 /2 = 0x4E  */


    /*   ==========  GPIO SPI Config  ==========  */
#define GPIO_SPI_BITS       24  /* total bits of spi */
#define GPIO_SPI_DATA_BITS  16  /* last x bits is data */


/* public interface */

int gpio_spi_init(ushort bandrate);

int gpio_spi_trans(int data);

int gpio_spi_release(void);


typedef struct gpio_spi_control {
    int     status;       // 
    int     edge;         // 0 fall, 1 rise
    int     index;        // current bit
    int     wdata;        // 
    int     rdata;        // 
    u8      scl_port;
    u8      scl_pin;
    u8      sdi_port;
    u8      sdi_pin;
    u8      sdo_port;
    u8      sdo_pin;
    u8      total_bits;
    u8      data_bits;
} __attribute__((packed)) GPIO_SPI_CCTRL ;

/*
current_bit_index :   [n ... 7 6 5 4 3 2 1 0 ]
*/

#endif // __GPIO_SPI_H__



