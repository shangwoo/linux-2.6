/***********************************************
 *  linux/arch/arm/mach-as9260/asm9260_canserial.c
 *  Copyright (C) 2014 Alpscale
 *
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <mach/system.h>
#include <asm/hardware/iomd.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/uart_reg.h>
#include <mach/irqs.h>
#include <mach/asm9260_uart.h>
#include <mach/asm9260_canserial.h>
#include <mach/hardware.h>
#include <mach/pincontrol.h>
/***************************************CAN ****************************************************/

# define ASM9260_MAX_CAN 2
static struct platform_device *__initdata asm9260_cans[ASM9260_MAX_CAN];


/* 
 * CAN0 resource definition.
 */
static struct resource can0_resources[] = {
	[0] = {
		.start	=HW_CAN0,
		.end	=HW_CAN0+ SZ_16K - 1,
		.flags	=IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_CAN0,
		.end	= INT_CAN0,
		.flags	= IORESOURCE_IRQ,
	},
};

/* 
 * CAN0 data definition.
 */

static struct asm9260_data can0_data = {
        .clock   =0,	
	.ocr     =0,		
	.cdr     =0,	
	.regs = (void __iomem *)IO_ADDRESS(HW_CAN0),
};

static u64 can0_dmamask = DMA_BIT_MASK(32);

/* 
 * CAN0 platform_device definition.
 */
static struct platform_device asm9260_can0_device = {
	.name		= "asm9260_platform",
	.id			= 0,
	.dev		= {
				.dma_mask		= &can0_dmamask,
				.coherent_dma_mask	= DMA_BIT_MASK(32),
				.platform_data		= &can0_data,
	},
	.resource	= can0_resources,
	.num_resources	= ARRAY_SIZE(can0_resources),
};


/* 
 * can0 pin configuration.
 */
static inline void configure_can0_pins(unsigned pins)
{
	/*TXD & RXD*/

}

/* 
 * CAN1 resource definition.
 */
static struct resource can1_resources[] = {
	[0] = {
		.start	= HW_CAN1,
		.end	= HW_CAN1+ SZ_16K - 1,
		.flags	= IORESOURCE_MEM ,
	},
	[1] = {
		.start	= INT_CAN1,
		.end	=INT_CAN1 ,
		.flags	= IORESOURCE_IRQ,
	},
};

/* 
 * CAN1 data definition.
 */
static struct asm9260_data can1_data = {
        .clock   =0,	
	.ocr     =0,		
	.cdr     =0,
	.regs = (void __iomem *)IO_ADDRESS(HW_CAN1),
};

static u64 can1_dmamask = DMA_BIT_MASK(32);

/* 
 * CAN1 platform_device definition.
 */
static struct platform_device asm9260_can1_device = {
	.name		= "asm9260_platform",
	.id			= 1,
	.dev		= {
				.dma_mask		= &can1_dmamask,
				.coherent_dma_mask	= DMA_BIT_MASK(32),
				.platform_data		= &can1_data,
	},
	.resource	= can1_resources,
	.num_resources	= ARRAY_SIZE(can1_resources),
};

/* 
 * can1 pin configuration.
 */
static inline void configure_can1_pins (unsigned pins)
{
   	/*TXD & RXD*/
	set_pin_mux(10, 5, 7);
	set_pin_mux(10, 6, 7);
}

/* 
 * can register function.
 */
void __init asm9260_register_can(unsigned id, unsigned portnr, unsigned pins)
{
	struct platform_device *pdev;
	switch (id)
	{
		case 0:
			pdev = &asm9260_can0_device;
			configure_can0_pins(pins);
			break;

		case 1:
			pdev = &asm9260_can1_device;
			configure_can1_pins(pins);
			break;
		default:
			return;
		
	}
	
	pdev->id = id;

	if (id < ASM9260_MAX_CAN)
		asm9260_cans[id] = pdev;
}

/* 
 * this function can add can dev.
 */
void __init asm9260_add_device_can(void)
{
	int i;

	for (i = 0; i < ASM9260_MAX_CAN; i++) {
		if (asm9260_cans[i])
                {
			platform_device_register(asm9260_cans[i]);
                     printk("asm9260 can  register!!!!\n");
                }
	}

}

/****************************************CAN**********************************************************/
