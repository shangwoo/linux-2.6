/*
 * linux/arch/arm/mach-as9260/pincontrol.c
 *
 * Copyright (C) 2014 Alpscale
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/pincontrol.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>





/* Not all hardware capabilities are exposed through these calls; they
 * only encapsulate the most common features and modes.
 * 
 */

/*
 * GPIO pull_up register function, but this should be NULL, because it's not implemented indeed, informed by LOGIC team.
 */
void set_GPIO_pull_up(int port,int pin)
{
	u32 val, addr;

	addr = HW_IOCON_PIO0_0 + port * 0x20 + pin * 4;
	val = as3310_readl(addr);
	val = (val & 0xffffe7) | 0x10;

	as3310_writel(val, addr);
}
EXPORT_SYMBOL(set_GPIO_pull_up);

/*
 * GPIO pull_down register function.
 */
void set_GPIO_pull_down(int port,int pin)
{
	u32 val, addr;

	addr = HW_IOCON_PIO0_0 + port * 0x20 + pin * 4;
	val = as3310_readl(addr);
	val = (val & 0xffffe7) | 0x08;

	as3310_writel(val, addr);
}
EXPORT_SYMBOL(set_GPIO_pull_down);

/* 
 * Pin mux chosen function.
 */
void set_pin_mux(int port,int pin,int mux_type)
{
	uint32_t val,addr;

	addr = HW_IOCON_PIO_BASE + (port * 32) + (pin * 4);
	val = as3310_readl(addr);   // read org val
	val &= 0xFFFFFFF8; // clear mux feild
	val |= mux_type; // set mux feild with new value

	as3310_writel(val, addr);   // Set Pin mux
}
EXPORT_SYMBOL(set_pin_mux);

/*
 * Get the Pin mux value.
 */
int get_pin_mux_val(int port,int pin)
{
	uint32_t val,addr;

	val = 0xFFFFFFF8; // clear mux feild
	addr = HW_IOCON_PIO_BASE + (port * 32) + (pin * 4);
	return ((as3310_readl(addr) & val));   //read Pin mux
}
EXPORT_SYMBOL(get_pin_mux_val);

static int gpio_bit_offs(int port, int pin)
{
	return (1 << ((port % 4) * 8 + pin));
}

/*
 * PIN input or output func chosen.
 */
void set_pin_dir(int port, int pin, int bOut)
{
	if (bOut)
		as3310_writel_gpio(gpio_bit_offs(port, pin),
				HW_GPIO_DATA_BASE+0x8000 + (port/4)*0x10000 + 4); 
	else
		as3310_writel_gpio(gpio_bit_offs(port, pin),
				HW_GPIO_DATA_BASE+0x8000 + (port/4)*0x10000  + 8); 
}
EXPORT_SYMBOL(set_pin_dir);

/*
 * Pin output as high level
 */
void set_GPIO(int port, int pin)
{
	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_DATA_BASE + (port/4) * 0x10000 + 4);//set GPIO
	set_pin_dir(port,pin,1);    //output en
}
EXPORT_SYMBOL(set_GPIO);

/*
 * Pin output as low level.
 */
void clear_GPIO(int port, int pin)
{
	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_DATA_BASE + (port/4)*0x10000+8);//clear GPIO
	set_pin_dir(port, pin, 1);//output en
}
EXPORT_SYMBOL(clear_GPIO);

/*
 * Pin output function
 */
void write_GPIO(int port,int pin,int value)
{
	if (value)
		set_GPIO(port,pin);
	else
		clear_GPIO(port,pin);
}
EXPORT_SYMBOL(write_GPIO);

/*
 * Get the Pin value.
 */
int read_GPIO(int port,int pin)
{
	uint32_t read_val, val, addr;

	val = gpio_bit_offs(port, pin);
	addr = (HW_GPIO_DATA_BASE + (port/4)*0x10000);  // direction input
	as3310_writel_gpio(val,
			HW_GPIO_DIR0 + (port/4)*0x10000 + 8);  // input Enable

	read_val = as3310_readl_gpio(addr);   // read port

	return (read_val & val) ? 1 : 0;
}
EXPORT_SYMBOL(read_GPIO);

/*************************************************
 ================  IRQ Funtions  ================
**************************************************/

/*
 * GPIO IRQ edge trigger setting function.
 */
void io_irq_enable_edge(int port,int pin,int type)
{

	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_DIR0 + (port/4) * 0x10000 + 8);  // input Enable
	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IC0 + (port/4) * 0x10000 + 4); // Clear pin's interrupt
	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IS0 + (port/4) * 0x10000 + 8); // choose edge trig
	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IBE0 + (port/4) * 0x10000 + 8); //not DoubleEdge

	if (type == GPIO_IRQ_EDGE_FALLING)
		as3310_writel_gpio(gpio_bit_offs(port, pin),
				HW_GPIO_IEV0 + (port/4) * 0x10000 + 8); // choose falling edge 
	else
		as3310_writel_gpio(gpio_bit_offs(port, pin),
				HW_GPIO_IEV0 + (port/4) * 0x10000 + 4); // choose rising edge

	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IE0+(port/4)*0x10000+4);// Enable pin as interrupt source
}
EXPORT_SYMBOL(io_irq_enable_edge);

/*
 * GPIO IRQ level trigger setting function
 */
void io_irq_enable_level(int port,int pin,int type){

	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_DIR0 + (port/4)* 0x10000 + 8);  // input Enable
	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IC0 + (port/4) * 0x10000 + 4);// Clear pin's interrupt
	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IS0 + (port/4) * 0x10000 + 4);// choose level trig
	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IBE0 + (port/4) * 0x10000 + 8); //not DoubleEdge

	if (type == GPIO_IRQ_LEVEL_LOW)
		as3310_writel_gpio(gpio_bit_offs(port, pin),
				HW_GPIO_IEV0 + (port/4) * 0x10000 + 8);    // choose low level 
	else
		as3310_writel_gpio(gpio_bit_offs(port, pin),
				HW_GPIO_IEV0 + (port/4) * 0x10000 + 4);   // choose high level 

	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IE0 + (port/4) * 0x10000 + 4);// Enable pin as interrupt source
}
EXPORT_SYMBOL(io_irq_enable_level);

/*
 * Disable one pin's interrupt function.
 */
void io_irq_disable(int port,int pin){

	io_irq_mask(port, pin);   // Disable IRQ, ie Mask IRQ.
}
EXPORT_SYMBOL(io_irq_disable);

/*
 * Mask a pin's interrupt line.
 */
void io_irq_mask(int port,int pin){

	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IE0 + (port/4) * 0x10000 + 8);   // Mask IRQ
}
EXPORT_SYMBOL(io_irq_mask);

/*
 * Activate a pin's interrupt line.
 */
void io_irq_unmask(int port,int pin){

	as3310_writel_gpio(gpio_bit_offs(port, pin),
			HW_GPIO_IE0 + (port/4) * 0x10000 + 4);  // Unmask IRQ
}
EXPORT_SYMBOL(io_irq_unmask);

/*
 * Clear a pin's interrupt status bit.
 */
void io_irq_clr(int port,int pin)
{
	int pin_offs = gpio_bit_offs(port, pin);
	BUG_ON(as3310_readl_gpio(HW_GPIO_IS0 + (port/4) * 0x10000 + 4)
			& pin_offs);
	as3310_writel_gpio(pin_offs,
			HW_GPIO_IC0 + (port/4) * 0x10000 + 4); // Clear IRQ
}
EXPORT_SYMBOL(io_irq_clr);

/*
 * Check a pin's interrupt status bit.
 */
int get_io_irq_status(int port,int pin)
{
	int val, addr;

	/* count bank reg addr */
	addr = (HW_GPIO_MIS0 + (port/4) * 0x10000 + 4);
	/* Read status */
	val = as3310_readl_gpio(addr);

	return ((val & gpio_bit_offs(port, pin)) != 0);
}
EXPORT_SYMBOL(get_io_irq_status);

void asm9260_gpio_init(void)
{
	printk("%s\n", __func__);

	as3310_writel(0x1 << 4, HW_AHBCLKCTRL0 + 8); /* gpio clk gate */
	as3310_writel(0x1 << 4, HW_AHBCLKCTRL0 + 4); /* gpio clk gate */

	as3310_writel(0x1 << 4, HW_PRESETCTRL0 + 8); /* gpio reset */
	as3310_writel(0x1 << 4, HW_PRESETCTRL0 + 4); /* gpio reset */

	as3310_writel(0x1 << 23, HW_AHBCLKCTRL0 + 4); /* IOCONFIG */
}
