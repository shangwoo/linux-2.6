/*
 * linux/arch/arm/mach-as9260/include/mach/pincontrol.h
 *
 * Copyright (C) 2014 Alpscale
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _PIN_CONTROL_H_
#define _PIN_CONTROL_H_

#include <linux/module.h>
#include <linux/kernel.h>

#define GPIO_TOTAL_PORTS           18    /* total banks */
#define GPIO_PINS_OF_EACH_PORTS    8


/* GPIO TYPE */
#define PIN_FUNCTION_0      0
#define PIN_FUNCTION_1      1
#define PIN_FUNCTION_2      2
#define PIN_FUNCTION_3      3
#define PIN_FUNCTION_GPIO   3



void set_pin_mux(int port,int pin,int mux_type);
void set_GPIO_pull_up(int port,int pin);
void set_GPIO_pull_down(int port,int pin);
void asm9260_gpio_init(void);
int get_pin_mux_val(int port,int pin);
void set_GPIO(int port,int pin);
void clear_GPIO(int port,int pin);
void write_GPIO(int port,int pin,int value);
int read_GPIO(int port,int pin);



#define GPIO_IRQ_LEVEL_LOW          0
#define GPIO_IRQ_LEVEL_HIGH         1
#define GPIO_IRQ_EDGE_FALLING       0
#define GPIO_IRQ_EDGE_RISING        1

void io_irq_enable_edge(int port,int pin,int type);
void io_irq_enable_level(int port,int pin,int type);
void io_irq_disable(int port,int pin);
void io_irq_mask(int port,int pin);
void io_irq_unmask(int port,int pin);
void io_irq_clr(int port,int pin);
int get_io_irq_status(int port,int pin);


#endif // _PIN_CONTROL_H_
 
