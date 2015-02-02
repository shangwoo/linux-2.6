/*
 * Pinctrl driver for the Alphascale ASM9260 SoC
 *
 * Copyright (c) 2014, Oleksij Rempel <linux@rempel-privat.de>
 *
 * Driver from Toumaz Xenif TZ1090 SoC:
 * Copyright (c) 2013, Imagination Technologies Ltd.
 *
 * Derived from Tegra code:
 * Copyright (c) 2011-2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * Derived from code:
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 NVIDIA Corporation
 * Copyright (C) 2009-2011 ST-Ericsson AB
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/clk.h>

/*
 * The registers may be shared with other threads/cores, so we need to use the
 * metag global lock2 for atomicity.
 */
//#include <asm/global_lock.h>

#include "core.h"
#include "pinconf.h"

/* pinctrl register */
#define IOCON_PIO0_0			0x0000
/* only two modes are supported NONE and PULL UP */
#define IOCON_MODE_SHIFT		3
#define IOCON_MODE_MASK			(0x3 << IOCON_MODE_SHIFT)
/* Only GPIO0_* pins support pull up. */
#define IOCON_MODE_PULL_UP		(0x2 << IOCON_MODE_SHIFT)
/* Only GPIO0_* pins don't support pull down. */
#define IOCON_MODE_PULL_DOWN		(0x1 << IOCON_MODE_SHIFT)
#define IOCON_MODE_NONE			(0x0 << IOCON_MODE_SHIFT)
/* up to 8 functions per pin */
#define IOCON_PINMUX_MASK		(0x7 << 0)

#define ASM9260_PINCTRL_PIN(pin)	PINCTRL_PIN(pin, #pin)
#define MUX_OFFSET(bank, pin)		((bank) * 32 + (pin) * 4)

/**
 * struct asm9260_function - ASM9260 pinctrl mux function
 * @name:	The name of the function, exported to pinctrl core.
 * @groups:	An array of pin groups that may select this function.
 * @ngroups:	The number of entries in @groups.
 */
struct asm9260_function {
	const char		*name;
	const char		**groups;
	unsigned int		ngroups;
};

/**
 * struct asm9260_pingroup - ASM9260 pin group
 * @name:	Name of pin group.
 * @pins:	Array of pin numbers in this pin group.
 * @npins:	Number of pins in this pin group.
 * @mux:	Top level mux.
 * @drv:	Drive control supported, 0 if unsupported.
 *		This means Schmitt, Slew, and Drive strength.
 * @slw_bit:	Slew register bit. 0 if unsupported.
 *		The same bit is used for Schmitt, and Drive (*2).
 * @func:	Currently muxed function.
 * @func_count:	Number of pins using current mux function.
 *
 * A representation of a group of pins (possibly just one pin) in the ASM9260
 * pin controller. Each group allows some parameter or parameters to be
 * configured. The most common is mux function selection.
 */
struct asm9260_pingroup {
	const char		*name;
	const unsigned int	number;
	const unsigned int	bank;
	const unsigned int	pin;

#define MAX_FUNCS_PER_PIN	8
	const int		funcs[MAX_FUNCS_PER_PIN];
};


/* Mux functions that can be used by a mux */
enum asm9260_mux {
	ASM9260_MUX_na = -1,

	ASM9260_MUX_gpio0,
	ASM9260_MUX_cam0,
	ASM9260_MUX_can0,
	ASM9260_MUX_can1,
	ASM9260_MUX_ct0,
	ASM9260_MUX_ct1,
	ASM9260_MUX_ct2,
	ASM9260_MUX_ct3,
	ASM9260_MUX_i2c0,
	ASM9260_MUX_i2c1,
	ASM9260_MUX_i2s0,
	ASM9260_MUX_i2s1,
	ASM9260_MUX_jtag,
	ASM9260_MUX_lcd0,
	ASM9260_MUX_lcd_if0,
	ASM9260_MUX_mc,
	ASM9260_MUX_mc0,
	ASM9260_MUX_mii0,
	ASM9260_MUX_nand0,
	ASM9260_MUX_outclk,
	ASM9260_MUX_qei0,
	ASM9260_MUX_qspi0,
	ASM9260_MUX_rmii0,
	ASM9260_MUX_sd0,
	ASM9260_MUX_spi0,
	ASM9260_MUX_spi1,
	ASM9260_MUX_uart0,
	ASM9260_MUX_uart1,
	ASM9260_MUX_uart2,
	ASM9260_MUX_uart3,
	ASM9260_MUX_uart4,
	ASM9260_MUX_uart5,
	ASM9260_MUX_uart6,
	ASM9260_MUX_uart7,
	ASM9260_MUX_uart8,
	ASM9260_MUX_uart9,
};

#define FUNCTION(mux)			\
	[(ASM9260_MUX_ ## mux)] = {		\
		.name = #mux,			\
		.groups = NULL,			\
		.ngroups = 0,		\
	}

/* Must correlate with enum asm9260_mux */
static struct asm9260_function asm9260_functions[] = {
	FUNCTION(gpio0),
	FUNCTION(cam0),
	FUNCTION(can0),
	FUNCTION(can1),
	FUNCTION(ct0),
	FUNCTION(ct1),
	FUNCTION(ct2),
	FUNCTION(ct3),
	FUNCTION(i2c0),
	FUNCTION(i2c1),
	FUNCTION(i2s0),
	FUNCTION(i2s1),
	FUNCTION(jtag),
	FUNCTION(lcd0),
	FUNCTION(lcd_if0),
	FUNCTION(mc),
	FUNCTION(mc0),
	FUNCTION(mii0),
	FUNCTION(nand0),
	FUNCTION(outclk),
	FUNCTION(qei0),
	FUNCTION(qspi0),
	FUNCTION(rmii0),
	FUNCTION(sd0),
	FUNCTION(spi0),
	FUNCTION(spi1),
	FUNCTION(uart0),
	FUNCTION(uart1),
	FUNCTION(uart2),
	FUNCTION(uart3),
	FUNCTION(uart4),
	FUNCTION(uart5),
	FUNCTION(uart6),
	FUNCTION(uart7),
	FUNCTION(uart8),
	FUNCTION(uart9),
};


#define PMUX(p_number, p_bank, p_pin, p_name, f1, f2, f3, f4, f5, f6, f7) \
	{						\
		.number = p_number,			\
		.bank = p_bank,				\
		.pin = p_pin,				\
		.name = #p_name,			\
		.funcs = {				\
			ASM9260_MUX_gpio0,		\
			ASM9260_MUX_ ## f1,		\
			ASM9260_MUX_ ## f2,		\
			ASM9260_MUX_ ## f3,		\
			ASM9260_MUX_ ## f4,		\
			ASM9260_MUX_ ## f5,		\
			ASM9260_MUX_ ## f6,		\
			ASM9260_MUX_ ## f7,		\
		},					\
	}

static struct asm9260_pingroup asm9260_mux_table[] = {
	PMUX(120,	0,	0,	GPIO0_0,	na,	uart1,	i2s0,	spi1,	na,	jtag,	na),
	PMUX(121,	0,	1,	GPIO0_1,	na,	uart1,	i2s0,	spi1,	na,	jtag,	na),
	PMUX(122,	0,	2,	GPIO0_2,	na,	uart1,	i2s0,	spi1,	na,	jtag,	na),
	PMUX(123,	0,	3,	GPIO0_3,	na,	uart1,	i2s0,	spi1,	na,	jtag,	na),
	PMUX(124,	0,	4,	GPIO0_4,	na,	uart1,	i2s0,	spi0,	na,	jtag,	i2c0),
	PMUX(128,	1,	4,	GPIO1_4,	ct0,	uart0,	lcd_if0,	spi0,	mii0,	lcd0,	na),
	PMUX(129,	1,	5,	GPIO1_5,	na,	uart0,	lcd_if0,	spi0,	rmii0,	lcd0,	na),
	PMUX(130,	1,	6,	GPIO1_6,	na,	uart0,	lcd_if0,	spi0,	rmii0,	lcd0,	i2c1),
	PMUX(131,	1,	7,	GPIO1_7,	na,	uart0,	lcd_if0,	spi0,	rmii0,	lcd0,	i2c1),
	PMUX(132,	2,	0,	GPIO2_0,	ct1,	uart2,	lcd_if0,	spi1,	mii0,	lcd0,	can0),
	PMUX(133,	2,	1,	GPIO2_1,	ct1,	uart2,	lcd_if0,	spi1,	mii0,	lcd0,	can0),
	PMUX(134,	2,	2,	GPIO2_2,	ct1,	uart3,	lcd_if0,	spi1,	mii0,	lcd0,	na),
	PMUX(135,	2,	3,	GPIO2_3,	ct1,	uart3,	lcd_if0,	spi1,	mii0,	lcd0,	na),
	PMUX(136,	2,	4,	GPIO2_4,	ct1,	uart3,	lcd_if0,	na,	mii0,	lcd0,	na),
	PMUX(137,	2,	5,	GPIO2_5,	na,	uart3,	lcd_if0,	na,	mii0,	lcd0,	outclk),
	PMUX(138,	2,	6,	GPIO2_6,	na,	uart3,	lcd_if0,	na,	mii0,	lcd0,	can1),
	PMUX(139,	2,	7,	GPIO2_7,	na,	uart4,	lcd_if0,	na,	mii0,	lcd0,	can1),
	PMUX(140,	3,	0,	GPIO3_0,	ct2,	uart4,	lcd_if0,	sd0,	rmii0,	lcd0,	na),
	PMUX(141,	3,	1,	GPIO3_1,	ct2,	uart4,	lcd_if0,	sd0,	rmii0,	lcd0,	na),
	PMUX(142,	3,	2,	GPIO3_2,	ct2,	uart4,	lcd_if0,	sd0,	rmii0,	lcd0,	can1),
	PMUX(143,	3,	3,	GPIO3_3,	ct2,	uart4,	lcd_if0,	sd0,	rmii0,	lcd0,	can1),
	PMUX(144,	3,	4,	GPIO3_4,	ct2,	uart5,	lcd_if0,	sd0,	rmii0,	lcd0,	outclk),
	PMUX(145,	3,	5,	GPIO3_5,	na,	uart5,	lcd_if0,	sd0,	rmii0,	lcd0,	i2c0),
	PMUX(146,	3,	6,	GPIO3_6,	na,	uart5,	lcd_if0,	na,	rmii0,	lcd0,	i2c0),
	PMUX(147,	3,	7,	GPIO3_7,	na,	uart5,	lcd_if0,	na,	rmii0,	lcd0,	na),
	PMUX(151,	4,	0,	GPIO4_0,	ct3,	uart5,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(152,	4,	1,	GPIO4_1,	ct3,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(153,	4,	2,	GPIO4_2,	ct3,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(154,	4,	3,	GPIO4_3,	ct3,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(155,	4,	4,	GPIO4_4,	ct3,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(156,	4,	5,	GPIO4_5,	na,	uart6,	na,	qspi0,	mii0,	lcd0,	na),
	PMUX(157,	4,	6,	GPIO4_6,	na,	na,	na,	na,	mii0,	lcd0,	i2c1),
	PMUX(158,	4,	7,	GPIO4_7,	na,	na,	na,	na,	mii0,	lcd0,	i2c1),
	PMUX(169,	5,	0,	GPIO5_0,	mc0,	uart7,	i2s1,	sd0,	rmii0,	na,	na),
	PMUX(170,	5,	1,	GPIO5_1,	mc0,	uart7,	i2s1,	sd0,	rmii0,	na,	na),
	PMUX(171,	5,	2,	GPIO5_2,	mc0,	uart7,	i2s1,	sd0,	rmii0,	na,	can0),
	PMUX(172,	5,	3,	GPIO5_3,	mc0,	uart7,	i2s1,	sd0,	rmii0,	na,	can0),
	PMUX(173,	5,	4,	GPIO5_4,	mc0,	uart8,	i2s1,	sd0,	rmii0,	na,	na),
	PMUX(51,	8,	1,	GPIO8_1,	na,	uart2,	cam0,	na,	mii0,	na,	na),
	PMUX(52,	8,	2,	GPIO8_2,	na,	uart2,	cam0,	na,	mii0,	na,	na),
	PMUX(53,	8,	3,	GPIO8_3,	na,	uart2,	cam0,	na,	mii0,	na,	na),
	PMUX(54,	8,	4,	GPIO8_4,	na,	uart2,	cam0,	na,	mii0,	na,	na),
	PMUX(55,	8,	5,	GPIO8_5,	na,	uart3,	cam0,	na,	mii0,	na,	na),
	PMUX(56,	8,	6,	GPIO8_6,	na,	uart3,	cam0,	na,	mii0,	na,	na),
	PMUX(57,	8,	7,	GPIO8_7,	na,	uart3,	cam0,	na,	mii0,	na,	na),
	PMUX(45,	9,	0,	GPIO9_0,	ct0,	uart3,	cam0,	na,	rmii0,	na,	i2c0),
	PMUX(46,	9,	1,	GPIO9_1,	ct0,	uart3,	cam0,	na,	rmii0,	na,	i2c0),
	PMUX(47,	9,	2,	GPIO9_2,	ct0,	uart4,	cam0,	na,	rmii0,	na,	na),
	PMUX(48,	9,	3,	GPIO9_3,	ct0,	uart4,	cam0,	na,	rmii0,	na,	na),
	PMUX(49,	9,	4,	GPIO9_4,	ct0,	uart4,	cam0,	na,	rmii0,	na,	na),
	PMUX(50,	9,	5,	GPIO9_5,	na,	uart4,	cam0,	na,	rmii0,	na,	i2c1),
	PMUX(4,	10,	0,	GPIO10_0,	ct1,	uart5,	i2s0,	spi0,	na,	na,	na),
	PMUX(5,	10,	1,	GPIO10_1,	ct1,	uart5,	i2s0,	spi0,	rmii0,	na,	na),
	PMUX(6,	10,	2,	GPIO10_2,	ct1,	uart5,	i2s0,	spi0,	rmii0,	na,	na),
	PMUX(7,	10,	3,	GPIO10_3,	ct1,	uart5,	i2s0,	spi0,	na,	na,	can0),
	PMUX(8,	10,	4,	GPIO10_4,	ct1,	uart6,	i2s0,	spi1,	rmii0,	na,	na),
	PMUX(9,	10,	5,	GPIO10_5,	na,	uart6,	i2s0,	spi1,	rmii0,	na,	can1),
	PMUX(10,	10,	6,	GPIO10_6,	na,	uart6,	i2s0,	spi1,	rmii0,	na,	can1),
	PMUX(11,	10,	7,	GPIO10_7,	na,	uart6,	cam0,	spi1,	rmii0,	na,	na),
	PMUX(12,	11,	0,	GPIO11_0,	ct2,	uart7,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(13,	11,	1,	GPIO11_1,	ct2,	uart7,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(14,	11,	2,	GPIO11_2,	ct2,	uart7,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(15,	11,	3,	GPIO11_3,	ct2,	uart7,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(16,	11,	4,	GPIO11_4,	ct2,	uart8,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(17,	11,	5,	GPIO11_5,	na,	uart8,	i2s1,	qspi0,	nand0,	na,	na),
	PMUX(18,	11,	6,	GPIO11_6,	na,	uart9,	i2s1,	na,	nand0,	na,	i2c0),
	PMUX(19,	11,	7,	GPIO11_7,	na,	uart9,	cam0,	na,	nand0,	na,	i2c0),
	PMUX(23,	12,	0,	GPIO12_0,	ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(24,	12,	1,	GPIO12_1,	ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(25,	12,	2,	GPIO12_2,	ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(26,	12,	3,	GPIO12_3,	ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(27,	12,	4,	GPIO12_4,	ct3,	uart1,	na,	sd0,	nand0,	na,	na),
	PMUX(28,	12,	5,	GPIO12_5,	na,	uart8,	na,	sd0,	nand0,	na,	na),
	PMUX(29,	12,	6,	GPIO12_6,	na,	uart8,	cam0,	na,	nand0,	na,	i2c1),
	PMUX(30,	12,	7,	GPIO12_7,	na,	na,	cam0,	na,	nand0,	na,	i2c1),
	PMUX(31,	13,	4,	GPIO13_4,	mc,	uart2,	na,	spi1,	nand0,	na,	na),
	PMUX(32,	13,	5,	GPIO13_5,	mc0,	uart9,	na,	spi1,	nand0,	na,	na),
	PMUX(33,	13,	6,	GPIO13_6,	mc0,	uart9,	na,	spi1,	nand0,	na,	na),
	PMUX(34,	13,	7,	GPIO13_7,	mc0,	na,	na,	spi1,	nand0,	na,	na),
	PMUX(38,	14,	0,	GPIO14_0,	mc0,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(39,	14,	1,	GPIO14_1,	mc0,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(40,	14,	2,	GPIO14_2,	na,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(41,	14,	3,	GPIO14_3,	na,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(42,	14,	4,	GPIO14_4,	na,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(43,	14,	5,	GPIO14_5,	na,	uart0,	i2s0,	sd0,	nand0,	na,	na),
	PMUX(44,	15,	0,	GPIO15_0,	na,	uart4,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(61,	15,	1,	GPIO15_1,	na,	uart4,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(62,	15,	2,	GPIO15_2,	na,	uart5,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(63,	15,	3,	GPIO15_3,	na,	uart5,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(64,	15,	4,	GPIO15_4,	na,	uart6,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(65,	15,	5,	GPIO15_5,	na,	uart6,	i2s0,	sd0,	rmii0,	na,	na),
	PMUX(66,	15,	6,	GPIO15_6,	na,	uart7,	i2s0,	na,	rmii0,	na,	na),
	PMUX(67,	15,	7,	GPIO15_7,	na,	uart7,	na,	na,	rmii0,	na,	na),
	PMUX(73,	16,	0,	GPIO16_0,	ct2,	uart4,	na,	na,	mii0,	na,	na),
	PMUX(74,	16,	1,	GPIO16_1,	ct2,	uart4,	na,	na,	mii0,	na,	na),
	PMUX(75,	16,	2,	GPIO16_2,	ct2,	uart5,	na,	na,	mii0,	na,	na),
	PMUX(76,	16,	3,	GPIO16_3,	ct2,	uart5,	na,	na,	mii0,	na,	na),
	PMUX(77,	16,	4,	GPIO16_4,	ct2,	uart6,	na,	na,	mii0,	na,	na),
	PMUX(78,	16,	5,	GPIO16_5,	qei0,	uart6,	na,	na,	mii0,	na,	na),
	PMUX(79,	16,	6,	GPIO16_6,	qei0,	uart6,	na,	na,	mii0,	na,	can1),
	PMUX(80,	16,	7,	GPIO16_7,	qei0,	uart6,	na,	na,	mii0,	na,	can1),
	PMUX(81,	17,	0,	GPIO17_0,	ct3,	uart7,	i2s1,	na,	rmii0,	na,	na),
	PMUX(82,	17,	1,	GPIO17_1,	ct3,	uart7,	i2s1,	na,	na,	na,	na),
	PMUX(83,	17,	2,	GPIO17_2,	ct3,	uart7,	i2s1,	na,	na,	na,	i2c1),
	PMUX(84,	17,	3,	GPIO17_3,	ct3,	uart7,	i2s1,	na,	na,	na,	i2c1),
	PMUX(85,	17,	4,	GPIO17_4,	ct3,	uart8,	i2s1,	na,	na,	na,	na),
	PMUX(86,	17,	5,	GPIO17_5,	qei0,	uart8,	i2s1,	na,	rmii0,	na,	na),
	PMUX(87,	17,	6,	GPIO17_6,	qei0,	uart9,	i2s1,	na,	mii0,	na,	na),
	PMUX(88,	17,	7,	GPIO17_7,	qei0,	uart9,	na,	na,	mii0,	na,	na),
};

#define MUX_TABLE_SIZE		ARRAY_SIZE(asm9260_mux_table)
struct asm9260_pmx_priv {
	struct device		*dev;
	struct pinctrl_dev	*pctl;
	void __iomem		*regs;
	spinlock_t		lock;
	u32			pin_en[3];
	u32			gpio_en[3];

	struct clk		*clk;

	struct pinctrl_pin_desc	pin_desc[MUX_TABLE_SIZE];
};

static void __init asm9260_init_mux_pins(struct asm9260_pmx_priv *priv)
{
	unsigned int i;

	for (i = 0; i < MUX_TABLE_SIZE; i++) {
		priv->pin_desc[i].name = asm9260_mux_table[i].name;
		priv->pin_desc[i].number = asm9260_mux_table[i].number;
	}
}

static inline u32 priv_read(struct asm9260_pmx_priv *priv, u32 reg)
{
	return ioread32(priv->regs + reg);
}

static inline void priv_write(struct asm9260_pmx_priv *priv, u32 val, u32 reg)
{
	iowrite32(val, priv->regs + reg);
}

/*
 * Pin control operations
 */

/* each GPIO pin has it's own pseudo pingroup containing only itself */

static int asm9260_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return MUX_TABLE_SIZE;
}

static const char *asm9260_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						 unsigned int group)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->pin_desc[group].name;
}

static int asm9260_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					 unsigned int group,
					 const unsigned int **pins,
					 unsigned int *num_pins)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);

	*pins = &priv->pin_desc[group].number;
	*num_pins = 1;

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static void asm9260_pinctrl_pin_dbg_show(struct pinctrl_dev *pctldev,
					struct seq_file *s,
					unsigned int offset)
{
	seq_printf(s, " %s", dev_name(pctldev->dev));
}
#endif

static int reserve_map(struct device *dev, struct pinctrl_map **map,
		       unsigned int *reserved_maps, unsigned int *num_maps,
		       unsigned int reserve)
{
	unsigned int old_num = *reserved_maps;
	unsigned int new_num = *num_maps + reserve;
	struct pinctrl_map *new_map;

	if (old_num >= new_num)
		return 0;

	new_map = krealloc(*map, sizeof(*new_map) * new_num, GFP_KERNEL);
	if (!new_map) {
		dev_err(dev, "krealloc(map) failed\n");
		return -ENOMEM;
	}

	memset(new_map + old_num, 0, (new_num - old_num) * sizeof(*new_map));

	*map = new_map;
	*reserved_maps = new_num;

	return 0;
}

static int add_map_mux(struct pinctrl_map **map, unsigned int *reserved_maps,
		       unsigned int *num_maps, const char *group,
		       const char *function)
{
	if (WARN_ON(*num_maps == *reserved_maps))
		return -ENOSPC;

	(*map)[*num_maps].type = PIN_MAP_TYPE_MUX_GROUP;
	(*map)[*num_maps].data.mux.group = group;
	(*map)[*num_maps].data.mux.function = function;
	(*num_maps)++;

	return 0;
}

static int add_map_configs(struct device *dev,
			   struct pinctrl_map **map,
			   unsigned int *reserved_maps, unsigned int *num_maps,
			   const char *group, unsigned long *configs,
			   unsigned int num_configs)
{
	unsigned long *dup_configs;

	if (WARN_ON(*num_maps == *reserved_maps))
		return -ENOSPC;

	dup_configs = kmemdup(configs, num_configs * sizeof(*dup_configs),
			      GFP_KERNEL);
	if (!dup_configs) {
		dev_err(dev, "kmemdup(configs) failed\n");
		return -ENOMEM;
	}

	(*map)[*num_maps].type = PIN_MAP_TYPE_CONFIGS_GROUP;
	(*map)[*num_maps].data.configs.group_or_pin = group;
	(*map)[*num_maps].data.configs.configs = dup_configs;
	(*map)[*num_maps].data.configs.num_configs = num_configs;
	(*num_maps)++;

	return 0;
}

static void asm9260_pinctrl_dt_free_map(struct pinctrl_dev *pctldev,
				       struct pinctrl_map *map,
				       unsigned int num_maps)
{
	int i;

	for (i = 0; i < num_maps; i++)
		if (map[i].type == PIN_MAP_TYPE_CONFIGS_GROUP)
			kfree(map[i].data.configs.configs);

	kfree(map);
}

static int asm9260_pinctrl_dt_subnode_to_map(struct device *dev,
					    struct device_node *np,
					    struct pinctrl_map **map,
					    unsigned int *reserved_maps,
					    unsigned int *num_maps)
{
	int ret;
	const char *function;
	unsigned long *configs = NULL;
	unsigned int num_configs = 0;
	unsigned int reserve;
	struct property *prop;
	const char *group;

	ret = of_property_read_string(np, "alphascale,function", &function);
	if (ret < 0) {
		/* EINVAL=missing, which is fine since it's optional */
		if (ret != -EINVAL)
			dev_err(dev, "could not parse property function\n");
		function = NULL;
	}

	ret = pinconf_generic_parse_dt_config(np, &configs, &num_configs);
	if (ret)
		return ret;

	reserve = 0;
	if (function != NULL)
		reserve++;
	if (num_configs)
		reserve++;
	ret = of_property_count_strings(np, "alphascale,pins");
	if (ret < 0) {
		dev_err(dev, "could not parse property pins\n");
		goto exit;
	}
	reserve *= ret;

	ret = reserve_map(dev, map, reserved_maps, num_maps, reserve);
	if (ret < 0)
		goto exit;

	of_property_for_each_string(np, "alphascale,pins", prop, group) {
		if (function) {
			ret = add_map_mux(map, reserved_maps, num_maps,
					  group, function);
			if (ret < 0)
				goto exit;
		}

		if (num_configs) {
			ret = add_map_configs(dev, map, reserved_maps,
					      num_maps, group, configs,
					      num_configs);
			if (ret < 0)
				goto exit;
		}
	}

	ret = 0;

exit:
	kfree(configs);
	return ret;
}

static int asm9260_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
					 struct device_node *np_config,
					 struct pinctrl_map **map,
					 unsigned int *num_maps)
{
	unsigned int reserved_maps;
	struct device_node *np = np_config;
	int ret;

	reserved_maps = 0;
	*map = NULL;
	*num_maps = 0;

	//for_each_child_of_node(np_config, np) {
		ret = asm9260_pinctrl_dt_subnode_to_map(pctldev->dev, np, map,
						       &reserved_maps,
						       num_maps);
		if (ret < 0) {
			asm9260_pinctrl_dt_free_map(pctldev, *map, *num_maps);
			return ret;
		}
	//}

	return 0;
}

static struct pinctrl_ops asm9260_pinctrl_ops = {
	.get_groups_count	= asm9260_pinctrl_get_groups_count,
	.get_group_name		= asm9260_pinctrl_get_group_name,
	.get_group_pins		= asm9260_pinctrl_get_group_pins,
#ifdef CONFIG_DEBUG_FS
	.pin_dbg_show		= asm9260_pinctrl_pin_dbg_show,
#endif
	.dt_node_to_map		= asm9260_pinctrl_dt_node_to_map,
	.dt_free_map		= asm9260_pinctrl_dt_free_map,
};

/*
 * Pin mux operations
 */
static int asm9260_pinctrl_get_funcs_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(asm9260_functions);
}

static const char *asm9260_pinctrl_get_func_name(struct pinctrl_dev *pctldev,
						unsigned int function)
{
	return asm9260_functions[function].name;
}

static int asm9260_pinctrl_get_func_groups(struct pinctrl_dev *pctldev,
					  unsigned int function,
					  const char * const **groups,
					  unsigned int * const num_groups)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct asm9260_pingroup *table;
	int a, b, count = 0, *tmp;
	const char **gr;

	if (asm9260_functions[function].groups != NULL)
		goto done;

	tmp = devm_kmalloc(priv->dev, sizeof(tmp) * MUX_TABLE_SIZE, GFP_KERNEL);
	if (!tmp) {
		dev_err(priv->dev, "Can't allocate func/pin array\n");
		return PTR_ERR(tmp);
	}

	for (a = 0; a < MUX_TABLE_SIZE; a++) {
		table = &asm9260_mux_table[a];

		for (b = 0; b < MAX_FUNCS_PER_PIN; b++) {
			if (table->funcs[b] == function) {
				tmp[count] = a;
				count++;
			}

		}

	}

	gr = devm_kmalloc(priv->dev,
			sizeof(gr) * count, GFP_KERNEL);
	if (!gr) {
		dev_err(priv->dev, "Can't allocate func group\n");
		devm_kfree(priv->dev, tmp);
		return PTR_ERR(gr);
	}

	for (a = 0; a < count; a++)
		gr[a] = asm9260_mux_table[tmp[a]].name;

	asm9260_functions[function].groups = gr;
	asm9260_functions[function].ngroups = count;
done:
	*groups = asm9260_functions[function].groups;
	*num_groups = asm9260_functions[function].ngroups;
	return 0;
}

#if 0
/**
 * asm9260_pinctrl_select() - update bit in SELECT register
 * @priv:		Pinmux data
 * @pin:		Pin number (must be within GPIO range)
 */
static void asm9260_pinctrl_select(struct asm9260_pmx_priv *priv,
				  unsigned int pin)
{
	u32 reg, reg_shift, select, val;
	unsigned int priv_index, priv_shift;
	unsigned long flags;
	printk("%s:%i\n", __func__, __LINE__);

	/* uses base 32 instead of base 30 */
	priv_index = pin >> 5;
	priv_shift = pin & 0x1f;

	/* select = !perip || gpio */
	select = ((~priv->pin_en[priv_index] |
		   priv->gpio_en[priv_index]) >> priv_shift) & 1;

	/* find register and bit offset (base 30) */
	reg = REG_PINCTRL_SELECT + 4*(pin / 30);
	reg_shift = pin % 30;

	/* modify gpio select bit */
	__global_lock2(flags);
	val = priv_read(priv, reg);
	val &= ~BIT(reg_shift);
	val |= select << reg_shift;
	priv_write(priv, val, reg);
	__global_unlock2(flags);
}

/**
 * asm9260_pinctrl_gpio_select() - enable/disable GPIO usage for a pin
 * @priv:		Pinmux data
 * @pin:		Pin number
 * @gpio_select:	true to enable pin as GPIO,
 *			false to leave control to whatever function is enabled
 *
 * Records that GPIO usage is enabled/disabled so that enabling a function
 * doesn't override the SELECT register bit.
 */
static void asm9260_pinctrl_gpio_select(struct asm9260_pmx_priv *priv,
				       unsigned int pin,
				       bool gpio_select)
{
	unsigned int index, shift;
	u32 gpio_en;

	printk("%s:%i\n", __func__, __LINE__);
	if (pin >= ARRAY_SIZE(asm9260_pins))
		return;

	/* uses base 32 instead of base 30 */
	index = pin >> 5;
	shift = pin & 0x1f;

	spin_lock(&priv->lock);

	/* keep a record whether gpio is selected */
	gpio_en = priv->gpio_en[index];
	gpio_en &= ~BIT(shift);
	if (gpio_select)
		gpio_en |= BIT(shift);
	priv->gpio_en[index] = gpio_en;

	/* update the select bit */
	asm9260_pinctrl_select(priv, pin);

	spin_unlock(&priv->lock);
}

#endif

static int asm9260_pinctrl_set_mux(struct pinctrl_dev *pctldev,
				 unsigned int function, unsigned int pin)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct asm9260_pingroup *table;
	void __iomem            *offset;
	int mux;
	u32 val;

	table = &asm9260_mux_table[pin];
	for (mux = 0; mux < MAX_FUNCS_PER_PIN; ++mux) {
		if (table->funcs[mux] == function)
			goto found_mux;
	}

	return -EINVAL;
found_mux:

	//__global_lock2(flags);
	offset = priv->regs + MUX_OFFSET(table->bank, table->pin);
	val = ioread32(offset);
	val &= ~IOCON_PINMUX_MASK;
	val |= mux;
	iowrite32(val, offset);
	//__global_unlock2(flags);

	return 0;
}

static struct pinmux_ops asm9260_pinmux_ops = {
	.get_functions_count	= asm9260_pinctrl_get_funcs_count,
	.get_function_name	= asm9260_pinctrl_get_func_name,
	.get_function_groups	= asm9260_pinctrl_get_func_groups,
	.set_mux		= asm9260_pinctrl_set_mux,
};

static int asm9260_pinconf_reg(struct pinctrl_dev *pctldev,
			      unsigned int pin,
			      enum pin_config_param param,
			      void __iomem **reg, u32 *val)
{
	struct asm9260_pmx_priv *priv = pinctrl_dev_get_drvdata(pctldev);
	struct asm9260_pingroup *table;
	int a;

	for (a = 0; a < MUX_TABLE_SIZE; a++) {
		table = &asm9260_mux_table[a];
		if (table->number == pin)
			break;
	}

	*reg = priv->regs + MUX_OFFSET(table->bank, table->pin);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		*val = IOCON_MODE_NONE;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (table->bank != 0)
			return -ENOTSUPP;
		*val = IOCON_MODE_PULL_UP;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (table->bank == 0)
			return -ENOTSUPP;
		*val = IOCON_MODE_PULL_DOWN;
		break;
	default:
		return -ENOTSUPP;
	};

	return 0;
}

static int asm9260_pinconf_get(struct pinctrl_dev *pctldev,
			      unsigned int pin, unsigned long *config)
{
	enum pin_config_param param = pinconf_to_config_param(*config);
	int ret;
	u32 val, tmp, arg;
	void __iomem *reg;

	/* Get register information */
	ret = asm9260_pinconf_reg(pctldev, pin, param,
				 &reg, &val);
	if (ret < 0)
		return ret;

	/* Extract field from register */
	tmp = ioread32(reg);
	arg = ((tmp & IOCON_MODE_MASK) >> IOCON_MODE_SHIFT) == val;

	/* Config not active */
	if (!arg)
		return -EINVAL;

	/* And pack config */
	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int asm9260_pinconf_set(struct pinctrl_dev *pctldev,
			      unsigned int pin, unsigned long *configs,
			      unsigned num_configs)
{
	enum pin_config_param param;
	unsigned int arg;
	int ret;
	u32 val, tmp;
	void __iomem *reg;
	int i;

	for (i = 0; i < num_configs; i++) {

		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		/* Get register information */
		ret = asm9260_pinconf_reg(pctldev, pin, param,
					 &reg, &val);
		if (ret < 0)
			return ret;

		/* Write register field */
		//__global_lock2(flags);
		tmp = ioread32(reg);
		tmp &= ~IOCON_MODE_MASK;
		if (arg)
			tmp |= val;
		iowrite32(tmp, reg);
		//__global_unlock2(flags);
	} /* for each config */

	return 0;
}

static struct pinconf_ops asm9260_pinconf_ops = {
	.is_generic			= true,
	.pin_config_get			= asm9260_pinconf_get,
	.pin_config_set			= asm9260_pinconf_set,
};

/*
 * Pin control driver setup
 */

/* OK */
static struct pinctrl_desc asm9260_pinctrl_desc = {
	.pctlops	= &asm9260_pinctrl_ops,
	.pmxops		= &asm9260_pinmux_ops,
	.confops	= &asm9260_pinconf_ops,
	.owner		= THIS_MODULE,
};

/* OK */
static int asm9260_pinctrl_probe(struct platform_device *pdev)
{
	struct asm9260_pmx_priv *priv;
	struct resource	*res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "Can't alloc asm9260_priv\n");
		return -ENOMEM;
	}
	priv->dev = &pdev->dev;
	spin_lock_init(&priv->lock);

	asm9260_init_mux_pins(priv);

	asm9260_pinctrl_desc.name = dev_name(&pdev->dev);
	asm9260_pinctrl_desc.pins = priv->pin_desc;
	asm9260_pinctrl_desc.npins = MUX_TABLE_SIZE;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	priv->clk = devm_clk_get(&pdev->dev, "ahb");
	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clk!\n");
		return ret;
	}

	priv->pctl = pinctrl_register(&asm9260_pinctrl_desc, &pdev->dev, priv);
	if (!priv->pctl) {
		dev_err(&pdev->dev, "Couldn't register pinctrl driver\n");
		ret = -ENODEV;
		goto err_return;
	}

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "ASM9260 pinctrl driver initialised\n");

	return 0;
err_return:
	clk_disable_unprepare(priv->clk);
	return ret;
}

static int asm9260_pinctrl_remove(struct platform_device *pdev)
{
	struct asm9260_pmx_priv *priv = platform_get_drvdata(pdev);

	pinctrl_unregister(priv->pctl);

	return 0;
}

static struct of_device_id asm9260_pinctrl_of_match[] = {
	{ .compatible = "alphascale,asm9260-pinctrl", },
	{ },
};

static struct platform_driver asm9260_pinctrl_driver = {
	.driver = {
		.name		= "asm9260-pinctrl",
		.owner		= THIS_MODULE,
		.of_match_table	= asm9260_pinctrl_of_match,
	},
	.probe	= asm9260_pinctrl_probe,
	.remove	= asm9260_pinctrl_remove,
};

static int __init asm9260_pinctrl_init(void)
{
	return platform_driver_register(&asm9260_pinctrl_driver);
}
arch_initcall(asm9260_pinctrl_init);

static void __exit asm9260_pinctrl_exit(void)
{
	platform_driver_unregister(&asm9260_pinctrl_driver);
}
module_exit(asm9260_pinctrl_exit);

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("Alphascale ASM9260 pinctrl driver");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, asm9260_pinctrl_of_match);
