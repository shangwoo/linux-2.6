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

/*
 * The registers may be shared with other threads/cores, so we need to use the
 * metag global lock2 for atomicity.
 */
//#include <asm/global_lock.h>

#include "core.h"
#include "pinconf.h"


////////////////////////////////////////////////////
/* pinctrl register */
#define IOCON_PIO0_0			0x0000
/* only two modes are supported NONE and PULL UP */
#define IOCON_MODE_SHIFT		3
#define IOCON_MODE_MASK			(0x3 << IOCON_MODE_SHIFT)
#define IOCON_MODE_PULL_UP		(0x2 << IOCON_MODE_SHIFT)
#define IOCON_MODE_PULL_NONE		(0x0 << IOCON_MODE_SHIFT)
/* up to 8 functions per pin */
#define IOCON_PINMUX_MASK		(0x7 << 0)

#define ASM9260_PINCTRL_PIN(pin)	PINCTRL_PIN(pin, #pin)
#define PINID(bank, pin)		((bank) * 32 + (pin) * 4)
//////////////////////////////////////////////////////

/* Register offsets from bank base address */
#define REG_PINCTRL_SELECT	0x10
#define REG_PINCTRL_SCHMITT	0x90
#define REG_PINCTRL_PU_PD	0xa0
#define REG_PINCTRL_SR		0xc0
#define REG_PINCTRL_DR		0xd0
#define REG_PINCTRL_IF_CTL	0xe0

/* REG_PINCTRL_DR field values */
#define REG_DR_2mA		0
#define REG_DR_4mA		1
#define REG_DR_8mA		2
#define REG_DR_12mA		3

/**
 * struct asm9260_function - ASM9260 pinctrl mux function
 * @name:	The name of the function, exported to pinctrl core.
 * @groups:	An array of pin groups that may select this function.
 * @ngroups:	The number of entries in @groups.
 */
struct asm9260_function {
	const char		*name;
	const char * const	*groups;
	unsigned int		ngroups;
};

/**
 * struct asm9260_muxdesc - ASM9260 individual mux description
 * @funcs:	Function for each mux value.
 * @reg:	Mux register offset. 0 if unsupported.
 * @bit:	Mux register bit. 0 if unsupported.
 * @width:	Mux field width. 0 if unsupported.
 *
 * A representation of a group of signals (possibly just one signal) in the
 * ASM9260 which can be muxed to a set of functions or sub muxes.
 */
struct asm9260_muxdesc {
	int	funcs[5];
	u16	reg;
	u8	bit;
	u8	width;
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
	const unsigned int	*pins;
	unsigned int		npins;
	struct asm9260_muxdesc	mux;

	bool			drv;
	u8			slw_bit;

	int			func;
	unsigned int		func_count;
};

////////////////////////////////////////////////////
/*
 * all pinctrl register offsets are based on GPIO names. So we will
 * use GPIOs for pins to be close to documentation.
 */
/*      PIN Name;   PinID = Reg Offset; Pin number */
enum asm9260_pin_enum {
	GPIO0_0		= PINID(0, 0), /* 120 */
	GPIO0_1		= PINID(0, 1), /* 121 */
	GPIO0_2		= PINID(0, 2), /* 122 */
	GPIO0_3		= PINID(0, 3), /* 123 */
	GPIO0_4		= PINID(0, 4), /* 124 */

	GPIO1_4		= PINID(1, 4), /* 128 */
	GPIO1_5		= PINID(1, 5), /* 129 */
	GPIO1_6		= PINID(1, 6), /* 130 */
	GPIO1_7		= PINID(1, 7), /* 131 */

	GPIO2_0		= PINID(2, 0), /* 132 */
	GPIO2_1		= PINID(2, 1), /* 133 */
	GPIO2_2		= PINID(2, 2), /* 134 */
	GPIO2_3		= PINID(2, 3), /* 135 */
	GPIO2_4		= PINID(2, 4), /* 136 */
	GPIO2_5		= PINID(2, 5), /* 137 */
	GPIO2_6		= PINID(2, 6), /* 138 */
	GPIO2_7		= PINID(2, 7), /* 139 */

	GPIO3_0		= PINID(3, 0), /* 140 */
	GPIO3_1		= PINID(3, 1), /* 141 */
	GPIO3_2		= PINID(3, 2), /* 142 */
	GPIO3_3		= PINID(3, 3), /* 143 */
	GPIO3_4		= PINID(3, 4), /* 144 */
	GPIO3_5		= PINID(3, 5), /* 145 */
	GPIO3_6		= PINID(3, 6), /* 146 */
	GPIO3_7		= PINID(3, 7), /* 147 */

	GPIO4_0		= PINID(4, 0), /* 151 */
	GPIO4_1		= PINID(4, 1), /* 152 */
	GPIO4_2		= PINID(4, 2), /* 153 */
	GPIO4_3		= PINID(4, 3), /* 154 */
	GPIO4_4		= PINID(4, 4), /* 155 */
	GPIO4_5		= PINID(4, 5), /* 156 */
	GPIO4_6		= PINID(4, 6), /* 157 */
	GPIO4_7		= PINID(4, 7), /* 158 */

	GPIO5_0		= PINID(5, 0), /* 169 */
	GPIO5_1		= PINID(5, 1), /* 170 */
	GPIO5_2		= PINID(5, 2), /* 171 */
	GPIO5_3		= PINID(5, 3), /* 172 */
	GPIO5_4		= PINID(5, 4), /* 173 */

	GPIO8_1		= PINID(8, 1), /* 51 */
	GPIO8_2		= PINID(8, 2), /* 52 */
	GPIO8_3		= PINID(8, 3), /* 53 */
	GPIO8_4		= PINID(8, 4), /* 54 */
	GPIO8_5		= PINID(8, 5), /* 55 */
	GPIO8_6		= PINID(8, 6), /* 56 */
	GPIO8_7		= PINID(8, 7), /* 57 */

	GPIO9_0		= PINID(9, 0), /* 45 */
	GPIO9_1		= PINID(9, 1), /* 46 */
	GPIO9_2		= PINID(9, 2), /* 47 */
	GPIO9_3		= PINID(9, 3), /* 48 */
	GPIO9_4		= PINID(9, 4), /* 49 */
	GPIO9_5		= PINID(9, 5), /* 50 */

	GPIO10_0	= PINID(10, 0), /* 4 */
	GPIO10_1	= PINID(10, 1), /* 5 */
	GPIO10_2	= PINID(10, 2), /* 6 */
	GPIO10_3	= PINID(10, 3), /* 7 */
	GPIO10_4	= PINID(10, 4), /* 8 */
	GPIO10_5	= PINID(10, 5), /* 9 */
	GPIO10_6	= PINID(10, 6), /* 10 */
	GPIO10_7	= PINID(10, 7), /* 11 */

	GPIO11_0	= PINID(11, 0), /* 12 */
	GPIO11_1	= PINID(11, 1), /* 13 */
	GPIO11_2	= PINID(11, 2), /* 14 */
	GPIO11_3	= PINID(11, 3), /* 15 */
	GPIO11_4	= PINID(11, 4), /* 16 */
	GPIO11_5	= PINID(11, 5), /* 17 */
	GPIO11_6	= PINID(11, 6), /* 18 */
	GPIO11_7	= PINID(11, 7), /* 19 */

	GPIO12_0	= PINID(12, 0), /* 23 */
	GPIO12_1	= PINID(12, 1), /* 24 */
	GPIO12_2	= PINID(12, 2), /* 25 */
	GPIO12_3	= PINID(12, 3), /* 26 */
	GPIO12_4	= PINID(12, 4), /* 27 */
	GPIO12_5	= PINID(12, 5), /* 28 */
	GPIO12_6	= PINID(12, 6), /* 29 */
	GPIO12_7	= PINID(12, 7), /* 30 */

	GPIO13_4	= PINID(13, 4), /* 31 */
	GPIO13_5	= PINID(13, 5), /* 32 */
	GPIO13_6	= PINID(13, 6), /* 33 */
	GPIO13_7	= PINID(13, 7), /* 34 */

	GPIO14_0	= PINID(14, 0), /* 38 */
	GPIO14_1	= PINID(14, 1), /* 39 */
	GPIO14_2	= PINID(14, 2), /* 40 */
	GPIO14_3	= PINID(14, 3), /* 41 */
	GPIO14_4	= PINID(14, 4), /* 42 */
	GPIO14_5	= PINID(14, 5), /* 43 */

	GPIO15_0	= PINID(15, 0), /* 44 */
	GPIO15_1	= PINID(15, 1), /* 61 */
	GPIO15_2	= PINID(15, 2), /* 62 */
	GPIO15_3	= PINID(15, 3), /* 63 */
	GPIO15_4	= PINID(15, 4), /* 64 */
	GPIO15_5	= PINID(15, 5), /* 65 */
	GPIO15_6	= PINID(15, 6), /* 66 */
	GPIO15_7	= PINID(15, 7), /* 67 */

	GPIO16_0	= PINID(16, 0), /* 73 */
	GPIO16_1	= PINID(16, 1), /* 74 */
	GPIO16_2	= PINID(16, 2), /* 75 */
	GPIO16_3	= PINID(16, 3), /* 76 */
	GPIO16_4	= PINID(16, 4), /* 77 */
	GPIO16_5	= PINID(16, 5), /* 78 */
	GPIO16_6	= PINID(16, 6), /* 79 */
	GPIO16_7	= PINID(16, 7), /* 80 */

	GPIO17_0	= PINID(17, 0), /* 81 */
	GPIO17_1	= PINID(17, 1), /* 82 */
	GPIO17_2	= PINID(17, 2), /* 83 */
	GPIO17_3	= PINID(17, 3), /* 84 */
	GPIO17_4	= PINID(17, 4), /* 85 */
	GPIO17_5	= PINID(17, 5), /* 86 */
	GPIO17_6	= PINID(17, 6), /* 87 */
	GPIO17_7	= PINID(17, 7), /* 88 */
};
/////////////////////////////////////////////////

static const struct pinctrl_pin_desc asm9260_pins[] = {
	ASM9260_PINCTRL_PIN(GPIO0_0),
	ASM9260_PINCTRL_PIN(GPIO0_1),
	ASM9260_PINCTRL_PIN(GPIO0_2),
	ASM9260_PINCTRL_PIN(GPIO0_3),
	ASM9260_PINCTRL_PIN(GPIO0_4),

	ASM9260_PINCTRL_PIN(GPIO1_4),
	ASM9260_PINCTRL_PIN(GPIO1_5),
	ASM9260_PINCTRL_PIN(GPIO1_6),
	ASM9260_PINCTRL_PIN(GPIO1_7),

	ASM9260_PINCTRL_PIN(GPIO2_0),
	ASM9260_PINCTRL_PIN(GPIO2_1),
	ASM9260_PINCTRL_PIN(GPIO2_2),
	ASM9260_PINCTRL_PIN(GPIO2_3),
	ASM9260_PINCTRL_PIN(GPIO2_4),
	ASM9260_PINCTRL_PIN(GPIO2_5),
	ASM9260_PINCTRL_PIN(GPIO2_6),
	ASM9260_PINCTRL_PIN(GPIO2_7),

	ASM9260_PINCTRL_PIN(GPIO3_0),
	ASM9260_PINCTRL_PIN(GPIO3_1),
	ASM9260_PINCTRL_PIN(GPIO3_2),
	ASM9260_PINCTRL_PIN(GPIO3_3),
	ASM9260_PINCTRL_PIN(GPIO3_4),
	ASM9260_PINCTRL_PIN(GPIO3_5),
	ASM9260_PINCTRL_PIN(GPIO3_6),
	ASM9260_PINCTRL_PIN(GPIO3_7),

	ASM9260_PINCTRL_PIN(GPIO4_0),
	ASM9260_PINCTRL_PIN(GPIO4_1),
	ASM9260_PINCTRL_PIN(GPIO4_2),
	ASM9260_PINCTRL_PIN(GPIO4_3),
	ASM9260_PINCTRL_PIN(GPIO4_4),
	ASM9260_PINCTRL_PIN(GPIO4_5),
	ASM9260_PINCTRL_PIN(GPIO4_6),
	ASM9260_PINCTRL_PIN(GPIO4_7),

	ASM9260_PINCTRL_PIN(GPIO5_0),
	ASM9260_PINCTRL_PIN(GPIO5_1),
	ASM9260_PINCTRL_PIN(GPIO5_2),
	ASM9260_PINCTRL_PIN(GPIO5_3),
	ASM9260_PINCTRL_PIN(GPIO5_4),

	ASM9260_PINCTRL_PIN(GPIO8_1),
	ASM9260_PINCTRL_PIN(GPIO8_2),
	ASM9260_PINCTRL_PIN(GPIO8_3),
	ASM9260_PINCTRL_PIN(GPIO8_4),
	ASM9260_PINCTRL_PIN(GPIO8_5),
	ASM9260_PINCTRL_PIN(GPIO8_6),
	ASM9260_PINCTRL_PIN(GPIO8_7),

	ASM9260_PINCTRL_PIN(GPIO9_0),
	ASM9260_PINCTRL_PIN(GPIO9_1),
	ASM9260_PINCTRL_PIN(GPIO9_2),
	ASM9260_PINCTRL_PIN(GPIO9_3),
	ASM9260_PINCTRL_PIN(GPIO9_4),
	ASM9260_PINCTRL_PIN(GPIO9_5),

	ASM9260_PINCTRL_PIN(GPIO10_0),
	ASM9260_PINCTRL_PIN(GPIO10_1),
	ASM9260_PINCTRL_PIN(GPIO10_2),
	ASM9260_PINCTRL_PIN(GPIO10_3),
	ASM9260_PINCTRL_PIN(GPIO10_4),
	ASM9260_PINCTRL_PIN(GPIO10_5),
	ASM9260_PINCTRL_PIN(GPIO10_6),
	ASM9260_PINCTRL_PIN(GPIO10_7),

	ASM9260_PINCTRL_PIN(GPIO11_0),
	ASM9260_PINCTRL_PIN(GPIO11_1),
	ASM9260_PINCTRL_PIN(GPIO11_2),
	ASM9260_PINCTRL_PIN(GPIO11_3),
	ASM9260_PINCTRL_PIN(GPIO11_4),
	ASM9260_PINCTRL_PIN(GPIO11_5),
	ASM9260_PINCTRL_PIN(GPIO11_6),
	ASM9260_PINCTRL_PIN(GPIO11_7),

	ASM9260_PINCTRL_PIN(GPIO12_0),
	ASM9260_PINCTRL_PIN(GPIO12_1),
	ASM9260_PINCTRL_PIN(GPIO12_2),
	ASM9260_PINCTRL_PIN(GPIO12_3),
	ASM9260_PINCTRL_PIN(GPIO12_4),
	ASM9260_PINCTRL_PIN(GPIO12_5),
	ASM9260_PINCTRL_PIN(GPIO12_6),
	ASM9260_PINCTRL_PIN(GPIO12_7),

	ASM9260_PINCTRL_PIN(GPIO13_4),
	ASM9260_PINCTRL_PIN(GPIO13_5),
	ASM9260_PINCTRL_PIN(GPIO13_6),
	ASM9260_PINCTRL_PIN(GPIO13_7),

	ASM9260_PINCTRL_PIN(GPIO14_0),
	ASM9260_PINCTRL_PIN(GPIO14_1),
	ASM9260_PINCTRL_PIN(GPIO14_2),
	ASM9260_PINCTRL_PIN(GPIO14_3),
	ASM9260_PINCTRL_PIN(GPIO14_4),
	ASM9260_PINCTRL_PIN(GPIO14_5),

	ASM9260_PINCTRL_PIN(GPIO15_0),
	ASM9260_PINCTRL_PIN(GPIO15_1),
	ASM9260_PINCTRL_PIN(GPIO15_2),
	ASM9260_PINCTRL_PIN(GPIO15_3),
	ASM9260_PINCTRL_PIN(GPIO15_4),
	ASM9260_PINCTRL_PIN(GPIO15_5),
	ASM9260_PINCTRL_PIN(GPIO15_6),
	ASM9260_PINCTRL_PIN(GPIO15_7),

	ASM9260_PINCTRL_PIN(GPIO16_0),
	ASM9260_PINCTRL_PIN(GPIO16_1),
	ASM9260_PINCTRL_PIN(GPIO16_2),
	ASM9260_PINCTRL_PIN(GPIO16_3),
	ASM9260_PINCTRL_PIN(GPIO16_4),
	ASM9260_PINCTRL_PIN(GPIO16_5),
	ASM9260_PINCTRL_PIN(GPIO16_6),
	ASM9260_PINCTRL_PIN(GPIO16_7),

	ASM9260_PINCTRL_PIN(GPIO17_0),
	ASM9260_PINCTRL_PIN(GPIO17_1),
	ASM9260_PINCTRL_PIN(GPIO17_2),
	ASM9260_PINCTRL_PIN(GPIO17_3),
	ASM9260_PINCTRL_PIN(GPIO17_4),
	ASM9260_PINCTRL_PIN(GPIO17_5),
	ASM9260_PINCTRL_PIN(GPIO17_6),
	ASM9260_PINCTRL_PIN(GPIO17_7),
};

static const unsigned int uart1_pins[] = {
	GPIO0_0, /* clk */
	GPIO0_1, /* tx */
	GPIO0_2, /* rx */
	GPIO0_3, /* rts */
	GPIO0_4, /* cts */
};

static const unsigned int uart4_0_pins[] = {
	GPIO2_7, /* clk */
	GPIO3_0, /* tx */
	GPIO3_1, /* rx */
	GPIO3_2, /* rts */
	GPIO3_3, /* cts */
};

static const unsigned int uart4_1_pins[] = {
	GPIO9_2, /* clk */
	GPIO9_3, /* tx */
	GPIO9_4, /* rx */
	GPIO9_5, /* rts */
	/* TODO: no cts, probably ASM9260_PIN_GPIO3_3 should be used */
};

/* Pins in each pin group */
static const char * const CAM_DAT0_groups[] = {
	"GPIO8_4",
};

static const char * const CAM_DAT1_groups[] = {
	"GPIO8_5",
};

static const char * const CAM_DAT2_groups[] = {
	"GPIO8_6",
};

static const char * const CAM_DAT3_groups[] = {
	"GPIO8_7",
};

static const char * const CAM_DAT4_groups[] = {
	"GPIO9_0",
};

static const char * const CAM_DAT5_groups[] = {
	"GPIO9_1",
};

static const char * const CAM_DAT6_groups[] = {
	"GPIO9_2",
};

static const char * const CAM_DAT7_groups[] = {
	"GPIO9_3",
};

static const char * const CAM_DAT8_groups[] = {
	"GPIO9_4",
};

static const char * const CAM_DAT9_groups[] = {
	"GPIO9_5",
};

static const char * const CAM_HREF_groups[] = {
	"GPIO8_3",
};

static const char * const CAM_MCLK_groups[] = {
	"GPIO10_7", "GPIO11_7", "GPIO12_6", "GPIO12_7",
};

static const char * const CAM_PCLK_groups[] = {
	"GPIO8_1",
};

static const char * const CAM_VSYN_groups[] = {
	"GPIO8_2",
};

static const char * const CAN0_RX_groups[] = {
	"GPIO5_3", "GPIO10_3",
};

static const char * const CAN0_TX_groups[] = {
	"GPIO5_2",
};

static const char * const CAN1_RX_groups[] = {
	"GPIO2_7", "GPIO10_5", "GPIO16_7",
};

static const char * const CAN1_TX_groups[] = {
	"GPIO2_6", "GPIO10_6", "GPIO16_6",
};

static const char * const CT0_CAP_groups[] = {
	"GPIO1_4", "GPIO9_4",
};

static const char * const CT0_MAT0_groups[] = {
	"GPIO9_0",
};

static const char * const CT0_MAT1_groups[] = {
	"GPIO9_1",
};

static const char * const CT0_MAT2_groups[] = {
	"GPIO9_2",
};

static const char * const CT0_MAT3_groups[] = {
	"GPIO9_3",
};

static const char * const CT1_CAP_groups[] = {
	"GPIO2_4", "GPIO10_4",
};

static const char * const CT1_MAT0_groups[] = {
	"GPIO2_0", "GPIO10_0",
};

static const char * const CT1_MAT1_groups[] = {
	"GPIO2_1", "GPIO10_1",
};

static const char * const CT1_MAT2_groups[] = {
	"GPIO2_2", "GPIO10_2",
};

static const char * const CT1_MAT3_groups[] = {
	"GPIO2_3", "GPIO10_3",
};

static const char * const CT2_CAP_groups[] = {
	"GPIO3_4", "GPIO11_4", "GPIO16_4",
};

static const char * const CT2_MAT0_groups[] = {
	"GPIO3_0", "GPIO11_0", "GPIO16_0",
};

static const char * const CT2_MAT1_groups[] = {
	"GPIO3_1", "GPIO11_1", "GPIO16_1",
};

static const char * const CT2_MAT2_groups[] = {
	"GPIO3_2", "GPIO11_2", "GPIO16_2",
};

static const char * const CT2_MAT3_groups[] = {
	"GPIO3_3", "GPIO11_3", "GPIO16_3",
};

static const char * const CT3_CAP_groups[] = {
	"GPIO4_4", "GPIO12_4", "GPIO17_4",
};

static const char * const CT3_MAT0_groups[] = {
	"GPIO4_0", "GPIO12_0", "GPIO17_0",
};

static const char * const CT3_MAT1_groups[] = {
	"GPIO4_1", "GPIO12_1", "GPIO17_1",
};

static const char * const CT3_MAT2_groups[] = {
	"GPIO4_2", "GPIO12_2", "GPIO17_2",
};

static const char * const CT3_MAT3_groups[] = {
	"GPIO4_3", "GPIO12_3", "GPIO17_3",
};

static const char * const I2C0_SCL_groups[] = {
	"GPIO0_4", "GPIO3_5", "GPIO9_0", "GPIO11_6",
};

static const char * const I2C0_SDA_groups[] = {
	"GPIO3_6", "GPIO9_1", "GPIO11_7",
};

static const char * const I2C1_SCL_groups[] = {
	"GPIO1_6", "GPIO4_6", "GPIO9_5", "GPIO12_6", "GPIO17_2",
};

static const char * const I2C1_SDA_groups[] = {
	"GPIO1_7", "GPIO4_7", "GPIO12_7", "GPIO17_3",
};

static const char * const I2S0_BCLK_groups[] = {
	"GPIO0_1", "GPIO10_1", "GPIO14_1", "GPIO15_1",
};

static const char * const I2S0_LRC_groups[] = {
	"GPIO0_2", "GPIO10_2", "GPIO14_2", "GPIO15_2",
};

static const char * const I2S0_MCLK_groups[] = {
	"GPIO0_0", "GPIO10_0", "GPIO14_0", "GPIO15_0",
};

static const char * const I2S0_RX0_groups[] = {
	"GPIO0_3", "GPIO10_3", "GPIO14_3", "GPIO15_3",
};

static const char * const I2S0_TX0_groups[] = {
	"GPIO0_4", "GPIO10_4", "GPIO14_4", "GPIO15_4",
};

static const char * const I2S0_TX1_groups[] = {
	"GPIO10_5", "GPIO14_5", "GPIO15_5",
};

static const char * const I2S0_TX2_groups[] = {
	"GPIO10_6", "GPIO15_6",
};

static const char * const I2S1_BCLK_groups[] = {
	"GPIO5_1", "GPIO11_1", "GPIO17_1",
};

static const char * const I2S1_LRC_groups[] = {
	"GPIO5_2", "GPIO11_2", "GPIO17_2",
};

static const char * const I2S1_MCLK_groups[] = {
	"GPIO5_0", "GPIO11_0", "GPIO17_0",
};

static const char * const I2S1_RX0_groups[] = {
	"GPIO5_3", "GPIO11_3", "GPIO17_3",
};

static const char * const I2S1_TX0_groups[] = {
	"GPIO5_4", "GPIO11_4", "GPIO17_4",
};

static const char * const I2S1_TX1_groups[] = {
	"GPIO11_5", "GPIO17_5",
};

static const char * const I2S1_TX2_groups[] = {
	"GPIO11_6", "GPIO17_6",
};

static const char * const JTAG_groups[] = {
	"GPIO0_0", "GPIO0_1", "GPIO0_2", "GPIO0_3", "GPIO0_4",
};

static const char * const LCD_AC_OQ_groups[] = {
	"GPIO1_6",
};

static const char * const LCD_CP_OQ_groups[] = {
	"GPIO1_4",
};

static const char * const LCD_FP_OQ_groups[] = {
	"GPIO1_5",
};

static const char * const LCD_IF_BUSY_groups[] = {
	"GPIO1_4",
};

static const char * const LCD_IF_CS_groups[] = {
	"GPIO1_7",
};

static const char * const LCD_IF_DAT0_groups[] = {
	"GPIO2_0",
};

static const char * const LCD_IF_DAT1_groups[] = {
	"GPIO2_1",
};

static const char * const LCD_IF_DAT10_groups[] = {
	"GPIO3_2",
};

static const char * const LCD_IF_DAT11_groups[] = {
	"GPIO3_3",
};

static const char * const LCD_IF_DAT12_groups[] = {
	"GPIO3_4",
};

static const char * const LCD_IF_DAT13_groups[] = {
	"GPIO3_5",
};

static const char * const LCD_IF_DAT14_groups[] = {
	"GPIO3_6",
};

static const char * const LCD_IF_DAT15_groups[] = {
	"GPIO3_7",
};

static const char * const LCD_IF_DAT2_groups[] = {
	"GPIO2_2",
};

static const char * const LCD_IF_DAT3_groups[] = {
	"GPIO2_3",
};

static const char * const LCD_IF_DAT4_groups[] = {
	"GPIO2_4",
};

static const char * const LCD_IF_DAT5_groups[] = {
	"GPIO2_5",
};

static const char * const LCD_IF_DAT6_groups[] = {
	"GPIO2_6",
};

static const char * const LCD_IF_DAT7_groups[] = {
	"GPIO2_7",
};

static const char * const LCD_IF_DAT8_groups[] = {
	"GPIO3_0",
};

static const char * const LCD_IF_DAT9_groups[] = {
	"GPIO3_1",
};

static const char * const LCD_IF_RS_groups[] = {
	"GPIO1_6",
};

static const char * const LCD_IF_WR_groups[] = {
	"GPIO1_5",
};

static const char * const LCD_LP_OQ_groups[] = {
	"GPIO1_7",
};

static const char * const LCD_PIXEL_OQ0_groups[] = {
	"GPIO2_0",
};

static const char * const LCD_PIXEL_OQ1_groups[] = {
	"GPIO2_1",
};

static const char * const LCD_PIXEL_OQ10_groups[] = {
	"GPIO3_2",
};

static const char * const LCD_PIXEL_OQ11_groups[] = {
	"GPIO3_3",
};

static const char * const LCD_PIXEL_OQ12_groups[] = {
	"GPIO3_4",
};

static const char * const LCD_PIXEL_OQ13_groups[] = {
	"GPIO3_5",
};

static const char * const LCD_PIXEL_OQ14_groups[] = {
	"GPIO3_6",
};

static const char * const LCD_PIXEL_OQ15_groups[] = {
	"GPIO3_7",
};

static const char * const LCD_PIXEL_OQ16_groups[] = {
	"GPIO4_0",
};

static const char * const LCD_PIXEL_OQ17_groups[] = {
	"GPIO4_1",
};

static const char * const LCD_PIXEL_OQ18_groups[] = {
	"GPIO4_2",
};

static const char * const LCD_PIXEL_OQ19_groups[] = {
	"GPIO4_3",
};

static const char * const LCD_PIXEL_OQ2_groups[] = {
	"GPIO2_2",
};

static const char * const LCD_PIXEL_OQ20_groups[] = {
	"GPIO4_4",
};

static const char * const LCD_PIXEL_OQ21_groups[] = {
	"GPIO4_5",
};

static const char * const LCD_PIXEL_OQ22_groups[] = {
	"GPIO4_6",
};

static const char * const LCD_PIXEL_OQ23_groups[] = {
	"GPIO4_7",
};

static const char * const LCD_PIXEL_OQ3_groups[] = {
	"GPIO2_3",
};

static const char * const LCD_PIXEL_OQ4_groups[] = {
	"GPIO2_4",
};

static const char * const LCD_PIXEL_OQ5_groups[] = {
	"GPIO2_5",
};

static const char * const LCD_PIXEL_OQ6_groups[] = {
	"GPIO2_6",
};

static const char * const LCD_PIXEL_OQ7_groups[] = {
	"GPIO2_7",
};

static const char * const LCD_PIXEL_OQ8_groups[] = {
	"GPIO3_0",
};

static const char * const LCD_PIXEL_OQ9_groups[] = {
	"GPIO3_1",
};

static const char * const MCI0_groups[] = {
	"GPIO5_1",
};

static const char * const MCI1_groups[] = {
	"GPIO5_4", "GPIO13_4",
};

static const char * const MCI2_groups[] = {
	"GPIO13_7",
};

static const char * const MCIABORT_groups[] = {
	"GPIO5_0",
};

static const char * const MCOA0_groups[] = {
	"GPIO5_2",
};

static const char * const MCOA1_groups[] = {
	"GPIO13_5",
};

static const char * const MCOA2_groups[] = {
	"GPIO14_0",
};

static const char * const MCOB0_groups[] = {
	"GPIO5_3",
};

static const char * const MCOB1_groups[] = {
	"GPIO13_6",
};

static const char * const MCOB2_groups[] = {
	"GPIO14_1",
};

static const char * const MII_COL_groups[] = {
	"GPIO2_6", "GPIO4_6", "GPIO8_6", "GPIO16_6",
};

static const char * const MII_CRS_groups[] = {
	"GPIO2_5", "GPIO4_5", "GPIO8_5", "GPIO16_5",
};

static const char * const MII_PPS_OUT_groups[] = {
	"GPIO1_4", "GPIO17_6", "GPIO17_7",
};

static const char * const MII_RXD2_groups[] = {
	"GPIO2_0", "GPIO4_0", "GPIO16_0",
};

static const char * const MII_RXD3_groups[] = {
	"GPIO2_1", "GPIO4_1", "GPIO8_1", "GPIO16_1",
};

static const char * const MII_RX_ER_groups[] = {
	"GPIO2_7", "GPIO4_7", "GPIO8_7", "GPIO16_7",
};

static const char * const MII_TX_CLK_groups[] = {
	"GPIO2_4", "GPIO4_4", "GPIO8_4", "GPIO16_4",
};

static const char * const MII_TXD2_groups[] = {
	"GPIO2_2", "GPIO4_2", "GPIO8_2", "GPIO16_2",
};

static const char * const MII_TXD3_groups[] = {
	"GPIO2_3", "GPIO4_3", "GPIO8_3", "GPIO16_3",
};

static const char * const NAND_ALE_groups[] = {
	"GPIO11_2",
};

static const char * const NAND_CE0N_groups[] = {
	"GPIO11_6", "GPIO13_6",
};

static const char * const NAND_CE1N_groups[] = {
	"GPIO11_7", "GPIO13_7",
};

static const char * const NAND_CLE_groups[] = {
	"GPIO11_3",
};

static const char * const NAND_D0_groups[] = {
	"GPIO12_0", "GPIO14_0",
};

static const char * const NAND_D1_groups[] = {
	"GPIO12_1", "GPIO14_1",
};

static const char * const NAND_D2_groups[] = {
	"GPIO12_2", "GPIO14_2",
};

static const char * const NAND_D3_groups[] = {
	"GPIO12_3", "GPIO14_3",
};

static const char * const NAND_D4_groups[] = {
	"GPIO12_4", "GPIO14_4",
};

static const char * const NAND_D5_groups[] = {
	"GPIO12_5", "GPIO14_5",
};

static const char * const NAND_D6_groups[] = {
	"GPIO12_6",
};

static const char * const NAND_D7_groups[] = {
	"GPIO12_7",
};

static const char * const NAND_RDY0_groups[] = {
	"GPIO11_4", "GPIO13_4",
};

static const char * const NAND_RDY1_groups[] = {
	"GPIO11_5", "GPIO13_5",
};

static const char * const NAND_REN_groups[] = {
	"GPIO11_0",
};

static const char * const NAND_WEN_groups[] = {
	"GPIO11_1",
};

static const char * const OUTCLK_groups[] = {
	"GPIO2_5",
};

static const char * const QEI_A_groups[] = {
	"GPIO16_5", "GPIO17_5",
};

static const char * const QEI_B_groups[] = {
	"GPIO16_6", "GPIO17_6",
};

static const char * const QEI_INDEX_groups[] = {
	"GPIO16_7", "GPIO17_7",
};

static const char * const QSPI0_DAT0_groups[] = {
	"GPIO4_2", "GPIO11_2",
};

static const char * const QSPI0_DAT1_groups[] = {
	"GPIO4_3", "GPIO11_3",
};

static const char * const QSPI0_DAT2_groups[] = {
	"GPIO4_4", "GPIO11_4",
};

static const char * const QSPI0_DAT3_groups[] = {
	"GPIO4_5", "GPIO11_5",
};

static const char * const QSPI0_SCK_groups[] = {
	"GPIO4_0", "GPIO11_0",
};

static const char * const QSPI0_SEL_groups[] = {
	"GPIO4_1", "GPIO11_1",
};

static const char * const RMII_CRS_DV_groups[] = {
	"GPIO3_2", "GPIO5_2", "GPIO9_2", "GPIO15_2",
};

static const char * const RMII_MDC_groups[] = {
	"GPIO3_0", "GPIO5_0", "GPIO9_0", "GPIO15_0",
};

static const char * const RMII_MDIO_groups[] = {
	"GPIO3_1", "GPIO5_1", "GPIO9_1", "GPIO15_1",
};

static const char * const RMII_REFCLK_groups[] = {
	"GPIO1_5", "GPIO1_6", "GPIO1_7", "GPIO10_1", "GPIO10_2", "GPIO10_4",
	"GPIO10_5", "GPIO10_6", "GPIO10_7", "GPIO17_0", "GPIO17_5",
};

static const char * const RMII_RXD0_groups[] = {
	"GPIO3_3", "GPIO5_3", "GPIO9_3", "GPIO15_3",
};

static const char * const RMII_RXD1_groups[] = {
	"GPIO3_4", "GPIO5_4", "GPIO9_4", "GPIO15_4",
};

static const char * const RMII_TXD0_groups[] = {
	"GPIO3_6", "GPIO15_6",
};

static const char * const RMII_TXD1_groups[] = {
	"GPIO3_7", "GPIO15_7",
};

static const char * const RMII_TX_EN_groups[] = {
	"GPIO3_5", "GPIO9_5", "GPIO15_5",
};

static const char * const SD0_CLK_groups[] = {
	"GPIO3_0", "GPIO5_0", "GPIO12_0", "GPIO14_0", "GPIO15_0",
};

static const char * const SD0_CMD_groups[] = {
	"GPIO3_1", "GPIO5_1", "GPIO12_1", "GPIO14_1", "GPIO15_1",
};

static const char * const SD0_DAT0_groups[] = {
	"GPIO3_2", "GPIO5_2", "GPIO12_2", "GPIO14_2", "GPIO15_2",
};

static const char * const SD0_DAT1_groups[] = {
	"GPIO3_3", "GPIO5_3", "GPIO12_3", "GPIO14_3", "GPIO15_3",
};

static const char * const SD0_DAT2_groups[] = {
	"GPIO3_4", "GPIO5_4", "GPIO12_4", "GPIO14_4", "GPIO15_4",
};

static const char * const SD0_DAT3_groups[] = {
	"GPIO3_5", "GPIO12_5", "GPIO14_5", "GPIO15_5",
};

static const char * const SPI0_MISO_groups[] = {
	"GPIO1_6", "GPIO10_2",
};

static const char * const SPI0_MOSI_groups[] = {
	"GPIO1_7", "GPIO10_3",
};

static const char * const SPI0_SCK_groups[] = {
	"GPIO0_4", "GPIO1_4", "GPIO10_0",
};

static const char * const SPI0_SEL_groups[] = {
	"GPIO1_5", "GPIO10_1",
};

static const char * const SPI1_MISO_groups[] = {
	"GPIO0_2", "GPIO2_2", "GPIO10_6", "GPIO13_6",
};

static const char * const SPI1_MOSI_groups[] = {
	"GPIO0_3", "GPIO2_3", "GPIO10_7", "GPIO13_7",
};

static const char * const SPI1_SCK_groups[] = {
	"GPIO0_0", "GPIO2_0", "GPIO10_4", "GPIO13_4",
};

static const char * const SPI1_SEL_groups[] = {
	"GPIO0_1", "GPIO2_1", "GPIO10_5", "GPIO13_5",
};

static const char * const UART0_CTS_groups[] = {
	"GPIO14_3",
};

static const char * const UART0_DCD_groups[] = {
	"GPIO1_6",
};

static const char * const UART0_DSR_groups[] = {
	"GPIO1_5", "GPIO14_5",
};

static const char * const UART0_DTR_groups[] = {
	"GPIO1_4", "GPIO14_4",
};

static const char * const UART0_RI_groups[] = {
	"GPIO1_7",
};

static const char * const UART0_RTS_groups[] = {
	"GPIO14_2",
};

static const char * const UART0_RXD_groups[] = {
	"GPIO14_1",
};

static const char * const UART0_TXD_groups[] = {
	"GPIO14_0",
};

static const char * const UART1_CLK_groups[] = {
	"GPIO0_0", "GPIO12_0",
};

static const char * const UART1_CTS_groups[] = {
	"GPIO0_4", "GPIO12_4",
};

static const char * const UART1_RTS_groups[] = {
	"GPIO0_3", "GPIO12_3",
};

static const char * const UART1_RXD_groups[] = {
	"GPIO0_2", "GPIO12_2",
};

static const char * const UART1_TXD_groups[] = {
	"GPIO0_1", "GPIO12_1",
};

static const char * const UART2_CTS_groups[] = {
	"GPIO2_1", "GPIO8_4", "GPIO13_4",
};

static const char * const UART2_RTS_groups[] = {
	"GPIO2_0", "GPIO8_3",
};

static const char * const UART2_RXD_groups[] = {
	"GPIO8_2",
};

static const char * const UART2_TXD_groups[] = {
	"GPIO8_1",
};

static const char * const UART3_CLK_groups[] = {
	"GPIO2_2", "GPIO8_5",
};

static const char * const UART3_CTS_groups[] = {
	"GPIO2_6", "GPIO9_1",
};

static const char * const UART3_RTS_groups[] = {
	"GPIO2_5", "GPIO9_0",
};

static const char * const UART3_RXD_groups[] = {
	"GPIO2_4", "GPIO8_7",
};

static const char * const UART3_TXD_groups[] = {
	"GPIO2_3", "GPIO8_6",
};

static const char * const UART4_CLK_groups[] = {
	"GPIO2_7", "GPIO9_2",
};

static const char * const UART4_CTS_groups[] = {
	"GPIO3_3",
};

static const char * const UART4_RTS_groups[] = {
	"GPIO3_2", "GPIO9_5",
};

static const char * const UART4_RXD_groups[] = {
	"GPIO3_1", "GPIO9_4", "GPIO15_1", "GPIO16_1",
};

static const char * const UART4_TXD_groups[] = {
	"GPIO3_0", "GPIO9_3", "GPIO15_0", "GPIO16_0",
};

static const char * const UART5_CLK_groups[] = {
	"GPIO3_4",
};

static const char * const UART5_CTS_groups[] = {
	"GPIO4_0", "GPIO10_3",
};

static const char * const UART5_RTS_groups[] = {
	"GPIO3_7", "GPIO10_2",
};

static const char * const UART5_RXD_groups[] = {
	"GPIO3_6", "GPIO10_1", "GPIO15_3", "GPIO16_3",
};

static const char * const UART5_TXD_groups[] = {
	"GPIO3_5", "GPIO10_0", "GPIO15_2", "GPIO16_2",
};

static const char * const UART6_CLK_groups[] = {
	"GPIO4_1",
};

static const char * const UART6_CTS_groups[] = {
	"GPIO4_5", "GPIO10_7", "GPIO16_7",
};

static const char * const UART6_RTS_groups[] = {
	"GPIO4_4", "GPIO10_6", "GPIO16_6",
};

static const char * const UART6_RXD_groups[] = {
	"GPIO4_3", "GPIO10_5", "GPIO15_5", "GPIO16_5",
};

static const char * const UART6_TXD_groups[] = {
	"GPIO4_2", "GPIO10_4", "GPIO15_4", "GPIO16_4",
};

static const char * const UART7_CTS_groups[] = {
	"GPIO5_3", "GPIO11_3", "GPIO17_3",
};

static const char * const UART7_RTS_groups[] = {
	"GPIO5_2", "GPIO11_2", "GPIO17_2",
};

static const char * const UART7_RXD_groups[] = {
	"GPIO5_1", "GPIO11_1", "GPIO15_7", "GPIO17_1",
};

static const char * const UART7_TXD_groups[] = {
	"GPIO5_0", "GPIO11_0", "GPIO15_6", "GPIO17_0",
};

static const char * const UART8_RXD_groups[] = {
	"GPIO11_5", "GPIO12_6", "GPIO17_5",
};

static const char * const UART8_TXD_groups[] = {
	"GPIO5_4", "GPIO11_4", "GPIO12_5", "GPIO17_4",
};

static const char * const UART9_RXD_groups[] = {
	"GPIO11_7", "GPIO13_6", "GPIO17_7",
};

static const char * const UART9_TXD_groups[] = {
	"GPIO11_6", "GPIO13_5", "GPIO17_6",
};


/* Mux functions that can be used by a mux */

enum asm9260_mux {
	ASM9260_MUX_NA = -1,

	ASM9260_MUX_CAM_DAT0,
	ASM9260_MUX_CAM_DAT1,
	ASM9260_MUX_CAM_DAT2,
	ASM9260_MUX_CAM_DAT3,
	ASM9260_MUX_CAM_DAT4,
	ASM9260_MUX_CAM_DAT5,
	ASM9260_MUX_CAM_DAT6,
	ASM9260_MUX_CAM_DAT7,
	ASM9260_MUX_CAM_DAT8,
	ASM9260_MUX_CAM_DAT9,
	ASM9260_MUX_CAM_HREF,
	ASM9260_MUX_CAM_MCLK,
	ASM9260_MUX_CAM_PCLK,
	ASM9260_MUX_CAM_VSYN,
	ASM9260_MUX_CAN0_RX,
	ASM9260_MUX_CAN0_TX,
	ASM9260_MUX_CAN1_RX,
	ASM9260_MUX_CAN1_TX,
	ASM9260_MUX_CT0_CAP,
	ASM9260_MUX_CT0_MAT0,
	ASM9260_MUX_CT0_MAT1,
	ASM9260_MUX_CT0_MAT2,
	ASM9260_MUX_CT0_MAT3,
	ASM9260_MUX_CT1_CAP,
	ASM9260_MUX_CT1_MAT0,
	ASM9260_MUX_CT1_MAT1,
	ASM9260_MUX_CT1_MAT2,
	ASM9260_MUX_CT1_MAT3,
	ASM9260_MUX_CT2_CAP,
	ASM9260_MUX_CT2_MAT0,
	ASM9260_MUX_CT2_MAT1,
	ASM9260_MUX_CT2_MAT2,
	ASM9260_MUX_CT2_MAT3,
	ASM9260_MUX_CT3_CAP,
	ASM9260_MUX_CT3_MAT0,
	ASM9260_MUX_CT3_MAT1,
	ASM9260_MUX_CT3_MAT2,
	ASM9260_MUX_CT3_MAT3,
	ASM9260_MUX_I2C0_SCL,
	ASM9260_MUX_I2C0_SDA,
	ASM9260_MUX_I2C1_SCL,
	ASM9260_MUX_I2C1_SDA,
	ASM9260_MUX_I2S0_BCLK,
	ASM9260_MUX_I2S0_LRC,
	ASM9260_MUX_I2S0_MCLK,
	ASM9260_MUX_I2S0_RX0,
	ASM9260_MUX_I2S0_TX0,
	ASM9260_MUX_I2S0_TX1,
	ASM9260_MUX_I2S0_TX2,
	ASM9260_MUX_I2S1_BCLK,
	ASM9260_MUX_I2S1_LRC,
	ASM9260_MUX_I2S1_MCLK,
	ASM9260_MUX_I2S1_RX0,
	ASM9260_MUX_I2S1_TX0,
	ASM9260_MUX_I2S1_TX1,
	ASM9260_MUX_I2S1_TX2,
	ASM9260_MUX_JTAG,
	ASM9260_MUX_LCD_AC_OQ,
	ASM9260_MUX_LCD_CP_OQ,
	ASM9260_MUX_LCD_FP_OQ,
	ASM9260_MUX_LCD_IF_BUSY,
	ASM9260_MUX_LCD_IF_CS,
	ASM9260_MUX_LCD_IF_DAT0,
	ASM9260_MUX_LCD_IF_DAT1,
	ASM9260_MUX_LCD_IF_DAT10,
	ASM9260_MUX_LCD_IF_DAT11,
	ASM9260_MUX_LCD_IF_DAT12,
	ASM9260_MUX_LCD_IF_DAT13,
	ASM9260_MUX_LCD_IF_DAT14,
	ASM9260_MUX_LCD_IF_DAT15,
	ASM9260_MUX_LCD_IF_DAT2,
	ASM9260_MUX_LCD_IF_DAT3,
	ASM9260_MUX_LCD_IF_DAT4,
	ASM9260_MUX_LCD_IF_DAT5,
	ASM9260_MUX_LCD_IF_DAT6,
	ASM9260_MUX_LCD_IF_DAT7,
	ASM9260_MUX_LCD_IF_DAT8,
	ASM9260_MUX_LCD_IF_DAT9,
	ASM9260_MUX_LCD_IF_RS,
	ASM9260_MUX_LCD_IF_WR,
	ASM9260_MUX_LCD_LP_OQ,
	ASM9260_MUX_LCD_PIXEL_OQ0,
	ASM9260_MUX_LCD_PIXEL_OQ1,
	ASM9260_MUX_LCD_PIXEL_OQ10,
	ASM9260_MUX_LCD_PIXEL_OQ11,
	ASM9260_MUX_LCD_PIXEL_OQ12,
	ASM9260_MUX_LCD_PIXEL_OQ13,
	ASM9260_MUX_LCD_PIXEL_OQ14,
	ASM9260_MUX_LCD_PIXEL_OQ15,
	ASM9260_MUX_LCD_PIXEL_OQ16,
	ASM9260_MUX_LCD_PIXEL_OQ17,
	ASM9260_MUX_LCD_PIXEL_OQ18,
	ASM9260_MUX_LCD_PIXEL_OQ19,
	ASM9260_MUX_LCD_PIXEL_OQ2,
	ASM9260_MUX_LCD_PIXEL_OQ20,
	ASM9260_MUX_LCD_PIXEL_OQ21,
	ASM9260_MUX_LCD_PIXEL_OQ22,
	ASM9260_MUX_LCD_PIXEL_OQ23,
	ASM9260_MUX_LCD_PIXEL_OQ3,
	ASM9260_MUX_LCD_PIXEL_OQ4,
	ASM9260_MUX_LCD_PIXEL_OQ5,
	ASM9260_MUX_LCD_PIXEL_OQ6,
	ASM9260_MUX_LCD_PIXEL_OQ7,
	ASM9260_MUX_LCD_PIXEL_OQ8,
	ASM9260_MUX_LCD_PIXEL_OQ9,
	ASM9260_MUX_MCI0,
	ASM9260_MUX_MCI1,
	ASM9260_MUX_MCI2,
	ASM9260_MUX_MCIABORT,
	ASM9260_MUX_MCOA0,
	ASM9260_MUX_MCOA1,
	ASM9260_MUX_MCOA2,
	ASM9260_MUX_MCOB0,
	ASM9260_MUX_MCOB1,
	ASM9260_MUX_MCOB2,
	ASM9260_MUX_MII_COL,
	ASM9260_MUX_MII_CRS,
	ASM9260_MUX_MII_PPS_OUT,
	ASM9260_MUX_MII_RXD2,
	ASM9260_MUX_MII_RXD3,
	ASM9260_MUX_MII_RX_ER,
	ASM9260_MUX_MII_TX_CLK,
	ASM9260_MUX_MII_TXD2,
	ASM9260_MUX_MII_TXD3,
	ASM9260_MUX_NAND_ALE,
	ASM9260_MUX_NAND_CE0N,
	ASM9260_MUX_NAND_CE1N,
	ASM9260_MUX_NAND_CLE,
	ASM9260_MUX_NAND_D0,
	ASM9260_MUX_NAND_D1,
	ASM9260_MUX_NAND_D2,
	ASM9260_MUX_NAND_D3,
	ASM9260_MUX_NAND_D4,
	ASM9260_MUX_NAND_D5,
	ASM9260_MUX_NAND_D6,
	ASM9260_MUX_NAND_D7,
	ASM9260_MUX_NAND_RDY0,
	ASM9260_MUX_NAND_RDY1,
	ASM9260_MUX_NAND_REN,
	ASM9260_MUX_NAND_WEN,
	ASM9260_MUX_OUTCLK,
	ASM9260_MUX_QEI_A,
	ASM9260_MUX_QEI_B,
	ASM9260_MUX_QEI_INDEX,
	ASM9260_MUX_QSPI0_DAT0,
	ASM9260_MUX_QSPI0_DAT1,
	ASM9260_MUX_QSPI0_DAT2,
	ASM9260_MUX_QSPI0_DAT3,
	ASM9260_MUX_QSPI0_SCK,
	ASM9260_MUX_QSPI0_SEL,
	ASM9260_MUX_RMII_CRS_DV,
	ASM9260_MUX_RMII_MDC,
	ASM9260_MUX_RMII_MDIO,
	ASM9260_MUX_RMII_REFCLK,
	ASM9260_MUX_RMII_RXD0,
	ASM9260_MUX_RMII_RXD1,
	ASM9260_MUX_RMII_TXD0,
	ASM9260_MUX_RMII_TXD1,
	ASM9260_MUX_RMII_TX_EN,
	ASM9260_MUX_SD0_CLK,
	ASM9260_MUX_SD0_CMD,
	ASM9260_MUX_SD0_DAT0,
	ASM9260_MUX_SD0_DAT1,
	ASM9260_MUX_SD0_DAT2,
	ASM9260_MUX_SD0_DAT3,
	ASM9260_MUX_SPI0_MISO,
	ASM9260_MUX_SPI0_MOSI,
	ASM9260_MUX_SPI0_SCK,
	ASM9260_MUX_SPI0_SEL,
	ASM9260_MUX_SPI1_MISO,
	ASM9260_MUX_SPI1_MOSI,
	ASM9260_MUX_SPI1_SCK,
	ASM9260_MUX_SPI1_SEL,
	ASM9260_MUX_UART0_CTS,
	ASM9260_MUX_UART0_DCD,
	ASM9260_MUX_UART0_DSR,
	ASM9260_MUX_UART0_DTR,
	ASM9260_MUX_UART0_RI,
	ASM9260_MUX_UART0_RTS,
	ASM9260_MUX_UART0_RXD,
	ASM9260_MUX_UART0_TXD,
	ASM9260_MUX_UART1_CLK,
	ASM9260_MUX_UART1_CTS,
	ASM9260_MUX_UART1_RTS,
	ASM9260_MUX_UART1_RXD,
	ASM9260_MUX_UART1_TXD,
	ASM9260_MUX_UART2_CTS,
	ASM9260_MUX_UART2_RTS,
	ASM9260_MUX_UART2_RXD,
	ASM9260_MUX_UART2_TXD,
	ASM9260_MUX_UART3_CLK,
	ASM9260_MUX_UART3_CTS,
	ASM9260_MUX_UART3_RTS,
	ASM9260_MUX_UART3_RXD,
	ASM9260_MUX_UART3_TXD,
	ASM9260_MUX_UART4_CLK,
	ASM9260_MUX_UART4_CTS,
	ASM9260_MUX_UART4_RTS,
	ASM9260_MUX_UART4_RXD,
	ASM9260_MUX_UART4_TXD,
	ASM9260_MUX_UART5_CLK,
	ASM9260_MUX_UART5_CTS,
	ASM9260_MUX_UART5_RTS,
	ASM9260_MUX_UART5_RXD,
	ASM9260_MUX_UART5_TXD,
	ASM9260_MUX_UART6_CLK,
	ASM9260_MUX_UART6_CTS,
	ASM9260_MUX_UART6_RTS,
	ASM9260_MUX_UART6_RXD,
	ASM9260_MUX_UART6_TXD,
	ASM9260_MUX_UART7_CTS,
	ASM9260_MUX_UART7_RTS,
	ASM9260_MUX_UART7_RXD,
	ASM9260_MUX_UART7_TXD,
	ASM9260_MUX_UART8_RXD,
	ASM9260_MUX_UART8_TXD,
	ASM9260_MUX_UART9_RXD,
	ASM9260_MUX_UART9_TXD,
};

#define FUNCTION(mux)			\
	[(ASM9260_MUX_ ## mux)] = {			\
		.name = #mux,				\
		.groups = mux##_groups,		\
		.ngroups = ARRAY_SIZE(mux##_groups),	\
	}

/* Must correlate with enum asm9260_mux */
static const struct asm9260_function asm9260_functions[] = {
	FUNCTION(CAM_DAT0),
	FUNCTION(CAM_DAT1),
	FUNCTION(CAM_DAT2),
	FUNCTION(CAM_DAT3),
	FUNCTION(CAM_DAT4),
	FUNCTION(CAM_DAT5),
	FUNCTION(CAM_DAT6),
	FUNCTION(CAM_DAT7),
	FUNCTION(CAM_DAT8),
	FUNCTION(CAM_DAT9),
	FUNCTION(CAM_HREF),
	FUNCTION(CAM_MCLK),
	FUNCTION(CAM_PCLK),
	FUNCTION(CAM_VSYN),
	FUNCTION(CAN0_RX),
	FUNCTION(CAN0_TX),
	FUNCTION(CAN1_RX),
	FUNCTION(CAN1_TX),
	FUNCTION(CT0_CAP),
	FUNCTION(CT0_MAT0),
	FUNCTION(CT0_MAT1),
	FUNCTION(CT0_MAT2),
	FUNCTION(CT0_MAT3),
	FUNCTION(CT1_CAP),
	FUNCTION(CT1_MAT0),
	FUNCTION(CT1_MAT1),
	FUNCTION(CT1_MAT2),
	FUNCTION(CT1_MAT3),
	FUNCTION(CT2_CAP),
	FUNCTION(CT2_MAT0),
	FUNCTION(CT2_MAT1),
	FUNCTION(CT2_MAT2),
	FUNCTION(CT2_MAT3),
	FUNCTION(CT3_CAP),
	FUNCTION(CT3_MAT0),
	FUNCTION(CT3_MAT1),
	FUNCTION(CT3_MAT2),
	FUNCTION(CT3_MAT3),
	FUNCTION(I2C0_SCL),
	FUNCTION(I2C0_SDA),
	FUNCTION(I2C1_SCL),
	FUNCTION(I2C1_SDA),
	FUNCTION(I2S0_BCLK),
	FUNCTION(I2S0_LRC),
	FUNCTION(I2S0_MCLK),
	FUNCTION(I2S0_RX0),
	FUNCTION(I2S0_TX0),
	FUNCTION(I2S0_TX1),
	FUNCTION(I2S0_TX2),
	FUNCTION(I2S1_BCLK),
	FUNCTION(I2S1_LRC),
	FUNCTION(I2S1_MCLK),
	FUNCTION(I2S1_RX0),
	FUNCTION(I2S1_TX0),
	FUNCTION(I2S1_TX1),
	FUNCTION(I2S1_TX2),
	FUNCTION(JTAG),
	FUNCTION(LCD_AC_OQ),
	FUNCTION(LCD_CP_OQ),
	FUNCTION(LCD_FP_OQ),
	FUNCTION(LCD_IF_BUSY),
	FUNCTION(LCD_IF_CS),
	FUNCTION(LCD_IF_DAT0),
	FUNCTION(LCD_IF_DAT1),
	FUNCTION(LCD_IF_DAT10),
	FUNCTION(LCD_IF_DAT11),
	FUNCTION(LCD_IF_DAT12),
	FUNCTION(LCD_IF_DAT13),
	FUNCTION(LCD_IF_DAT14),
	FUNCTION(LCD_IF_DAT15),
	FUNCTION(LCD_IF_DAT2),
	FUNCTION(LCD_IF_DAT3),
	FUNCTION(LCD_IF_DAT4),
	FUNCTION(LCD_IF_DAT5),
	FUNCTION(LCD_IF_DAT6),
	FUNCTION(LCD_IF_DAT7),
	FUNCTION(LCD_IF_DAT8),
	FUNCTION(LCD_IF_DAT9),
	FUNCTION(LCD_IF_RS),
	FUNCTION(LCD_IF_WR),
	FUNCTION(LCD_LP_OQ),
	FUNCTION(LCD_PIXEL_OQ0),
	FUNCTION(LCD_PIXEL_OQ1),
	FUNCTION(LCD_PIXEL_OQ10),
	FUNCTION(LCD_PIXEL_OQ11),
	FUNCTION(LCD_PIXEL_OQ12),
	FUNCTION(LCD_PIXEL_OQ13),
	FUNCTION(LCD_PIXEL_OQ14),
	FUNCTION(LCD_PIXEL_OQ15),
	FUNCTION(LCD_PIXEL_OQ16),
	FUNCTION(LCD_PIXEL_OQ17),
	FUNCTION(LCD_PIXEL_OQ18),
	FUNCTION(LCD_PIXEL_OQ19),
	FUNCTION(LCD_PIXEL_OQ2),
	FUNCTION(LCD_PIXEL_OQ20),
	FUNCTION(LCD_PIXEL_OQ21),
	FUNCTION(LCD_PIXEL_OQ22),
	FUNCTION(LCD_PIXEL_OQ23),
	FUNCTION(LCD_PIXEL_OQ3),
	FUNCTION(LCD_PIXEL_OQ4),
	FUNCTION(LCD_PIXEL_OQ5),
	FUNCTION(LCD_PIXEL_OQ6),
	FUNCTION(LCD_PIXEL_OQ7),
	FUNCTION(LCD_PIXEL_OQ8),
	FUNCTION(LCD_PIXEL_OQ9),
	FUNCTION(MCI0),
	FUNCTION(MCI1),
	FUNCTION(MCI2),
	FUNCTION(MCIABORT),
	FUNCTION(MCOA0),
	FUNCTION(MCOA1),
	FUNCTION(MCOA2),
	FUNCTION(MCOB0),
	FUNCTION(MCOB1),
	FUNCTION(MCOB2),
	FUNCTION(MII_COL),
	FUNCTION(MII_CRS),
	FUNCTION(MII_PPS_OUT),
	FUNCTION(MII_RXD2),
	FUNCTION(MII_RXD3),
	FUNCTION(MII_RX_ER),
	FUNCTION(MII_TX_CLK),
	FUNCTION(MII_TXD2),
	FUNCTION(MII_TXD3),
	FUNCTION(NAND_ALE),
	FUNCTION(NAND_CE0N),
	FUNCTION(NAND_CE1N),
	FUNCTION(NAND_CLE),
	FUNCTION(NAND_D0),
	FUNCTION(NAND_D1),
	FUNCTION(NAND_D2),
	FUNCTION(NAND_D3),
	FUNCTION(NAND_D4),
	FUNCTION(NAND_D5),
	FUNCTION(NAND_D6),
	FUNCTION(NAND_D7),
	FUNCTION(NAND_RDY0),
	FUNCTION(NAND_RDY1),
	FUNCTION(NAND_REN),
	FUNCTION(NAND_WEN),
	FUNCTION(OUTCLK),
	FUNCTION(QEI_A),
	FUNCTION(QEI_B),
	FUNCTION(QEI_INDEX),
	FUNCTION(QSPI0_DAT0),
	FUNCTION(QSPI0_DAT1),
	FUNCTION(QSPI0_DAT2),
	FUNCTION(QSPI0_DAT3),
	FUNCTION(QSPI0_SCK),
	FUNCTION(QSPI0_SEL),
	FUNCTION(RMII_CRS_DV),
	FUNCTION(RMII_MDC),
	FUNCTION(RMII_MDIO),
	FUNCTION(RMII_REFCLK),
	FUNCTION(RMII_RXD0),
	FUNCTION(RMII_RXD1),
	FUNCTION(RMII_TXD0),
	FUNCTION(RMII_TXD1),
	FUNCTION(RMII_TX_EN),
	FUNCTION(SD0_CLK),
	FUNCTION(SD0_CMD),
	FUNCTION(SD0_DAT0),
	FUNCTION(SD0_DAT1),
	FUNCTION(SD0_DAT2),
	FUNCTION(SD0_DAT3),
	FUNCTION(SPI0_MISO),
	FUNCTION(SPI0_MOSI),
	FUNCTION(SPI0_SCK),
	FUNCTION(SPI0_SEL),
	FUNCTION(SPI1_MISO),
	FUNCTION(SPI1_MOSI),
	FUNCTION(SPI1_SCK),
	FUNCTION(SPI1_SEL),
	FUNCTION(UART0_CTS),
	FUNCTION(UART0_DCD),
	FUNCTION(UART0_DSR),
	FUNCTION(UART0_DTR),
	FUNCTION(UART0_RI),
	FUNCTION(UART0_RTS),
	FUNCTION(UART0_RXD),
	FUNCTION(UART0_TXD),
	FUNCTION(UART1_CLK),
	FUNCTION(UART1_CTS),
	FUNCTION(UART1_RTS),
	FUNCTION(UART1_RXD),
	FUNCTION(UART1_TXD),
	FUNCTION(UART2_CTS),
	FUNCTION(UART2_RTS),
	FUNCTION(UART2_RXD),
	FUNCTION(UART2_TXD),
	FUNCTION(UART3_CLK),
	FUNCTION(UART3_CTS),
	FUNCTION(UART3_RTS),
	FUNCTION(UART3_RXD),
	FUNCTION(UART3_TXD),
	FUNCTION(UART4_CLK),
	FUNCTION(UART4_CTS),
	FUNCTION(UART4_RTS),
	FUNCTION(UART4_RXD),
	FUNCTION(UART4_TXD),
	FUNCTION(UART5_CLK),
	FUNCTION(UART5_CTS),
	FUNCTION(UART5_RTS),
	FUNCTION(UART5_RXD),
	FUNCTION(UART5_TXD),
	FUNCTION(UART6_CLK),
	FUNCTION(UART6_CTS),
	FUNCTION(UART6_RTS),
	FUNCTION(UART6_RXD),
	FUNCTION(UART6_TXD),
	FUNCTION(UART7_CTS),
	FUNCTION(UART7_RTS),
	FUNCTION(UART7_RXD),
	FUNCTION(UART7_TXD),
	FUNCTION(UART8_RXD),
	FUNCTION(UART8_TXD),
	FUNCTION(UART9_RXD),
	FUNCTION(UART9_TXD),
};

#if 0
/* Sub muxes */

/**
 * MUX() - Initialise a mux description.
 * @f0:		Function 0 (ASM9260_MUX_ is prepended, NA for none)
 * @f1:		Function 1 (ASM9260_MUX_ is prepended, NA for none)
 * @f2:		Function 2 (ASM9260_MUX_ is prepended, NA for none)
 * @f3:		Function 3 (ASM9260_MUX_ is prepended, NA for none)
 * @f4:		Function 4 (ASM9260_MUX_ is prepended, NA for none)
 */
#define MUX(f0, f1, f2, f3, f4, mux_r, mux_b, mux_w)		\
	{							\
		.funcs = {					\
			ASM9260_MUX_ ## f0,			\
			ASM9260_MUX_ ## f1,			\
			ASM9260_MUX_ ## f2,			\
			ASM9260_MUX_ ## f3,			\
			ASM9260_MUX_ ## f4,			\
			ASM9260_MUX_ ## f5,			\
			ASM9260_MUX_ ## f6,			\
		},						\

/**
 * MUX_PG() - Initialise a pin group with mux control
 * @pg_name:	Pin group name (stringified, _pins appended to get pins array)
 * @f0:		Function 0 (ASM9260_MUX_ is prepended, NA for none)
 * @f1:		Function 1 (ASM9260_MUX_ is prepended, NA for none)
 * @f2:		Function 2 (ASM9260_MUX_ is prepended, NA for none)
 * @f3:		Function 3 (ASM9260_MUX_ is prepended, NA for none)
 * @f4:		Function 4 (ASM9260_MUX_ is prepended, NA for none)
 */
#define MUX_PG(pg_name, f0, f1, f2, f3, f4, f5, f6)				\
	{							\
		.name = #pg_name,				\
		.pins = pg_name##_pins,				\
		.npins = ARRAY_SIZE(pg_name##_pins),		\
		.mux = MUX(f0, f1, f2, f3, f4, f5, f6),		\
	}

/*
 * Define main muxing pin groups
 */


/*
 * These are the pin mux groups. Pin muxing can be enabled and disabled for each
 * pin individually so these groups are internal. The mapping of pins to pin mux
 * group is below (asm9260_mux_pins).
 */
static struct asm9260_pingroup asm9260_mux_groups[] = {
	/* Muxing pin groups */
	/*     pg,		f0,	f1,	f2,	f3,	f4,	f5,	f6, */
	MUX_PG(GPIO0_0,		UART1_CLK,	I2S0_MCLK,	SPI1_SCK,	JTAG,	NA,	NA,	NA),
	MUX_PG(GPIO0_1,		UART1_TXD,	I2S0_BCLK,	SPI1_SEL,	JTAG,	NA,	NA,	NA),
	MUX_PG(GPIO0_2,		UART1_RXD,	I2S0_LRC,	SPI1_MISO,	JTAG,	NA,	NA,	NA),
	MUX_PG(GPIO0_3,		UART1_RTS,	I2S0_RX0,	SPI1_MOSI,	JTAG,	NA,	NA,	NA),
	MUX_PG(GPIO0_4,		UART1_CTS,	I2S0_TX0,	SPI0_SCK,	JTAG,	I2C0_SCL,	NA,	NA),
	MUX_PG(GPIO1_4,		CT0_CAP,	UART0_DTR,	LCD_IF_BUSY,	SPI0_SCK,	MII_PPS_OUT,	LCD_CP_OQ,	NA),
	MUX_PG(GPIO1_5,		UART0_DSR,	LCD_IF_WR,	SPI0_SEL,	RMII_REFCLK,	LCD_FP_OQ,	NA,	NA),
	MUX_PG(GPIO1_6,		UART0_DCD,	LCD_IF_RS,	SPI0_MISO,	RMII_REFCLK,	LCD_AC_OQ,	I2C1_SCL,	NA),
	MUX_PG(GPIO1_7,		UART0_RI,	LCD_IF_CS,	SPI0_MOSI,	RMII_REFCLK,	LCD_LP_OQ,	I2C1_SDA,	NA),
	MUX_PG(GPIO2_0,		CT1_MAT0,	UART2_RTS,	LCD_IF_DAT0,	SPI1_SCK,	MII_RXD2,	LCD_PIXEL_OQ0,	CAN0_TX),
	MUX_PG(GPIO2_1,		CT1_MAT1,	UART2_CTS,	LCD_IF_DAT1,	SPI1_SEL,	MII_RXD3,	LCD_PIXEL_OQ1,	CAN0_RX),
	MUX_PG(GPIO2_2,		CT1_MAT2,	UART3_CLK,	LCD_IF_DAT2,	SPI1_MISO,	MII_TXD2,	LCD_PIXEL_OQ2,	NA),
	MUX_PG(GPIO2_3,		CT1_MAT3,	UART3_TXD,	LCD_IF_DAT3,	SPI1_MOSI,	MII_TXD3,	LCD_PIXEL_OQ3,	NA),
	MUX_PG(GPIO2_4,		CT1_CAP,	UART3_RXD,	LCD_IF_DAT4,	MII_TX_CLK,	LCD_PIXEL_OQ4,	NA,	NA),
	MUX_PG(GPIO2_5,		UART3_RTS,	LCD_IF_DAT5,	MII_CRS,	LCD_PIXEL_OQ5,	OUTCLK,	NA,	NA),
	MUX_PG(GPIO2_6,		UART3_CTS,	LCD_IF_DAT6,	MII_COL,	LCD_PIXEL_OQ6,	CAN1_TX,	NA,	NA),
	MUX_PG(GPIO2_7,		UART4_CLK,	LCD_IF_DAT7,	MII_RX_ER,	LCD_PIXEL_OQ7,	CAN1_RX,	NA,	NA),
	MUX_PG(GPIO3_0,		CT2_MAT0,	UART4_TXD,	LCD_IF_DAT8,	SD0_CLK,	RMII_MDC,	LCD_PIXEL_OQ8,	NA),
	MUX_PG(GPIO3_1,		CT2_MAT1,	UART4_RXD,	LCD_IF_DAT9,	SD0_CMD,	RMII_MDIO,	LCD_PIXEL_OQ9,	NA),
	MUX_PG(GPIO3_2,		CT2_MAT2,	UART4_RTS,	LCD_IF_DAT10,	SD0_DAT0,	RMII_CRS_DV,	LCD_PIXEL_OQ10,	CAN1_TX),
	MUX_PG(GPIO3_3,		CT2_MAT3,	UART4_CTS,	LCD_IF_DAT11,	SD0_DAT1,	RMII_RXD0,	LCD_PIXEL_OQ11,	CAN1_RX),
	MUX_PG(GPIO3_4,		CT2_CAP,	UART5_CLK,	LCD_IF_DAT12,	SD0_DAT2,	RMII_RXD1,	LCD_PIXEL_OQ12,	OUTCLK),
	MUX_PG(GPIO3_5,		UART5_TXD,	LCD_IF_DAT13,	SD0_DAT3,	RMII_TX_EN,	LCD_PIXEL_OQ13,	I2C0_SCL,	NA),
	MUX_PG(GPIO3_6,		UART5_RXD,	LCD_IF_DAT14,	RMII_TXD0,	LCD_PIXEL_OQ14,	I2C0_SDA,	NA,	NA),
	MUX_PG(GPIO3_7,		UART5_RTS,	LCD_IF_DAT15,	RMII_TXD1,	LCD_PIXEL_OQ15,	NA,	NA,	NA),
	MUX_PG(GPIO4_0,		CT3_MAT0,	UART5_CTS,	QSPI0_SCK,	MII_RXD2,	LCD_PIXEL_OQ16,	NA,	NA),
	MUX_PG(GPIO4_1,		CT3_MAT1,	UART6_CLK,	QSPI0_SEL,	MII_RXD3,	LCD_PIXEL_OQ17,	NA,	NA),
	MUX_PG(GPIO4_2,		CT3_MAT2,	UART6_TXD,	QSPI0_DAT0,	MII_TXD2,	LCD_PIXEL_OQ18,	NA,	NA),
	MUX_PG(GPIO4_3,		CT3_MAT3,	UART6_RXD,	QSPI0_DAT1,	MII_TXD3,	LCD_PIXEL_OQ19,	NA,	NA),
	MUX_PG(GPIO4_4,		CT3_CAP,	UART6_RTS,	QSPI0_DAT2,	MII_TX_CLK,	LCD_PIXEL_OQ20,	NA,	NA),
	MUX_PG(GPIO4_5,		UART6_CTS,	QSPI0_DAT3,	MII_CRS,	LCD_PIXEL_OQ21,	NA,	NA,	NA),
	MUX_PG(GPIO4_6,		MII_COL,	LCD_PIXEL_OQ22,	I2C1_SCL,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO4_7,		MII_RX_ER,	LCD_PIXEL_OQ23,	I2C1_SDA,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO5_0,		MCIABORT,	UART7_TXD,	I2S1_MCLK,	SD0_CLK,	RMII_MDC,	NA,	NA),
	MUX_PG(GPIO5_1,		MCI0,	UART7_RXD,	I2S1_BCLK,	SD0_CMD,	RMII_MDIO,	NA,	NA),
	MUX_PG(GPIO5_2,		MCOA0,	UART7_RTS,	I2S1_LRC,	SD0_DAT0,	RMII_CRS_DV,	CAN0_TX,	NA),
	MUX_PG(GPIO5_3,		MCOB0,	UART7_CTS,	I2S1_RX0,	SD0_DAT1,	RMII_RXD0,	CAN0_RX,	NA),
	MUX_PG(GPIO5_4,		MCI1,	UART8_TXD,	I2S1_TX0,	SD0_DAT2,	RMII_RXD1,	NA,	NA),
	MUX_PG(GPIO8_1,		UART2_TXD,	CAM_PCLK,	MII_RXD3,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_2,		UART2_RXD,	CAM_VSYN,	MII_TXD2,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_3,		UART2_RTS,	CAM_HREF,	MII_TXD3,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_4,		UART2_CTS,	CAM_DAT0,	MII_TX_CLK,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_5,		UART3_CLK,	CAM_DAT1,	MII_CRS,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_6,		UART3_TXD,	CAM_DAT2,	MII_COL,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_7,		UART3_RXD,	CAM_DAT3,	MII_RX_ER,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO9_0,		CT0_MAT0,	UART3_RTS,	CAM_DAT4,	RMII_MDC,	I2C0_SCL,	NA,	NA),
	MUX_PG(GPIO9_1,		CT0_MAT1,	UART3_CTS,	CAM_DAT5,	RMII_MDIO,	I2C0_SDA,	NA,	NA),
	MUX_PG(GPIO9_2,		CT0_MAT2,	UART4_CLK,	CAM_DAT6,	RMII_CRS_DV,	NA,	NA,	NA),
	MUX_PG(GPIO9_3,		CT0_MAT3,	UART4_TXD,	CAM_DAT7,	RMII_RXD0,	NA,	NA,	NA),
	MUX_PG(GPIO9_4,		CT0_CAP,	UART4_RXD,	CAM_DAT8,	RMII_RXD1,	NA,	NA,	NA),
	MUX_PG(GPIO9_5,		UART4_RTS,	CAM_DAT9,	RMII_TX_EN,	I2C1_SCL,	NA,	NA,	NA),
	MUX_PG(GPIO10_0,	CT1_MAT0,	UART5_TXD,	I2S0_MCLK,	SPI0_SCK,	NA,	NA,	NA),
	MUX_PG(GPIO10_1,	CT1_MAT1,	UART5_RXD,	I2S0_BCLK,	SPI0_SEL,	RMII_REFCLK,	NA,	NA),
	MUX_PG(GPIO10_2,	CT1_MAT2,	UART5_RTS,	I2S0_LRC,	SPI0_MISO,	RMII_REFCLK,	NA,	NA),
	MUX_PG(GPIO10_3,	CT1_MAT3,	UART5_CTS,	I2S0_RX0,	SPI0_MOSI,	CAN0_RX,	NA,	NA),
	MUX_PG(GPIO10_4,	CT1_CAP,	UART6_TXD,	I2S0_TX0,	SPI1_SCK,	RMII_REFCLK,	NA,	NA),
	MUX_PG(GPIO10_5,	UART6_RXD,	I2S0_TX1,	SPI1_SEL,	RMII_REFCLK,	CAN1_RX,	NA,	NA),
	MUX_PG(GPIO10_6,	UART6_RTS,	I2S0_TX2,	SPI1_MISO,	RMII_REFCLK,	CAN1_TX,	NA,	NA),
	MUX_PG(GPIO10_7,	UART6_CTS,	CAM_MCLK,	SPI1_MOSI,	RMII_REFCLK,	NA,	NA,	NA),
	MUX_PG(GPIO11_0,	CT2_MAT0,	UART7_TXD,	I2S1_MCLK,	QSPI0_SCK,	NAND_REN,	NA,	NA),
	MUX_PG(GPIO11_1,	CT2_MAT1,	UART7_RXD,	I2S1_BCLK,	QSPI0_SEL,	NAND_WEN,	NA,	NA),
	MUX_PG(GPIO11_2,	CT2_MAT2,	UART7_RTS,	I2S1_LRC,	QSPI0_DAT0,	NAND_ALE,	NA,	NA),
	MUX_PG(GPIO11_3,	CT2_MAT3,	UART7_CTS,	I2S1_RX0,	QSPI0_DAT1,	NAND_CLE,	NA,	NA),
	MUX_PG(GPIO11_4,	CT2_CAP,	UART8_TXD,	I2S1_TX0,	QSPI0_DAT2,	NAND_RDY0,	NA,	NA),
	MUX_PG(GPIO11_5,	UART8_RXD,	I2S1_TX1,	QSPI0_DAT3,	NAND_RDY1,	NA,	NA,	NA),
	MUX_PG(GPIO11_6,	UART9_TXD,	I2S1_TX2,	NAND_CE0N,	I2C0_SCL,	NA,	NA,	NA),
	MUX_PG(GPIO11_7,	UART9_RXD,	CAM_MCLK,	NAND_CE1N,	I2C0_SDA,	NA,	NA,	NA),
	MUX_PG(GPIO12_0,	CT3_MAT0,	UART1_CLK,	SD0_CLK,	NAND_D0,	NA,	NA,	NA),
	MUX_PG(GPIO12_1,	CT3_MAT1,	UART1_TXD,	SD0_CMD,	NAND_D1,	NA,	NA,	NA),
	MUX_PG(GPIO12_2,	CT3_MAT2,	UART1_RXD,	SD0_DAT0,	NAND_D2,	NA,	NA,	NA),
	MUX_PG(GPIO12_3,	CT3_MAT3,	UART1_RTS,	SD0_DAT1,	NAND_D3,	NA,	NA,	NA),
	MUX_PG(GPIO12_4,	CT3_CAP,	UART1_CTS,	SD0_DAT2,	NAND_D4,	NA,	NA,	NA),
	MUX_PG(GPIO12_5,	UART8_TXD,	SD0_DAT3,	NAND_D5,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO12_6,	UART8_RXD,	CAM_MCLK,	NAND_D6,	I2C1_SCL,	NA,	NA,	NA),
	MUX_PG(GPIO12_7,	CAM_MCLK,	NAND_D7,	I2C1_SDA,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO13_4,	MCI1,	UART2_CTS,	SPI1_SCK,	NAND_RDY0,	NA,	NA,	NA),
	MUX_PG(GPIO13_5,	MCOA1,	UART9_TXD,	SPI1_SEL,	NAND_RDY1,	NA,	NA,	NA),
	MUX_PG(GPIO13_6,	MCOB1,	UART9_RXD,	SPI1_MISO,	NAND_CE0N,	NA,	NA,	NA),
	MUX_PG(GPIO13_7,	MCI2,	SPI1_MOSI,	NAND_CE1N,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO14_0,	MCOA2,	UART0_TXD,	I2S0_MCLK,	SD0_CLK,	NAND_D0,	NA,	NA),
	MUX_PG(GPIO14_1,	MCOB2,	UART0_RXD,	I2S0_BCLK,	SD0_CMD,	NAND_D1,	NA,	NA),
	MUX_PG(GPIO14_2,	UART0_RTS,	I2S0_LRC,	SD0_DAT0,	NAND_D2,	NA,	NA,	NA),
	MUX_PG(GPIO14_3,	UART0_CTS,	I2S0_RX0,	SD0_DAT1,	NAND_D3,	NA,	NA,	NA),
	MUX_PG(GPIO14_4,	UART0_DTR,	I2S0_TX0,	SD0_DAT2,	NAND_D4,	NA,	NA,	NA),
	MUX_PG(GPIO14_5,	UART0_DSR,	I2S0_TX1,	SD0_DAT3,	NAND_D5,	NA,	NA,	NA),
	MUX_PG(GPIO15_0,	UART4_TXD,	I2S0_MCLK,	SD0_CLK,	RMII_MDC,	NA,	NA,	NA),
	MUX_PG(GPIO15_1,	UART4_RXD,	I2S0_BCLK,	SD0_CMD,	RMII_MDIO,	NA,	NA,	NA),
	MUX_PG(GPIO15_2,	UART5_TXD,	I2S0_LRC,	SD0_DAT0,	RMII_CRS_DV,	NA,	NA,	NA),
	MUX_PG(GPIO15_3,	UART5_RXD,	I2S0_RX0,	SD0_DAT1,	RMII_RXD0,	NA,	NA,	NA),
	MUX_PG(GPIO15_4,	UART6_TXD,	I2S0_TX0,	SD0_DAT2,	RMII_RXD1,	NA,	NA,	NA),
	MUX_PG(GPIO15_5,	UART6_RXD,	I2S0_TX1,	SD0_DAT3,	RMII_TX_EN,	NA,	NA,	NA),
	MUX_PG(GPIO15_6,	UART7_TXD,	I2S0_TX2,	RMII_TXD0,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO15_7,	UART7_RXD,	RMII_TXD1,	NA,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_0,	CT2_MAT0,	UART4_TXD,	MII_RXD2,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_1,	CT2_MAT1,	UART4_RXD,	MII_RXD3,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_2,	CT2_MAT2,	UART5_TXD,	MII_TXD2,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_3,	CT2_MAT3,	UART5_RXD,	MII_TXD3,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_4,	CT2_CAP,	UART6_TXD,	MII_TX_CLK,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_5,	QEI_A,	UART6_RXD,	MII_CRS,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_6,	QEI_B,	UART6_RTS,	MII_COL,	CAN1_TX,	NA,	NA,	NA),
	MUX_PG(GPIO16_7,	QEI_INDEX,	UART6_CTS,	MII_RX_ER,	CAN1_RX,	NA,	NA,	NA),
	MUX_PG(GPIO17_0,	CT3_MAT0,	UART7_TXD,	I2S1_MCLK,	RMII_REFCLK,	NA,	NA,	NA),
	MUX_PG(GPIO17_1,	CT3_MAT1,	UART7_RXD,	I2S1_BCLK,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO17_2,	CT3_MAT2,	UART7_RTS,	I2S1_LRC,	I2C1_SCL,	NA,	NA,	NA),
	MUX_PG(GPIO17_3,	CT3_MAT3,	UART7_CTS,	I2S1_RX0,	I2C1_SDA,	NA,	NA,	NA),
	MUX_PG(GPIO17_4,	CT3_CAP,	UART8_TXD,	I2S1_TX0,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO17_5,	QEI_A,	UART8_RXD,	I2S1_TX1,	RMII_REFCLK,	NA,	NA,	NA),
	MUX_PG(GPIO17_6,	QEI_B,	UART9_TXD,	I2S1_TX2,	MII_PPS_OUT,	NA,	NA,	NA),
	MUX_PG(GPIO17_7,	QEI_INDEX,	UART9_RXD,	MII_PPS_OUT,	NA,	NA,	NA,	NA),
};

/*
 * This is the mapping from GPIO pins to pin mux groups in asm9260_mux_groups[].
 * Pins which aren't muxable to multiple peripherals are set to
 * ASM9260_MUX_GROUP_MAX to enable the "perip" function to enable/disable
 * peripheral control of the pin.
 *
 * This array is initialised in asm9260_init_mux_pins().
 */
static u8 asm9260_mux_pins[ARRAY_SIZE(asm9260_pins)];

/* ASM9260_MUX_GROUP_MAX is used in asm9260_mux_pins[] for non-muxing pins */
#define ASM9260_MUX_GROUP_MAX ARRAY_SIZE(asm9260_mux_groups)

/**
 * asm9260_init_mux_pins() - Initialise GPIO pin to mux group mapping.
 *
 * Initialises the asm9260_mux_pins[] array to be the inverse of the pin lists in
 * each pin mux group in asm9260_mux_groups[].
 *
 * It is assumed that no pin mux groups overlap (share pins).
 */
static void __init asm9260_init_mux_pins(void)
{
	unsigned int g, p;
	const struct asm9260_pingroup *grp;
	const unsigned int *pin;

	for (p = 0; p < ARRAY_SIZE(asm9260_pins); ++p)
		asm9260_mux_pins[p] = ASM9260_MUX_GROUP_MAX;

	grp = asm9260_mux_groups;
	for (g = 0, grp = asm9260_mux_groups;
	     g < ARRAY_SIZE(asm9260_mux_groups); ++g, ++grp)
		for (pin = grp->pins, p = 0; p < grp->npins; ++p, ++pin)
			asm9260_mux_pins[*pin] = g;
}

#endif

/**
 * SIMPLE_PG() - Initialise a simple convenience pin group
 * @pg_name:	Pin group name (stringified, _pins appended to get pins array)
 *
 * A simple pin group is simply used for binding pins together so they can be
 * referred to by a single name instead of having to list every pin
 * individually.
 */
#define SIMPLE_PG(pg_name)					\
	{							\
		.name = #pg_name,				\
		.pins = pg_name##_pins,				\
		.npins = ARRAY_SIZE(pg_name##_pins),		\
	}

static struct asm9260_pingroup asm9260_groups[] = {
	/*        pg_name */
	SIMPLE_PG(uart1),
	SIMPLE_PG(uart4_0),
	SIMPLE_PG(uart4_1),
};

/**
 * struct asm9260_pmx - Private pinctrl data
 * @dev:	Platform device
 * @pctl:	Pin control device
 * @regs:	Register region
 * @lock:	Lock protecting coherency of pin_en, gpio_en, and SELECT regs
 * @pin_en:	Pins that have been enabled (32 pins packed into each element)
 * @gpio_en:	GPIOs that have been enabled (32 pins packed into each element)
 */
struct asm9260_pmx {
	struct device		*dev;
	struct pinctrl_dev	*pctl;
	void __iomem		*regs;
	spinlock_t		lock;
	u32			pin_en[3];
	u32			gpio_en[3];
};

static inline u32 pmx_read(struct asm9260_pmx *pmx, u32 reg)
{
	return ioread32(pmx->regs + reg);
}

static inline void pmx_write(struct asm9260_pmx *pmx, u32 val, u32 reg)
{
	iowrite32(val, pmx->regs + reg);
}

/*
 * Pin control operations
 */

/* each GPIO pin has it's own pseudo pingroup containing only itself */

static int asm9260_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(asm9260_pins);
}

static const char *asm9260_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						 unsigned int group)
{
	if (group < ARRAY_SIZE(asm9260_groups)) {
		/* normal pingroup */
		return asm9260_groups[group].name;
	} else {
		/* individual gpio pin pseudo-pingroup */
		unsigned int pin = group - ARRAY_SIZE(asm9260_groups);
		return asm9260_pins[pin].name;
	}
}

static int asm9260_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					 unsigned int group,
					 const unsigned int **pins,
					 unsigned int *num_pins)
{
	if (group < ARRAY_SIZE(asm9260_groups)) {
		/* normal pingroup */
		*pins = asm9260_groups[group].pins;
		*num_pins = asm9260_groups[group].npins;
	} else {
		/* individual gpio pin pseudo-pingroup */
		unsigned int pin = group - ARRAY_SIZE(asm9260_groups);
		*pins = &asm9260_pins[pin].number;
		*num_pins = 1;
	}

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

	ret = of_property_read_string(np, "asm9260,function", &function);
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
	ret = of_property_count_strings(np, "asm9260,pins");
	if (ret < 0) {
		dev_err(dev, "could not parse property pins\n");
		goto exit;
	}
	reserve *= ret;

	ret = reserve_map(dev, map, reserved_maps, num_maps, reserve);
	if (ret < 0)
		goto exit;

	of_property_for_each_string(np, "asm9260,pins", prop, group) {
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
	struct device_node *np;
	int ret;

	reserved_maps = 0;
	*map = NULL;
	*num_maps = 0;

	for_each_child_of_node(np_config, np) {
		ret = asm9260_pinctrl_dt_subnode_to_map(pctldev->dev, np, map,
						       &reserved_maps,
						       num_maps);
		if (ret < 0) {
			asm9260_pinctrl_dt_free_map(pctldev, *map, *num_maps);
			return ret;
		}
	}

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
	/* pingroup functions */
	*groups = asm9260_functions[function].groups;
	*num_groups = asm9260_functions[function].ngroups;
	return 0;
}

#if 0
/**
 * asm9260_pinctrl_select() - update bit in SELECT register
 * @pmx:		Pinmux data
 * @pin:		Pin number (must be within GPIO range)
 */
static void asm9260_pinctrl_select(struct asm9260_pmx *pmx,
				  unsigned int pin)
{
	u32 reg, reg_shift, select, val;
	unsigned int pmx_index, pmx_shift;
	unsigned long flags;

	/* uses base 32 instead of base 30 */
	pmx_index = pin >> 5;
	pmx_shift = pin & 0x1f;

	/* select = !perip || gpio */
	select = ((~pmx->pin_en[pmx_index] |
		   pmx->gpio_en[pmx_index]) >> pmx_shift) & 1;

	/* find register and bit offset (base 30) */
	reg = REG_PINCTRL_SELECT + 4*(pin / 30);
	reg_shift = pin % 30;

	/* modify gpio select bit */
	__global_lock2(flags);
	val = pmx_read(pmx, reg);
	val &= ~BIT(reg_shift);
	val |= select << reg_shift;
	pmx_write(pmx, val, reg);
	__global_unlock2(flags);
}

/**
 * asm9260_pinctrl_gpio_select() - enable/disable GPIO usage for a pin
 * @pmx:		Pinmux data
 * @pin:		Pin number
 * @gpio_select:	true to enable pin as GPIO,
 *			false to leave control to whatever function is enabled
 *
 * Records that GPIO usage is enabled/disabled so that enabling a function
 * doesn't override the SELECT register bit.
 */
static void asm9260_pinctrl_gpio_select(struct asm9260_pmx *pmx,
				       unsigned int pin,
				       bool gpio_select)
{
	unsigned int index, shift;
	u32 gpio_en;

	if (pin >= ARRAY_SIZE(asm9260_pins))
		return;

	/* uses base 32 instead of base 30 */
	index = pin >> 5;
	shift = pin & 0x1f;

	spin_lock(&pmx->lock);

	/* keep a record whether gpio is selected */
	gpio_en = pmx->gpio_en[index];
	gpio_en &= ~BIT(shift);
	if (gpio_select)
		gpio_en |= BIT(shift);
	pmx->gpio_en[index] = gpio_en;

	/* update the select bit */
	asm9260_pinctrl_select(pmx, pin);

	spin_unlock(&pmx->lock);
}

/**
 * asm9260_pinctrl_perip_select() - enable/disable peripheral interface for a pin
 * @pmx:		Pinmux data
 * @pin:		Pin number
 * @perip_select:	true to enable peripheral interface when not GPIO,
 *			false to leave pin in GPIO mode
 *
 * Records that peripheral usage is enabled/disabled so that SELECT register can
 * be set appropriately when GPIO is disabled.
 */
static void asm9260_pinctrl_perip_select(struct asm9260_pmx *pmx,
					unsigned int pin,
					bool perip_select)
{
	unsigned int index, shift;
	u32 pin_en;

	if (pin >= ARRAY_SIZE(asm9260_pins))
		return;

	/* uses base 32 instead of base 30 */
	index = pin >> 5;
	shift = pin & 0x1f;

	spin_lock(&pmx->lock);

	/* keep a record whether peripheral is selected */
	pin_en = pmx->pin_en[index];
	pin_en &= ~BIT(shift);
	if (perip_select)
		pin_en |= BIT(shift);
	pmx->pin_en[index] = pin_en;

	/* update the select bit */
	asm9260_pinctrl_select(pmx, pin);

	spin_unlock(&pmx->lock);
}

/**
 * asm9260_pinctrl_enable_mux() - Switch a pin mux group to a function.
 * @pmx:		Pinmux data
 * @desc:		Pinmux description
 * @function:		Function to switch to
 *
 * Enable a particular function on a pin mux group. Since pin mux descriptions
 * are nested this function is recursive.
 */
static int asm9260_pinctrl_enable_mux(struct asm9260_pmx *pmx,
				     const struct asm9260_muxdesc *desc,
				     unsigned int function)
{
	const int *fit;
	unsigned long flags;
	int mux;
	unsigned int func, ret;
	u32 reg, mask;

	/* find the mux value for this function, searching recursively */
	for (mux = 0, fit = desc->funcs;
	     mux < ARRAY_SIZE(desc->funcs); ++mux, ++fit) {
		func = *fit;
		if (func == function)
			goto found_mux;
	}

	return -EINVAL;
found_mux:

	/* Set up the mux */
	if (desc->width) {
		mask = (BIT(desc->width) - 1) << desc->bit;
		__global_lock2(flags);
		reg = pmx_read(pmx, desc->reg);
		reg &= ~mask;
		reg |= (mux << desc->bit) & mask;
		pmx_write(pmx, reg, desc->reg);
		__global_unlock2(flags);
	}

	return 0;
}

#endif
/**
 * asm9260_pinctrl_enable() - Enable a function on a pin group.
 * @pctldev:		Pin control data
 * @function:		Function index to enable
 * @group:		Group index to enable
 *
 * Enable a particular function on a group of pins. The per GPIO pin pseudo pin
 * groups can be used (in which case the pin will be enabled in peripheral mode
 * and if it belongs to a pin mux group the mux will be switched if it isn't
 * already in use. Some convenience pin groups can also be used in which case
 * the effect is the same as enabling the function on each individual pin in the
 * group.
 */
static int asm9260_pinctrl_enable(struct pinctrl_dev *pctldev,
				 unsigned int function, unsigned int group)
{
#if 0
	struct asm9260_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	struct asm9260_pingroup *grp;
	int ret;
	unsigned int pin_num, mux_group, i, npins;
	const unsigned int *pins;

	/* group of pins? */
	if (group < ARRAY_SIZE(asm9260_groups)) {
		grp = &asm9260_groups[group];
		npins = grp->npins;
		pins = grp->pins;
		/*
		 * All pins in the group must belong to the same mux group,
		 * which allows us to just use the mux group of the first pin.
		 * By explicitly listing permitted pingroups for each function
		 * the pinmux core should ensure this is always the case.
		 */
	} else {
		pin_num = group - ARRAY_SIZE(asm9260_groups);
		npins = 1;
		pins = &pin_num;
	}
	mux_group = asm9260_mux_pins[*pins];

	/* no mux group, but can still be individually muxed to peripheral */
	if (mux_group >= ASM9260_MUX_GROUP_MAX) {
		if (function == ASM9260_MUX_PERIP)
			goto mux_pins;
		return -EINVAL;
	}

	/* mux group already set to a different function? */
	grp = &asm9260_mux_groups[mux_group];
	if (grp->func_count && grp->func != function) {
		dev_err(pctldev->dev,
			"%s: can't mux pin(s) to '%s', group already muxed to '%s'\n",
			__func__, asm9260_functions[function].name,
			asm9260_functions[grp->func].name);
		return -EBUSY;
	}

	dev_dbg(pctldev->dev, "%s: muxing %u pin(s) in '%s' to '%s'\n",
		__func__, npins, grp->name, asm9260_functions[function].name);

	/* if first pin in mux group to be enabled, enable the group mux */
	if (!grp->func_count) {
		grp->func = function;
		ret = asm9260_pinctrl_enable_mux(pmx, &grp->mux, function);
		if (ret)
			return ret;
	}
	/* add pins to ref count and mux individually to peripheral */
	grp->func_count += npins;
mux_pins:
	for (i = 0; i < npins; ++i)
		asm9260_pinctrl_perip_select(pmx, pins[i], true);

#endif
	return 0;
}
#if 0

/**
 * asm9260_pinctrl_gpio_request_enable() - Put pin in GPIO mode.
 * @pctldev:		Pin control data
 * @range:		GPIO range
 * @pin:		Pin number
 *
 * Puts a particular pin into GPIO mode, disabling peripheral control until it's
 * disabled again.
 */
static int asm9260_pinctrl_gpio_request_enable(struct pinctrl_dev *pctldev,
					      struct pinctrl_gpio_range *range,
					      unsigned int pin)
{
	struct asm9260_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	asm9260_pinctrl_gpio_select(pmx, pin, true);
	return 0;
}

/**
 * asm9260_pinctrl_gpio_disable_free() - Take pin out of GPIO mode.
 * @pctldev:		Pin control data
 * @range:		GPIO range
 * @pin:		Pin number
 *
 * Take a particular pin out of GPIO mode. If the pin is enabled for a
 * peripheral it will return to peripheral mode.
 */
static void asm9260_pinctrl_gpio_disable_free(struct pinctrl_dev *pctldev,
					     struct pinctrl_gpio_range *range,
					     unsigned int pin)
{
	struct asm9260_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	asm9260_pinctrl_gpio_select(pmx, pin, false);
}
#endif
static struct pinmux_ops asm9260_pinmux_ops = {
	.get_functions_count	= asm9260_pinctrl_get_funcs_count,
	.get_function_name	= asm9260_pinctrl_get_func_name,
	.get_function_groups	= asm9260_pinctrl_get_func_groups,
	.enable			= asm9260_pinctrl_enable,
//	.gpio_request_enable	= asm9260_pinctrl_gpio_request_enable,
//	.gpio_disable_free	= asm9260_pinctrl_gpio_disable_free,
};

#if 0
static int asm9260_pinconf_reg(struct pinctrl_dev *pctldev,
			      unsigned int pin,
			      enum pin_config_param param,
			      u32 *reg, u32 *val)
{
	/* All supported pins have controllable input bias */
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		*val = IOCON_MODE_PULL_NONE;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		*val = IOCON_MODE_PULL_UP;
		break;
	default:
		return -ENOTSUPP;
	};

	*reg = pin;
	return 0;
}

static int asm9260_pinconf_get(struct pinctrl_dev *pctldev,
			      unsigned int pin, unsigned long *config)
{
	struct asm9260_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	int ret;
	u32 reg, width, mask, shift, val, tmp, arg;

	/* Get register information */
	ret = asm9260_pinconf_reg(pctldev, pin, param,
				 &reg, &val);
	if (ret < 0)
		return ret;

	/* Extract field from register */
	tmp = pmx_read(pmx, reg);
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
	struct asm9260_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;
	unsigned int arg;
	int ret;
	u32 reg, width, mask, shift, val, tmp;
	unsigned long flags;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		dev_dbg(pctldev->dev, "%s(pin=%s, config=%#lx)\n",
			__func__, asm9260_pins[pin].name, configs[i]);

		/* Get register information */
		ret = asm9260_pinconf_reg(pctldev, pin, param,
					 &reg, &val);
		if (ret < 0)
			return ret;

		/* Unpack argument and range check it */
		if (arg > 1) {
			dev_dbg(pctldev->dev, "%s: arg %u out of range\n",
				__func__, arg);
			return -EINVAL;
		}

		/* Write register field */
		__global_lock2(flags);
		tmp = pmx_read(pmx, reg);
		tmp &= ~IOCON_MODE_MASK;
		if (arg)
			tmp |= val << IOCON_MODE_SHIFT;
		pmx_write(pmx, tmp, reg);
		__global_unlock2(flags);
	} /* for each config */

	return 0;
}

static const int asm9260_boolean_map[] = {
	[0]		= -EINVAL,
	[1]		= 1,
};

static const int asm9260_dr_map[] = {
	[REG_DR_2mA]	= 2,
	[REG_DR_4mA]	= 4,
	[REG_DR_8mA]	= 8,
	[REG_DR_12mA]	= 12,
};

static int asm9260_pinconf_group_reg(struct pinctrl_dev *pctldev,
				    const struct asm9260_pingroup *g,
				    enum pin_config_param param,
				    bool report_err,
				    u32 *reg, u32 *width, u32 *mask, u32 *shift,
				    const int **map)
{
	/* Drive configuration applies in groups, but not to all groups. */
	if (!g->drv) {
		if (report_err)
			dev_dbg(pctldev->dev,
				"%s: group %s has no drive control\n",
				__func__, g->name);
		return -ENOTSUPP;
	}

	/* Find information about drive parameter's register */
	switch (param) {
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		*reg = REG_PINCTRL_SCHMITT;
		*width = 1;
		*map = asm9260_boolean_map;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		*reg = REG_PINCTRL_DR;
		*width = 2;
		*map = asm9260_dr_map;
		break;
	default:
		return -ENOTSUPP;
	};

	/* Calculate field information */
	*shift = g->slw_bit * *width;
	*mask = (BIT(*width) - 1) << *shift;

	return 0;
}

static int asm9260_pinconf_group_get(struct pinctrl_dev *pctldev,
				    unsigned int group,
				    unsigned long *config)
{
	struct asm9260_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	const struct asm9260_pingroup *g;
	enum pin_config_param param = pinconf_to_config_param(*config);
	int ret, arg;
	unsigned int pin;
	u32 reg, width, mask, shift, val;
	const int *map;

	if (group >= ARRAY_SIZE(asm9260_groups)) {
		pin = group - ARRAY_SIZE(asm9260_groups);
		return asm9260_pinconf_get(pctldev, pin, config);
	}

	g = &asm9260_groups[group];
	if (g->npins == 1) {
		pin = g->pins[0];
		ret = asm9260_pinconf_get(pctldev, pin, config);
		if (ret != -ENOTSUPP)
			return ret;
	}

	/* Get register information */
	ret = asm9260_pinconf_group_reg(pctldev, g, param, true,
				       &reg, &width, &mask, &shift, &map);
	if (ret < 0)
		return ret;

	/* Extract field from register */
	val = pmx_read(pmx, reg);
	arg = map[(val & mask) >> shift];
	if (arg < 0)
		return arg;

	/* And pack config */
	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int asm9260_pinconf_group_set(struct pinctrl_dev *pctldev,
				    unsigned int group, unsigned long *configs,
				    unsigned num_configs)
{
	struct asm9260_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	const struct asm9260_pingroup *g;
	enum pin_config_param param;
	unsigned int arg, pin, i;
	const unsigned int *pit;
	int ret;
	u32 reg, width, mask, shift, val;
	unsigned long flags;
	const int *map;
	int j;

	if (group >= ARRAY_SIZE(asm9260_groups)) {
		pin = group - ARRAY_SIZE(asm9260_groups);
		return asm9260_pinconf_set(pctldev, pin, configs, num_configs);
	}

	g = &asm9260_groups[group];
	if (g->npins == 1) {
		pin = g->pins[0];
		ret = asm9260_pinconf_set(pctldev, pin, configs, num_configs);
		if (ret != -ENOTSUPP)
			return ret;
	}

	for (j = 0; j < num_configs; j++) {
		param = pinconf_to_config_param(configs[j]);

		dev_dbg(pctldev->dev, "%s(group=%s, config=%#lx)\n",
			__func__, g->name, configs[j]);

		/* Get register information */
		ret = asm9260_pinconf_group_reg(pctldev, g, param, true, &reg,
						&width, &mask, &shift, &map);
		if (ret < 0) {
			/*
			 * Maybe we're trying to set a per-pin configuration
			 * of a group, so do the pins one by one. This is
			 * mainly as a convenience.
			 */
			for (i = 0, pit = g->pins; i < g->npins; ++i, ++pit) {
				ret = asm9260_pinconf_set(pctldev, *pit, configs,
					num_configs);
				if (ret)
					return ret;
			}
			return 0;
		}

		/* Unpack argument and map it to register value */
		arg = pinconf_to_config_argument(configs[j]);
		for (i = 0; i < BIT(width); ++i) {
			if (map[i] == arg || (map[i] == -EINVAL && !arg)) {
				/* Write register field */
				__global_lock2(flags);
				val = pmx_read(pmx, reg);
				val &= ~mask;
				val |= i << shift;
				pmx_write(pmx, val, reg);
				__global_unlock2(flags);
				goto next_config;
			}
		}

		dev_dbg(pctldev->dev, "%s: arg %u not supported\n",
			__func__, arg);
		return -EINVAL;

next_config:
		;
	} /* for each config */

	return 0;
}

static struct pinconf_ops asm9260_pinconf_ops = {
	.is_generic			= true,
	.pin_config_get			= asm9260_pinconf_get,
	.pin_config_set			= asm9260_pinconf_set,
	.pin_config_group_get		= asm9260_pinconf_group_get,
	.pin_config_group_set		= asm9260_pinconf_group_set,
	.pin_config_config_dbg_show	= pinconf_generic_dump_config,
};
#endif
/*
 * Pin control driver setup
 */

/* OK */
static struct pinctrl_desc asm9260_pinctrl_desc = {
	.pctlops	= &asm9260_pinctrl_ops,
	.pmxops		= &asm9260_pinmux_ops,
//	.confops	= &asm9260_pinconf_ops,
	.owner		= THIS_MODULE,
};

/* OK */
static int asm9260_pinctrl_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct asm9260_pmx *pmx;

	pmx = devm_kzalloc(&pdev->dev, sizeof(*pmx), GFP_KERNEL);
	if (!pmx) {
		dev_err(&pdev->dev, "Can't alloc asm9260_pmx\n");
		return -ENOMEM;
	}
	pmx->dev = &pdev->dev;
	spin_lock_init(&pmx->lock);

	asm9260_pinctrl_desc.name = dev_name(&pdev->dev);
	asm9260_pinctrl_desc.pins = asm9260_pins;
	asm9260_pinctrl_desc.npins = ARRAY_SIZE(asm9260_pins);

	pmx->regs = of_io_request_and_map(np, 0, dev_name(&pdev->dev));
	if (IS_ERR(pmx->regs))
		return PTR_ERR(pmx->regs);

	pmx->pctl = pinctrl_register(&asm9260_pinctrl_desc, &pdev->dev, pmx);
	if (!pmx->pctl) {
		dev_err(&pdev->dev, "Couldn't register pinctrl driver\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, pmx);

	dev_info(&pdev->dev, "ASM9260 pinctrl driver initialised\n");

	return 0;
}

static int asm9260_pinctrl_remove(struct platform_device *pdev)
{
	struct asm9260_pmx *pmx = platform_get_drvdata(pdev);

	pinctrl_unregister(pmx->pctl);

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
	//asm9260_init_mux_pins();
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
