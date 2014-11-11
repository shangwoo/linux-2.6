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

#if 0
/* Pins in each pin group */

/*
 * The magic "perip" function allows otherwise non-muxing pins to be enabled in
 * peripheral mode.
 */
static const char * const perip_groups[] = {
	/* non-muxing convenient gpio pingroups */
	"uart",
	"uart0",
	"uart1",
	"spi0",
	"spi1",
	"scb0",
	"scb1",
	"scb2",
	"i2s",
	/* individual pins not part of a pin mux group */
	"spi0_mclk",
	"spi0_cs0",
	"spi0_cs1",
	"spi0_cs2",
	"spi0_dout",
	"spi0_din",
	"spi1_mclk",
	"spi1_cs0",
	"spi1_cs1",
	"spi1_dout",
	"spi1_din",
	"uart0_rxd",
	"uart0_txd",
	"uart0_cts",
	"uart0_rts",
	"uart1_rxd",
	"uart1_txd",
	"scb0_sdat",
	"scb0_sclk",
	"scb1_sdat",
	"scb1_sclk",
	"scb2_sdat",
	"scb2_sclk",
	"i2s_mclk",
	"i2s_bclk_out",
	"i2s_lrclk_out",
	"i2s_dout0",
	"i2s_dout1",
	"i2s_dout2",
	"i2s_din",
	"pdm_a",
	"pdm_b",
	"pdm_c",
};

static const char * const sdh_sdio_groups[] = {
	"sdh",
	"sdio",
	/* sdh pins */
	"sdh_cd",
	"sdh_wp",
	"sdh_clk_in",
	/* sdio pins */
	"sdio_clk",
	"sdio_cmd",
	"sdio_d0",
	"sdio_d1",
	"sdio_d2",
	"sdio_d3",
};

static const char * const spi1_cs2_groups[] = {
	"spi1_cs2",
};

static const char * const pdm_dac_groups[] = {
	"pdm_d",
};

static const char * const usb_vbus_groups[] = {
	"spi1_cs2",
	"pdm_d",
};

static const char * const afe_groups[] = {
	"afe",
	/* afe pins */
	"tx_on",
	"rx_on",
	"pll_on",
	"pa_on",
	"rx_hp",
	"ant_sel0",
	"ant_sel1",
	"gain0",
	"gain1",
	"gain2",
	"gain3",
	"gain4",
	"gain5",
	"gain6",
	"gain7",
};

static const char * const tft_groups[] = {
	"tft",
	/* tft pins */
	"tft_red0",
	"tft_red1",
	"tft_red2",
	"tft_red3",
	"tft_red4",
	"tft_red5",
	"tft_red6",
	"tft_red7",
	"tft_green0",
	"tft_green1",
	"tft_green2",
	"tft_green3",
	"tft_green4",
	"tft_green5",
	"tft_green6",
	"tft_green7",
	"tft_blue0",
	"tft_blue1",
	"tft_blue2",
	"tft_blue3",
	"tft_blue4",
	"tft_blue5",
	"tft_blue6",
	"tft_blue7",
	"tft_vdden_gd",
	"tft_panelclk",
	"tft_blank_ls",
	"tft_vsync_ns",
	"tft_hsync_nr",
	"tft_vd12acb",
	"tft_pwrsave",
};

/* Mux functions that can be used by a mux */

enum asm9260_mux {
	ASM9260_MUX_NA = -1,

	ASM9260_MUX_CAM,
	ASM9260_MUX_CAN,
	ASM9260_MUX_CT,
	ASM9260_MUX_I2C,
	ASM9260_MUX_I2S,
	ASM9260_MUX_JTAG,
	ASM9260_MUX_LCD,
	ASM9260_MUX_LCD_MCU,
	ASM9260_MUX_MC,
	ASM9260_MUX_MII,
	ASM9260_MUX_NAND,
	ASM9260_MUX_OUTCLK,
	ASM9260_MUX_QEI,
	ASM9260_MUX_QSPI,
	ASM9260_MUX_RMII,
	ASM9260_MUX_SD,
	ASM9260_MUX_SPI,
	ASM9260_MUX_UART,
};

#define FUNCTION(mux, fname, group)			\
	[(ASM9260_MUX_ ## mux)] = {			\
		.name = #fname,				\
		.groups = group##_groups,		\
		.ngroups = ARRAY_SIZE(group##_groups),	\
	}
/* For intermediate functions with submuxes */
#define NULL_FUNCTION(mux, fname)			\
	[(ASM9260_MUX_ ## mux)] = {			\
		.name = #fname,				\
	}

/* Must correlate with enum asm9260_mux */
static const struct asm9260_function asm9260_functions[] = {
	/*	 FUNCTION	function name	pingroups */
	FUNCTION(PERIP,		perip,		perip),
	FUNCTION(SDH,		sdh,		sdh_sdio),
	FUNCTION(SDIO,		sdio,		sdh_sdio),
	FUNCTION(SPI1_CS2,	spi1_cs2,	spi1_cs2),
	FUNCTION(PDM_DAC,	pdm_dac,	pdm_dac),
	FUNCTION(USB_VBUS,	usb_vbus,	usb_vbus),
	FUNCTION(AFE,		afe,		afe),
	FUNCTION(TS_OUT_0,	ts_out_0,	afe),
	FUNCTION(DAC,		ext_dac,	tft),
	FUNCTION(NOT_IQADC_STB,	not_iqadc_stb,	tft),
	FUNCTION(IQDAC_STB,	iqdac_stb,	tft),
	FUNCTION(TFT,		tft,		tft),
	NULL_FUNCTION(EXT_DAC,	_ext_dac),
	FUNCTION(TS_OUT_1,	ts_out_1,	tft),
	FUNCTION(LCD_TRACE,	lcd_trace,	tft),
	FUNCTION(PHY_RINGOSC,	phy_ringosc,	tft),
};

/* Sub muxes */

/**
 * MUX() - Initialise a mux description.
 * @f0:		Function 0 (ASM9260_MUX_ is prepended, NA for none)
 * @f1:		Function 1 (ASM9260_MUX_ is prepended, NA for none)
 * @f2:		Function 2 (ASM9260_MUX_ is prepended, NA for none)
 * @f3:		Function 3 (ASM9260_MUX_ is prepended, NA for none)
 * @f4:		Function 4 (ASM9260_MUX_ is prepended, NA for none)
 * @mux_r:	Mux register (REG_PINCTRL_ is prepended)
 * @mux_b:	Bit number in register that the mux field begins
 * @mux_w:	Width of mux field in register
 */
#define MUX(f0, f1, f2, f3, f4, mux_r, mux_b, mux_w)		\
	{							\
		.funcs = {					\
			ASM9260_MUX_ ## f0,			\
			ASM9260_MUX_ ## f1,			\
			ASM9260_MUX_ ## f2,			\
			ASM9260_MUX_ ## f3,			\
			ASM9260_MUX_ ## f4,			\
		},						\
		.reg = (REG_PINCTRL_ ## mux_r),			\
		.bit = (mux_b),					\
		.width = (mux_w),				\
	}

/**
 * MUX_PG() - Initialise a pin group with mux control
 * @pg_name:	Pin group name (stringified, _pins appended to get pins array)
 * @f0:		Function 0 (ASM9260_MUX_ is prepended, NA for none)
 * @f1:		Function 1 (ASM9260_MUX_ is prepended, NA for none)
 * @f2:		Function 2 (ASM9260_MUX_ is prepended, NA for none)
 * @f3:		Function 3 (ASM9260_MUX_ is prepended, NA for none)
 * @f4:		Function 4 (ASM9260_MUX_ is prepended, NA for none)
 * @mux_r:	Mux register (REG_PINCTRL_ is prepended)
 * @mux_b:	Bit number in register that the mux field begins
 * @mux_w:	Width of mux field in register
 */
#define MUX_PG(pg_name, f0, f1, f2, f3, f4, f5, f6,		\
	       mux_r, mux_b, mux_w)				\
	{							\
		.name = #pg_name,				\
		.pins = pg_name##_pins,				\
		.npins = ARRAY_SIZE(pg_name##_pins),		\
		.mux = MUX(f0, f1, f2, f3, f4, f5, f6,		\
			   mux_r, mux_b, mux_w),		\
	}


/**
 * DRV_PG() - Initialise a pin group with drive control
 * @pg_name:	Pin group name (stringified, _pins appended to get pins array)
 * @slw_b:	Slew register bit.
 *		The same bit is used for Schmitt, and Drive (*2).
 */
#define DRV_PG(pg_name, slw_b)					\
	{							\
		.name = #pg_name,				\
		.pins = pg_name##_pins,				\
		.npins = ARRAY_SIZE(pg_name##_pins),		\
		.drv = true,					\
		.slw_bit = (slw_b),				\
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
	/*     pg_name,  f0,       f1,       f2,       f3,        f4,          mux r/b/w */
//	MUX_PG(sdh,      SDH,      SDIO,     NA,       NA,        NA,          IF_CTL, 20, 2),
//	MUX_PG(sdio,     SDIO,     SDH,      NA,       NA,        NA,          IF_CTL, 16, 2),
//	MUX_PG(spi1_cs2, SPI1_CS2, USB_VBUS, NA,       NA,        NA,          IF_CTL, 10, 2),
//	MUX_PG(pdm_d,    PDM_DAC,  USB_VBUS, NA,       NA,        NA,          IF_CTL,  8, 2),
//	MUX_PG(afe,      AFE,      TS_OUT_0, NA,       NA,        NA,          IF_CTL,  4, 2),
//	MUX_PG(tft,      TFT,      EXT_DAC,  TS_OUT_1, LCD_TRACE, PHY_RINGOSC, IF_CTL,  0, 3),

	/*     pg,		f0,	f1,	f2,	f3,	f4,	f5,	f6, */
	MUX_PG(GPIO0_0,		UART,	I2S,	SPI,	JTAG,	NA,	NA,	NA),
	MUX_PG(GPIO0_1,		UART,	I2S,	SPI,	JTAG,	NA,	NA,	NA),
	MUX_PG(GPIO0_2,		UART,	I2S,	SPI,	JTAG,	NA,	NA,	NA),
	MUX_PG(GPIO0_3,		UART,	I2S,	SPI,	JTAG,	NA,	NA,	NA),
	MUX_PG(GPIO0_4,		UART,	I2S,	SPI,	JTAG,	I2C,	NA,	NA),
	MUX_PG(GPIO1_4,		CT,	UART,	LCD_MCU,SPI,	MII,	LCD,	NA),
	MUX_PG(GPIO1_5,		UART,	LCD_MCU,SPI,	RMII,	LCD,	NA,	NA),
	MUX_PG(GPIO1_6,		UART,	LCD_MCU,SPI,	RMII,	LCD,	I2C,	NA),
	MUX_PG(GPIO1_7,		UART,	LCD_MCU,SPI,	RMII,	LCD,	I2C,	NA),
	MUX_PG(GPIO2_0,		CT,	UART,	LCD_MCU,SPI,	MII,	LCD,	CAN),
	MUX_PG(GPIO2_1,		CT,	UART,	LCD_MCU,SPI,	MII,	LCD,	CAN),
	MUX_PG(GPIO2_2,		CT,	UART,	LCD_MCU,SPI,	MII,	LCD,	NA),
	MUX_PG(GPIO2_3,		CT,	UART,	LCD_MCU,SPI,	MII,	LCD,	NA),
	MUX_PG(GPIO2_4,		CT,	UART,	LCD_MCU,MII,	LCD,	NA,	NA),
	MUX_PG(GPIO2_5,		UART,	LCD_MCU,MII,	LCD,	OUTCLK,	NA,	NA),
	MUX_PG(GPIO2_6,		UART,	LCD_MCU,MII,	LCD,	CAN,	NA,	NA),
	MUX_PG(GPIO2_7,		UART,	LCD_MCU,MII,	LCD,	CAN,	NA,	NA),
	MUX_PG(GPIO3_0,		CT,	UART,	LCD_MCU,SD,	RMII,	LCD,	NA),
	MUX_PG(GPIO3_1,		CT,	UART,	LCD_MCU,SD,	RMII,	LCD,	NA),
	MUX_PG(GPIO3_2,		CT,	UART,	LCD_MCU,SD,	RMII,	LCD,	CAN),
	MUX_PG(GPIO3_3,		CT,	UART,	LCD_MCU,SD,	RMII,	LCD,	CAN),
	MUX_PG(GPIO3_4,		CT,	UART,	LCD_MCU,SD,	RMII,	LCD,	OUTCLK),
	MUX_PG(GPIO3_5,		UART,	LCD_MCU,SD,	RMII,	LCD,	I2C,	NA),
	MUX_PG(GPIO3_6,		UART,	LCD_MCU,RMII,	LCD,	I2C,	NA,	NA),
	MUX_PG(GPIO3_7,		UART,	LCD_MCU,RMII,	LCD,	NA,	NA,	NA),
	MUX_PG(GPIO4_0,		CT,	UART,	QSPI,	MII,	LCD,	NA,	NA),
	MUX_PG(GPIO4_1,		CT,	UART,	QSPI,	MII,	LCD,	NA,	NA),
	MUX_PG(GPIO4_2,		CT,	UART,	QSPI,	MII,	LCD,	NA,	NA),
	MUX_PG(GPIO4_3,		CT,	UART,	QSPI,	MII,	LCD,	NA,	NA),
	MUX_PG(GPIO4_4,		CT,	UART,	QSPI,	MII,	LCD,	NA,	NA),
	MUX_PG(GPIO4_5,		UART,	QSPI,	MII,	LCD,	NA,	NA,	NA),
	MUX_PG(GPIO4_6,		MII,	LCD,	I2C,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO4_7,		MII,	LCD,	I2C,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO5_0,		MC,	UART,	I2S,	SD,	RMII,	NA,	NA),
	MUX_PG(GPIO5_1,		MC,	UART,	I2S,	SD,	RMII,	NA,	NA),
	MUX_PG(GPIO5_2,		MC,	UART,	I2S,	SD,	RMII,	CAN,	NA),
	MUX_PG(GPIO5_3,		MC,	UART,	I2S,	SD,	RMII,	CAN,	NA),
	MUX_PG(GPIO5_4,		MC,	UART,	I2S,	SD,	RMII,	NA,	NA),
	MUX_PG(GPIO8_1,		UART,	CAM,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_2,		UART,	CAM,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_3,		UART,	CAM,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_4,		UART,	CAM,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_5,		UART,	CAM,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_6,		UART,	CAM,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO8_7,		UART,	CAM,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO9_0,		CT,	UART,	CAM,	RMII,	I2C,	NA,	NA),
	MUX_PG(GPIO9_1,		CT,	UART,	CAM,	RMII,	I2C,	NA,	NA),
	MUX_PG(GPIO9_2,		CT,	UART,	CAM,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO9_3,		CT,	UART,	CAM,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO9_4,		CT,	UART,	CAM,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO9_5,		UART,	CAM,	RMII,	I2C,	NA,	NA,	NA),
	MUX_PG(GPIO10_0,	CT,	UART,	I2S,	SPI,	NA,	NA,	NA),
	MUX_PG(GPIO10_1,	CT,	UART,	I2S,	SPI,	RMII,	NA,	NA),
	MUX_PG(GPIO10_2,	CT,	UART,	I2S,	SPI,	RMII,	NA,	NA),
	MUX_PG(GPIO10_3,	CT,	UART,	I2S,	SPI,	CAN,	NA,	NA),
	MUX_PG(GPIO10_4,	CT,	UART,	I2S,	SPI,	RMII,	NA,	NA),
	MUX_PG(GPIO10_5,	UART,	I2S,	SPI,	RMII,	CAN,	NA,	NA),
	MUX_PG(GPIO10_6,	UART,	I2S,	SPI,	RMII,	CAN,	NA,	NA),
	MUX_PG(GPIO10_7,	UART,	CAM,	SPI,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO11_0,	CT,	UART,	I2S,	QSPI,	NAND,	NA,	NA),
	MUX_PG(GPIO11_1,	CT,	UART,	I2S,	QSPI,	NAND,	NA,	NA),
	MUX_PG(GPIO11_2,	CT,	UART,	I2S,	QSPI,	NAND,	NA,	NA),
	MUX_PG(GPIO11_3,	CT,	UART,	I2S,	QSPI,	NAND,	NA,	NA),
	MUX_PG(GPIO11_4,	CT,	UART,	I2S,	QSPI,	NAND,	NA,	NA),
	MUX_PG(GPIO11_5,	UART,	I2S,	QSPI,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO11_6,	UART,	I2S,	NAND,	I2C,	NA,	NA,	NA),
	MUX_PG(GPIO11_7,	UART,	CAM,	NAND,	I2C,	NA,	NA,	NA),
	MUX_PG(GPIO12_0,	CT,	UART,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO12_1,	CT,	UART,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO12_2,	CT,	UART,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO12_3,	CT,	UART,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO12_4,	CT,	UART,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO12_5,	UART,	SD,	NAND,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO12_6,	UART,	CAM,	NAND,	I2C,	NA,	NA,	NA),
	MUX_PG(GPIO12_7,	CAM,	NAND,	I2C,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO13_4,	MC,	UART,	SPI,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO13_5,	MC,	UART,	SPI,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO13_6,	MC,	UART,	SPI,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO13_7,	MC,	SPI,	NAND,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO14_0,	MC,	UART,	I2S,	SD,	NAND,	NA,	NA),
	MUX_PG(GPIO14_1,	MC,	UART,	I2S,	SD,	NAND,	NA,	NA),
	MUX_PG(GPIO14_2,	UART,	I2S,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO14_3,	UART,	I2S,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO14_4,	UART,	I2S,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO14_5,	UART,	I2S,	SD,	NAND,	NA,	NA,	NA),
	MUX_PG(GPIO15_0,	UART,	I2S,	SD,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO15_1,	UART,	I2S,	SD,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO15_2,	UART,	I2S,	SD,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO15_3,	UART,	I2S,	SD,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO15_4,	UART,	I2S,	SD,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO15_5,	UART,	I2S,	SD,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO15_6,	UART,	I2S,	RMII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO15_7,	UART,	RMII,	NA,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_0,	CT,	UART,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_1,	CT,	UART,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_2,	CT,	UART,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_3,	CT,	UART,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_4,	CT,	UART,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_5,	QEI,	UART,	MII,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO16_6,	QEI,	UART,	MII,	CAN,	NA,	NA,	NA),
	MUX_PG(GPIO16_7,	QEI,	UART,	MII,	CAN,	NA,	NA,	NA),
	MUX_PG(GPIO17_0,	CT,	UART,	I2S,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO17_1,	CT,	UART,	I2S,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO17_2,	CT,	UART,	I2S,	I2C,	NA,	NA,	NA),
	MUX_PG(GPIO17_3,	CT,	UART,	I2S,	I2C,	NA,	NA,	NA),
	MUX_PG(GPIO17_4,	CT,	UART,	I2S,	NA,	NA,	NA,	NA),
	MUX_PG(GPIO17_5,	QEI,	UART,	I2S,	RMII,	NA,	NA,	NA),
	MUX_PG(GPIO17_6,	QEI,	UART,	I2S,	MII,	NA,	NA,	NA),
	MUX_PG(GPIO17_7,	QEI,	UART,	MII,	NA,	NA,	NA,	NA),
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
#if 0
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

	return 0;
}

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

static struct pinmux_ops asm9260_pinmux_ops = {
	.get_functions_count	= asm9260_pinctrl_get_funcs_count,
	.get_function_name	= asm9260_pinctrl_get_func_name,
	.get_function_groups	= asm9260_pinctrl_get_func_groups,
	.enable			= asm9260_pinctrl_enable,
	.gpio_request_enable	= asm9260_pinctrl_gpio_request_enable,
	.gpio_disable_free	= asm9260_pinctrl_gpio_disable_free,
};

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
//	.pmxops		= &asm9260_pinmux_ops,
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
