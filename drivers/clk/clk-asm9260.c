/*
 * U300 clock implementation
 * Copyright (C) 2007-2012 ST-Ericsson AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Linus Walleij <linus.walleij@stericsson.com>
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com>
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define HW_AHBCLKCTRL0		0x0020
#define HW_AHBCLKCTRL1		0x0030
#define HW_SYSPLLCTRL		0x0100
#define HW_MAINCLKSEL		0x0120
#define HW_MAINCLKUEN		0x0124
#define HW_UARTCLKSEL		0x0128
#define HW_UARTCLKUEN		0x012c
#define HW_I2S0CLKSEL		0x0130
#define HW_I2S0CLKUEN		0x0134
#define HW_I2S1CLKSEL		0x0138
#define HW_I2S1CLKUEN		0x013c
#define HW_WDTCLKSEL		0x0160
#define HW_WDTCLKUEN		0x0164
#define HW_CLKOUTCLKSEL		0x0170
#define HW_CLKOUTCLKUEN		0x0174
#define HW_CPUCLKDIV		0x017c
#define HW_SYSAHBCLKDIV		0x0180
#define HW_I2S0MCLKDIV		0x0190
#define HW_I2S0SCLKDIV		0x0194
#define HW_I2S1MCLKDIV		0x0188
#define HW_I2S1SCLKDIV		0x018c
#define HW_UART0CLKDIV		0x0198
#define HW_UART1CLKDIV		0x019c
#define HW_UART2CLKDIV		0x01a0
#define HW_UART3CLKDIV		0x01a4
#define HW_UART4CLKDIV		0x01a8
#define HW_UART5CLKDIV		0x01ac
#define HW_UART6CLKDIV		0x01b0
#define HW_UART7CLKDIV		0x01b4
#define HW_UART8CLKDIV		0x01b8
#define HW_UART9CLKDIV		0x01bc
#define HW_SPI0CLKDIV		0x01c0
#define HW_SPI1CLKDIV		0x01c4
#define HW_QUADSPICLKDIV	0x01c8
#define HW_SSP0CLKDIV		0x01d0
#define HW_NANDCLKDIV		0x01d4
#define HW_TRACECLKDIV		0x01e0
#define HW_CAMMCLKDIV		0x01e8
#define HW_WDTCLKDIV		0x01ec
#define HW_CLKOUTCLKDIV		0x01f4
#define HW_MACCLKDIV		0x01f8
#define HW_LCDCLKDIV		0x01fc
#define HW_ADCANACLKDIV		0x0200

static DEFINE_SPINLOCK(asm9260_clk_lock);

struct asm9260_gate_data {
	const char *name;
	const char *parent_name;
	unsigned long flags;
	u32 reg;
	u8 bit_idx;
};

static void __iomem *base;

static const struct asm9260_gate_data asm9260_gates[] __initconst = {
	{ "main_gate",	"main_mux",	CLK_SET_RATE_PARENT,	HW_MAINCLKUEN,	0 },
	{ "uart_gate",	"uart_mux",	CLK_SET_RATE_PARENT,	HW_UARTCLKUEN,	0 },
	{ "i2s0_gate",	"i2s0_mux",	CLK_SET_RATE_PARENT,	HW_I2S0CLKUEN,	0 },
	{ "i2s1_gate",	"i2s1_mux",	CLK_SET_RATE_PARENT,	HW_I2S1CLKUEN,	0 },
	{ "wdt_gate",	"wdt_mux",	CLK_SET_RATE_PARENT,	HW_WDTCLKUEN,	0 },
	{ "clkout_gate",	"clkout_mux",	CLK_SET_RATE_PARENT,	HW_CLKOUTCLKUEN, 0 },

	/* ahb gates */
	{ "rom",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	1 },
	{ "ram",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	2 },
	{ "gpio",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	4 },
	{ "mac",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	5 },
	{ "emi",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	6 },
	{ "usb0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	7 },
	{ "usb1",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	8 },
	{ "dma0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	9 },
	{ "dma1",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	10 },
	{ "uart0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	11 },
	{ "uart1",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	12 },
	{ "uart2",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	13 },
	{ "uart3",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	14 },
	{ "uart4",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	15 },
	{ "uart5",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	16 },
	{ "uart6",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	17 },
	{ "uart7",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	18 },
	{ "uart8",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	19 },
	{ "uart9",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	20 },
	{ "i2s0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	21 },
	{ "i2c0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	22 },
	{ "i2c1",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	23 },
	{ "ssp0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	24 },
	{ "ioconf",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	25 },
	{ "wdt",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	26 },
	{ "can0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	27 },
	{ "can1",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	28 },
	{ "mpwm",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	29 },
	{ "spi0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	30 },
	{ "spi1",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL0,	31 },

	{ "qei",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	0 },
	{ "quadspi0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	1 },
	{ "capmif",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	2 },
	{ "lcdif",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	3 },
	{ "timer0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	4 },
	{ "timer1",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	5 },
	{ "timer2",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	6 },
	{ "timer3",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	7 },
	{ "irq",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	8 },
	{ "rtc",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	9 },
	{ "nand",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	10 },
	{ "adc0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	11 },
	{ "led",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	12 },
	{ "dac0",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	13 },
	{ "lcd",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	14 },
	{ "i2s1",	"ahb_div",	CLK_SET_RATE_PARENT,	HW_AHBCLKCTRL1,	15 },
};

static void __iomem *asm9260_get_sreg(struct device_node *node)
{
	u32 reg;
	int ret;

	ret = of_property_read_u32(node, "reg", &reg);
	if (WARN_ON(ret))
		return NULL;

	return base + reg;
}

/*
 * On this chip gate used to disable or to update clock
 * after new source was selected
 */

static void __init asm9260_gate_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *iomem;
	const char *parent_name;
	u32 bit;
	int ret;

	iomem = asm9260_get_sreg(node);
	parent_name = of_clk_get_parent_name(node, 0);

	ret = of_property_read_u32(node, "bit-index", &bit);
	if (WARN_ON(ret))
		return;

	clk = clk_register_gate(NULL, clk_name, parent_name,
			CLK_SET_RATE_PARENT, iomem, bit, 0,
			&asm9260_clk_lock);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(asm9260_gate, "alphascale,asm9260-gate-clock", asm9260_gate_init);


static void __init asm9260_div_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *iomem;
	const char *parent_name;

	iomem = asm9260_get_sreg(node);

	parent_name = of_clk_get_parent_name(node, 0);
	clk = clk_register_divider(NULL, clk_name, parent_name,
			CLK_SET_RATE_PARENT, iomem, 0, 8, CLK_DIVIDER_ONE_BASED,
			&asm9260_clk_lock);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(asm9260_div, "alphascale,asm9260-div-clock", asm9260_div_init);

/*
 * Simple one bit MUX for two sources
 */
static void __init asm9260_bimux_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	u8 num_parents;
	void __iomem *iomem;
	const char **parent_names;
	int ret, i;
	u32 *table;

	iomem = asm9260_get_sreg(node);
	if (!iomem)
		return;

	num_parents = of_clk_get_parent_count(node);
	if (WARN_ON(!num_parents || num_parents > 2))
		return;

	parent_names = kzalloc(sizeof(char *) * num_parents, GFP_KERNEL);
	if (WARN_ON(!parent_names))
		return;

	table = kzalloc(sizeof(u32) * num_parents, GFP_KERNEL);
	if (WARN_ON(!table))
		return;

	ret = of_property_read_u32_array(node, "mux-table", table,
			num_parents);
	if (WARN_ON(ret))
		return;

	for (i = 0; i < num_parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	clk = clk_register_mux_table(NULL, clk_name, parent_names,
			num_parents, 0, iomem, 0, 1, 0, table,
			&asm9260_clk_lock);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(asm9260_bimux, "alphascale,asm9260-bimux-clock", asm9260_bimux_init);

static void __init asm9260_pll_init(struct device_node *np)
{
	struct clk *clk;
	struct resource res;
	const char *clk_name = np->name;
	u32 rate;
	const char *parent_name;
	u32 accuracy = 0;

	of_address_to_resource(np, 0, &res);
	//if (!request_mem_region(res.start, resource_size(&res), "asm9260-clk"))
	if (!request_mem_region(0x80040000, 0x504, "asm9260-clk"))
		panic("%s: unable to request mem region", np->name);

	//base = ioremap_nocache(res.start, resource_size(&res));
	base = ioremap_nocache(0x80040000, 0x504);
	if (!base)
		panic("%s: unable to map resource", np->name);


	/* register pll */
	rate = (ioread32(base + HW_SYSPLLCTRL) & 0xffff) * 1000000;

	parent_name = of_clk_get_parent_name(np, 0);
	accuracy = clk_get_accuracy(__clk_lookup(parent_name));
	clk = clk_register_fixed_rate_with_accuracy(NULL, clk_name, parent_name,
			0, rate,
			accuracy);

	if (!IS_ERR(clk))
		of_clk_add_provider(np, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(asm9260_pll, "alphascale,asm9260-pll-clock", asm9260_pll_init);
