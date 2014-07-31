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

#if 0
void __init asm9260_clk_init(void __iomem *base)
{
	of_clk_init(asm9260_clk_match);
}
#endif
static void __init asm9260_pll_init(struct device_node *node)
{
	printk("!!!!!!!!!dddd\n");
//        of_clk_init(asm9260_clk_match);
}
CLK_OF_DECLARE(asm9260_pll, "alpscale,asm9260-syspllctrl", asm9260_pll_init);
