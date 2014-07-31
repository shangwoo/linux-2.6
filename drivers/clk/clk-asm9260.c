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

struct asm9260_clk {
	void __iomem    *reg;
	char *parent_name;
};

static void __init asm9260_pll_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	u32 rate, reg;
	void __iomem *iomem;
	struct device_node *srnp;
	const char *parent_name;
	u32 accuracy = 0;
	int ret;

	ret = of_property_read_u32(node, "reg", &reg);
	if (WARN_ON(ret))
		return;

	srnp = of_find_compatible_node(NULL, NULL, "alpscale,asm9260-sregs");
	iomem = of_iomap(srnp, 0);
	iomem += reg;
	rate = (ioread32(iomem) & 0xffff) * 1000000;

	parent_name = of_clk_get_parent_name(node, 0);
	accuracy = clk_get_accuracy(__clk_lookup(parent_name));
	clk = clk_register_fixed_rate_with_accuracy(NULL, clk_name, parent_name,
			0, rate,
			accuracy);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(asm9260_pll, "alpscale,asm9260-pll-clock", asm9260_pll_init);
