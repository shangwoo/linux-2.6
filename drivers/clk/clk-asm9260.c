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

static DEFINE_SPINLOCK(asm9260_clk_lock);

struct asm9260_clk {
	void __iomem    *reg;
	char *parent_name;
};

static void __iomem *asm9260_get_sreg(struct device_node *node)
{
	u32 reg;
	void __iomem *iomem;
	struct device_node *srnp;
	int ret;

	ret = of_property_read_u32(node, "reg", &reg);
	if (WARN_ON(ret))
		return NULL;

	srnp = of_find_compatible_node(NULL, NULL, "alpscale,asm9260-sregs");
	iomem = of_iomap(srnp, 0);
	iomem += reg;

	return iomem;
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
			0, iomem, bit, 0,
			&asm9260_clk_lock);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(asm9260_gate, "alpscale,asm9260-gate-clock", asm9260_gate_init);


static void __init asm9260_div_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *iomem;
	const char *parent_name;

	iomem = asm9260_get_sreg(node);

	parent_name = of_clk_get_parent_name(node, 0);
	clk = clk_register_divider(NULL, clk_name, parent_name,
			0, iomem, 0, 8, CLK_DIVIDER_ONE_BASED,
			&asm9260_clk_lock);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(asm9260_div, "alpscale,asm9260-div-clock", asm9260_div_init);

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
CLK_OF_DECLARE(asm9260_bimux, "alpscale,asm9260-bimux-clock", asm9260_bimux_init);

static void __init asm9260_pll_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	u32 rate;
	void __iomem *iomem;
	const char *parent_name;
	u32 accuracy = 0;

	iomem = asm9260_get_sreg(node);
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
