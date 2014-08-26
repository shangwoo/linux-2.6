/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/stmp_device.h>
#include <asm/exception.h>
#include <linux/bitops.h>

#include "irqchip.h"

#define HW_ICOLL_VECTOR				0x0000
/* bits 31:2
 * This register presents the vector address for the interrupt currently
 * active on the CPU IRQ input. Writing to this register notifies the
 * interrupt collector that the interrupt service routine for the current
 * interrupt has been entered.
 * The exception trap should have a LDPC instruction from this address:
 * LDPC HW_ICOLL_VECTOR_ADDR; IRQ exception at 0xffff0018
 */

#define HW_ICOLL_LEVELACK			0x0010
#define BM_LEVELn(nr)				BIT(nr)

#define HW_ICOLL_CTRL				0x0020
#define HW_ICOLL_STAT_OFFSET			0x0030
#define HW_ICOLL_RAW0				0x0040
#define HW_ICOLL_RAW1				0x0050
#define	HW_ICOLL_PRIORITYn(n)			(0x0060 + ((n) >> 2) * 0x10)
#define	HW_ICOLL_INTERRUPTn_SET(n)		(0x0064 + ((n) >> 2) * 0x10)
#define	HW_ICOLL_INTERRUPTn_CLR(n)		(0x0068 + ((n) >> 2) * 0x10)

#define BM_ICOLL_INTERRUPTn_SHIFT(n)		(((n) & 0x3) << 3)
#define BM_ICOLL_INTERRUPTn_ENABLE(n)		(1 << (2 + \
			BM_ICOLL_INTERRUPTn_SHIFT(n)))

#define HW_ICOLL_VBASE				0x0160
#define HW_ICOLL_CLEAR0				0x01d0
#define	HW_ICOLL_CLEAR1				0x01e0
#define HW_ICOLL_UNDEF_VECTOR			0x01f0

#define ICOLL_NUM_IRQS		64

static void __iomem *icoll_base;
static struct irq_domain *icoll_domain;

static void asm9260_init_icall(void)
{
	unsigned int i;

	/* IRQ enable,RISC32_RSE_MODE */
	__raw_writel(0x5 << 16, icoll_base + HW_ICOLL_CTRL);

	/* set irq_table addr */
	__raw_writel(0x0, icoll_base + HW_ICOLL_VBASE);

	/* set irq_table addr */
	__raw_writel(0xffff0000, icoll_base + HW_ICOLL_UNDEF_VECTOR);

	for (i = 0; i < 16 * 0x10; i += 0x10)
		__raw_writel(0x00000000, icoll_base + HW_ICOLL_PRIORITYn(0) + i);

	/* set timer 0 priority level high */
        __raw_writel(0x00000300, icoll_base + HW_ICOLL_PRIORITYn(0) + 0x70);
}

static unsigned int irq_get_level(struct irq_data *d)
{
	unsigned int tmp;

	tmp = __raw_readl(icoll_base + HW_ICOLL_INTERRUPTn_SET(d->hwirq));
	return (tmp >> BM_ICOLL_INTERRUPTn_SHIFT(d->hwirq)) & 0x3;
}

static void icoll_ack_irq(struct irq_data *d)
{
	__raw_readl(icoll_base + HW_ICOLL_VECTOR);
}

static void icoll_mask_irq(struct irq_data *d)
{
	__raw_writel(BM_ICOLL_INTERRUPTn_ENABLE(d->hwirq),
			icoll_base + HW_ICOLL_INTERRUPTn_CLR(d->hwirq));
}

static void icoll_unmask_irq(struct irq_data *d)
{
	u32 level;

	__raw_writel((1 << (d->hwirq & 0x1f)),
			icoll_base + HW_ICOLL_CLEAR0 + ((d->hwirq >> 5) * 0x10) + 4);

	level = irq_get_level(d);
	__raw_writel(BM_LEVELn(level), icoll_base + HW_ICOLL_LEVELACK);

	__raw_writel(BM_ICOLL_INTERRUPTn_ENABLE(d->hwirq),
			icoll_base + HW_ICOLL_INTERRUPTn_SET(d->hwirq));
}

static struct irq_chip asm9260_icoll_chip = {
	.irq_ack = icoll_ack_irq,
	.irq_mask = icoll_mask_irq,
	.irq_unmask = icoll_unmask_irq,
};

#if 0
asmlinkage void __exception_irq_entry icoll_handle_irq(struct pt_regs *regs)
{
	u32 irqnr;

	irqnr = __raw_readl(icoll_base + HW_ICOLL_STAT_OFFSET);
	__raw_writel(irqnr, icoll_base + HW_ICOLL_VECTOR);
	irqnr = irq_find_mapping(icoll_domain, irqnr);

	handle_IRQ(irqnr, regs);
}
#endif

static int icoll_irq_domain_map(struct irq_domain *d, unsigned int virq,
				irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq, &asm9260_icoll_chip, handle_level_irq);
	set_irq_flags(virq, IRQF_VALID);

	return 0;
}

static struct irq_domain_ops icoll_irq_domain_ops = {
	.map = icoll_irq_domain_map,
	.xlate = irq_domain_xlate_onecell,
};

static int __init icoll_of_init(struct device_node *np,
			  struct device_node *interrupt_parent)
{
	icoll_base = of_iomap(np, 0);
	WARN_ON(!icoll_base);

	/*
	 * Interrupt Collector reset, which initializes the priority
	 * for each irq to level 0.
	 */
	stmp_reset_block(icoll_base + HW_ICOLL_CTRL);

	asm9260_init_icall();

	icoll_domain = irq_domain_add_linear(np, ICOLL_NUM_IRQS,
					     &icoll_irq_domain_ops, NULL);

	irq_set_default_host(icoll_domain);

	return icoll_domain ? 0 : -ENODEV;
}
IRQCHIP_DECLARE(mxs, "alpscale,asm9260-icall", icoll_of_init);
