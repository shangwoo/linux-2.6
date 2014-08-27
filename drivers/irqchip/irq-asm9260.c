/*
 * Copyright (C) 2014 Oleksij Rempel <linux@rempel-privat.de>
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
#include <linux/delay.h>

#include "irqchip.h"

#define SET_REG 4
#define CLR_REG 8

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
/* The Interrupt Collector Level Acknowledge Register is used by software to
 * indicate the completion of an interrupt on a specific level.
 * This register is written at the very end of an interrupt service routine. If
 * nesting is used then the CPU irq must be turned on before writing to this
 * register to avoid a race condition in the CPU interrupt hardware.
 */
#define BM_LEVELn(nr)				BIT(nr)

#define HW_ICOLL_CTRL				0x0020
#define BM_CTRL_SFTRST				BIT(31)
#define BM_CTRL_CLKGATE				BIT(30)
#define BM_CTRL_NO_NESTING			BIT(19)
/* disable interrupt level nesting */
#define BM_CTRL_ARM_RSE_MODE			BIT(18)
/* Set this bit to one enable the RISC32-style read side effect associated with
 * the vector address register. In this mode, interrupt in-service is signaled
 * by the read of the HW_ICOLL_VECTOR register to acquire the interrupt vector
 * address. Set this bit to zero for normal operation, in which the ISR signals
 * in-service explicitly by means of a write to the HW_ICOLL_VECTOR register.
 * 0 - Must Write to Vector register to go in-service.
 * 1 - Go in-service as a read side effect
 */
#define BM_CTRL_IRQ_ENABLE			BIT(16)

#define HW_ICOLL_STAT_OFFSET			0x0030
/* bits 5:0
 * Vector number of current interrupt. Multiply by 4 and add to vector base
 * address to obtain the value in HW_ICOLL_VECTOR.
 */

#define HW_ICOLL_RAW0				0x0040
#define HW_ICOLL_RAW1				0x0050
/* This register provides a read-only view of the raw interrupt request lines
 * coming from various parts of the chip. Its purpose is to improve diagnostic
 * observability.
 */

#define	HW_ICOLL_INTERRUPTn(n)			(0x0060 + ((n) >> 2) * 0x10)
#define	HW_ICOLL_INTERRUPTn_SET(n)		(HW_ICOLL_INTERRUPTn(n) \
		+ SET_REG)
#define	HW_ICOLL_INTERRUPTn_CLR(n)		(HW_ICOLL_INTERRUPTn(n) \
		+ CLR_REG)
#define BM_INT_PRIORITY_MASK			0x3
/* WARNING: Modifying the priority of an enabled interrupt may result in
 * undefined behavior. */
#define BM_INT_ENABLE				BIT(2)
#define BM_INT_SOFTIRQ				BIT(3)

#define BM_ICOLL_INTERRUPTn_SHIFT(n)		(((n) & 0x3) << 3)
#define BM_ICOLL_INTERRUPTn_ENABLE(n)		(1 << (2 + \
			BM_ICOLL_INTERRUPTn_SHIFT(n)))

#define HW_ICOLL_VBASE				0x0160
/* bits 31:2
 * This bitfield holds the upper 30 bits of the base address of the vector
 * table. */

#define HW_ICOLL_CLEAR0				0x01d0
#define	HW_ICOLL_CLEAR1				0x01e0
#define HW_ICOLL_CLEARn(n)			(0x01d0 + ((n >> 5) * 0x10) \
							+ SET_REG)
#define BM_CLEAR_BIT(n)				BIT(n & 0x1f)

#define HW_ICOLL_UNDEF_VECTOR			0x01f0
/* Scratchpad */

#define ICOLL_NUM_IRQS		64

static void __iomem *icoll_base;
static struct irq_domain *icoll_domain;
static int use_cached_level = 1;
static u8 level_cache[ICOLL_NUM_IRQS];

static unsigned int irq_get_level(struct irq_data *d)
{
	unsigned int tmp;

	if (use_cached_level)
		return level_cache[d->hwirq];

	tmp = readl_relaxed(icoll_base + HW_ICOLL_INTERRUPTn_SET(d->hwirq));
	return (tmp >> BM_ICOLL_INTERRUPTn_SHIFT(d->hwirq)) & 0x3;
}

static void irq_set_level(int hwirq, int level)
{
	if (unlikely(level < 0 || level > 3)) {
		pr_err("%s Wrong level (%i) for irq (%i)!", __func__, level,
				hwirq);
		return;
	}
	if (use_cached_level)
		level_cache[hwirq] = level;

	writel_relaxed(level << BM_ICOLL_INTERRUPTn_SHIFT(hwirq),
			icoll_base + HW_ICOLL_INTERRUPTn(hwirq));
}

static void icoll_ack_irq(struct irq_data *d)
{
	readl_relaxed(icoll_base + HW_ICOLL_VECTOR);
}

static void icoll_mask_irq(struct irq_data *d)
{
	writel_relaxed(BM_ICOLL_INTERRUPTn_ENABLE(d->hwirq),
			icoll_base + HW_ICOLL_INTERRUPTn_CLR(d->hwirq));
}

static void icoll_unmask_irq(struct irq_data *d)
{
	u32 level;

	writel_relaxed(BM_CLEAR_BIT(d->hwirq),
			icoll_base + HW_ICOLL_CLEARn(d->hwirq));

	level = irq_get_level(d);
	writel_relaxed(BM_LEVELn(level), icoll_base + HW_ICOLL_LEVELACK);

	writel_relaxed(BM_ICOLL_INTERRUPTn_ENABLE(d->hwirq),
			icoll_base + HW_ICOLL_INTERRUPTn_SET(d->hwirq));
}

static struct irq_chip asm9260_icoll_chip = {
	.irq_ack = icoll_ack_irq,
	.irq_mask = icoll_mask_irq,
	.irq_unmask = icoll_unmask_irq,
};

asmlinkage void __exception_irq_entry icoll_handle_irq(struct pt_regs *regs)
{
	u32 irqnr;

	irqnr = irq_find_mapping(icoll_domain,
			readl_relaxed(icoll_base + HW_ICOLL_STAT_OFFSET));

	handle_IRQ(irqnr, regs);
}

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
	struct resource res;

	of_address_to_resource(np, 0, &res);
	if (!request_mem_region(res.start, resource_size(&res), np->name))
		panic("%s: unable to request mem region", np->name);

	icoll_base = ioremap(res.start, resource_size(&res));
	if (!icoll_base)
		panic("%s: unable to map resource", np->name);

	/*
	 * Interrupt Collector reset, which initializes the priority
	 * for each irq to level 0.
	 */
	writel(BM_CTRL_CLKGATE, icoll_base + HW_ICOLL_CTRL + CLR_REG);

	writel(BM_CTRL_SFTRST, icoll_base + HW_ICOLL_CTRL + SET_REG);
	udelay(100);

	writel(BM_CTRL_SFTRST, icoll_base + HW_ICOLL_CTRL + CLR_REG);
	udelay(100);

	/* enable IRQ controller */
	writel_relaxed(BM_CTRL_ARM_RSE_MODE | BM_CTRL_IRQ_ENABLE,
			icoll_base + HW_ICOLL_CTRL);

	/* set timer 0 priority level high. TODO: should be done by DT  */
	irq_set_level(29, 3);

	icoll_domain = irq_domain_add_linear(np, ICOLL_NUM_IRQS,
					     &icoll_irq_domain_ops, NULL);

	irq_set_default_host(icoll_domain);

	set_handle_irq(icoll_handle_irq);

	return icoll_domain ? 0 : -ENODEV;
}
IRQCHIP_DECLARE(asm9260, "alpscale,asm9260-icall", icoll_of_init);
