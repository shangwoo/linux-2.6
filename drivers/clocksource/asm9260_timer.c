/*
 * Copyright (C) 2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/bitops.h>

#define SET_REG 4
#define CLR_REG 8

#define HW_IR           0x0000 /* RW. Interrupt */
#define BM_IR_CR0	BIT(4)
#define BM_IR_MR3	BIT(3)
#define BM_IR_MR2	BIT(2)
#define BM_IR_MR1	BIT(1)
#define BM_IR_MR0	BIT(0)

#define HW_TCR		0x0010 /* RW. Timer controller */
/* BM_C*_RST
 * Timer Counter and the Prescale Counter are synchronously reset on the
 * next positive edge of PCLK. The counters remain reset until TCR[1] is
 * returned to zero. */
#define BM_C3_RST	BIT(7)
#define BM_C2_RST	BIT(6)
#define BM_C1_RST	BIT(5)
#define BM_C0_RST	BIT(4)
/* BM_C*_EN
 * 1 - Timer Counter and Prescale Counter are enabled for counting
 * 0 - counters are disabled */
#define BM_C3_EN	BIT(3)
#define BM_C2_EN	BIT(2)
#define BM_C1_EN	BIT(1)
#define BM_C0_EN	BIT(0)

#define HW_DIR		0x0020 /* RW. Direction? */
/* 00 - count up
 * 01 - count down 
 * 10 - ?? 2^n/2 */
#define BM_DIR0_SHIFT	0
#define BM_DIR1_SHIFT	4
#define BM_DIR2_SHIFT	8
#define BM_DIR3_SHIFT	12

#define HW_TC0		0x0030 /* RO. Timer counter 0 */
/* HW_TC*. Timer counter owerflow (0xffff.ffff to 0x0000.0000) do not generate
 * interrupt. This registers can be used to detect overflow */
#define HW_TC1          0x0040
#define HW_TC2		0x0050
#define HW_TC3		0x0060

#define HW_PR		0x0070 /* RW. prescaler */
#define HW_PC		0x0080 /* RO. Prescaler counter */
#define HW_MCR		0x0090 /* RW. Match control */
#define BM_MCR_INT_EN(n)	(1 << (n * 3 + 0)) /* enable interrupt on match */
#define BM_MCR_RES_EN(n)	(1 << (n * 3 + 1)) /* enable TC reset on match */
#define BM_MCR_STOP_EN(n)	(1 << (n * 3 + 2)) /* enable stop TC on match */

#define HW_MR0		0x00a0 /* RW. Match reg */
#define HW_MR1		0x00b0
#define HW_MR2		0x00C0
#define HW_MR3		0x00D0
#define HW_CCR		0x00E0 /* RW. Capture control */
#define HW_CR0		0x00F0 /* RO. Capture reg */
#define HW_CR1		0x0100
#define HW_CR2		0x0110
#define HW_CR3		0x0120
#define HW_EMR		0x0130 /* RW. External Match */
#define HW_PWMTH0	0x0140 /* RW. PWM width */
#define HW_PWMTH1	0x0150
#define HW_PWMTH2	0x0160
#define HW_PWMTH3	0x0170
#define HW_CTCR		0x0180 /* Counter control */
#define HW_PWMC		0x0190 /* PWM control */

static void __iomem *base;
static unsigned long ticks_per_jiffy;

static int asm9260_timer_set_next_event(unsigned long delta,
					 struct clock_event_device *dev)
{
	/* configure match count for TC0 */
	writel_relaxed(delta, base + HW_MR0);
	/* enable TC0 */
	writel_relaxed(BM_C0_EN, base + HW_TCR + SET_REG);
	return 0;
}

static void asm9260_timer_set_mode(enum clock_event_mode mode,
				    struct clock_event_device *evt)
{
	/* stop timer0 */
	writel_relaxed(BM_C0_EN, base + HW_TCR + CLR_REG);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		/* disable reset and stop on match */
		writel_relaxed(BM_MCR_RES_EN(0) | BM_MCR_STOP_EN(0),
				base + HW_MCR + CLR_REG);
		/* configure match count for TC0 */
		writel_relaxed(ticks_per_jiffy, base + HW_MR0);
		/* enable TC0 */
		writel_relaxed(BM_C0_EN, base + HW_TCR + SET_REG);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* enable reset and stop on match */
		writel_relaxed(BM_MCR_RES_EN(0) | BM_MCR_STOP_EN(0),
				base + HW_MCR + SET_REG);
		break;
	default:
		break;
	}
}

static struct clock_event_device asm9260_clockevent_device = {
	.name		= "asm9260-clockevent-dev",
	.rating		= 200,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_next_event	= asm9260_timer_set_next_event,
	.set_mode	= asm9260_timer_set_mode,
};

static irqreturn_t asm9260_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &asm9260_clockevent_device;

	evt->event_handler(evt);

	writel_relaxed(BM_IR_MR0, base + HW_IR);

	return IRQ_HANDLED;
}

static struct irqaction asm9260_timer_irq = {
	.name		= "asm9260-clockevent-dev",
	.flags		= IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= asm9260_timer_interrupt,
};

/*
 * ---------------------------------------------------------------------------
 * Timer initialization
 * ---------------------------------------------------------------------------
 */

static void __init asm9260_clockevent_init(struct clk *clk)
{
	unsigned long rate = clk_get_rate(clk);

	ticks_per_jiffy = DIV_ROUND_CLOSEST(rate, HZ);
	asm9260_clockevent_device.cpumask = cpumask_of(0);
	clockevents_config_and_register(&asm9260_clockevent_device,
					rate, 0x2c00, 0xfffffffe);
}

static void __init asm9260_clocksource_init(struct clk *clk)
{
	unsigned long hz = clk_get_rate(clk);

	clocksource_mmio_init(base + HW_TC1,
			"asm9260-clocksource", hz,
			200, 32, clocksource_mmio_readl_up);

	writel_relaxed(0xffffffff, base + HW_MR1);
	writel_relaxed(BM_C1_EN, base + HW_TCR + SET_REG);
}

static void __init asm9260_timer_init(struct device_node *np)
{
	int irq;
	struct clk *clk;
	struct resource res;
	int ret;

	of_address_to_resource(np, 0, &res);
	if (!request_mem_region(res.start, resource_size(&res), "asm9260-timer"))
		panic("%s: unable to request mem region", np->name);

	base = ioremap_nocache(res.start, resource_size(&res));
	if (!base)
		panic("%s: unable to map resource", np->name);

	clk = of_clk_get(np, 0);

	ret = clk_prepare_enable(clk);
	if (ret)
		panic("Failed to enable clk!\n");

	irq = irq_of_parse_and_map(np, 0);
	setup_irq(irq, &asm9260_timer_irq);

	/* set all timers for count-up */
	writel_relaxed(0, base + HW_DIR);
	/* disbale devider */
	writel_relaxed(0, base + HW_PR);
	/* make sure all timers use every rising PCLK edge. */
	writel_relaxed(0, base + HW_CTCR);
	/* enable interrupt for TC0 and clean setting for all other lines */
	writel_relaxed(BM_MCR_INT_EN(0) , base + HW_MCR);

	asm9260_clocksource_init(clk);
	asm9260_clockevent_init(clk);
}
CLOCKSOURCE_OF_DECLARE(asm9260_timer, "alpscale,asm9260-timer",
		asm9260_timer_init);
