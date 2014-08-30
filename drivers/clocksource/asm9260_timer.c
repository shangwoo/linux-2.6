/*
 * Copyright (C) 2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/bitops.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

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
#define HW_MR0		0x00a0 /* RW. Match reg */
#define HW_MR1		0x00b0
#define HW_MR2		0x00C0
#define HW_MR3		0x00D0
#define HW_CCR		0x00E0 /* RW. Capture control */
#define HW_CR0		0x00F0 /* RO. Capture reg */
#define HW_CR1               0x0100
#define HW_CR2               0x0110
#define HW_CR3               0x0120
#define HW_EMR               0x0130 /* RW. External Match */
#define HW_PWMTH0            0x0140 /* RW. PWM width */
#define HW_PWMTH1            0x0150
#define HW_PWMTH2            0x0160
#define HW_PWMTH3            0x0170
#define HW_CTCR              0x0180 /* Counter control */
#define HW_PWMC              0x0190 /* PWM control */


/*
 * ---------------------------------------------------------------------------
 * 32KHz OS timer
 *
 * This currently works only on 16xx, as 1510 does not have the continuous
 * 32KHz synchronous timer. The 32KHz synchronous timer is used to keep track
 * of time in addition to the 32KHz OS timer. Using only the 32KHz OS timer
 * on 1510 would be possible, but the timer would not be as accurate as
 * with the 32KHz synchronized timer.
 * ---------------------------------------------------------------------------
 */

/* 16xx specific defines */
#define OMAP1_32K_TIMER_BASE		0xfffb9000
#define OMAP1_32KSYNC_TIMER_BASE	0xfffbc400
#define OMAP1_32K_TIMER_CR		0x08
#define OMAP1_32K_TIMER_TVR		0x00
#define OMAP1_32K_TIMER_TCR		0x04

#define OMAP_32K_TICKS_PER_SEC		(32768)

/*
 * TRM says 1 / HZ = ( TVR + 1) / 32768, so TRV = (32768 / HZ) - 1
 * so with HZ = 128, TVR = 255.
 */
#define OMAP_32K_TIMER_TICK_PERIOD	((OMAP_32K_TICKS_PER_SEC / HZ) - 1)

#define JIFFIES_TO_HW_TICKS(nr_jiffies, clock_rate)			\
				(((nr_jiffies) * (clock_rate)) / HZ)

static void __iomem *timer_base;

static inline void omap_32k_timer_write(int val, int reg)
{
	omap_writew(val, OMAP1_32K_TIMER_BASE + reg);
}

static inline unsigned long omap_32k_timer_read(int reg)
{
	return omap_readl(OMAP1_32K_TIMER_BASE + reg) & 0xffffff;
}

static inline void omap_32k_timer_start(unsigned long load_val)
{
	if (!load_val)
		load_val = 1;
	omap_32k_timer_write(load_val, OMAP1_32K_TIMER_TVR);
	omap_32k_timer_write(0x0f, OMAP1_32K_TIMER_CR);
}

static inline void omap_32k_timer_stop(void)
{
	omap_32k_timer_write(0x0, OMAP1_32K_TIMER_CR);
}

static int omap_32k_timer_set_next_event(unsigned long delta,
					 struct clock_event_device *dev)
{
	omap_32k_timer_start(delta);

	return 0;
}

static void omap_32k_timer_set_mode(enum clock_event_mode mode,
				    struct clock_event_device *evt)
{
	omap_32k_timer_stop();

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		omap_32k_timer_start(OMAP_32K_TIMER_TICK_PERIOD);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device clockevent_32k_timer = {
	.name		= "32k-timer",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_next_event	= omap_32k_timer_set_next_event,
	.set_mode	= omap_32k_timer_set_mode,
};

static irqreturn_t omap_32k_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_32k_timer;

	/* TODO: need to ack timer register */

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction omap_32k_timer_irq = {
	.name		= "32KHz timer",
	.flags		= IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= omap_32k_timer_interrupt,
};

static __init void omap_init_32k_timer(void)
{
	setup_irq(INT_OS_TIMER, &omap_32k_timer_irq);

	clockevent_32k_timer.cpumask = cpumask_of(0);
	clockevents_config_and_register(&clockevent_32k_timer,
					OMAP_32K_TICKS_PER_SEC, 1, 0xfffffffe);
}

/*
 * ---------------------------------------------------------------------------
 * Timer initialization
 * ---------------------------------------------------------------------------
 */
int __init omap_32k_timer_init(void)
{
	int ret = -ENODEV;

	of_address_to_resource(np, 0, &res);
	if (!request_mem_region(res.start, resource_size(&res), np->name))
		panic("%s: unable to request mem region", np->name);

	icoll_base = ioremap_nocache(res.start, resource_size(&res));
	if (!icoll_base)
		panic("%s: unable to map resource", np->name);

	clk = of_clk_get(np, 0);

	err = clk_prepare_enable(port->clk_ahb);
	if (err)
		dev_err(uport->dev, "Failed to enable ahb_clk!\n");

	ret = omap_init_clocksource_32k(base);

	if (!ret)
		omap_init_32k_timer();

	return ret;
}
