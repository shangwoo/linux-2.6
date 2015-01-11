/*
 * Watchdog driver for Alphascale ASM9260. Same WD can be found on NXP LPC176x.
 *
 * Copyright (c) 2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#define CLOCK_FREQ	1000000

/* Watchdog Mode register */
#define HW_WDMOD			0x00
#define BM_MOD_WDINT			BIT(3)
#define BM_MOD_WDTOF			BIT(2)
#define BM_MOD_WDRESET			BIT(1)
#define BM_MOD_WDEN			BIT(0)

/*
 * Watchdog Timer Constant register
 * Minimal value is 0xff, the meaning of this value
 * depends on used clock: T = WDCLK * (0xff + 1) * 4
 */
#define HW_WDTC				0x04
#define HW_WDTC				0x04

/* Watchdog Feed register */
#define HW_WDFEED			0x08

/* Watchdog Timer Value register */
#define HW_WDTV				0x0c



#define SIRFSOC_TIMER_COUNTER_LO	0x0000
#define SIRFSOC_TIMER_MATCH_0		0x0008
#define SIRFSOC_TIMER_INT_EN		0x0024
#define SIRFSOC_TIMER_WATCHDOG_EN	0x0028
#define SIRFSOC_TIMER_LATCH		0x0030
#define SIRFSOC_TIMER_LATCHED_LO	0x0034

#define SIRFSOC_TIMER_WDT_INDEX		5

#define SIRFSOC_WDT_MIN_TIMEOUT		30		/* 30 secs */
#define SIRFSOC_WDT_MAX_TIMEOUT		(10 * 60)	/* 10 mins */
#define SIRFSOC_WDT_DEFAULT_TIMEOUT	30		/* 30 secs */

static unsigned int timeout = SIRFSOC_WDT_DEFAULT_TIMEOUT;
static bool nowayout = WATCHDOG_NOWAYOUT;

module_param(timeout, uint, 0);
module_param(nowayout, bool, 0);

MODULE_PARM_DESC(timeout, "Default watchdog timeout (in seconds)");
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int asm9260_wdt_gettimeleft(struct watchdog_device *wdd)
{
	u32 counter, match;
	void __iomem *wdt_base;
	int time_left;

	wdt_base = watchdog_get_drvdata(wdd);
	counter = ioread32(wdt_base + SIRFSOC_TIMER_COUNTER_LO);
	match = ioread32(wdt_base +
		SIRFSOC_TIMER_MATCH_0 + (SIRFSOC_TIMER_WDT_INDEX << 2));

	time_left = match - counter;

	return time_left / CLOCK_FREQ;
}

static int asm9260_wdt_updatetimeout(struct watchdog_device *wdd)
{
	u32 counter, timeout_ticks;
	void __iomem *wdt_base;

	timeout_ticks = wdd->timeout * CLOCK_FREQ;
	wdt_base = watchdog_get_drvdata(wdd);

	/* Enable the latch before reading the LATCH_LO register */
	iowrite32(1, wdt_base + SIRFSOC_TIMER_LATCH);

	/* Set the TO value */
	counter = ioread32(wdt_base + SIRFSOC_TIMER_LATCHED_LO);

	counter += timeout_ticks;

	iowrite32(counter, wdt_base +
		SIRFSOC_TIMER_MATCH_0 + (SIRFSOC_TIMER_WDT_INDEX << 2));

	return 0;
}

static int asm9260_wdt_enable(struct watchdog_device *wdd)
{
	void __iomem *wdt_base = watchdog_get_drvdata(wdd);
	asm9260_wdt_updatetimeout(wdd);

	/*
	 * NOTE: If interrupt is not enabled
	 * then WD-Reset doesn't get generated at all.
	 */
	iowrite32(ioread32(wdt_base + SIRFSOC_TIMER_INT_EN)
		| (1 << SIRFSOC_TIMER_WDT_INDEX),
		wdt_base + SIRFSOC_TIMER_INT_EN);
	iowrite32(1, wdt_base + SIRFSOC_TIMER_WATCHDOG_EN);

	return 0;
}

static int asm9260_wdt_disable(struct watchdog_device *wdd)
{
	void __iomem *wdt_base = watchdog_get_drvdata(wdd);

	iowrite32(0, wdt_base + SIRFSOC_TIMER_WATCHDOG_EN);
	iowrite32(ioread32(wdt_base + SIRFSOC_TIMER_INT_EN)
		& (~(1 << SIRFSOC_TIMER_WDT_INDEX)),
		wdt_base + SIRFSOC_TIMER_INT_EN);

	return 0;
}

static int asm9260_wdt_settimeout(struct watchdog_device *wdd, unsigned int to)
{
	wdd->timeout = to;
	asm9260_wdt_updatetimeout(wdd);

	return 0;
}

#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE)

static const struct watchdog_info asm9260_wdt_ident = {
	.options          =     OPTIONS,
	.firmware_version =	0,
	.identity         =	"SiRFSOC Watchdog",
};

static struct watchdog_ops asm9260_wdt_ops = {
	.owner = THIS_MODULE,
	.start = asm9260_wdt_enable,
	.stop = asm9260_wdt_disable,
	.get_timeleft = asm9260_wdt_gettimeleft,
	.ping = asm9260_wdt_updatetimeout,
	.set_timeout = asm9260_wdt_settimeout,
};

static struct watchdog_device asm9260_wdd = {
	.info = &asm9260_wdt_ident,
	.ops = &asm9260_wdt_ops,
	.timeout = SIRFSOC_WDT_DEFAULT_TIMEOUT,
	.min_timeout = SIRFSOC_WDT_MIN_TIMEOUT,
	.max_timeout = SIRFSOC_WDT_MAX_TIMEOUT,
};

static int asm9260_wdt_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	void __iomem *base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	watchdog_set_drvdata(&asm9260_wdd, base);

	watchdog_init_timeout(&asm9260_wdd, timeout, &pdev->dev);
	watchdog_set_nowayout(&asm9260_wdd, nowayout);

	ret = watchdog_register_device(&asm9260_wdd);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, &asm9260_wdd);

	return 0;
}

static void asm9260_wdt_shutdown(struct platform_device *pdev)
{
	struct watchdog_device *wdd = platform_get_drvdata(pdev);

	asm9260_wdt_disable(wdd);
}

static int asm9260_wdt_remove(struct platform_device *pdev)
{
	asm9260_wdt_shutdown(pdev);
	return 0;
}

#ifdef	CONFIG_PM_SLEEP
static int asm9260_wdt_suspend(struct device *dev)
{
	return 0;
}

static int asm9260_wdt_resume(struct device *dev)
{
	struct watchdog_device *wdd = dev_get_drvdata(dev);

	/*
	 * NOTE: Since timer controller registers settings are saved
	 * and restored back by the timer-prima2.c, so we need not
	 * update WD settings except refreshing timeout.
	 */
	asm9260_wdt_updatetimeout(wdd);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(asm9260_wdt_pm_ops,
		asm9260_wdt_suspend, asm9260_wdt_resume);

static const struct of_device_id asm9260_wdt_of_match[] = {
	{ .compatible = "alphascale,asm9260-wdt"},
	{},
};
MODULE_DEVICE_TABLE(of, asm9260_wdt_of_match);

static struct platform_driver asm9260_wdt_driver = {
	.driver = {
		.name = "asm9260-wdt",
		.owner = THIS_MODULE,
		.pm = &asm9260_wdt_pm_ops,
		.of_match_table	= asm9260_wdt_of_match,
	},
	.probe = asm9260_wdt_probe,
	.remove = asm9260_wdt_remove,
	.shutdown = asm9260_wdt_shutdown,
};
module_platform_driver(asm9260_wdt_driver);

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_LICENSE("GPL v2");
