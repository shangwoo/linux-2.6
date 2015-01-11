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
#include <linux/clk.h>
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
#define BM_WDTC_MIN(freq)		((0xff + 1) / (freq))
#define BM_WDTC_MAX(freq)		(0x7fffffff / (freq))

/* Watchdog Feed register */
#define HW_WDFEED			0x08

/* Watchdog Timer Value register */
#define HW_WDTV				0x0c

#define ASM9260_WDT_DEFAULT_TIMEOUT	30		/* 30 secs */

static unsigned int timeout = ASM9260_WDT_DEFAULT_TIMEOUT;
static bool nowayout = WATCHDOG_NOWAYOUT;

module_param(timeout, uint, 0);
module_param(nowayout, bool, 0);

MODULE_PARM_DESC(timeout, "Default watchdog timeout (in seconds)");
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct asm9260_wdt_priv {
	struct device		*dev;
	struct watchdog_device	wdd;
	void __iomem		*iobase;

	struct clk		*clk;
	struct clk		*clk_ahb;
	unsigned long		wdt_freq;
};

static int asm9260_wdt_feed(struct watchdog_device *wdd)
{
	struct asm9260_wdt_priv *priv = watchdog_get_drvdata(wdd);

	iowrite32(0xaa, priv->iobase + HW_WDFEED);
	iowrite32(0x55, priv->iobase + HW_WDFEED);

	return 0;
}

static unsigned int asm9260_wdt_gettimeleft(struct watchdog_device *wdd)
{
	struct asm9260_wdt_priv *priv = watchdog_get_drvdata(wdd);
	u32 counter;

	counter = ioread32(priv->iobase + HW_WDTV);

	return DIV_ROUND_CLOSEST(counter, priv->wdt_freq);
}

static int asm9260_wdt_updatetimeout(struct watchdog_device *wdd)
{
	struct asm9260_wdt_priv *priv = watchdog_get_drvdata(wdd);
	u32 counter;

	counter = wdd->timeout * priv->wdt_freq;

	iowrite32(counter, priv->iobase + HW_WDTC);

	return 0;
}

static int asm9260_wdt_enable(struct watchdog_device *wdd)
{
	struct asm9260_wdt_priv *priv = watchdog_get_drvdata(wdd);

	iowrite32(BM_MOD_WDEN | BM_MOD_WDRESET, priv->iobase + HW_WDMOD);

	asm9260_wdt_updatetimeout(wdd);

	asm9260_wdt_feed(wdd);

	return 0;
}

static int asm9260_wdt_disable(struct watchdog_device *wdd)
{
	struct asm9260_wdt_priv *priv = watchdog_get_drvdata(wdd);

	iowrite32(BM_MOD_WDEN | BM_MOD_WDRESET, priv->iobase + HW_WDMOD);
	/* FIXME: disable clocks here? */
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
	.identity         =	"Alphascale asm9260 Watchdog",
};

static struct watchdog_ops asm9260_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= asm9260_wdt_enable,
	.stop		= asm9260_wdt_disable,
	.get_timeleft	= asm9260_wdt_gettimeleft,
	.ping		= asm9260_wdt_feed,
	.set_timeout	= asm9260_wdt_settimeout,
};

static int __init asm9260_wdt_get_dt_clks(struct asm9260_wdt_priv *priv)
{
	int clk_idx = 0, err;

	priv->clk = devm_clk_get(priv->dev, "mod");
	if (IS_ERR(priv->clk))
		goto out_err;

	/* configure AHB clock */
	clk_idx = 1;
	priv->clk_ahb = devm_clk_get(priv->dev, "ahb");
	if (IS_ERR(priv->clk_ahb))
		goto out_err;

	err = clk_prepare_enable(priv->clk_ahb);
	if (err)
		dev_err(priv->dev, "Failed to enable ahb_clk!\n");

	err = clk_set_rate(priv->clk, CLOCK_FREQ);
	if (err) {
		clk_disable_unprepare(priv->clk_ahb);
		dev_err(priv->dev, "Failed to set rate!\n");
	}

	err = clk_prepare_enable(priv->clk);
	if (err) {
		clk_disable_unprepare(priv->clk_ahb);
		dev_err(priv->dev, "Failed to enable clk!\n");
	}

	/* wdt has internal divider */
	priv->wdt_freq = clk_get_rate(priv->clk) / 4;

	return 0;
out_err:
	dev_err(priv->dev, "%s: Failed to get clk (%i)\n", __func__, clk_idx);
	return 1;
}

static int asm9260_wdt_probe(struct platform_device *pdev)
{
	struct asm9260_wdt_priv *priv;
	struct watchdog_device *wdd;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct asm9260_wdt_priv),
			GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->iobase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->iobase))
		return PTR_ERR(priv->iobase);

	ret = asm9260_wdt_get_dt_clks(priv);
	if (ret)
		return ret;

	wdd = &priv->wdd;
	wdd->info = &asm9260_wdt_ident;
	wdd->ops = &asm9260_wdt_ops;
	wdd->timeout = ASM9260_WDT_DEFAULT_TIMEOUT;
	wdd->min_timeout = BM_WDTC_MIN(priv->wdt_freq);
	wdd->max_timeout = BM_WDTC_MAX(priv->wdt_freq);

	watchdog_set_drvdata(wdd, priv);

	watchdog_init_timeout(wdd, timeout, &pdev->dev);
	watchdog_set_nowayout(wdd, nowayout);

	ret = watchdog_register_device(wdd);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "initialized.\n");
	return 0;
}

static void asm9260_wdt_shutdown(struct platform_device *pdev)
{
	struct asm9260_wdt_priv *priv = platform_get_drvdata(pdev);

	asm9260_wdt_disable(&priv->wdd);
}

static int asm9260_wdt_remove(struct platform_device *pdev)
{
	struct asm9260_wdt_priv *priv = platform_get_drvdata(pdev);

	asm9260_wdt_shutdown(pdev);
	clk_disable_unprepare(priv->clk);
	clk_disable_unprepare(priv->clk_ahb);

	return 0;
}

#ifdef	CONFIG_PM_SLEEP
static int asm9260_wdt_suspend(struct device *dev)
{
	return 0;
}

static int asm9260_wdt_resume(struct device *dev)
{
	struct asm9260_wdt_priv *priv = dev_get_drvdata(dev);

	/*
	 * FIXME: need this?
	 * NOTE: Since timer controller registers settings are saved
	 * and restored back by the timer-prima2.c, so we need not
	 * update WD settings except refreshing timeout.
	 */
	asm9260_wdt_updatetimeout(&priv->wdd);

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
