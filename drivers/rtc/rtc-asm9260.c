/*
 * Copyright (C) 2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/clk.h>

/* Miscellaneous registers */
/* Interrupt Location Register */
#define HW_ILR			0x00
#define BM_RTCALF		BIT(1)
#define BM_RTCCIF		BIT(0)

/* Clock Control Register */
#define HW_CCR			0x08
/* Calibration counter disable */
#define BM_CCALOFF		BIT(4)
/* Reset internal oscillator divider */
#define BM_CTCRST		BIT(1)
/* Clock Enable */
#define BM_CLKEN		BIT(0)

/* Counter Increment Interrupt Register */
#define HW_CIIR			0x0C
#define BM_CIIR_IMYEAR		BIT(7)
#define BM_CIIR_IMMON		BIT(6)
#define BM_CIIR_IMDOY		BIT(5)
#define BM_CIIR_IMDOW		BIT(4)
#define BM_CIIR_IMDOM		BIT(3)
#define BM_CIIR_IMHOUR		BIT(2)
#define BM_CIIR_IMMIN		BIT(1)
#define BM_CIIR_IMSEC		BIT(0)

/* Alarm Mask Register */
#define HW_AMR			0x10
#define BM_AMR_IMYEAR		BIT(7)
#define BM_AMR_IMMON		BIT(6)
#define BM_AMR_IMDOY		BIT(5)
#define BM_AMR_IMDOW		BIT(4)
#define BM_AMR_IMDOM		BIT(3)
#define BM_AMR_IMHOUR		BIT(2)
#define BM_AMR_IMMIN		BIT(1)
#define BM_AMR_IMSEC		BIT(0)
#define BM_AMR_OFF		0xff

/* Consolidated time registers */
#define HW_CTIME0		0x14
#define BM_CTIME0_DOW_S		24
#define BM_CTIME0_DOW_M		0x7
#define BM_CTIME0_HOUR_S	16
#define BM_CTIME0_HOUR_M	0x1f
#define BM_CTIME0_MIN_S		8
#define BM_CTIME0_MIN_M		0x3f
#define BM_CTIME0_SEC_S		0
#define BM_CTIME0_SEC_M		0x3f

#define HW_CTIME1		0x18
#define BM_CTIME1_YEAR_S	16
#define BM_CTIME1_YEAR_M	0xfff
#define BM_CTIME1_MON_S		8
#define BM_CTIME1_MON_M		0xf
#define BM_CTIME1_DOM_S		0
#define BM_CTIME1_DOM_M		0x1f

#define HW_CTIME2		0x1C
#define BM_CTIME2_DOY_S		0
#define BM_CTIME2_DOY_M		0xfff

/* Time counter registers */
#define HW_SEC			0x20
#define HW_MIN			0x24
#define HW_HOUR			0x28
#define HW_DOM			0x2C
#define HW_DOW			0x30
#define HW_DOY			0x34
#define HW_MONTH		0x38
#define HW_YEAR			0x3C
#define HW_CALIBRATION		0x40

/* General purpose registers */
/* We can use one of this registers to detect if battery was removed/off
 * so far we in */
#define HW_GPREG0		0x44
#define BM_GPREG0_MAGIC		0x12345678

#define HW_GPREG1		0x48
#define HW_GPREG2		0x4C
#define HW_GPREG3		0x50
#define HW_GPREG4		0x54

/* RTC Auxiliary control register */
#define HW_RTC_AUX		0x5C
/*
 * RTC Oscillator Fail detect flag.
 * Read: this bit is set if the RTC oscillator stops, and when RTC power is
 * first turned on. An interrupt will occur when this bit is set, the
 * RTC_OSCFEN bit in RTC_AUXEN is a 1, and the RTC interrupt is enabled
 * in the NVIC.
 * Write: writing a 1 to this bit clears the flag.
 */
#define BM_RTC_OSCF		BIT(4)
/* RTC Auxiliary Enable register */
#define HW_RTC_AUXEN		0x58
/* Oscillator Fail Detect interrupt enable. */
#define BM_RTC_OSCFEN		BIT(4)

/* Alarm register group */
#define HW_ALSEC		0x60
#define HW_ALMIN		0x64
#define HW_ALHOUR		0x68
#define HW_ALDOM		0x6C
#define HW_ALDOW		0x70
#define HW_ALDOY		0x74
#define HW_ALMON		0x78
#define HW_ALYEAR		0x7C

struct asm9260_rtc_priv {
	void __iomem		*iobase;
	int			irq_alarm;
	struct rtc_device	*rtc;
	spinlock_t		lock;		/* Protects this structure */
	struct clk		*clk;
};

#if 0
static irqreturn_t asm9260_rtc_irq(int irq, void *dev_id)
{
	struct asm9260_rtc_priv *asm9260_rtc_priv = dev_id;
	u32 isr;
	unsigned long events = 0;

	spin_lock(&asm9260_rtc_priv->lock);

	/* clear interrupt sources */
	isr = ioread32(asm9260_rtc_priv->iobase + VT8500_RTC_IS);
	iowrite32(isr, asm9260_rtc_priv->iobase + VT8500_RTC_IS);

	spin_unlock(&asm9260_rtc_priv->lock);

	if (isr & VT8500_RTC_IS_ALARM)
		events |= RTC_AF | RTC_IRQF;

	rtc_update_irq(asm9260_rtc_priv->rtc, 1, events);

	return IRQ_HANDLED;
}
#endif

static int asm9260_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct asm9260_rtc_priv *priv = dev_get_drvdata(dev);
	u32 ctime0, ctime1, ctime2;

	ctime0 = ioread32(priv->iobase + HW_CTIME0);
	ctime1 = ioread32(priv->iobase + HW_CTIME1);
	ctime2 = ioread32(priv->iobase + HW_CTIME2);

	if (ctime1 != ioread32(priv->iobase + HW_CTIME1)) {
		printk("... reread rtc\n");
		/*
		 * woops, counter flipped right now. Now we are safe
		 * to reread.
		 */
		ctime0 = ioread32(priv->iobase + HW_CTIME0);
		ctime1 = ioread32(priv->iobase + HW_CTIME1);
		ctime2 = ioread32(priv->iobase + HW_CTIME2);
	}

	tm->tm_sec  = (ctime0 >> BM_CTIME0_SEC_S)  & BM_CTIME0_SEC_M;
	tm->tm_min  = (ctime0 >> BM_CTIME0_MIN_S)  & BM_CTIME0_MIN_M;
	tm->tm_hour = (ctime0 >> BM_CTIME0_HOUR_S) & BM_CTIME0_HOUR_M;
	tm->tm_wday = (ctime0 >> BM_CTIME0_DOW_S)  & BM_CTIME0_DOW_M;

	tm->tm_mday = (ctime1 >> BM_CTIME1_DOM_S)  & BM_CTIME1_DOM_M;
	tm->tm_mon  = (ctime1 >> BM_CTIME1_MON_S)  & BM_CTIME1_MON_M;
	tm->tm_year = (ctime1 >> BM_CTIME1_YEAR_S) & BM_CTIME1_YEAR_M;

	tm->tm_yday = (ctime2 >> BM_CTIME2_DOY_S)  & BM_CTIME2_DOY_M;

	return 0;
}

static int asm9260_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct asm9260_rtc_priv *priv = dev_get_drvdata(dev);

	/*
	 * make sure SEC counter will not flip other counter on write time,
	 * real value will be written at the enf of sequence.
	 */
	iowrite32(0, priv->iobase + HW_SEC);

	iowrite32(tm->tm_year, priv->iobase + HW_YEAR);
	iowrite32(tm->tm_mon,  priv->iobase + HW_MONTH);
	iowrite32(tm->tm_mday, priv->iobase + HW_DOM);
	iowrite32(tm->tm_wday, priv->iobase + HW_DOW);
	iowrite32(tm->tm_yday, priv->iobase + HW_DOY);
	iowrite32(tm->tm_hour, priv->iobase + HW_HOUR);
	iowrite32(tm->tm_min,  priv->iobase + HW_MIN);
	iowrite32(tm->tm_sec,  priv->iobase + HW_SEC);

	return 0;
}

#if 0
static int asm9260_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct asm9260_rtc_priv *asm9260_rtc_priv = dev_get_drvdata(dev);
	u32 isr, alarm;

	alarm = ioread32(asm9260_rtc_priv->iobase + VT8500_RTC_AS);
	isr = ioread32(asm9260_rtc_priv->iobase + VT8500_RTC_IS);

	alrm->time.tm_mday = bcd2bin((alarm & ALARM_DAY_MASK) >> ALARM_DAY_S);
	alrm->time.tm_hour = bcd2bin((alarm & TIME_HOUR_MASK) >> TIME_HOUR_S);
	alrm->time.tm_min = bcd2bin((alarm & TIME_MIN_MASK) >> TIME_MIN_S);
	alrm->time.tm_sec = bcd2bin((alarm & TIME_SEC_MASK));

	alrm->enabled = (alarm & ALARM_ENABLE_MASK) ? 1 : 0;
	alrm->pending = (isr & VT8500_RTC_IS_ALARM) ? 1 : 0;

	return rtc_valid_tm(&alrm->time);
}

static int asm9260_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct asm9260_rtc_priv *asm9260_rtc_priv = dev_get_drvdata(dev);

	iowrite32((alrm->enabled ? ALARM_ENABLE_MASK : 0)
		| (bin2bcd(alrm->time.tm_mday) << ALARM_DAY_S)
		| (bin2bcd(alrm->time.tm_hour) << TIME_HOUR_S)
		| (bin2bcd(alrm->time.tm_min) << TIME_MIN_S)
		| (bin2bcd(alrm->time.tm_sec)),
		asm9260_rtc_priv->iobase + VT8500_RTC_AS);

	return 0;
}

static int asm9260_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct asm9260_rtc_priv *asm9260_rtc_priv = dev_get_drvdata(dev);
	unsigned long tmp = ioread32(asm9260_rtc_priv->iobase + VT8500_RTC_AS);

	if (enabled)
		tmp |= ALARM_ENABLE_MASK;
	else
		tmp &= ~ALARM_ENABLE_MASK;

	iowrite32(tmp, asm9260_rtc_priv->iobase + VT8500_RTC_AS);
	return 0;
}
#endif

static const struct rtc_class_ops asm9260_rtc_ops = {
	.read_time		= asm9260_rtc_read_time,
	.set_time		= asm9260_rtc_set_time,
//	.read_alarm		= asm9260_rtc_read_alarm,
//	.set_alarm		= asm9260_rtc_set_alarm,
//	.alarm_irq_enable	= asm9260_alarm_irq_enable,
};


static int __init asm9260_rtc_init(struct asm9260_rtc_priv *priv)
{
	/* FIXME: do sanity checks. Do clock is ok? */
	/* Enable RTC and set it to 24-hour mode */
	iowrite32(BM_CTCRST, priv->iobase + HW_CCR);
	msleep(1);
	iowrite32(0, priv->iobase + HW_CCR);
	iowrite32(BM_CLKEN, priv->iobase + HW_CCR);

	iowrite32(0, priv->iobase + HW_CIIR);
	iowrite32(BM_AMR_OFF, priv->iobase + HW_AMR);

	return 0;
}

static int __init asm9260_rtc_probe(struct platform_device *pdev)
{
	struct asm9260_rtc_priv *priv;
	struct resource	*res;
	int ret;

	priv = devm_kzalloc(&pdev->dev,
			   sizeof(struct asm9260_rtc_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

#if 0
	priv->irq_alarm = platform_get_irq(pdev, 0);
	if (priv->irq_alarm < 0) {
		dev_err(&pdev->dev, "No alarm IRQ resource defined\n");
		return priv->irq_alarm;
	}
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->iobase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->iobase))
		return PTR_ERR(priv->iobase);

	priv->clk = devm_clk_get(&pdev->dev, "ahb");
	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clk!\n");
		return ret;
	}

	ret = asm9260_rtc_init(priv);
	if (ret)
		return ret;

	priv->rtc = devm_rtc_device_register(&pdev->dev, "asm9260-rtc",
					      &asm9260_rtc_ops, THIS_MODULE);
	if (IS_ERR(priv->rtc)) {
		ret = PTR_ERR(priv->rtc);
		dev_err(&pdev->dev,
			"Failed to register RTC device -> %d\n", ret);
		goto err_return;
	}

#if 0
	ret = devm_request_irq(&pdev->dev, priv->irq_alarm,
				asm9260_rtc_irq, 0, "rtc alarm", priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't get irq %i, err %d\n",
			priv->irq_alarm, ret);
		goto err_return;
	}
#endif

	return 0;

err_return:
	return ret;
}

static int asm9260_rtc_remove(struct platform_device *pdev)
{
	struct asm9260_rtc_priv *priv = platform_get_drvdata(pdev);

	/* Disable alarm matching */
	iowrite32(BM_AMR_OFF, priv->iobase + HW_AMR);
	clk_disable_unprepare(priv->clk);
	return 0;
}

static const struct of_device_id wmt_dt_ids[] = {
	{ .compatible = "alphascale,asm9260-rtc", },
	{}
};

static struct platform_driver asm9260_rtc_driver = {
	.probe		= asm9260_rtc_probe,
	.remove		= asm9260_rtc_remove,
	.driver		= {
		.name	= "asm9260-rtc",
		.owner	= THIS_MODULE,
		.of_match_table = wmt_dt_ids,
	},
};

module_platform_driver(asm9260_rtc_driver);

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("Alphascale asm9260 SoC Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL v2");
