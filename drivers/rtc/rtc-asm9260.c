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
#define BM_CMR_IMYEAR		BIT(7)
#define BM_CMR_IMMON		BIT(6)
#define BM_CMR_IMDOY		BIT(5)
#define BM_CMR_IMDOW		BIT(4)
#define BM_CMR_IMDOM		BIT(3)
#define BM_CMR_IMHOUR		BIT(2)
#define BM_CMR_IMMIN		BIT(1)
#define BM_CMR_IMSEC		BIT(0)

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
#define HW_GPREG0		0x44
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



/*
 * Register definitions
 */
#define VT8500_RTC_TS		0x00	/* Time set */
#define VT8500_RTC_DS		0x04	/* Date set */
#define VT8500_RTC_AS		0x08	/* Alarm set */
#define VT8500_RTC_CR		0x0c	/* Control */
#define VT8500_RTC_TR		0x10	/* Time read */
#define VT8500_RTC_DR		0x14	/* Date read */
#define VT8500_RTC_WS		0x18	/* Write status */
#define VT8500_RTC_CL		0x20	/* Calibration */
#define VT8500_RTC_IS		0x24	/* Interrupt status */
#define VT8500_RTC_ST		0x28	/* Status */

#define INVALID_TIME_BIT	(1 << 31)

#define DATE_CENTURY_S		19
#define DATE_YEAR_S		11
#define DATE_YEAR_MASK		(0xff << DATE_YEAR_S)
#define DATE_MONTH_S		6
#define DATE_MONTH_MASK		(0x1f << DATE_MONTH_S)
#define DATE_DAY_MASK		0x3f

#define TIME_DOW_S		20
#define TIME_DOW_MASK		(0x07 << TIME_DOW_S)
#define TIME_HOUR_S		14
#define TIME_HOUR_MASK		(0x3f << TIME_HOUR_S)
#define TIME_MIN_S		7
#define TIME_MIN_MASK		(0x7f << TIME_MIN_S)
#define TIME_SEC_MASK		0x7f

#define ALARM_DAY_S		20
#define ALARM_DAY_MASK		(0x3f << ALARM_DAY_S)

#define ALARM_DAY_BIT		(1 << 29)
#define ALARM_HOUR_BIT		(1 << 28)
#define ALARM_MIN_BIT		(1 << 27)
#define ALARM_SEC_BIT		(1 << 26)

#define ALARM_ENABLE_MASK	(ALARM_DAY_BIT \
				| ALARM_HOUR_BIT \
				| ALARM_MIN_BIT \
				| ALARM_SEC_BIT)

#define VT8500_RTC_CR_ENABLE	(1 << 0)	/* Enable RTC */
#define VT8500_RTC_CR_12H	(1 << 1)	/* 12h time format */
#define VT8500_RTC_CR_SM_ENABLE	(1 << 2)	/* Enable periodic irqs */
#define VT8500_RTC_CR_SM_SEC	(1 << 3)	/* 0: 1Hz/60, 1: 1Hz */
#define VT8500_RTC_CR_CALIB	(1 << 4)	/* Enable calibration */

#define VT8500_RTC_IS_ALARM	(1 << 0)	/* Alarm interrupt status */

struct asm9260_rtc_priv {
	void __iomem		*iobase;
	int			irq_alarm;
	struct rtc_device	*rtc;
	spinlock_t		lock;		/* Protects this structure */
};

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

static int asm9260_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct asm9260_rtc_priv *priv = dev_get_drvdata(dev);
	u32 ctime0, ctime1, ctime2;

	/* FIXME: should we recheck if ctime0 flipped? with day, year filp or what
	 * ever. */
	ctime0 = ioread32(priv->iobase + HW_CTIME0);
	ctime1 = ioread32(priv->iobase + HW_CTIME1);
	ctime2 = ioread32(priv->iobase + HW_CTIME2);

	tm->tm_sec  = bcd2bin((ctime0 >> BM_CTIME0_SEC_S)  & BM_CTIME0_SEC_M);
	tm->tm_min  = bcd2bin((ctime0 >> BM_CTIME0_MIN_S)  & BM_CTIME0_MIN_M);
	tm->tm_hour = bcd2bin((ctime0 >> BM_CTIME0_HOUR_S) & BM_CTIME0_HOUR_M);
	tm->tm_wday = bcd2bin((ctime0 >> BM_CTIME0_DOW_S)  & BM_CTIME0_DOW_M);

	tm->tm_mday = bcd2bin((ctime1 >> BM_CTIME1_DOM_S)  & BM_CTIME1_DOM_M);
	tm->tm_mon  = bcd2bin((ctime1 >> BM_CTIME1_MON_S)  & BM_CTIME1_MON_M);
	tm->tm_year = bcd2bin((ctime1 >> BM_CTIME1_YEAR_S) & BM_CTIME1_YEAR_M);

	tm->tm_yday = bcd2bin((ctime2 >> BM_CTIME2_DOY_S)  & BM_CTIME2_DOY_M);

	return 0;
}

static int asm9260_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct asm9260_rtc_priv *asm9260_rtc_priv = dev_get_drvdata(dev);

	if (tm->tm_year < 100) {
		dev_warn(dev, "Only years 2000-2199 are supported by the "
			      "hardware!\n");
		return -EINVAL;
	}

	iowrite32((bin2bcd(tm->tm_year % 100) << DATE_YEAR_S)
		| (bin2bcd(tm->tm_mon + 1) << DATE_MONTH_S)
		| (bin2bcd(tm->tm_mday))
		| ((tm->tm_year >= 200) << DATE_CENTURY_S),
		asm9260_rtc_priv->iobase + VT8500_RTC_DS);
	iowrite32((bin2bcd(tm->tm_wday) << TIME_DOW_S)
		| (bin2bcd(tm->tm_hour) << TIME_HOUR_S)
		| (bin2bcd(tm->tm_min) << TIME_MIN_S)
		| (bin2bcd(tm->tm_sec)),
		asm9260_rtc_priv->iobase + VT8500_RTC_TS);

	return 0;
}

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

static const struct rtc_class_ops asm9260_rtc_ops = {
	.read_time		= asm9260_rtc_read_time,
	.set_time		= asm9260_rtc_set_time,
	.read_alarm		= asm9260_rtc_read_alarm,
	.set_alarm		= asm9260_rtc_set_alarm,
	.alarm_irq_enable	= asm9260_alarm_irq_enable,
};

static int asm9260_rtc_probe(struct platform_device *pdev)
{
	struct asm9260_rtc_priv *asm9260_rtc_priv;
	struct resource	*res;
	int ret;

	asm9260_rtc_priv = devm_kzalloc(&pdev->dev,
			   sizeof(struct asm9260_rtc_priv), GFP_KERNEL);
	if (!asm9260_rtc_priv)
		return -ENOMEM;

	spin_lock_init(&asm9260_rtc_priv->lock);
	platform_set_drvdata(pdev, asm9260_rtc_priv);

	asm9260_rtc_priv->irq_alarm = platform_get_irq(pdev, 0);
	if (asm9260_rtc_priv->irq_alarm < 0) {
		dev_err(&pdev->dev, "No alarm IRQ resource defined\n");
		return asm9260_rtc_priv->irq_alarm;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	asm9260_rtc_priv->iobase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(asm9260_rtc_priv->iobase))
		return PTR_ERR(asm9260_rtc_priv->iobase);

	/* Enable RTC and set it to 24-hour mode */
	iowrite32(VT8500_RTC_CR_ENABLE,
	       asm9260_rtc_priv->iobase + VT8500_RTC_CR);

	asm9260_rtc_priv->rtc = devm_rtc_device_register(&pdev->dev, "asm9260-rtc",
					      &asm9260_rtc_ops, THIS_MODULE);
	if (IS_ERR(asm9260_rtc_priv->rtc)) {
		ret = PTR_ERR(asm9260_rtc_priv->rtc);
		dev_err(&pdev->dev,
			"Failed to register RTC device -> %d\n", ret);
		goto err_return;
	}

	ret = devm_request_irq(&pdev->dev, asm9260_rtc_priv->irq_alarm,
				asm9260_rtc_irq, 0, "rtc alarm", asm9260_rtc_priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't get irq %i, err %d\n",
			asm9260_rtc_priv->irq_alarm, ret);
		goto err_return;
	}

	return 0;

err_return:
	return ret;
}

static int asm9260_rtc_remove(struct platform_device *pdev)
{
	struct asm9260_rtc_priv *asm9260_rtc_priv = platform_get_drvdata(pdev);

	/* Disable alarm matching */
	iowrite32(0, asm9260_rtc_priv->iobase + VT8500_RTC_IS);

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
