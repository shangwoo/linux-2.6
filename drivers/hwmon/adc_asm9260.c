/*
 * Driver for the Alpscale asm9260 adc.
 *
 * Copyright 2010 Alpscale. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <linux/platform_device.h>
#include <linux/hwmon-sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>

#define AS9260_ADC_NAME	"as9260-adc"



struct as9260_adc_priv {
	struct device *hwmon_dev;
};

static ssize_t as9260_adc_show_name(struct device *dev, struct device_attribute
			      *devattr, char *buf)
{
	return sprintf(buf, "as9260_adc\n");
}


void lradc_adc_init(void)
{           
        as3310_writel(1<<11, HW_AHBCLKCTRL1+4);
        as3310_writel(20, HW_ADCCLKDIV);

        as3310_writel(as3310_readl(HW_PDRUNCFG)&0xffffffaf, HW_PDRUNCFG);


        as3310_writel(0xc0000000, HW_LRADC_CTRL0_CLR);
        as3310_writel(0x00a00000, HW_LRADC_CTRL3_SET);
        
        as3310_writel(1<<2, HW_LRADC_CTRL4_SET);
        mdelay(1);
        as3310_writel(1<<1, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<14, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<14, HW_LRADC_CTRL4_CLR);
        as3310_writel(1<<15, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<15, HW_LRADC_CTRL4_CLR);
        as3310_writel(1<<11, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<11, HW_LRADC_CTRL4_CLR);

        while( as3310_readl(HW_LRADC_STATUS)&(1<<10) ) {
            ;
        }

        as3310_writel(0x40<<16, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<13, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<13, HW_LRADC_CTRL4_CLR);

        mdelay(1);
        as3310_writel(3<<8, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<10, HW_LRADC_CTRL4_SET);


        as3310_writel(0x01000000, HW_LRADC_CTRL3);
}

static int as9260_adc_raw_read(struct device *dev,
		struct device_attribute *devattr, unsigned int *val)
{
    volatile int wait_adc = 0x1000000;
	struct platform_device *pdev = to_platform_device(dev);
	struct as9260_adc_priv *priv = platform_get_drvdata(pdev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	unsigned int channel = attr->index;


    as3310_writel( 0, HW_LRADC_CH0+channel*0x10);
    as3310_writel( (1<<(16+channel))+(1<<channel), HW_LRADC_CTRL1+8);

    /*start conversion*/
    as3310_writel( 1<<channel, HW_LRADC_CTRL0+4);

    //not enough
    while( (as3310_readl(HW_LRADC_CTRL0)&(1<<channel))&&(wait_adc --) ) {
        ;
    }

        
    if(wait_adc<0) {
        return -ETIME;
    }else{
        //must delay for a while
        mdelay(100);
        *val = as3310_readl(HW_LRADC_CH0+channel*0x10)&0x3ffff;
        return 0;
    }
	
}

static ssize_t as9260_adc_read(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	unsigned val;
	int ret = as9260_adc_raw_read(dev, devattr, &val);

	if (ret)
		return ret;
    
	return sprintf(buf, "%u\n", val);
}

static DEVICE_ATTR(name, S_IRUGO, as9260_adc_show_name, NULL);
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, as9260_adc_read, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, as9260_adc_read, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, as9260_adc_read, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, as9260_adc_read, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, as9260_adc_read, NULL, 4);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, as9260_adc_read, NULL, 5);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, as9260_adc_read, NULL, 6);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, as9260_adc_read, NULL, 7);


static struct attribute *as9260_attr[] = {
	&dev_attr_name.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	NULL
};

static const struct attribute_group as9260_group = {
	.attrs = as9260_attr,
};


static int __init as9260_adc_probe(struct platform_device *pdev)
{
	struct as9260_adc_priv *priv;
	int ret;

    printk("as9260_adc_probe start...\n");

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	/* Register sysfs hooks */
	ret = sysfs_create_group(&pdev->dev.kobj, &as9260_group);
	if (ret)
		goto out_err_create1;

	priv->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(priv->hwmon_dev)) {
		ret = PTR_ERR(priv->hwmon_dev);
		dev_err(&pdev->dev,
				"hwmon_device_register failed with %d.\n", ret);
		goto out_err_register;
	}

    lradc_adc_init();

	return 0;

out_err_register:
	sysfs_remove_group(&pdev->dev.kobj, &as9260_group);

out_err_create1:
	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return ret;
}

static int __devexit as9260_adc_remove(struct platform_device *pdev)
{
	struct as9260_adc_priv *priv = platform_get_drvdata(pdev);

	hwmon_device_unregister(priv->hwmon_dev);

	sysfs_remove_group(&pdev->dev.kobj, &as9260_group);

	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return 0;
}

static struct platform_driver as9260_adc_driver = {
	.remove 	= __devexit_p(as9260_adc_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= AS9260_ADC_NAME,
	},
};

static int __init as9260_adc_init(void)
{
	return platform_driver_probe(&as9260_adc_driver, as9260_adc_probe);
}

static void __exit as9260_adc_exit(void)
{
	platform_driver_unregister(&as9260_adc_driver);
}

module_init(as9260_adc_init);
module_exit(as9260_adc_exit);

MODULE_DESCRIPTION("as9260 touchscreen driver");
MODULE_LICENSE("GPL");
