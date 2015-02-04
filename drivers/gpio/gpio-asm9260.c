/*
 * Alphascale asm9260t GPIO driver
 *
 * Copyright (C) 2015 Oleksij Rempel <linux@rempel-privat.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>

#define SET_REG			0x4
#define CLR_REG			0x8

#define HW_DATA0		0x00000
#define HW_DMA_CTRL		0x00010
#define HW_DMA_DATA		0x00020
#define HW_DMA_PADCTRL0		0x00030
#define HW_DMA_PADCTRL1		0x00040
#define HW_DMA_PADCTRL2		0x00050
#define HW_DMA_PADCTRL3		0x00060
#define HW_DMA_CTRL1		0x00070
#define HW_DMA_CTRL2		0x00080
#define HW_DMA_CTRL3		0x00090
#define HW_DMA_CTRL4		0x000a0
#define HW_DIR0			0x08000
#define HW_IS0			0x08010
#define HW_IBE0			0x08020
#define HW_IEV0			0x08030
#define HW_IE0			0x08040
#define HW_RIS0			0x08050
#define HW_MIS0			0x08060
#define HW_IC0			0x08070
#define HW_DATAMASK0		0x08080

#define PORTn(reg, port)	((reg) + ((port) / 4) * 0x10000)
#define PINn(port, pin)		(1 << (((port) % 4) * 8 + (pin)))

enum asm9260_gpios {
	ASM9260_GPIO1,
	ASM9260_GPIO2,
	ASM9260_GPIO3,
	ASM9260_GPIO4,
	ASM9260_GPIO5,
	ASM9260_GPIO6,
	ASM9260_GPIO7,
	ASM9260_GPIO8,
	ASM9260_GPIO9,
	ASM9260_GPIO10,
	ASM9260_GPIO11,
	ASM9260_GPIO12,
	ASM9260_GPIO13,
	ASM9260_GPIO14,
	ASM9260_GPIO15,
	ASM9260_GPIO16,
	ASM9260_MAX_GPIO,
};

struct asm9260_gpio_priv {
	struct gpio_chip	chip;
	void __iomem		*iobase;
	struct clk		*clk;
	u16 input_mask;		/* 1 = GPIO is input direction, 0 = output */
};

static inline struct asm9260_gpio_priv *to_asm9260_gpio(struct gpio_chip *_chip)
{
	return container_of(_chip, struct asm9260_gpio_priv, chip);
}

static int asm9260_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	printk("%s:%i\n", __func__, __LINE__);
	return 0;
}

static void asm9260_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	printk("%s:%i\n", __func__, __LINE__);
}

static int asm9260_gpio_set_mode(struct asm9260_gpio_priv *asm9260_gpio, u8 offset,
				u8 val)
{
	printk("%s:%i\n", __func__, __LINE__);
	return 0;
}

static int asm9260_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	printk("%s:%i\n", __func__, __LINE__);
	return 0;
}

static int asm9260_get_gpio_in_status(struct asm9260_gpio_priv *asm9260_gpio,
				     struct gpio_chip *chip, unsigned offset)
{
	printk("%s:%i\n", __func__, __LINE__);
	return 0;
}

static int asm9260_get_gpio_out_status(struct asm9260_gpio_priv *asm9260_gpio,
				      struct gpio_chip *chip, unsigned offset)
{
	printk("%s:%i\n", __func__, __LINE__);
	return 0;
}

static int asm9260_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	printk("%s:%i\n", __func__, __LINE__);
	return 0;
}

static void asm9260_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	printk("%s:%i\n", __func__, __LINE__);
}

static int asm9260_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	printk("%s:%i\n", __func__, __LINE__);
	return 0;
}

static const struct gpio_chip asm9260_gpio_chip = {
	.label			= "asm9260-gpio",
	.owner			= THIS_MODULE,
	.request		= asm9260_gpio_request,
	.free			= asm9260_gpio_free,
	.direction_input	= asm9260_gpio_direction_input,
	.get			= asm9260_gpio_get,
	.direction_output	= asm9260_gpio_direction_output,
	.set			= asm9260_gpio_set,
	.base			= -1,
	.ngpio			= ASM9260_MAX_GPIO,
	.can_sleep		= 1,
};

static int asm9260_gpio_probe(struct platform_device *pdev)
{
	struct asm9260_gpio_priv *priv;
	struct resource	*res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv),
				GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

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

	priv->chip = asm9260_gpio_chip;
	priv->chip.dev = &pdev->dev;

	platform_set_drvdata(pdev, priv);

	ret = gpiochip_add(&priv->chip);
	if (ret) {
		dev_err(&pdev->dev, "Failed to registr gpiochip!\n");
		goto err_return;
	}

	return ret;

err_return:
	clk_disable_unprepare(priv->clk);
	return ret;
}

static int asm9260_gpio_remove(struct platform_device *pdev)
{
	struct asm9260_gpio_priv *priv = platform_get_drvdata(pdev);

	gpiochip_remove(&priv->chip);
	clk_disable_unprepare(priv->clk);
	return 0;
}

static const struct of_device_id asm9260_gpio_of_match[] = {
	{ .compatible = "alphascale,asm9260-gpio", },
	{ }
};
MODULE_DEVICE_TABLE(of, asm9260_gpio_of_match);

static struct platform_driver asm9260_gpio_driver = {
	.probe = asm9260_gpio_probe,
	.remove = asm9260_gpio_remove,
	.driver = {
		.name = "asm9260-gpio",
		.of_match_table = asm9260_gpio_of_match,
	},
};
module_platform_driver(asm9260_gpio_driver);

MODULE_DESCRIPTION("ASM9260 GPIO driver");
MODULE_ALIAS("platform:asm9260-gpio");
MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_LICENSE("GPL v2");
