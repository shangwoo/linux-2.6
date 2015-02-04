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
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define HW_nDATA		0x0000
#define HW_DMA_CTRL		0x0010
#define HW_DMA_DATA		0x0020
#define HW_DMA_PADCTRL0		0x0030
#define HW_DMA_PADCTRL1		0x0040
#define HW_DMA_PADCTRL2		0x0050
#define HW_DMA_PADCTRL3		0x0060
#define HW_DMA_CTRL1		0x0070
#define HW_DMA_CTRL2		0x0080
#define HW_DMA_CTRL3		0x0090
#define HW_DMA_CTRL4		0x00a0
#define HW_nDIR			0x8000
#define HW_nIS			0x8010
#define HW_nIBE			0x8020
#define HW_nIEV			0x8030
#define HW_nIE			0x8040
#define HW_nRIS			0x8050
#define HW_nMIS			0x8060
#define HW_nIC			0x8070
#define HW_nDATAMASK		0x8080

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

struct asm9260_gpio {
	struct gpio_chip chip;
	struct asm9260 *asm9260;
	u16 input_mask;		/* 1 = GPIO is input direction, 0 = output */
};

static inline struct asm9260_gpio *to_asm9260_gpio(struct gpio_chip *_chip)
{
	return container_of(_chip, struct asm9260_gpio, chip);
}

static int asm9260_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct asm9260_gpio *asm9260_gpio = to_asm9260_gpio(chip);
	struct asm9260 *asm9260 = asm9260_gpio->asm9260;

	/* Return an error if the pin is already assigned */
	if (test_and_set_bit(offset, &asm9260->pin_used))
		return -EBUSY;

	return 0;
}

static void asm9260_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct asm9260_gpio *asm9260_gpio = to_asm9260_gpio(chip);
	struct asm9260 *asm9260 = asm9260_gpio->asm9260;

	clear_bit(offset, &asm9260->pin_used);
}

static int asm9260_gpio_set_mode(struct asm9260_gpio *asm9260_gpio, u8 offset,
				u8 val)
{
	struct asm9260 *asm9260 = asm9260_gpio->asm9260;
	const struct asm9260_reg_cfg *mux = asm9260->mux_cfg;

	return asm9260_update_bits(asm9260, mux[offset].reg, mux[offset].mask,
				  val << mux[offset].shift);
}

static int asm9260_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct asm9260_gpio *asm9260_gpio = to_asm9260_gpio(chip);

	asm9260_gpio->input_mask |= BIT(offset);

	return asm9260_gpio_set_mode(asm9260_gpio, offset, ASM9260_GPIO_IN);
}

static int asm9260_get_gpio_in_status(struct asm9260_gpio *asm9260_gpio,
				     struct gpio_chip *chip, unsigned offset)
{
	u8 addr, read;
	int err;

	switch (offset) {
	case ASM9260_GPIO1 ... ASM9260_GPIO8:
		addr = ASM9260_REG_GPIO_A;
		break;
	case ASM9260_GPIO9 ... ASM9260_GPIO16:
		addr = ASM9260_REG_GPIO_B;
		offset = offset - 8;
		break;
	default:
		return -EINVAL;
	}

	err = asm9260_read_byte(asm9260_gpio->asm9260, addr, &read);
	if (err)
		return err;

	return !!(read & BIT(offset));
}

static int asm9260_get_gpio_out_status(struct asm9260_gpio *asm9260_gpio,
				      struct gpio_chip *chip, unsigned offset)
{
	struct asm9260 *asm9260 = asm9260_gpio->asm9260;
	const struct asm9260_reg_cfg *mux = asm9260->mux_cfg;
	u8 read;
	int err;

	err = asm9260_read_byte(asm9260, mux[offset].reg, &read);
	if (err)
		return err;

	read = (read & mux[offset].mask) >> mux[offset].shift;

	if (read == ASM9260_GPIO_OUT_HIGH)
		return 1;
	else if (read == ASM9260_GPIO_OUT_LOW)
		return 0;
	else
		return -EINVAL;
}

static int asm9260_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct asm9260_gpio *asm9260_gpio = to_asm9260_gpio(chip);

	/*
	 * Limitation:
	 *   ASM9260 doesn't have the GPIO direction register. It provides
	 *   only input and output status registers.
	 *   So, direction info is required to handle the 'get' operation.
	 *   This variable is updated whenever the direction is changed and
	 *   it is used here.
	 */

	if (asm9260_gpio->input_mask & BIT(offset))
		return asm9260_get_gpio_in_status(asm9260_gpio, chip, offset);
	else
		return asm9260_get_gpio_out_status(asm9260_gpio, chip, offset);
}

static void asm9260_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct asm9260_gpio *asm9260_gpio = to_asm9260_gpio(chip);
	u8 data;

	if (value)
		data = ASM9260_GPIO_OUT_HIGH;
	else
		data = ASM9260_GPIO_OUT_LOW;

	asm9260_gpio_set_mode(asm9260_gpio, offset, data);
}

static int asm9260_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	struct asm9260_gpio *asm9260_gpio = to_asm9260_gpio(chip);

	asm9260_gpio_set(chip, offset, value);
	asm9260_gpio->input_mask &= ~BIT(offset);

	return 0;
}

static const struct gpio_chip asm9260_gpio_chip = {
	.label			= "asm9260",
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
	struct asm9260 *asm9260 = dev_get_drvdata(pdev->dev.parent);
	struct asm9260_gpio *asm9260_gpio;

	asm9260_gpio = devm_kzalloc(&pdev->dev, sizeof(*asm9260_gpio),
				GFP_KERNEL);
	if (!asm9260_gpio)
		return -ENOMEM;

	asm9260_gpio->asm9260 = asm9260;
	asm9260_gpio->chip = asm9260_gpio_chip;
	asm9260_gpio->chip.dev = &pdev->dev;

	platform_set_drvdata(pdev, asm9260_gpio);

	return gpiochip_add(&asm9260_gpio->chip);
}

static int asm9260_gpio_remove(struct platform_device *pdev)
{
	struct asm9260_gpio *asm9260_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&asm9260_gpio->chip);
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
