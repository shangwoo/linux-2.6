/*
 * asm9260_wm8731.c  --  
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include <asm/mach-types.h>
#include <linux/io.h>
#include <mach/dma.h>
#include <mach/pincontrol.h>

#include "../codecs/wm8731.h"
#include "asm9260-pcm.h"
#include "asm9260-i2s.h"

/* Debugging stuff */
#define ASM9260_SOC_WM8731_DEBUG 0
#if ASM9260_SOC_WM8731_DEBUG
#define DBG   printk 
#else
#define DBG(x...)
#endif

#define I2C_WM8731_INTERFACE_INDEX 1

static struct snd_soc_machine asm9260_snd;
// static struct i2c_client *i2c;
static int asm9260_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int pll_out = 0;
	int ret = 0;

	DBG("Entered %s\n", __func__);

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	pll_out = 12000000;
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, pll_out,0);
	if (ret < 0)
		return ret;

	return 0;
}

static int asm9260_hifi_hw_free(struct snd_pcm_substream *substream)
{
	DBG("Entered %s\n", __func__);

	/* disable the PLL */
	return 0;
}

/*
 * ASM9260 WM8731 HiFi DAI opserations.
 */
static struct snd_soc_ops asm9260_hifi_ops = {
	.hw_params = asm9260_hifi_hw_params,
	.hw_free = asm9260_hifi_hw_free,
};


static int asm9260_wm8731_init(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_dai_link asm9260_snd_dai[] = {
{
	.name = "WM8731",
	.stream_name = "WM8731 audio",
	.cpu_dai = &asm9260_i2s_dai,
	.codec_dai = &wm8731_dai,
	.init = asm9260_wm8731_init,
	.ops = &asm9260_hifi_ops,
},
};

static struct snd_soc_machine asm9260_snd = {
	.name = "asm9260_snd",
	.dai_link = asm9260_snd_dai,
	.num_links = ARRAY_SIZE(asm9260_snd_dai),
};

static struct wm8731_setup_data asm9260_wm8753_setup = {
	.i2c_bus = I2C_WM8731_INTERFACE_INDEX,
	.i2c_address = 0x34/2,
};

static struct snd_soc_device asm9260_snd_devdata = {
	.machine = &asm9260_snd,
	.platform = &asm9260_soc_platform,
	.codec_dev = &soc_codec_dev_wm8731,
	.codec_data = &asm9260_wm8753_setup,
};


static struct platform_device *asm9260_snd_device;

static int __init asm9260_audio_init(void)
{
	int ret;

	asm9260_snd_device = platform_device_alloc("soc-audio", -1);
	if (!asm9260_snd_device)
		return -ENOMEM;

	platform_set_drvdata(asm9260_snd_device, &asm9260_snd_devdata);
	asm9260_snd_devdata.dev = &asm9260_snd_device->dev;
	ret = platform_device_add(asm9260_snd_device);

	if (ret) {
		platform_device_put(asm9260_snd_device);
		return ret;
	}

	return ret;
}

static void __exit asm9260_audio_exit(void)
{
	DBG("Entered %s\n", __func__);

	platform_device_unregister(asm9260_snd_device);
}

module_init(asm9260_audio_init);
module_exit(asm9260_audio_exit);

/* Module information */
MODULE_AUTHOR("Lu, lujy@alpscale.cn");
MODULE_DESCRIPTION("ALSA SoC WM8731 ALPSCALE");
MODULE_LICENSE("GPL");
