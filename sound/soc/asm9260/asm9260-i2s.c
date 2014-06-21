/*
 * asm9260-i2s.c  --  ALSA Soc Audio Layer
 *
 * (c) 2006 Wolfson Microelectronics PLC.
 * Graeme Gregory graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * (c) 2004-2005 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/pincontrol.h>
#include <mach/dma.h>
#include <linux/dma-mapping.h>
#include <asm/dma-mapping.h>


#include "asm9260-pcm.h"
#include "asm9260-i2s.h"

#define ASM9260_I2S_DEBUG 0
#if ASM9260_I2S_DEBUG
#define DBG   printk 
#else
#define DBG(x...)
#endif

extern void set_pin_mux(int port,int pin,int mux_type);

static struct asm9260_pcm_dma_params asm9260_i2s_pcm_stereo_out = {
	.client		= "I2S PCM Stereo out",
	.channel	= 0,
	.dma_addr	= HW_I2S1_TXDMA,
	.dma_size	= 2,
};

static struct asm9260_pcm_dma_params asm9260_i2s_pcm_stereo_in = {
	.client		= "I2S PCM Stereo in",
	.channel	= 1,
	.dma_addr	= HW_I2S1_RXDMA,
	.dma_size	= 2,
};

/*
 * Set asm9260 I2S DAI format
 */
static int asm9260_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	DBG("Entered %s\n", __func__);

	return 0;
}

static int asm9260_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	u32 tmp=0;

	DBG("Entered %s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		tmp = as3310_readl(HW_DMA1_CTL0);
		rtd->dai->cpu_dai->dma_data = &asm9260_i2s_pcm_stereo_out;
	}
	else
	{
		tmp = as3310_readl(HW_DMA1_CTL1);
		rtd->dai->cpu_dai->dma_data = &asm9260_i2s_pcm_stereo_in;
	}

	switch (params_rate(params)) {
	case 32000:
		as3310_writel(0x80babb00,HW_I2S1_CCR); 
		break;
	case 48000:
		as3310_writel(0x807c7c00,HW_I2S1_CCR);
		break;
	case 96000:
		as3310_writel(0x803e3d00,HW_I2S1_CCR);
		break;
	case 44100:
		as3310_writel(0x80878700,HW_I2S1_CCR);
		break;
	case 88200:
		as3310_writel(0x80434300,HW_I2S1_CCR);
		break;
	default:
		break;
	}
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S32_LE:
		as3310_writel(0x5,HW_I2S1_RCR0);
		as3310_writel(0x5,HW_I2S1_TCR0);
		tmp = (tmp&~126)|(2<<1)|(2<<4);
		((struct asm9260_pcm_dma_params*)rtd->dai->cpu_dai->dma_data)->dma_size = 4;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		as3310_writel(0x2,HW_I2S1_RCR0);
		as3310_writel(0x2,HW_I2S1_TCR0);
		tmp = (tmp&~126)|(1<<1)|(1<<4);
		((struct asm9260_pcm_dma_params*)rtd->dai->cpu_dai->dma_data)->dma_size = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
        	as3310_writel(0x4,HW_I2S1_RCR0);
        	as3310_writel(0x4,HW_I2S1_TCR0);	
		tmp = (tmp&~126)|(2<<1)|(2<<4);
		((struct asm9260_pcm_dma_params*)rtd->dai->cpu_dai->dma_data)->dma_size = 4;
		break;	
	default:
		break;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		as3310_writel(tmp,HW_DMA1_CTL0);
	}
	else
	{		
		as3310_writel(tmp,HW_DMA1_CTL1);
	}

	return 0;
}

static int asm9260_i2s_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	DBG("Entered %s\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		as3310_writel(0x0,HW_I2S1_IMR0);
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			as3310_writel(0x1,HW_I2S1_IRER);
		else
			as3310_writel(0x1,HW_I2S1_ITER);
		as3310_writel(0x1,HW_I2S1_CER);
		as3310_writel(0x1,HW_I2S1_IER);	//enable I2S0				
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			as3310_writel(0x0,HW_I2S1_IRER);
		else
			as3310_writel(0x0,HW_I2S1_ITER);
		as3310_writel(0x0,HW_I2S1_CER);
		as3310_writel(0x0,HW_I2S1_IER);	//enable I2S0		
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * Set asm9260 Clock source
 */
static int asm9260_i2s_set_sysclk(struct snd_soc_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{

	DBG("Entered %s\n", __func__);

	as3310_writel(0x0,HW_I2S1CLKSEL);
	as3310_writel(0x0,HW_I2S1CLKUEN);	
	as3310_writel(0x1,HW_I2S1CLKUEN);
	
	return 0;
}

/*
 * Set asm9260 Clock dividers
 */
static int asm9260_i2s_set_clkdiv(struct snd_soc_dai *cpu_dai,
	int div_id, int div)
{
	DBG("Entered %s\n", __func__);

	switch (div_id) {
	case ASM9260_DIV_BCLK:
		as3310_writel(div,HW_I2S1_SCLKDIV);
		break;
	case ASM9260_DIV_MCLK:
		as3310_writel(div,HW_I2S1_MCLKDIV);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int asm9260_i2s_probe(struct platform_device *pdev,
			     struct snd_soc_dai *dai)
{
	DBG("Entered %s\n", __func__);

	set_pin_mux(5, 0, 3);	
	set_pin_mux(5, 1, 3);	
	set_pin_mux(5, 2, 3);	
	set_pin_mux(5, 3, 3);	
	set_pin_mux(5, 4, 3);	

	as3310_writel(0x1<<15,HW_AHBCLKCTRL1 + SET_OFFSET);
	as3310_writel(0x0,HW_I2S1CLKSEL);
	as3310_writel(0x0,HW_I2S1CLKUEN);	
	as3310_writel(0x1,HW_I2S1CLKUEN);
	as3310_writel(0x1,HW_I2S1_MCLKDIV);
	as3310_writel(0x1,HW_I2S1_SCLKDIV);

	as3310_writel(as3310_readl(HW_MACPHY_SEL) | (1<<5), HW_MACPHY_SEL);	
	as3310_writel(as3310_readl(HW_DMA_CTRL) | (1<<22) | (1<<23) ,HW_DMA_CTRL);	
	
	as3310_writel(0x0,HW_I2S1_IER);	//enable I2S0
	as3310_writel(0x1,HW_I2S1_TER0);    //enable channel 0 TX        
	as3310_writel(0x1,HW_I2S1_RER0);	//enable channel 0 RX       
	as3310_writel(0x0,HW_I2S1_TER1);	//disable channel 1 TX       
	as3310_writel(0x0,HW_I2S1_RER1);	//disable channel 1 RX        
	as3310_writel(0x0,HW_I2S1_TER2);	//disable channel 2 TX        
	as3310_writel(0x0,HW_I2S1_RER2);	//disable channel 2 RX        
	as3310_writel(0x2,HW_I2S1_RCR0);	//Rx bits=16        
	as3310_writel(0x2,HW_I2S1_TCR0);	//Tx bits=16        
	as3310_writel(0x4,HW_I2S1_TFCR0);   //trigger level=5       
	as3310_writel(0x4,HW_I2S1_RFCR0);   //trigger level=5
	as3310_writel(0x80878700,HW_I2S1_CCR);	//44.1kHz  sclk--12M
	return 0;
}


#define asm9260_i2s_suspend NULL
#define asm9260_i2s_resume NULL


#define ASM9260_I2S_RATES \
	(SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)
struct snd_soc_dai asm9260_i2s_dai = {
	.name = "asm9260-i2s",
	.id = 0,
	.type = SND_SOC_DAI_I2S,
	.probe = asm9260_i2s_probe,
	.suspend = asm9260_i2s_suspend,
	.resume = asm9260_i2s_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = ASM9260_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = ASM9260_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.trigger = asm9260_i2s_trigger,
		.hw_params = asm9260_i2s_hw_params,},
	.dai_ops = {
		.set_fmt = asm9260_i2s_set_fmt,
		.set_clkdiv = asm9260_i2s_set_clkdiv,
		.set_sysclk = asm9260_i2s_set_sysclk,
	},
};
EXPORT_SYMBOL_GPL(asm9260_i2s_dai);

/* Module information */
MODULE_AUTHOR("Lujy, <lujy@alpscale.cn>");
MODULE_DESCRIPTION("asm9260 I2S SoC Interface");
MODULE_LICENSE("GPL");
