/*
 * asm9260-pcm.c  --  ALSA Soc Audio Layer
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <asm/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dma.h>

#include "asm9260-pcm.h"

#define ASM9260_PCM_DEBUG 0
#if ASM9260_PCM_DEBUG
#define DBG   printk
#else
#define DBG(x...)
#endif

struct dma_llp
{
	u32 sar;
	u32 dar;
	u32 llp;
	u32 ctrl_l;
	u32 ctrl_h;
};
static struct dma_llp *dma_llp_tx_vir;
static struct dma_llp *dma_llp_rx_vir;
static dma_addr_t dma_llp_tx_phy;
static dma_addr_t dma_llp_rx_phy;
static uint32_t audio_dma_rx_size = 0x200;
static uint32_t audio_dma_tx_size = 0x200;
static int audio_fmt;

static const struct snd_pcm_hardware asm9260_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				    SNDRV_PCM_INFO_BLOCK_TRANSFER |
				    SNDRV_PCM_INFO_MMAP |
				    SNDRV_PCM_INFO_MMAP_VALID |
				    SNDRV_PCM_INFO_PAUSE |
				    SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_U16_LE |
				    SNDRV_PCM_FMTBIT_U8 |
				    SNDRV_PCM_FMTBIT_S8,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 64*1024,
	.period_bytes_min	= PAGE_SIZE,          
	.period_bytes_max	= PAGE_SIZE*2,
	.periods_min		= 2,                      
	.periods_max		= 128,
	.fifo_size		= 32,
};

struct asm9260_runtime_data {
	spinlock_t lock;
	int state;

	unsigned int dma_period;
	dma_addr_t dma_start;
	dma_addr_t dma_pos;
	dma_addr_t dma_end;
	struct asm9260_pcm_dma_params *params;
};


void dma_tx_init(dma_addr_t dma_src_addr, uint32_t size)
{
    as3310_writel(dma_src_addr, HW_DMA1_SAR0);
    as3310_writel(0x8104912, HW_DMA1_CTL0);
    as3310_writel(audio_dma_tx_size, HW_DMA1_CTL0+4);
    as3310_writel(dma_llp_tx_phy, HW_DMA1_LLP0);
    as3310_writel(0x1, HW_DMA1_DMACFGREG);
    as3310_writel(0x101, HW_DMA1_CHENREG);
}

void dma_rx_init(dma_addr_t dma_dar_addr, uint32_t size)
{
    as3310_writel(HW_I2S1_RXDMA,HW_DMA1_SAR1);  //DMA1 channel1 receive
    as3310_writel(dma_dar_addr,HW_DMA1_DAR1);
    as3310_writel(0x00000000
        +(0<<0)	   //INT_EN
        +(1<<1)	   //DST_TR_WIDTH 16bit
		+(1<<4)	   //SRC_TR_WIDTH 16bit
        +(0<<7)	   //DINC 00 = Increment
        +(2<<9)	   //SINC 1x = No change  
        +(1<<11)   //DEST_MSIZE
        +(1<<14)   //SRC_MSIZE
        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
        +(0<<23)   //DMS Destination Master interface.
        +(0<<25)   //SMS source Master interface
        +(0<<27)   //LLP_DST_DISABLE
        +(1<<28)   //LLP_SRC_DISABLE
		,HW_DMA1_CTL1);

    as3310_writel(audio_dma_rx_size, HW_DMA1_CTL1+4);
    as3310_writel(dma_llp_rx_phy, HW_DMA1_LLP1);
    as3310_writel(0x1,HW_DMA1_DMACFGREG);
    as3310_writel(0x202,HW_DMA1_CHENREG);

}

static irqreturn_t asm9260_iis_write_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct asm9260_runtime_data *prtd = runtime->private_data;

	DBG("Entered %s\n", __func__);

	as3310_writel(1<<0, HW_DMA1_ClearTFR);//clear
	prtd = substream->runtime->private_data;

	if (substream)
		snd_pcm_period_elapsed(substream);

	spin_lock(&prtd->lock);
	if (prtd->state & ST_RUNNING) {
		prtd->dma_pos += prtd->dma_period;
		if(prtd->dma_pos>=prtd->dma_end)
			prtd->dma_pos = prtd->dma_start;
		dma_tx_init(prtd->dma_pos, prtd->dma_period);
	}

	spin_unlock(&prtd->lock);

	return IRQ_HANDLED;
}

static irqreturn_t asm9260_iis_read_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct asm9260_runtime_data *prtd = runtime->private_data;

	DBG("Entered %s\n", __func__);
	as3310_writel(1<<1, HW_DMA1_ClearTFR);//clear
	prtd = substream->runtime->private_data;

	if (substream)
		snd_pcm_period_elapsed(substream);

	spin_lock(&prtd->lock);
	if (prtd->state & ST_RUNNING) {
		prtd->dma_pos += prtd->dma_period;
		if(prtd->dma_pos>=prtd->dma_end)
			prtd->dma_pos = prtd->dma_start;
		dma_rx_init(prtd->dma_pos, prtd->dma_period);
	}

	spin_unlock(&prtd->lock);

	return IRQ_HANDLED;
}


void i2s_dma_tx_pkg_config(uint32_t audio_frag_size)
{
	int i, pkg_num;
	if(audio_fmt == 16)
	{
		pkg_num = (audio_frag_size / audio_dma_tx_size) >>1;
	}
	else
	{
		pkg_num = (audio_frag_size / audio_dma_tx_size) >>2;
	}
	for (i=0; i<pkg_num - 1; i++)
	{
		dma_llp_tx_vir[i].dar    = HW_I2S1_TXDMA;
		dma_llp_tx_vir[i].llp    = dma_llp_tx_phy + sizeof(struct dma_llp) * (i + 1);
		if(audio_fmt == 16)
			dma_llp_tx_vir[i].ctrl_l = 0x00000000
							        +(0<<0)	   //INT_EN
							        +(1<<1)	   //DST_TR_WIDTH 
									+(1<<4)	   //SRC_TR_WIDTH
							        +(2<<7)	   //DINC 1x = No change
							        +(0<<9)	   //SINC 00 = Increment
							        +(1<<11)   //DEST_MSIZE
							        +(1<<14)   //SRC_MSIZE
							        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
							        +(0<<23)   //DMS Destination Master interface.
							        +(0<<25)   //SMS source Master interface
							        +(1<<27)   //LLP_DST_DISABLE
							        +(0<<28);   //LLP_SRC_DISABLE
		else
		    dma_llp_tx_vir[i].ctrl_l = 0x00000000
							        +(0<<0)	   //INT_EN
							        +(2<<1)	   //DST_TR_WIDTH 
									+(2<<4)	   //SRC_TR_WIDTH
							        +(2<<7)	   //DINC 1x = No change
							        +(0<<9)	   //SINC 00 = Increment
							        +(1<<11)   //DEST_MSIZE
							        +(1<<14)   //SRC_MSIZE
							        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
							        +(0<<23)   //DMS Destination Master interface.
							        +(0<<25)   //SMS source Master interface
							        +(1<<27)   //LLP_DST_DISABLE
							        +(0<<28);   //LLP_SRC_DISABLE
		dma_llp_tx_vir[i].ctrl_h = audio_dma_tx_size;
	}
	dma_llp_tx_vir[i].dar    = HW_I2S1_TXDMA;
	dma_llp_tx_vir[i].llp    = 0;
	if(audio_fmt == 16)
		dma_llp_tx_vir[i].ctrl_l = 0x00000000
						        +(1<<0)	   //INT_EN
						        +(1<<1)	   //DST_TR_WIDTH 
								+(1<<4)	   //SRC_TR_WIDTH
						        +(2<<7)	   //DINC 1x = No change
						        +(0<<9)	   //SINC 00 = Increment
						        +(1<<11)   //DEST_MSIZE
						        +(1<<14)   //SRC_MSIZE
						        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
						        +(0<<23)   //DMS Destination Master interface.
						        +(0<<25)   //SMS source Master interface
						        +(0<<27)   //LLP_DST_DISABLE
						        +(0<<28);   //LLP_SRC_DISABLE
	else
		dma_llp_tx_vir[i].ctrl_l = 0x00000000
						        +(1<<0)	   //INT_EN
						        +(2<<1)	   //DST_TR_WIDTH 
								+(2<<4)	   //SRC_TR_WIDTH
						        +(2<<7)	   //DINC 1x = No change
						        +(0<<9)	   //SINC 00 = Increment
						        +(1<<11)   //DEST_MSIZE
						        +(1<<14)   //SRC_MSIZE
						        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
						        +(0<<23)   //DMS Destination Master interface.
						        +(0<<25)   //SMS source Master interface
						        +(0<<27)   //LLP_DST_DISABLE
						        +(0<<28);   //LLP_SRC_DISABLE	
	dma_llp_tx_vir[i].ctrl_h = audio_dma_tx_size;
}

void i2s_dma_rx_pkg_config(uint32_t audio_frag_size)
{
	int i, pkg_num;
	if(audio_fmt == 16)
	    pkg_num = (audio_frag_size / audio_dma_rx_size) >>1;
	else
		pkg_num = (audio_frag_size / audio_dma_rx_size) >>2;
	for (i=0; i<pkg_num - 1; i++)
	{
		dma_llp_rx_vir[i].sar    = HW_I2S1_RXDMA;
		dma_llp_rx_vir[i].llp    = dma_llp_rx_phy + sizeof(struct dma_llp) * (i + 1);
		if(audio_fmt == 16)
			dma_llp_rx_vir[i].ctrl_l = 0x00000000
							        +(0<<0)	   //INT_EN
							        +(1<<1)	   //DST_TR_WIDTH 
									+(1<<4)	   //SRC_TR_WIDTH
							        +(0<<7)	   //DINC 00 = Increment
							        +(2<<9)	   //SINC 1x = No change  
							        +(1<<11)   //DEST_MSIZE
							        +(1<<14)   //SRC_MSIZE
							        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
							        +(0<<23)   //DMS Destination Master interface.
							        +(0<<25)   //SMS source Master interface
							        +(0<<27)   //LLP_DST_DISABLE
							        +(1<<28);   //LLP_SRC_DISABLE
		else
			dma_llp_rx_vir[i].ctrl_l = 0x00000000
							        +(0<<0)	   //INT_EN
							        +(2<<1)	   //DST_TR_WIDTH 
									+(2<<4)	   //SRC_TR_WIDTH
							        +(0<<7)	   //DINC 00 = Increment
							        +(2<<9)	   //SINC 1x = No change  
							        +(1<<11)   //DEST_MSIZE
							        +(1<<14)   //SRC_MSIZE
							        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
							        +(0<<23)   //DMS Destination Master interface.
							        +(0<<25)   //SMS source Master interface
							        +(0<<27)   //LLP_DST_DISABLE
							        +(1<<28);   //LLP_SRC_DISABLE			
		dma_llp_rx_vir[i].ctrl_h = audio_dma_rx_size;
	}
	dma_llp_rx_vir[i].sar    = HW_I2S1_RXDMA;
	dma_llp_rx_vir[i].llp    = 0;
	if(audio_fmt == 16)
		dma_llp_rx_vir[i].ctrl_l = 0x00000000
						        +(1<<0)	   //INT_EN
						        +(1<<1)	   //DST_TR_WIDTH 
								+(1<<4)	   //SRC_TR_WIDTH
						        +(0<<7)	   //DINC 1x = No change
						        +(7<<9)	   //SINC 00 = Increment
						        +(1<<11)   //DEST_MSIZE
						        +(1<<14)   //SRC_MSIZE
						        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
						        +(0<<23)   //DMS Destination Master interface.
						        +(0<<25)   //SMS source Master interface
						        +(0<<27)   //LLP_DST_DISABLE
						        +(0<<28);   //LLP_SRC_DISABLE
	else
		dma_llp_rx_vir[i].ctrl_l = 0x00000000
						        +(1<<0)	   //INT_EN
						        +(2<<1)	   //DST_TR_WIDTH 
								+(2<<4)	   //SRC_TR_WIDTH
						        +(0<<7)	   //DINC 1x = No change
						        +(7<<9)	   //SINC 00 = Increment
						        +(1<<11)   //DEST_MSIZE
						        +(1<<14)   //SRC_MSIZE
						        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
						        +(0<<23)   //DMS Destination Master interface.
						        +(0<<25)   //SMS source Master interface
						        +(0<<27)   //LLP_DST_DISABLE
						        +(0<<28);   //LLP_SRC_DISABLE		
	dma_llp_rx_vir[i].ctrl_h = audio_dma_rx_size;
}

static int asm9260_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct asm9260_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct asm9260_pcm_dma_params *dma = rtd->dai->cpu_dai->dma_data;
	unsigned long totbytes = params_buffer_bytes(params);
	int ret = 0;

	DBG("Entered %s\n", __func__);

	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	if (!dma)
		return 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S32_LE:
		audio_fmt = 32;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		audio_fmt = 16;;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		audio_fmt = 24;
	default:
		break;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->dma_bytes = totbytes;

	spin_lock_irq(&prtd->lock);
	prtd->dma_period = params_period_bytes(params);
	prtd->dma_start = runtime->dma_addr;
	prtd->dma_pos = prtd->dma_start;
	prtd->dma_end = prtd->dma_start + totbytes;

	spin_unlock_irq(&prtd->lock);

	/* this may get called several times by oss emulation
	 * with different params -HW */
	if (prtd->params == NULL) {
		/* prepare DMA */
		prtd->params = dma;
		if (prtd->params->channel == 0)
		{
			dma_llp_tx_vir = (struct dma_llp *)dma_alloc_coherent(NULL, sizeof(struct dma_llp) * 100, &dma_llp_tx_phy, GFP_KERNEL|GFP_DMA);
			if (!dma_llp_tx_vir)
			{
				printk("unable to alloc dma_llp_rx_vir\n");
				return -ENOMEM;
			}
	
			ret = request_irq(INT_DMA1_CH0, asm9260_iis_write_irq, IRQF_DISABLED, "asm9260_dma1_ch0", (void *)(substream));
			if (ret) 
			{
				dma_free_coherent(NULL, sizeof(struct dma_llp) * 100, dma_llp_tx_vir, dma_llp_tx_phy);
				return ret;
			}

		}


		if (prtd->params->channel == 1)
		{
			dma_llp_rx_vir = (struct dma_llp *)dma_alloc_coherent(NULL, sizeof(struct dma_llp) * 100, &dma_llp_rx_phy, GFP_KERNEL|GFP_DMA);
			if (!dma_llp_rx_vir)
			{
				printk("unable to alloc dma_llp_rx_vir\n");
				return -ENOMEM;
			}


			ret = request_irq(INT_DMA1_CH1, asm9260_iis_read_irq, IRQF_DISABLED, "asm9260_dma1_ch1", (void *)(substream));
			if (ret) 
			{
				dma_free_coherent(NULL, sizeof(struct dma_llp) * 100, dma_llp_rx_vir, dma_llp_rx_phy);
				return ret;
			}
		}
	}


	return 0;
}

static int asm9260_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct asm9260_runtime_data *prtd = substream->runtime->private_data;

	DBG("Entered %s\n", __func__);

	/* TODO - do we need to ensure DMA flushed */
	if (prtd->params) {
		
		if (prtd->params->channel == 0)
		{
			dma_free_coherent(NULL, sizeof(struct dma_llp) * 40, dma_llp_tx_vir, dma_llp_tx_phy);
			free_irq(INT_DMA1_CH0, (void *)(substream));
		}


		if (prtd->params->channel == 1)
		{
			dma_free_coherent(NULL, sizeof(struct dma_llp) * 100, dma_llp_rx_vir, dma_llp_rx_phy);
			free_irq(INT_DMA1_CH1, (void *)(substream));
		}
		prtd->params = NULL;
	}
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int asm9260_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct asm9260_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	DBG("Entered %s\n", __func__);


	/* channel needs configuring for mem=>device, increment memory addr,
	 * sync to pclk, half-word transfers to the IIS-FIFO. */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		as3310_writel(HW_I2S1_TXDMA,HW_DMA1_DAR0); //DMA1 channel0 Transfer
		as3310_writel(0x00000000
			        +(0<<0)	   //INT_EN
			        +(1<<1)	   //DST_TR_WIDTH 
					+(1<<4)	   //SRC_TR_WIDTH
			        +(2<<7)	   //DINC 1x = No change
			        +(0<<9)	   //SINC 00 = Increment
			        +(1<<11)   //DEST_MSIZE
			        +(1<<14)   //SRC_MSIZE
			        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
			        +(0<<23)   //DMS Destination Master interface.
			        +(0<<25)   //SMS source Master interface
			        +(1<<27)   //LLP_DST_DISABLE
			        +(0<<28)   //LLP_SRC_DISABLE
					,HW_DMA1_CTL0);
		as3310_writel(audio_dma_tx_size, HW_DMA1_CTL0+4);
		as3310_writel(dma_llp_tx_phy, HW_DMA1_LLP0);
		as3310_writel(((as3310_readl(HW_DMA1_CFG0) & (~(1<<10))) & (~(1<<30))) & (~(1<<31)),HW_DMA1_CFG0); //hardware handshaking
		as3310_writel(0x00000000+(6<<7)    //SRC_PRE req2 to channel0                   
						+(6<<11),HW_DMA1_CFG0+4); //DST_PRE req2 to channel0

		i2s_dma_tx_pkg_config(prtd->dma_period);
	
		as3310_writel(0x101,HW_DMA1_MaskTFR); //	channel0 and channel1 transfer finish interrupt open
	} else {
		as3310_writel(HW_I2S1_RXDMA,HW_DMA1_SAR1);  //DMA1 channel1 receive
		as3310_writel(0x00000000
			        +(0<<0)	   //INT_EN
			        +(1<<1)	   //DST_TR_WIDTH 
					+(1<<4)	   //SRC_TR_WIDTH
			        +(0<<7)	   //DINC 00 = Increment
			        +(2<<9)	   //SINC 1x = No change  
			        +(1<<11)   //DEST_MSIZE
			        +(1<<14)   //SRC_MSIZE
			        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
			        +(0<<23)   //DMS Destination Master interface.
			        +(0<<25)   //SMS source Master interface
			        +(0<<27)   //LLP_DST_DISABLE
			        +(1<<28)   //LLP_SRC_DISABLE
					,HW_DMA1_CTL1);

		as3310_writel(audio_dma_rx_size,HW_DMA1_CTL1+4);
		as3310_writel(dma_llp_rx_phy, HW_DMA1_LLP1);
		as3310_writel(((as3310_readl(HW_DMA1_CFG1) & (~(1<<11))) & (~(1<<31))) & (~(1<<31)),HW_DMA1_CFG1); //hardware handshaking
		as3310_writel(0x00000000+(7<<7)    //SRC_PRE req2 to channel0                   
						+(7<<11),HW_DMA1_CFG1+4); //DST_PRE req2 to channel0

		i2s_dma_rx_pkg_config(prtd->dma_period);

		as3310_writel(0x202,HW_DMA1_MaskTFR); //	channel0 and channel1 transfer finish interrupt open
	}

	/* flush the DMA channel */

	prtd->dma_pos = prtd->dma_start;

	return ret;
}

static int asm9260_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct asm9260_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;
	int cnt = 0;

	DBG("Entered %s\n", __func__);

	spin_lock(&prtd->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		prtd->state |= ST_RUNNING;
		if(!prtd->params->channel)
			dma_tx_init(prtd->dma_pos, prtd->dma_period);
		else
			dma_rx_init(prtd->dma_pos, prtd->dma_period);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		prtd->state &= ~ST_RUNNING;
		spin_unlock(&prtd->lock);
		if (prtd->params->channel == 0)
		{
			while(as3310_readl(HW_DMA1_CHENREG)&0x1)
			{
				if(cnt++>0x6000000)
				{
					printk("DMA0 BUSY\n");		
					return -1;
				}
			}
		}


		if (prtd->params->channel == 1)
		{
			while(as3310_readl(HW_DMA1_CHENREG)&0x2)
			{
				if(cnt++>0x6000000)
				{
					printk("DMA1 BUSY\n");		
					return -1;
				}
			}
		}
		spin_lock(&prtd->lock);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock(&prtd->lock);

	return ret;
}

static int asm9260_dma_getposition(dmach_t channel, dma_addr_t *src, dma_addr_t *dst)
{

	if (src != NULL){
		if(0 == channel)
 			*src = as3310_readl(HW_DMA1_SAR0);
		else 
			*src = as3310_readl(HW_DMA1_SAR1);
	}

 	if (dst != NULL){
		if(0 == channel)
 			*dst = as3310_readl(HW_DMA1_DAR0);
		else 
			*dst = as3310_readl(HW_DMA1_DAR1);
	}
 	return 0;
}

static snd_pcm_uframes_t
asm9260_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct asm9260_runtime_data *prtd = runtime->private_data;
	unsigned long res;
	dma_addr_t src, dst;

//	DBG("Entered %s\n", __func__);

	spin_lock(&prtd->lock);
	asm9260_dma_getposition(prtd->params->channel, &src, &dst);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		res = dst - prtd->dma_start;
	}
	else
	{
		res = src - prtd->dma_start;
	}

	spin_unlock(&prtd->lock);

	/* we seem to be getting the odd error from the pcm library due
	 * to out-of-bounds pointers. this is maybe due to the dma engine
	 * not having loaded the new values for the channel before being
	 * callled... (todo - fix )
	 */

	if (res >= snd_pcm_lib_buffer_bytes(substream)) {
		if (res == snd_pcm_lib_buffer_bytes(substream))
			res = 0;
	}

	return bytes_to_frames(substream->runtime, res);
}

static int asm9260_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct asm9260_runtime_data *prtd;

	DBG("Entered %s\n", __func__);

	snd_soc_set_runtime_hwparams(substream, &asm9260_pcm_hardware);

	prtd = kzalloc(sizeof(struct asm9260_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	spin_lock_init(&prtd->lock);

	runtime->private_data = prtd;
	return 0;
}

static int asm9260_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct asm9260_runtime_data *prtd = runtime->private_data;

	DBG("Entered %s\n", __func__);

	if (!prtd)
		DBG("asm9260_pcm_close called with prtd == NULL\n");

	kfree(prtd);

	return 0;
}

static int asm9260_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	DBG("Entered %s\n", __func__);

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

static struct snd_pcm_ops asm9260_pcm_ops = {
	.open		= asm9260_pcm_open,
	.close		= asm9260_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= asm9260_pcm_hw_params,
	.hw_free	= asm9260_pcm_hw_free,
	.prepare	= asm9260_pcm_prepare,
	.trigger	= asm9260_pcm_trigger,
	.pointer	= asm9260_pcm_pointer,
	.mmap		= asm9260_pcm_mmap,
};

static int asm9260_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = asm9260_pcm_hardware.buffer_bytes_max;

	DBG("Entered %s\n", __func__);

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void asm9260_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	DBG("Entered %s\n", __func__);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 asm9260_pcm_dmamask = DMA_32BIT_MASK;

static int asm9260_pcm_new(struct snd_card *card,
	struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret = 0;

	DBG("Entered %s\n", __func__);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &asm9260_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->playback.channels_min) {
		ret = asm9260_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = asm9260_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform asm9260_soc_platform = {
	.name		= "asm9260-audio",
	.pcm_ops 	= &asm9260_pcm_ops,
	.pcm_new	= asm9260_pcm_new,
	.pcm_free	= asm9260_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(asm9260_soc_platform);

MODULE_AUTHOR("Lujy, <lujy@alpscale.cn>");
MODULE_DESCRIPTION("ALPSCALE ASM9260 PCM DMA module");
MODULE_LICENSE("GPL");
