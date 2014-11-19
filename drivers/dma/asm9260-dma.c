/*
 * Copyright 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Refer to drivers/dma/imx-sdma.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/stmp_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/list.h>

#include <asm/irq.h>

#include "dmaengine.h"

/*
 * NOTE: The term "PIO" throughout the mxs-dma implementation means
 * PIO mode of mxs apbh-dma and apbx-dma.  With this working mode,
 * dma can program the controller registers of peripheral devices.
 */

#define dma_is_apbh(asm9260_dma)	((asm9260_dma)->type == MXS_DMA_APBH)
#define apbh_is_old(asm9260_dma)	((asm9260_dma)->dev_id == IMX23_DMA)

#define HW_APBHX_CTRL0				0x000
#define BM_APBH_CTRL0_APB_BURST8_EN		(1 << 29)
#define BM_APBH_CTRL0_APB_BURST_EN		(1 << 28)
#define BP_APBH_CTRL0_RESET_CHANNEL		16
#define HW_APBHX_CTRL1				0x010
#define HW_APBHX_CTRL2				0x020
#define HW_APBHX_CHANNEL_CTRL			0x030
#define BP_APBHX_CHANNEL_CTRL_RESET_CHANNEL	16
/*
 * The offset of NXTCMDAR register is different per both dma type and version,
 * while stride for each channel is all the same 0x70.
 */
#define HW_APBHX_CHn_NXTCMDAR(d, n) \
	(((dma_is_apbh(d) && apbh_is_old(d)) ? 0x050 : 0x110) + (n) * 0x70)
#define HW_APBHX_CHn_SEMA(d, n) \
	(((dma_is_apbh(d) && apbh_is_old(d)) ? 0x080 : 0x140) + (n) * 0x70)
#define HW_APBHX_CHn_BAR(d, n) \
	(((dma_is_apbh(d) && apbh_is_old(d)) ? 0x070 : 0x130) + (n) * 0x70)
#define HW_APBX_CHn_DEBUG1(d, n) (0x150 + (n) * 0x70)

/*
 * ccw bits definitions
 *
 * COMMAND:		0..1	(2)
 * CHAIN:		2	(1)
 * IRQ:			3	(1)
 * NAND_LOCK:		4	(1) - not implemented
 * NAND_WAIT4READY:	5	(1) - not implemented
 * DEC_SEM:		6	(1)
 * WAIT4END:		7	(1)
 * HALT_ON_TERMINATE:	8	(1)
 * TERMINATE_FLUSH:	9	(1)
 * RESERVED:		10..11	(2)
 * PIO_NUM:		12..15	(4)
 */
#define BP_CCW_COMMAND		0
#define BM_CCW_COMMAND		(3 << 0)
#define CCW_CHAIN		(1 << 2)
#define CCW_IRQ			(1 << 3)
#define CCW_DEC_SEM		(1 << 6)
#define CCW_WAIT4END		(1 << 7)
#define CCW_HALT_ON_TERM	(1 << 8)
#define CCW_TERM_FLUSH		(1 << 9)
#define BP_CCW_PIO_NUM		12
#define BM_CCW_PIO_NUM		(0xf << 12)

#define BF_CCW(value, field)	(((value) << BP_CCW_##field) & BM_CCW_##field)

#define MXS_DMA_CMD_NO_XFER	0
#define MXS_DMA_CMD_WRITE	1
#define MXS_DMA_CMD_READ	2
#define MXS_DMA_CMD_DMA_SENSE	3	/* not implemented */

struct asm9260_dma_ccw {
	u32		next;
	u16		bits;
	u16		xfer_bytes;
#define MAX_XFER_BYTES	0xff00
	u32		bufaddr;
#define MXS_PIO_WORDS	16
	u32		pio_words[MXS_PIO_WORDS];
};

#define CCW_BLOCK_SIZE	(4 * PAGE_SIZE)
#define NUM_CCW	(int)(CCW_BLOCK_SIZE / sizeof(struct asm9260_dma_ccw))

struct asm9260_dma_chan {
	struct asm9260_dma_engine		*asm9260_dma;
	struct dma_chan			chan;
	struct dma_async_tx_descriptor	desc;
	struct tasklet_struct		tasklet;
	unsigned int			chan_irq;
	struct asm9260_dma_ccw		*ccw;
	dma_addr_t			ccw_phys;
	int				desc_count;
	enum dma_status			status;
	unsigned int			flags;
	bool				reset;
#define MXS_DMA_SG_LOOP			(1 << 0)
#define MXS_DMA_USE_SEMAPHORE		(1 << 1)
};

#define MXS_DMA_CHANNELS		16
#define MXS_DMA_CHANNELS_MASK		0xffff

enum asm9260_dma_devtype {
	MXS_DMA_APBH,
	MXS_DMA_APBX,
};

enum asm9260_dma_id {
	IMX23_DMA,
};

struct asm9260_dma_engine {
	enum asm9260_dma_id			dev_id;
	enum asm9260_dma_devtype		type;
	void __iomem			*base;
	struct clk			*clk;
	struct dma_device		dma_device;
	struct device_dma_parameters	dma_parms;
	struct asm9260_dma_chan		asm9260_chans[MXS_DMA_CHANNELS];
	struct platform_device		*pdev;
	unsigned int			nr_channels;
};

struct asm9260_dma_type {
	enum asm9260_dma_id id;
	enum asm9260_dma_devtype type;
};

static struct asm9260_dma_type asm9260_dma_types[] = {
	{
		.id = IMX23_DMA,
		.type = MXS_DMA_APBH,
	}, {
		.id = IMX23_DMA,
		.type = MXS_DMA_APBX,
	}
};

static struct platform_device_id asm9260_dma_ids[] = {
	{
		.name = "imx23-dma-apbh",
		.driver_data = (kernel_ulong_t) &asm9260_dma_types[0],
	}, {
		.name = "imx23-dma-apbx",
		.driver_data = (kernel_ulong_t) &asm9260_dma_types[1],
	}, {
		/* end of list */
	}
};

static const struct of_device_id asm9260_dma_dt_ids[] = {
	{ .compatible = "fsl,imx23-dma-apbh", .data = &asm9260_dma_ids[0], },
	{ .compatible = "fsl,imx23-dma-apbx", .data = &asm9260_dma_ids[1], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, asm9260_dma_dt_ids);

static struct asm9260_dma_chan *to_asm9260_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct asm9260_dma_chan, chan);
}

static void asm9260_dma_reset_chan(struct asm9260_dma_chan *asm9260_chan)
{
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	int chan_id = asm9260_chan->chan.chan_id;

	/*
	 * mxs dma channel resets can cause a channel stall. To recover from a
	 * channel stall, we have to reset the whole DMA engine. To avoid this,
	 * we use cyclic DMA with semaphores, that are enhanced in
	 * asm9260_dma_int_handler. To reset the channel, we can simply stop writing
	 * into the semaphore counter.
	 */
	if (asm9260_chan->flags & MXS_DMA_USE_SEMAPHORE &&
			asm9260_chan->flags & MXS_DMA_SG_LOOP) {
		asm9260_chan->reset = true;
	} else if (dma_is_apbh(asm9260_dma) && apbh_is_old(asm9260_dma)) {
		writel(1 << (chan_id + BP_APBH_CTRL0_RESET_CHANNEL),
			asm9260_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_SET);
	} else {
		unsigned long elapsed = 0;
		const unsigned long max_wait = 50000; /* 50ms */
		void __iomem *reg_dbg1 = asm9260_dma->base +
				HW_APBX_CHn_DEBUG1(asm9260_dma, chan_id);

		/*
		 * On i.MX28 APBX, the DMA channel can stop working if we reset
		 * the channel while it is in READ_FLUSH (0x08) state.
		 * We wait here until we leave the state. Then we trigger the
		 * reset. Waiting a maximum of 50ms, the kernel shouldn't crash
		 * because of this.
		 */
		while ((readl(reg_dbg1) & 0xf) == 0x8 && elapsed < max_wait) {
			udelay(100);
			elapsed += 100;
		}

		if (elapsed >= max_wait)
			dev_err(&asm9260_chan->asm9260_dma->pdev->dev,
					"Failed waiting for the DMA channel %d to leave state READ_FLUSH, trying to reset channel in READ_FLUSH state now\n",
					chan_id);

		writel(1 << (chan_id + BP_APBHX_CHANNEL_CTRL_RESET_CHANNEL),
			asm9260_dma->base + HW_APBHX_CHANNEL_CTRL + STMP_OFFSET_REG_SET);
	}

	asm9260_chan->status = DMA_COMPLETE;
}

static void asm9260_dma_enable_chan(struct asm9260_dma_chan *asm9260_chan)
{
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	int chan_id = asm9260_chan->chan.chan_id;

	/* set cmd_addr up */
	writel(asm9260_chan->ccw_phys,
		asm9260_dma->base + HW_APBHX_CHn_NXTCMDAR(asm9260_dma, chan_id));

	/* write 1 to SEMA to kick off the channel */
	if (asm9260_chan->flags & MXS_DMA_USE_SEMAPHORE &&
			asm9260_chan->flags & MXS_DMA_SG_LOOP) {
		/* A cyclic DMA consists of at least 2 segments, so initialize
		 * the semaphore with 2 so we have enough time to add 1 to the
		 * semaphore if we need to */
		writel(2, asm9260_dma->base + HW_APBHX_CHn_SEMA(asm9260_dma, chan_id));
	} else {
		writel(1, asm9260_dma->base + HW_APBHX_CHn_SEMA(asm9260_dma, chan_id));
	}
	asm9260_chan->reset = false;
}

static void asm9260_dma_disable_chan(struct asm9260_dma_chan *asm9260_chan)
{
	asm9260_chan->status = DMA_COMPLETE;
}

static void asm9260_dma_pause_chan(struct asm9260_dma_chan *asm9260_chan)
{
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	int chan_id = asm9260_chan->chan.chan_id;

	/* freeze the channel */
	if (dma_is_apbh(asm9260_dma) && apbh_is_old(asm9260_dma))
		writel(1 << chan_id,
			asm9260_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_SET);
	else
		writel(1 << chan_id,
			asm9260_dma->base + HW_APBHX_CHANNEL_CTRL + STMP_OFFSET_REG_SET);

	asm9260_chan->status = DMA_PAUSED;
}

static void asm9260_dma_resume_chan(struct asm9260_dma_chan *asm9260_chan)
{
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	int chan_id = asm9260_chan->chan.chan_id;

	/* unfreeze the channel */
	if (dma_is_apbh(asm9260_dma) && apbh_is_old(asm9260_dma))
		writel(1 << chan_id,
			asm9260_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_CLR);
	else
		writel(1 << chan_id,
			asm9260_dma->base + HW_APBHX_CHANNEL_CTRL + STMP_OFFSET_REG_CLR);

	asm9260_chan->status = DMA_IN_PROGRESS;
}

static dma_cookie_t asm9260_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	return dma_cookie_assign(tx);
}

static void asm9260_dma_tasklet(unsigned long data)
{
	struct asm9260_dma_chan *asm9260_chan = (struct asm9260_dma_chan *) data;

	if (asm9260_chan->desc.callback)
		asm9260_chan->desc.callback(asm9260_chan->desc.callback_param);
}

static int asm9260_dma_irq_to_chan(struct asm9260_dma_engine *asm9260_dma, int irq)
{
	int i;

	for (i = 0; i != asm9260_dma->nr_channels; ++i)
		if (asm9260_dma->asm9260_chans[i].chan_irq == irq)
			return i;

	return -EINVAL;
}

static irqreturn_t asm9260_dma_int_handler(int irq, void *dev_id)
{
	struct asm9260_dma_engine *asm9260_dma = dev_id;
	struct asm9260_dma_chan *asm9260_chan;
	u32 completed;
	u32 err;
	int chan = asm9260_dma_irq_to_chan(asm9260_dma, irq);

	if (chan < 0)
		return IRQ_NONE;

	/* completion status */
	completed = readl(asm9260_dma->base + HW_APBHX_CTRL1);
	completed = (completed >> chan) & 0x1;

	/* Clear interrupt */
	writel((1 << chan),
			asm9260_dma->base + HW_APBHX_CTRL1 + STMP_OFFSET_REG_CLR);

	/* error status */
	err = readl(asm9260_dma->base + HW_APBHX_CTRL2);
	err &= (1 << (MXS_DMA_CHANNELS + chan)) | (1 << chan);

	/*
	 * error status bit is in the upper 16 bits, error irq bit in the lower
	 * 16 bits. We transform it into a simpler error code:
	 * err: 0x00 = no error, 0x01 = TERMINATION, 0x02 = BUS_ERROR
	 */
	err = (err >> (MXS_DMA_CHANNELS + chan)) + (err >> chan);

	/* Clear error irq */
	writel((1 << chan),
			asm9260_dma->base + HW_APBHX_CTRL2 + STMP_OFFSET_REG_CLR);

	/*
	 * When both completion and error of termination bits set at the
	 * same time, we do not take it as an error.  IOW, it only becomes
	 * an error we need to handle here in case of either it's a bus
	 * error or a termination error with no completion. 0x01 is termination
	 * error, so we can subtract err & completed to get the real error case.
	 */
	err -= err & completed;

	asm9260_chan = &asm9260_dma->asm9260_chans[chan];

	if (err) {
		dev_dbg(asm9260_dma->dma_device.dev,
			"%s: error in channel %d\n", __func__,
			chan);
		asm9260_chan->status = DMA_ERROR;
		asm9260_dma_reset_chan(asm9260_chan);
	} else if (asm9260_chan->status != DMA_COMPLETE) {
		if (asm9260_chan->flags & MXS_DMA_SG_LOOP) {
			asm9260_chan->status = DMA_IN_PROGRESS;
			if (asm9260_chan->flags & MXS_DMA_USE_SEMAPHORE)
				writel(1, asm9260_dma->base +
					HW_APBHX_CHn_SEMA(asm9260_dma, chan));
		} else {
			asm9260_chan->status = DMA_COMPLETE;
		}
	}

	if (asm9260_chan->status == DMA_COMPLETE) {
		if (asm9260_chan->reset)
			return IRQ_HANDLED;
		dma_cookie_complete(&asm9260_chan->desc);
	}

	/* schedule tasklet on this channel */
	tasklet_schedule(&asm9260_chan->tasklet);

	return IRQ_HANDLED;
}

static int asm9260_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct asm9260_dma_chan *asm9260_chan = to_asm9260_dma_chan(chan);
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	int ret;

	asm9260_chan->ccw = dma_zalloc_coherent(asm9260_dma->dma_device.dev,
					    CCW_BLOCK_SIZE,
					    &asm9260_chan->ccw_phys, GFP_KERNEL);
	if (!asm9260_chan->ccw) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	if (asm9260_chan->chan_irq != NO_IRQ) {
		ret = request_irq(asm9260_chan->chan_irq, asm9260_dma_int_handler,
					0, "mxs-dma", asm9260_dma);
		if (ret)
			goto err_irq;
	}

	ret = clk_prepare_enable(asm9260_dma->clk);
	if (ret)
		goto err_clk;

	asm9260_dma_reset_chan(asm9260_chan);

	dma_async_tx_descriptor_init(&asm9260_chan->desc, chan);
	asm9260_chan->desc.tx_submit = asm9260_dma_tx_submit;

	/* the descriptor is ready */
	async_tx_ack(&asm9260_chan->desc);

	return 0;

err_clk:
	free_irq(asm9260_chan->chan_irq, asm9260_dma);
err_irq:
	dma_free_coherent(asm9260_dma->dma_device.dev, CCW_BLOCK_SIZE,
			asm9260_chan->ccw, asm9260_chan->ccw_phys);
err_alloc:
	return ret;
}

static void asm9260_dma_free_chan_resources(struct dma_chan *chan)
{
	struct asm9260_dma_chan *asm9260_chan = to_asm9260_dma_chan(chan);
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;

	asm9260_dma_disable_chan(asm9260_chan);

	free_irq(asm9260_chan->chan_irq, asm9260_dma);

	dma_free_coherent(asm9260_dma->dma_device.dev, CCW_BLOCK_SIZE,
			asm9260_chan->ccw, asm9260_chan->ccw_phys);

	clk_disable_unprepare(asm9260_dma->clk);
}

/*
 * How to use the flags for ->device_prep_slave_sg() :
 *    [1] If there is only one DMA command in the DMA chain, the code should be:
 *            ......
 *            ->device_prep_slave_sg(DMA_CTRL_ACK);
 *            ......
 *    [2] If there are two DMA commands in the DMA chain, the code should be
 *            ......
 *            ->device_prep_slave_sg(0);
 *            ......
 *            ->device_prep_slave_sg(DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
 *            ......
 *    [3] If there are more than two DMA commands in the DMA chain, the code
 *        should be:
 *            ......
 *            ->device_prep_slave_sg(0);                                // First
 *            ......
 *            ->device_prep_slave_sg(DMA_PREP_INTERRUPT [| DMA_CTRL_ACK]);
 *            ......
 *            ->device_prep_slave_sg(DMA_PREP_INTERRUPT | DMA_CTRL_ACK); // Last
 *            ......
 */
static struct dma_async_tx_descriptor *asm9260_dma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct asm9260_dma_chan *asm9260_chan = to_asm9260_dma_chan(chan);
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	struct asm9260_dma_ccw *ccw;
	struct scatterlist *sg;
	u32 i, j;
	u32 *pio;
	bool append = flags & DMA_PREP_INTERRUPT;
	int idx = append ? asm9260_chan->desc_count : 0;

	if (asm9260_chan->status == DMA_IN_PROGRESS && !append)
		return NULL;

	if (sg_len + (append ? idx : 0) > NUM_CCW) {
		dev_err(asm9260_dma->dma_device.dev,
				"maximum number of sg exceeded: %d > %d\n",
				sg_len, NUM_CCW);
		goto err_out;
	}

	asm9260_chan->status = DMA_IN_PROGRESS;
	asm9260_chan->flags = 0;

	/*
	 * If the sg is prepared with append flag set, the sg
	 * will be appended to the last prepared sg.
	 */
	if (append) {
		BUG_ON(idx < 1);
		ccw = &asm9260_chan->ccw[idx - 1];
		ccw->next = asm9260_chan->ccw_phys + sizeof(*ccw) * idx;
		ccw->bits |= CCW_CHAIN;
		ccw->bits &= ~CCW_IRQ;
		ccw->bits &= ~CCW_DEC_SEM;
	} else {
		idx = 0;
	}

	if (direction == DMA_TRANS_NONE) {
		ccw = &asm9260_chan->ccw[idx++];
		pio = (u32 *) sgl;

		for (j = 0; j < sg_len;)
			ccw->pio_words[j++] = *pio++;

		ccw->bits = 0;
		ccw->bits |= CCW_IRQ;
		ccw->bits |= CCW_DEC_SEM;
		if (flags & DMA_CTRL_ACK)
			ccw->bits |= CCW_WAIT4END;
		ccw->bits |= CCW_HALT_ON_TERM;
		ccw->bits |= CCW_TERM_FLUSH;
		ccw->bits |= BF_CCW(sg_len, PIO_NUM);
		ccw->bits |= BF_CCW(MXS_DMA_CMD_NO_XFER, COMMAND);
	} else {
		for_each_sg(sgl, sg, sg_len, i) {
			if (sg_dma_len(sg) > MAX_XFER_BYTES) {
				dev_err(asm9260_dma->dma_device.dev, "maximum bytes for sg entry exceeded: %d > %d\n",
						sg_dma_len(sg), MAX_XFER_BYTES);
				goto err_out;
			}

			ccw = &asm9260_chan->ccw[idx++];

			ccw->next = asm9260_chan->ccw_phys + sizeof(*ccw) * idx;
			ccw->bufaddr = sg->dma_address;
			ccw->xfer_bytes = sg_dma_len(sg);

			ccw->bits = 0;
			ccw->bits |= CCW_CHAIN;
			ccw->bits |= CCW_HALT_ON_TERM;
			ccw->bits |= CCW_TERM_FLUSH;
			ccw->bits |= BF_CCW(direction == DMA_DEV_TO_MEM ?
					MXS_DMA_CMD_WRITE : MXS_DMA_CMD_READ,
					COMMAND);

			if (i + 1 == sg_len) {
				ccw->bits &= ~CCW_CHAIN;
				ccw->bits |= CCW_IRQ;
				ccw->bits |= CCW_DEC_SEM;
				if (flags & DMA_CTRL_ACK)
					ccw->bits |= CCW_WAIT4END;
			}
		}
	}
	asm9260_chan->desc_count = idx;

	return &asm9260_chan->desc;

err_out:
	asm9260_chan->status = DMA_ERROR;
	return NULL;
}

static struct dma_async_tx_descriptor *asm9260_dma_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct asm9260_dma_chan *asm9260_chan = to_asm9260_dma_chan(chan);
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	u32 num_periods = buf_len / period_len;
	u32 i = 0, buf = 0;

	if (asm9260_chan->status == DMA_IN_PROGRESS)
		return NULL;

	asm9260_chan->status = DMA_IN_PROGRESS;
	asm9260_chan->flags |= MXS_DMA_SG_LOOP;
	asm9260_chan->flags |= MXS_DMA_USE_SEMAPHORE;

	if (num_periods > NUM_CCW) {
		dev_err(asm9260_dma->dma_device.dev,
				"maximum number of sg exceeded: %d > %d\n",
				num_periods, NUM_CCW);
		goto err_out;
	}

	if (period_len > MAX_XFER_BYTES) {
		dev_err(asm9260_dma->dma_device.dev,
				"maximum period size exceeded: %d > %d\n",
				period_len, MAX_XFER_BYTES);
		goto err_out;
	}

	while (buf < buf_len) {
		struct asm9260_dma_ccw *ccw = &asm9260_chan->ccw[i];

		if (i + 1 == num_periods)
			ccw->next = asm9260_chan->ccw_phys;
		else
			ccw->next = asm9260_chan->ccw_phys + sizeof(*ccw) * (i + 1);

		ccw->bufaddr = dma_addr;
		ccw->xfer_bytes = period_len;

		ccw->bits = 0;
		ccw->bits |= CCW_CHAIN;
		ccw->bits |= CCW_IRQ;
		ccw->bits |= CCW_HALT_ON_TERM;
		ccw->bits |= CCW_TERM_FLUSH;
		ccw->bits |= CCW_DEC_SEM;
		ccw->bits |= BF_CCW(direction == DMA_DEV_TO_MEM ?
				MXS_DMA_CMD_WRITE : MXS_DMA_CMD_READ, COMMAND);

		dma_addr += period_len;
		buf += period_len;

		i++;
	}
	asm9260_chan->desc_count = i;

	return &asm9260_chan->desc;

err_out:
	asm9260_chan->status = DMA_ERROR;
	return NULL;
}

static int asm9260_dma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		unsigned long arg)
{
	struct asm9260_dma_chan *asm9260_chan = to_asm9260_dma_chan(chan);
	int ret = 0;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		asm9260_dma_reset_chan(asm9260_chan);
		asm9260_dma_disable_chan(asm9260_chan);
		break;
	case DMA_PAUSE:
		asm9260_dma_pause_chan(asm9260_chan);
		break;
	case DMA_RESUME:
		asm9260_dma_resume_chan(asm9260_chan);
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static enum dma_status asm9260_dma_tx_status(struct dma_chan *chan,
			dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct asm9260_dma_chan *asm9260_chan = to_asm9260_dma_chan(chan);
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	u32 residue = 0;

	if (asm9260_chan->status == DMA_IN_PROGRESS &&
			asm9260_chan->flags & MXS_DMA_SG_LOOP) {
		struct asm9260_dma_ccw *last_ccw;
		u32 bar;

		last_ccw = &asm9260_chan->ccw[asm9260_chan->desc_count - 1];
		residue = last_ccw->xfer_bytes + last_ccw->bufaddr;

		bar = readl(asm9260_dma->base +
				HW_APBHX_CHn_BAR(asm9260_dma, chan->chan_id));
		residue -= bar;
	}

	dma_set_tx_state(txstate, chan->completed_cookie, chan->cookie,
			residue);

	return asm9260_chan->status;
}

static void asm9260_dma_issue_pending(struct dma_chan *chan)
{
	struct asm9260_dma_chan *asm9260_chan = to_asm9260_dma_chan(chan);

	asm9260_dma_enable_chan(asm9260_chan);
}

static int __init asm9260_dma_init(struct asm9260_dma_engine *asm9260_dma)
{
	int ret;

	ret = clk_prepare_enable(asm9260_dma->clk);
	if (ret)
		return ret;

	ret = stmp_reset_block(asm9260_dma->base);
	if (ret)
		goto err_out;

	/* enable apbh burst */
	if (dma_is_apbh(asm9260_dma)) {
		writel(BM_APBH_CTRL0_APB_BURST_EN,
			asm9260_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_SET);
		writel(BM_APBH_CTRL0_APB_BURST8_EN,
			asm9260_dma->base + HW_APBHX_CTRL0 + STMP_OFFSET_REG_SET);
	}

	/* enable irq for all the channels */
	writel(MXS_DMA_CHANNELS_MASK << MXS_DMA_CHANNELS,
		asm9260_dma->base + HW_APBHX_CTRL1 + STMP_OFFSET_REG_SET);

err_out:
	clk_disable_unprepare(asm9260_dma->clk);
	return ret;
}

struct asm9260_dma_filter_param {
	struct device_node *of_node;
	unsigned int chan_id;
};

static bool asm9260_dma_filter_fn(struct dma_chan *chan, void *fn_param)
{
	struct asm9260_dma_filter_param *param = fn_param;
	struct asm9260_dma_chan *asm9260_chan = to_asm9260_dma_chan(chan);
	struct asm9260_dma_engine *asm9260_dma = asm9260_chan->asm9260_dma;
	int chan_irq;

	if (asm9260_dma->dma_device.dev->of_node != param->of_node)
		return false;

	if (chan->chan_id != param->chan_id)
		return false;

	chan_irq = platform_get_irq(asm9260_dma->pdev, param->chan_id);
	if (chan_irq < 0)
		return false;

	asm9260_chan->chan_irq = chan_irq;

	return true;
}

static struct dma_chan *asm9260_dma_xlate(struct of_phandle_args *dma_spec,
			       struct of_dma *ofdma)
{
	struct asm9260_dma_engine *asm9260_dma = ofdma->of_dma_data;
	dma_cap_mask_t mask = asm9260_dma->dma_device.cap_mask;
	struct asm9260_dma_filter_param param;

	if (dma_spec->args_count != 1)
		return NULL;

	param.of_node = ofdma->of_node;
	param.chan_id = dma_spec->args[0];

	if (param.chan_id >= asm9260_dma->nr_channels)
		return NULL;

	return dma_request_channel(mask, asm9260_dma_filter_fn, &param);
}

static int __init asm9260_dma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct platform_device_id *id_entry;
	const struct of_device_id *of_id;
	const struct asm9260_dma_type *dma_type;
	struct asm9260_dma_engine *asm9260_dma;
	struct resource *iores;
	int ret, i;

	asm9260_dma = devm_kzalloc(&pdev->dev, sizeof(*asm9260_dma), GFP_KERNEL);
	if (!asm9260_dma)
		return -ENOMEM;

	ret = of_property_read_u32(np, "dma-channels", &asm9260_dma->nr_channels);
	if (ret) {
		dev_err(&pdev->dev, "failed to read dma-channels\n");
		return ret;
	}

	of_id = of_match_device(asm9260_dma_dt_ids, &pdev->dev);
	if (of_id)
		id_entry = of_id->data;
	else
		id_entry = platform_get_device_id(pdev);

	dma_type = (struct asm9260_dma_type *)id_entry->driver_data;
	asm9260_dma->type = dma_type->type;
	asm9260_dma->dev_id = dma_type->id;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	asm9260_dma->base = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(asm9260_dma->base))
		return PTR_ERR(asm9260_dma->base);

	asm9260_dma->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(asm9260_dma->clk))
		return PTR_ERR(asm9260_dma->clk);

	dma_cap_set(DMA_SLAVE, asm9260_dma->dma_device.cap_mask);
	dma_cap_set(DMA_CYCLIC, asm9260_dma->dma_device.cap_mask);

	INIT_LIST_HEAD(&asm9260_dma->dma_device.channels);

	/* Initialize channel parameters */
	for (i = 0; i < MXS_DMA_CHANNELS; i++) {
		struct asm9260_dma_chan *asm9260_chan = &asm9260_dma->asm9260_chans[i];

		asm9260_chan->asm9260_dma = asm9260_dma;
		asm9260_chan->chan.device = &asm9260_dma->dma_device;
		dma_cookie_init(&asm9260_chan->chan);

		tasklet_init(&asm9260_chan->tasklet, asm9260_dma_tasklet,
			     (unsigned long) asm9260_chan);


		/* Add the channel to asm9260_chan list */
		list_add_tail(&asm9260_chan->chan.device_node,
			&asm9260_dma->dma_device.channels);
	}

	ret = asm9260_dma_init(asm9260_dma);
	if (ret)
		return ret;

	asm9260_dma->pdev = pdev;
	asm9260_dma->dma_device.dev = &pdev->dev;

	/* asm9260_dma gets 65535 bytes maximum sg size */
	asm9260_dma->dma_device.dev->dma_parms = &asm9260_dma->dma_parms;
	dma_set_max_seg_size(asm9260_dma->dma_device.dev, MAX_XFER_BYTES);

	asm9260_dma->dma_device.device_alloc_chan_resources = asm9260_dma_alloc_chan_resources;
	asm9260_dma->dma_device.device_free_chan_resources = asm9260_dma_free_chan_resources;
	asm9260_dma->dma_device.device_tx_status = asm9260_dma_tx_status;
	asm9260_dma->dma_device.device_prep_slave_sg = asm9260_dma_prep_slave_sg;
	asm9260_dma->dma_device.device_prep_dma_cyclic = asm9260_dma_prep_dma_cyclic;
	asm9260_dma->dma_device.device_control = asm9260_dma_control;
	asm9260_dma->dma_device.device_issue_pending = asm9260_dma_issue_pending;

	ret = dma_async_device_register(&asm9260_dma->dma_device);
	if (ret) {
		dev_err(asm9260_dma->dma_device.dev, "unable to register\n");
		return ret;
	}

	ret = of_dma_controller_register(np, asm9260_dma_xlate, asm9260_dma);
	if (ret) {
		dev_err(asm9260_dma->dma_device.dev,
			"failed to register controller\n");
		dma_async_device_unregister(&asm9260_dma->dma_device);
	}

	dev_info(asm9260_dma->dma_device.dev, "initialized\n");

	return 0;
}

static struct platform_driver asm9260_dma_driver = {
	.driver		= {
		.name	= "mxs-dma",
		.of_match_table = asm9260_dma_dt_ids,
	},
	.id_table	= asm9260_dma_ids,
};

static int __init asm9260_dma_module_init(void)
{
	return platform_driver_probe(&asm9260_dma_driver, asm9260_dma_probe);
}
subsys_initcall(asm9260_dma_module_init);
