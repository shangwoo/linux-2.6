/*******************************************************************************
  DWC Ether MAC 10/100 Universal version 4.0 has been used for developing
  this code.

  This contains the functions to handle the dma.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: AlphaScale
*******************************************************************************/

#include <asm/io.h>
#include <mach/mac.h>
#include <mach/io.h>
#include "dwmac100.h"
#include "dwmac_dma.h"

#define DMABNR_INIT     0x02C16000

static int dwmac100_dma_init(void __iomem *ioaddr, int pbl, int fb,
			     int mb, int burst_len, u32 dma_tx, u32 dma_rx)
{
	/* Enable Application Access by writing to DMABMR */
        as3310_writel(DMABNR_INIT,HW_ETH_DMABMR);

	/* Mask interrupts by writing to CSR7 */
	as3310_writel(DMA_INTR_DEFAULT_MASK, HW_ETH_BASE_ADDR + DMA_INTR_ENA);

	/* The base address of the RX/TX descriptor lists must be written into
	 * DMA CSR3 and CSR4, respectively. */
	as3310_writel(dma_tx, HW_ETH_BASE_ADDR + DMA_TX_BASE_ADDR);
	as3310_writel(dma_rx, HW_ETH_BASE_ADDR + DMA_RCV_BASE_ADDR);

	return 0;
}

/* Store and Forward capability is not used at all..
 * The transmit threshold can be programmed by
 * setting the TTC bits in the DMA control register.*/
static void dwmac100_dma_operation_mode(void __iomem *ioaddr, int txmode,
					int rxmode)
{
	u32 csr6 = as3310_readl(HW_ETH_BASE_ADDR + DMA_CONTROL);

	if (txmode <= 32)
		csr6 |= DMA_CONTROL_TTC_32;
	else if (txmode <= 64)
		csr6 |= DMA_CONTROL_TTC_64;
	else
		csr6 |= DMA_CONTROL_TTC_128;

	as3310_writel(csr6, HW_ETH_BASE_ADDR + DMA_CONTROL);
}

static void dwmac100_dump_dma_regs(void __iomem *ioaddr)
{
	int i;

	CHIP_DBG(KERN_DEBUG "DWMAC 100 DMA CSR\n");
	for (i = 0; i < 9; i++)
		pr_debug("\t CSR%d (offset 0x%x): 0x%08x\n", i,
		       (DMA_BUS_MODE + i * 4),
		       as3310_readl(HW_ETH_BASE_ADDR + DMA_BUS_MODE + i * 4));
	CHIP_DBG(KERN_DEBUG "\t CSR20 (offset 0x%x): 0x%08x\n",
	    DMA_CUR_TX_BUF_ADDR, readl(ioaddr + DMA_CUR_TX_BUF_ADDR));
	CHIP_DBG(KERN_DEBUG "\t CSR21 (offset 0x%x): 0x%08x\n",
	    DMA_CUR_RX_BUF_ADDR, readl(ioaddr + DMA_CUR_RX_BUF_ADDR));
}

/* DMA controller has two counters to track the number of
 * the receive missed frames. */
static void dwmac100_dma_diagnostic_fr(void *data, struct asmmac_extra_stats *x,
				       void __iomem *ioaddr)
{
	struct net_device_stats *stats = (struct net_device_stats *)data;
	u32 csr8 = as3310_readl(HW_ETH_BASE_ADDR + DMA_MISSED_FRAME_CTR);

	if (unlikely(csr8)) {
		if (csr8 & DMA_MISSED_FRAME_OVE) {
			stats->rx_over_errors += 0x800;
			x->rx_overflow_cntr += 0x800;
		} else {
			unsigned int ove_cntr;
			ove_cntr = ((csr8 & DMA_MISSED_FRAME_OVE_CNTR) >> 17);
			stats->rx_over_errors += ove_cntr;
			x->rx_overflow_cntr += ove_cntr;
		}

		if (csr8 & DMA_MISSED_FRAME_OVE_M) {
			stats->rx_missed_errors += 0xffff;
			x->rx_missed_cntr += 0xffff;
		} else {
			unsigned int miss_f = (csr8 & DMA_MISSED_FRAME_M_CNTR);
			stats->rx_missed_errors += miss_f;
			x->rx_missed_cntr += miss_f;
		}
	}
}

const struct asmmac_dma_ops dwmac100_dma_ops = {
	.init = dwmac100_dma_init,
	.dump_regs = dwmac100_dump_dma_regs,
	.dma_mode = dwmac100_dma_operation_mode,
	.dma_diagnostic_fr = dwmac100_dma_diagnostic_fr,
	.enable_dma_transmission = dwmac_enable_dma_transmission,
	.enable_dma_irq = dwmac_enable_dma_irq,
	.disable_dma_irq = dwmac_disable_dma_irq,
	.start_tx = dwmac_dma_start_tx,
	.stop_tx = dwmac_dma_stop_tx,
	.start_rx = dwmac_dma_start_rx,
	.stop_rx = dwmac_dma_stop_rx,
	.dma_interrupt = dwmac_dma_interrupt,
};
