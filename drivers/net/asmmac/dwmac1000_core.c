/*******************************************************************************
  This is the driver for the GMAC on-chip Ethernet controller for AlphaScale SoCs.

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

#include <linux/crc32.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <mach/io.h>
#include <mach/mac.h>
#include "dwmac1000.h"

#define MAC1000_HASH_HIGH        0xffffffff
#define MAC1000_HASH_LOW         0xffffffff

static void dwmac1000_core_init(void __iomem *ioaddr)
{
	u32 value = as3310_readl(HW_ETH_BASE_ADDR + GMAC_CONTROL);
	value |= GMAC_CORE_INIT;
	as3310_writel(value, HW_ETH_BASE_ADDR + GMAC_CONTROL);

	/* Mask GMAC interrupts */
	as3310_writel(0x207, HW_ETH_BASE_ADDR + GMAC_INT_MASK);

#ifdef STMMAC_VLAN_TAG_USED
	/* Tag detection without filtering */
	as3310_writel(0x0, HW_ETH_BASE_ADDR + GMAC_VLAN_TAG);
#endif
}

static int dwmac1000_rx_ipc_enable(void __iomem *ioaddr)
{
	u32 value = as3310_readl(HW_ETH_BASE_ADDR + GMAC_CONTROL);

	value |= GMAC_CONTROL_IPC;
	as3310_writel(value, HW_ETH_BASE_ADDR + GMAC_CONTROL);

	value = as3310_readl(HW_ETH_BASE_ADDR + GMAC_CONTROL);

	return !!(value & GMAC_CONTROL_IPC);
}

static void dwmac1000_dump_regs(void __iomem *ioaddr)
{
	int i;
	pr_info("\tDWMAC1000 regs (base addr = 0x%p)\n", ioaddr);

	for (i = 0; i < 55; i++) {
		int offset = i * 4;
		pr_info("\tReg No. %d (offset 0x%x): 0x%08x\n", i,
			offset, as3310_readl(HW_ETH_BASE_ADDR + offset));
	}
}

static void dwmac1000_set_umac_addr(void __iomem *ioaddr, unsigned char *addr,
				unsigned int reg_n)
{
	asmmac_set_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
				GMAC_ADDR_LOW(reg_n));
}

static void dwmac1000_get_umac_addr(void __iomem *ioaddr, unsigned char *addr,
				unsigned int reg_n)
{
	asmmac_get_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
				GMAC_ADDR_LOW(reg_n));
}

static void dwmac1000_set_filter(struct net_device *dev, int id)
{
        int i;
	unsigned int value = 0;
	unsigned int perfect_addr_number;
        struct dev_mc_list *mcptr = dev->mc_list;

	CHIP_DBG(KERN_INFO "%s: # mcasts %d, # unicast %d\n",
		 __func__, dev->mc_count, netdev_uc_count(dev));

	if (dev->flags & IFF_PROMISC)
		value = GMAC_FRAME_FILTER_PR;
	else if ((dev->mc_count > HASH_TABLE_SIZE)
		   || (dev->flags & IFF_ALLMULTI)) {
		value = GMAC_FRAME_FILTER_PM;	/* pass all multi */
		as3310_writel(MAC1000_HASH_HIGH, HW_ETH_BASE_ADDR + GMAC_HASH_HIGH);
		as3310_writel(MAC1000_HASH_LOW, HW_ETH_BASE_ADDR + GMAC_HASH_LOW);
	} else if (dev->mc_count!=0) {
		u32 mc_filter[2];

		/* Hash filter for multicast */
		value = GMAC_FRAME_FILTER_HMC;

		memset(mc_filter, 0, sizeof(mc_filter));
		//netdev_for_each_mc_addr(ha, dev) {
                for (i = 0; i < dev->mc_count; i++, mcptr = mcptr->next) {
			/* The upper 6 bits of the calculated CRC are used to
			   index the contens of the hash table */
                        int bit_nr=0;
			/* The most significant bit determines the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register. */
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		}
		as3310_writel(mc_filter[0], HW_ETH_BASE_ADDR + GMAC_HASH_LOW);
		as3310_writel(mc_filter[1], HW_ETH_BASE_ADDR + GMAC_HASH_HIGH);
	}

	/* Extra 16 regs are available in cores newer than the 3.40. */
	if (id > DWMAC_CORE_3_40)
		perfect_addr_number = GMAC_MAX_PERFECT_ADDRESSES;
	else
		perfect_addr_number = GMAC_MAX_PERFECT_ADDRESSES / 2;

	/* Handle multiple unicast addresses (perfect filtering)*/
	if (dev->uc_count > perfect_addr_number)
		/* Switch to promiscuous mode is more than 16 addrs
		   are required */
		value |= GMAC_FRAME_FILTER_PR;
	else {

	}

#ifdef FRAME_FILTER_DEBUG
	/* Enable Receive all mode (to debug filtering_fail errors) */
	value |= GMAC_FRAME_FILTER_RA;
#endif
	as3310_writel(value, HW_ETH_BASE_ADDR + GMAC_FRAME_FILTER);

	CHIP_DBG(KERN_INFO "\tFrame Filter reg: 0x%08x\n\tHash regs: "
	    "HI 0x%08x, LO 0x%08x\n", as3310_readl(HW_ETH_BASE_ADDR + GMAC_FRAME_FILTER),
	    as3310_readl(HW_ETH_BASE_ADDR + GMAC_HASH_HIGH), as3310_readl(HW_ETH_BASE_ADDR + GMAC_HASH_LOW));
}

static void dwmac1000_flow_ctrl(void __iomem *ioaddr, unsigned int duplex,
			   unsigned int fc, unsigned int pause_time)
{
	unsigned int flow = 0;

	CHIP_DBG(KERN_DEBUG "GMAC Flow-Control:\n");
	if (fc & FLOW_RX) {
		CHIP_DBG(KERN_DEBUG "\tReceive Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_RFE;
	}
	if (fc & FLOW_TX) {
		CHIP_DBG(KERN_DEBUG "\tTransmit Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_TFE;
	}

	if (duplex) {
		CHIP_DBG(KERN_DEBUG "\tduplex mode: PAUSE %d\n", pause_time);
		flow |= (pause_time << GMAC_FLOW_CTRL_PT_SHIFT);
	}

	as3310_writel(flow, HW_ETH_BASE_ADDR + GMAC_FLOW_CTRL);
}

static void dwmac1000_pmt(void __iomem *ioaddr, unsigned long mode)
{
	unsigned int pmt = 0;

	if (mode & WAKE_MAGIC) {
		CHIP_DBG(KERN_DEBUG "GMAC: WOL Magic frame\n");
		pmt |= power_down | magic_pkt_en;
	}
	if (mode & WAKE_UCAST) {
		CHIP_DBG(KERN_DEBUG "GMAC: WOL on global unicast\n");
		pmt |= global_unicast;
	}

	as3310_writel(pmt, HW_ETH_BASE_ADDR + GMAC_PMT);
}


static int dwmac1000_irq_status(void __iomem *ioaddr)
{
	u32 intr_status = as3310_readl(HW_ETH_BASE_ADDR + GMAC_INT_STATUS);
	int status = 0;

	/* Not used events (e.g. MMC interrupts) are not handled. */
	if ((intr_status & mmc_tx_irq)) {
		CHIP_DBG(KERN_INFO "GMAC: MMC tx interrupt: 0x%08x\n",
		    as3310_readl(HW_ETH_BASE_ADDR + GMAC_MMC_TX_INTR));
		status |= core_mmc_tx_irq;
	}
	if (unlikely(intr_status & mmc_rx_irq)) {
		CHIP_DBG(KERN_INFO "GMAC: MMC rx interrupt: 0x%08x\n",
		    as3310_readl(HW_ETH_BASE_ADDR + GMAC_MMC_RX_INTR));
		status |= core_mmc_rx_irq;
	}
	if (unlikely(intr_status & mmc_rx_csum_offload_irq)) {
		CHIP_DBG(KERN_INFO "GMAC: MMC rx csum offload: 0x%08x\n",
		    as3310_readl(HW_ETH_BASE_ADDR + GMAC_MMC_RX_CSUM_OFFLOAD));
		status |= core_mmc_rx_csum_offload_irq;
	}
	if (unlikely(intr_status & pmt_irq)) {
		CHIP_DBG(KERN_INFO "GMAC: received Magic frame\n");
		/* clear the PMT bits 5 and 6 by reading the PMT
		 * status register. */
		as3310_readl(HW_ETH_BASE_ADDR + GMAC_PMT);
		status |= core_irq_receive_pmt_irq;
	}
	/* MAC trx/rx EEE LPI entry/exit interrupts */
	if (intr_status & lpiis_irq) {
		/* Clean LPI interrupt by reading the Reg 12 */
		u32 lpi_status = as3310_readl(HW_ETH_BASE_ADDR + LPI_CTRL_STATUS);

		if (lpi_status & LPI_CTRL_STATUS_TLPIEN) {
			CHIP_DBG(KERN_INFO "GMAC TX entered in LPI\n");
			status |= core_irq_tx_path_in_lpi_mode;
		}
		if (lpi_status & LPI_CTRL_STATUS_TLPIEX) {
			CHIP_DBG(KERN_INFO "GMAC TX exit from LPI\n");
			status |= core_irq_tx_path_exit_lpi_mode;
		}
		if (lpi_status & LPI_CTRL_STATUS_RLPIEN) {
			CHIP_DBG(KERN_INFO "GMAC RX entered in LPI\n");
			status |= core_irq_rx_path_in_lpi_mode;
		}
		if (lpi_status & LPI_CTRL_STATUS_RLPIEX) {
			CHIP_DBG(KERN_INFO "GMAC RX exit from LPI\n");
			status |= core_irq_rx_path_exit_lpi_mode;
		}
	}

	return status;
}

static void  dwmac1000_set_eee_mode(void __iomem *ioaddr)
{
	u32 value;

	/* Enable the link status receive on RGMII, SGMII ore SMII
	 * receive path and instruct the transmit to enter in LPI
	 * state. */
	value = as3310_readl(HW_ETH_BASE_ADDR + LPI_CTRL_STATUS);
	value |= LPI_CTRL_STATUS_LPIEN | LPI_CTRL_STATUS_LPITXA;
	as3310_writel(value, HW_ETH_BASE_ADDR + LPI_CTRL_STATUS);
}

static void  dwmac1000_reset_eee_mode(void __iomem *ioaddr)
{
	u32 value;

	value = as3310_readl(HW_ETH_BASE_ADDR + LPI_CTRL_STATUS);
	value &= ~(LPI_CTRL_STATUS_LPIEN | LPI_CTRL_STATUS_LPITXA);
	as3310_writel(value, HW_ETH_BASE_ADDR + LPI_CTRL_STATUS);
}

static void  dwmac1000_set_eee_pls(void __iomem *ioaddr, int link)
{
	u32 value;

	value = as3310_readl(HW_ETH_BASE_ADDR + LPI_CTRL_STATUS);

	if (link)
		value |= LPI_CTRL_STATUS_PLS;
	else
		value &= ~LPI_CTRL_STATUS_PLS;

	as3310_writel(value, HW_ETH_BASE_ADDR + LPI_CTRL_STATUS);
}

static void  dwmac1000_set_eee_timer(void __iomem *ioaddr, int ls, int tw)
{
	int value = ((tw & 0xffff)) | ((ls & 0x7ff) << 16);

	/* Program the timers in the LPI timer control register:
	 * LS: minimum time (ms) for which the link
	 *  status from PHY should be ok before transmitting
	 *  the LPI pattern.
	 * TW: minimum time (us) for which the core waits
	 *  after it has stopped transmitting the LPI pattern.
	 */
	as3310_writel(value, HW_ETH_BASE_ADDR + LPI_TIMER_CTRL);
}

static const struct asmmac_ops dwmac1000_ops = {
	.core_init = dwmac1000_core_init,
	.rx_ipc = dwmac1000_rx_ipc_enable,
	.dump_regs = dwmac1000_dump_regs,
	.host_irq_status = dwmac1000_irq_status,
	.set_filter = dwmac1000_set_filter,
	.flow_ctrl = dwmac1000_flow_ctrl,
	.pmt = dwmac1000_pmt,
	.set_umac_addr = dwmac1000_set_umac_addr,
	.get_umac_addr = dwmac1000_get_umac_addr,
	.set_eee_mode =  dwmac1000_set_eee_mode,
	.reset_eee_mode =  dwmac1000_reset_eee_mode,
	.set_eee_timer =  dwmac1000_set_eee_timer,
	.set_eee_pls =  dwmac1000_set_eee_pls,
};

struct mac_device_info *dwmac1000_setup(void __iomem *ioaddr)
{
	struct mac_device_info *mac;
	u32 hwid = as3310_readl(HW_ETH_BASE_ADDR + GMAC_VERSION);

	mac = kzalloc(sizeof(const struct mac_device_info), GFP_KERNEL);
	if (!mac)
		return NULL;

	mac->mac = &dwmac1000_ops;
	mac->dma = &dwmac1000_dma_ops;

	mac->link.port = GMAC_CONTROL_PS;
	mac->link.duplex = GMAC_CONTROL_DM;
	mac->link.speed = GMAC_CONTROL_FES;
	mac->mii.addr = GMAC_MII_ADDR;
	mac->mii.data = GMAC_MII_DATA;
	mac->synopsys_uid = hwid;

	return mac;
}
