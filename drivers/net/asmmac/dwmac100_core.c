/*******************************************************************************
  DWC Ether MAC 10/100 Universal version 4.0 has been used for developing
  this code.

  This only implements the mac core functions for this chip.

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
#include <asm/io.h>
#include <mach/io.h>
#include <mach/mac.h>
#include "dwmac100.h"

#define HASH_HIGH_REGNUM      0xffffffff
#define HASH_LOW_REGNUM       0xffffffff

static void dwmac100_core_init(void __iomem *ioaddr)
{
	u32 value = as3310_readl(HW_ETH_BASE_ADDR + MAC_CONTROL);

	as3310_writel((value | MAC_CORE_INIT), HW_ETH_BASE_ADDR + MAC_CONTROL);

#ifdef STMMAC_VLAN_TAG_USED
	as3310_writel(ETH_P_8021Q, HW_ETH_BASE_ADDR + MAC_VLAN1);
#endif
}

static void dwmac100_dump_mac_regs(void __iomem *ioaddr)
{
	pr_info("\t----------------------------------------------\n"
		"\t  DWMAC 100 CSR (base addr = 0x%p)\n"
		"\t----------------------------------------------\n",
		ioaddr);
	pr_info("\tcontrol reg (offset 0x%x): 0x%08x\n", MAC_CONTROL,
		as3310_readl(HW_ETH_BASE_ADDR + MAC_CONTROL));
	pr_info("\taddr HI (offset 0x%x): 0x%08x\n ", MAC_ADDR_HIGH,
		as3310_readl(HW_ETH_BASE_ADDR + MAC_ADDR_HIGH));
	pr_info("\taddr LO (offset 0x%x): 0x%08x\n", MAC_ADDR_LOW,
		as3310_readl(HW_ETH_BASE_ADDR + MAC_ADDR_LOW));
	pr_info("\tmulticast hash HI (offset 0x%x): 0x%08x\n",
		MAC_HASH_HIGH, as3310_readl(HW_ETH_BASE_ADDR + MAC_HASH_HIGH));
	pr_info("\tmulticast hash LO (offset 0x%x): 0x%08x\n",
		MAC_HASH_LOW, as3310_readl(HW_ETH_BASE_ADDR + MAC_HASH_LOW));
	pr_info("\tflow control (offset 0x%x): 0x%08x\n",
		MAC_FLOW_CTRL, as3310_readl(HW_ETH_BASE_ADDR + MAC_FLOW_CTRL));
	pr_info("\tVLAN1 tag (offset 0x%x): 0x%08x\n", MAC_VLAN1,
		as3310_readl(HW_ETH_BASE_ADDR + MAC_VLAN1));
	pr_info("\tVLAN2 tag (offset 0x%x): 0x%08x\n", MAC_VLAN2,
		as3310_readl(HW_ETH_BASE_ADDR + MAC_VLAN2));
}

static int dwmac100_rx_ipc_enable(void __iomem *ioaddr)
{
	return 0;
}

static int dwmac100_irq_status(void __iomem *ioaddr)
{
	return 0;
}

static void dwmac100_set_umac_addr(void __iomem *ioaddr, unsigned char *addr,
				   unsigned int reg_n)
{
	asmmac_set_mac_addr(ioaddr, addr, MAC_ADDR_HIGH, MAC_ADDR_LOW);
}

static void dwmac100_get_umac_addr(void __iomem *ioaddr, unsigned char *addr,
				   unsigned int reg_n)
{
	asmmac_get_mac_addr(ioaddr, addr, MAC_ADDR_HIGH, MAC_ADDR_LOW);
}

static void dwmac100_set_filter(struct net_device *dev, int id)
{
        int i;
        int bit_nr;
	u32 value = as3310_readl(HW_ETH_BASE_ADDR + MAC_CONTROL);
        struct dev_mc_list *mcptr = dev->mc_list;

	if (dev->flags & IFF_PROMISC) {
		value |= MAC_CONTROL_PR;
		value &= ~(MAC_CONTROL_PM | MAC_CONTROL_IF | MAC_CONTROL_HO |
			   MAC_CONTROL_HP);
	} else if ((dev->mc_count > HASH_TABLE_SIZE)
		   || (dev->flags & IFF_ALLMULTI)) {
		value |= MAC_CONTROL_PM;
		value &= ~(MAC_CONTROL_PR | MAC_CONTROL_IF | MAC_CONTROL_HO);
		as3310_writel(HASH_HIGH_REGNUM, HW_ETH_BASE_ADDR + MAC_HASH_HIGH);
		as3310_writel(HASH_LOW_REGNUM, HW_ETH_BASE_ADDR + MAC_HASH_LOW);
	} else if (dev->mc_count == 0) {	/* no multicast */
		value &= ~(MAC_CONTROL_PM | MAC_CONTROL_PR | MAC_CONTROL_IF |
			   MAC_CONTROL_HO | MAC_CONTROL_HP);
	} else {
		u32 mc_filter[2];
		/* Perfect filter mode for physical address and Hash
		   filter for multicast */
		value |= MAC_CONTROL_HP;
		value &= ~(MAC_CONTROL_PM | MAC_CONTROL_PR |
			   MAC_CONTROL_IF | MAC_CONTROL_HO);

		memset(mc_filter, 0, sizeof(mc_filter));
		//netdev_for_each_mc_addr(ha, dev) {
                for (i = 0; i < dev->mc_count; i++, mcptr = mcptr->next){
			/* The upper 6 bits of the calculated CRC are used to
			 * index the contens of the hash table */
			bit_nr = ether_crc(ETH_ALEN, mcptr->dmi_addr) >> 26;
			/* The most significant bit determines the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register. */
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		}
		as3310_writel(mc_filter[0], HW_ETH_BASE_ADDR + MAC_HASH_LOW);
		as3310_writel(mc_filter[1], HW_ETH_BASE_ADDR + MAC_HASH_HIGH);
	}

	as3310_writel(value, HW_ETH_BASE_ADDR + MAC_CONTROL);

	CHIP_DBG(KERN_INFO "%s: CTRL reg: 0x%08x Hash regs: "
	    "HI 0x%08x, LO 0x%08x\n",
	    __func__, as3310_readl(HW_ETH_BASE_ADDR + MAC_CONTROL),
	    as3310_readl(HW_ETH_BASE_ADDR + MAC_HASH_HIGH), as3310_readl(HW_ETH_BASE_ADDR + MAC_HASH_LOW));
}

static void dwmac100_flow_ctrl(void __iomem *ioaddr, unsigned int duplex,
			       unsigned int fc, unsigned int pause_time)
{
	unsigned int flow = MAC_FLOW_CTRL_ENABLE;

	if (duplex)
		flow |= (pause_time << MAC_FLOW_CTRL_PT_SHIFT);
	as3310_writel(flow, HW_ETH_BASE_ADDR + MAC_FLOW_CTRL);
}

/* No PMT module supported for this Ethernet Controller.
 * Tested on ST platforms only.
 */
static void dwmac100_pmt(void __iomem *ioaddr, unsigned long mode)
{
	return;
}

static const struct asmmac_ops dwmac100_ops = {
	.core_init = dwmac100_core_init,
	.rx_ipc = dwmac100_rx_ipc_enable,
	.dump_regs = dwmac100_dump_mac_regs,
	.host_irq_status = dwmac100_irq_status,
	.set_filter = dwmac100_set_filter,
	.flow_ctrl = dwmac100_flow_ctrl,
	.pmt = dwmac100_pmt,
	.set_umac_addr = dwmac100_set_umac_addr,
	.get_umac_addr = dwmac100_get_umac_addr,
};

struct mac_device_info *dwmac100_setup(void __iomem *ioaddr)
{
	struct mac_device_info *mac;

	mac = kzalloc(sizeof(const struct mac_device_info), GFP_KERNEL);
	if (!mac)
		return NULL;

	pr_info("ASM9260 MAC100\n");

	mac->mac = &dwmac100_ops;
	mac->dma = &dwmac100_dma_ops;

	mac->link.port = MAC_CONTROL_PS;
	mac->link.duplex = MAC_CONTROL_F;
	mac->link.speed = 0;
	mac->mii.addr = MAC_MII_ADDR;
	mac->mii.data = MAC_MII_DATA;
	mac->synopsys_uid = 0;

	return mac;
}
