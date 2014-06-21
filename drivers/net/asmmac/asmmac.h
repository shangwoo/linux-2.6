/*******************************************************************************
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

#ifndef __ASMMAC_H__
#define __ASMMAC_H__

#define ASMMAC_RESOURCE_NAME   "mac9260"
#define DRV_MODULE_VERSION	"Sep_2013"

#define MODE_MII                 0
#define MODE_RMII_CRYSTAL        1
#define MODE_RMII_CHIP           2
#define FREQ_DIVIDER             8
#define PHY_ADDRESS              0x01
#define PHY_BCR                  0          	/*!< Tranceiver Basic Control Register */
#define PHY_BSR                  1  
#define PHY_AUTO_RESULT          0x1e    
#define ETH_MACMIIAR_PA   ((uint32_t)0x0000F800)  /* Physical layer address */ 
#define ETH_MACMIIAR_MR   ((uint32_t)0x000007C0)  /* MII register in the selected PHY */ 
#define ETH_MACMIIAR_MW   ((uint32_t)0x00000002)  /* MII write */ 
#define ETH_MACMIIAR_MB   ((uint32_t)0x00000001)  /* MII busy */ 
#define PHY_READ_TO       ((uint32_t)0x0004FFFF)
#define PHY_WRITE_TO      ((uint32_t)0x0004FFFF)
#define PHY_ResetDelay    ((uint32_t)0x000FFFFF) 

#define MAC_CLKEN                1<<5
#define MAC_RESET                1<<5
#define MAC_CSR                  (0x4<<2)
#define MAC_SOFTWARE_RESET       0x1

#define PHY_100M_FULL_DUPLEX        0x2100
#define PHY_100M_HALF_DUPLEX        0x2000
#define PHY_10M_FULL_DUPLEX         0x0100
#define PHY_10M_HALF_DUPLEX         0x0000
#define PHY_LINK                    0x0004
#define PHY_AUTORES_10Base_half     0x01
#define PHY_AUTORES_100Base_half    0x02
#define PHY_AUTORES_10Base_full     0x05
#define PHY_AUTORES_100Base_full    0x06
#define PHY_RESET                   0x8000

#define MAC_100M_FULL_DUPLEX     0x0000CA00
#define MAC_100M_HALF_DUPLEX     0x0000C200
#define MAC_10M_FULL_DUPLEX      0x00008A00
#define MAC_10M_HALF_DUPLEX      0x00008200

#define MAC_MII                  0x00
#define MAC_RMII_CRYSTAL         0x04
#define MAC_RMII_PLL_DIV         0x0c

#define ETH_MACFFR               0x00000040
#define ETH_MACFCR               0x00000080
#define ETH_DMAOMR               0x03200004
#define ETH_DMABMR               0x02C16000
#define ETH_DMAIER               0x00010041
#define ETH_MACCR                0x0000000c
#define ETH_DMAEN                0x00102002

#define ETH_SPEED_100M           0x4000
#define ETH_DUPLEX_FULL          0x800
#define ETH_AUTONEG              0x1000

#define MAC_10MBPS               0xffffbfff
#define MAC_100MBPS              0x00004000
#define MAC_HD                   0xfffff7ff
#define MAC_FD                   0x00000800
#define PHY_10MBPS               0xdfff
#define PHY_100MBPS              0x2000
#define PHY_HD                   0xfeff
#define PHY_FD                   0x0100
#define PHY_AUTONEG_OFF          0xefff
#define PHY_AUTONEG_ON           0x1000

#define MAC_INTERRUPT_EN_TR      0x00010041

#include <linux/clk.h>
#include <linux/asmnet.h>
#include <linux/phy.h>
#include <linux/pci.h>
#include <mach/mac.h>
#include "common.h"
#ifdef CONFIG_ASM9260_MAC_TIMER
#include "asmmac_timer.h"
#endif

struct asmmac_priv {
	/* Frequently used values are kept adjacent for cache effect */
	struct dma_desc *dma_tx ____cacheline_aligned;
	dma_addr_t dma_tx_phy;
	struct sk_buff **tx_skbuff;
	unsigned int cur_tx;
	unsigned int dirty_tx;
	unsigned int dma_tx_size;
	int tx_coalesce;

	struct dma_desc *dma_rx ;
	unsigned int cur_rx;
	unsigned int dirty_rx;
	struct sk_buff **rx_skbuff;
	dma_addr_t *rx_skbuff_dma;
	struct sk_buff_head rx_recycle;

	struct net_device *dev;
	dma_addr_t dma_rx_phy;
	unsigned int dma_rx_size;
	unsigned int dma_buf_sz;
	struct device *device;
	struct mac_device_info *hw;
	void __iomem *ioaddr;

	struct asmmac_extra_stats xstats;
	struct napi_struct napi;
	int no_csum_insertion;

	struct phy_device *phydev;
	int oldlink;
	int speed;
	int oldduplex;
	unsigned int flow_ctrl;
	unsigned int pause;
	struct mii_bus *mii;
	int mii_irq[PHY_MAX_ADDR];

	u32 msg_enable;
	spinlock_t lock;
	spinlock_t tx_lock;
	int wolopts;
	int wol_irq;
#ifdef CONFIG_ASM9260_MAC_TIMER
	struct asmmac_timer *tm;
#endif
	struct plat_asmmacenet_data *plat;
	struct asmmac_counters mmc;
	struct dma_features dma_cap;
	int hw_cap_support;
	struct clk *asmmac_clk;
	int clk_csr;
	int synopsys_id;
	struct timer_list eee_ctrl_timer;
	bool tx_path_in_lpi_mode;
	int lpi_irq;
	int eee_enabled;
	int eee_active;
	int tx_lpi_timer;
};

extern int phyaddr;

extern int asmmac_mdio_unregister(struct net_device *ndev);
extern int asmmac_mdio_register(struct net_device *ndev);
extern void asmmac_set_ethtool_ops(struct net_device *netdev);
extern const struct asmmac_desc_ops enh_desc_ops;
extern const struct asmmac_desc_ops ndesc_ops;
int asmmac_freeze(struct net_device *ndev);
int asmmac_restore(struct net_device *ndev);
int asmmac_resume(struct net_device *ndev);
int asmmac_suspend(struct net_device *ndev);
int asmmac_dvr_remove(struct net_device *ndev);
struct asmmac_priv *asmmac_dvr_probe(struct device *device,
				     struct plat_asmmacenet_data *plat_dat,
				     void __iomem *addr);
void asmmac_disable_eee_mode(struct asmmac_priv *priv);
bool asmmac_eee_init(struct asmmac_priv *priv);

#ifdef CONFIG_ASM9260_MAC_PLATFORM
extern struct platform_driver asmmac_pltfr_driver;
static inline int asmmac_register_platform(void)
{
	int err;

	err = platform_driver_register(&asmmac_pltfr_driver);
	if (err)
		pr_err("asmmac: failed to register the platform driver\n");

	return err;
}
static inline void asmmac_unregister_platform(void)
{
	platform_driver_register(&asmmac_pltfr_driver);
}
#else
static inline int asmmac_register_platform(void)
{
	pr_debug("asmmac: do not register the platf driver\n");

	return -EINVAL;
}
static inline void asmmac_unregister_platform(void)
{
}
#endif /* CONFIG_ASMMAC_PLATFORM */

#ifdef CONFIG_ASM9260_MAC_PCI
extern struct pci_driver asmmac_pci_driver;
static inline int asmmac_register_pci(void)
{
	int err;

	err = pci_register_driver(&asmmac_pci_driver);
	if (err)
		pr_err("asmmac: failed to register the PCI driver\n");

	return err;
}
static inline void asmmac_unregister_pci(void)
{
	pci_unregister_driver(&asmmac_pci_driver);
}
#else
static inline int asmmac_register_pci(void)
{
	pr_debug("asmmac: do not register the PCI driver\n");

	return -EINVAL;
}
static inline void asmmac_unregister_pci(void)
{
}
#endif /* CONFIG_ASMMAC_PCI */

#endif /* __ASMMAC_H__ */
