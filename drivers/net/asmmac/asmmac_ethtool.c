/*******************************************************************************
  ASMMAC Ethtool support

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".
*******************************************************************************/

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <asm/io.h>
#include <mach/io.h>
#include <mach/mac.h>

#include "asmmac.h"
#include "dwmac_dma.h"

#define REG_SPACE_SIZE	      0x1054
#define PHY_AUTONEG_ENABLE    0x0200

#define MAC100_ETHTOOL_NAME	"asm_mac100"
#define GMAC_ETHTOOL_NAME	"asm_gmac"
#define CARDNAME	"asm9260_mac"
#define DRV_VERSION	"1.02"

struct asmmac_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define ASMMAC_STAT(m)	\
	{ #m, FIELD_SIZEOF(struct asmmac_extra_stats, m),	\
	offsetof(struct asmmac_priv, xstats.m)}

static const struct asmmac_stats asmmac_gstrings_stats[] = {
	/* Transmit errors */
	ASMMAC_STAT(tx_underflow),
	ASMMAC_STAT(tx_carrier),
	ASMMAC_STAT(tx_losscarrier),
	ASMMAC_STAT(vlan_tag),
	ASMMAC_STAT(tx_deferred),
	ASMMAC_STAT(tx_vlan),
	ASMMAC_STAT(tx_jabber),
	ASMMAC_STAT(tx_frame_flushed),
	ASMMAC_STAT(tx_payload_error),
	ASMMAC_STAT(tx_ip_header_error),
	/* Receive errors */
	ASMMAC_STAT(rx_desc),
	ASMMAC_STAT(sa_filter_fail),
	ASMMAC_STAT(overflow_error),
	ASMMAC_STAT(ipc_csum_error),
	ASMMAC_STAT(rx_collision),
	ASMMAC_STAT(rx_crc),
	ASMMAC_STAT(dribbling_bit),
	ASMMAC_STAT(rx_length),
	ASMMAC_STAT(rx_mii),
	ASMMAC_STAT(rx_multicast),
	ASMMAC_STAT(rx_gmac_overflow),
	ASMMAC_STAT(rx_watchdog),
	ASMMAC_STAT(da_rx_filter_fail),
	ASMMAC_STAT(sa_rx_filter_fail),
	ASMMAC_STAT(rx_missed_cntr),
	ASMMAC_STAT(rx_overflow_cntr),
	ASMMAC_STAT(rx_vlan),
	/* Tx/Rx IRQ errors */
	ASMMAC_STAT(tx_undeflow_irq),
	ASMMAC_STAT(tx_process_stopped_irq),
	ASMMAC_STAT(tx_jabber_irq),
	ASMMAC_STAT(rx_overflow_irq),
	ASMMAC_STAT(rx_buf_unav_irq),
	ASMMAC_STAT(rx_process_stopped_irq),
	ASMMAC_STAT(rx_watchdog_irq),
	ASMMAC_STAT(tx_early_irq),
	ASMMAC_STAT(fatal_bus_error_irq),
	/* Extra info */
	ASMMAC_STAT(threshold),
	ASMMAC_STAT(tx_pkt_n),
	ASMMAC_STAT(rx_pkt_n),
	ASMMAC_STAT(poll_n),
	ASMMAC_STAT(sched_timer_n),
	ASMMAC_STAT(normal_irq_n),
	ASMMAC_STAT(normal_irq_n),
	ASMMAC_STAT(mmc_tx_irq_n),
	ASMMAC_STAT(mmc_rx_irq_n),
	ASMMAC_STAT(mmc_rx_csum_offload_irq_n),
	ASMMAC_STAT(irq_receive_pmt_irq_n),
	ASMMAC_STAT(irq_tx_path_in_lpi_mode_n),
	ASMMAC_STAT(irq_tx_path_exit_lpi_mode_n),
	ASMMAC_STAT(irq_rx_path_in_lpi_mode_n),
	ASMMAC_STAT(irq_rx_path_exit_lpi_mode_n),
	ASMMAC_STAT(phy_eee_wakeup_error_n),
};
#define ASMMAC_STATS_LEN ARRAY_SIZE(asmmac_gstrings_stats)

/* HW MAC Management counters (if supported) */
#define ASMMAC_MMC_STAT(m)	\
	{ #m, FIELD_SIZEOF(struct asmmac_counters, m),	\
	offsetof(struct asmmac_priv, mmc.m)}

static const struct asmmac_stats asmmac_mmc[] = {
	ASMMAC_MMC_STAT(mmc_tx_octetcount_gb),
	ASMMAC_MMC_STAT(mmc_tx_framecount_gb),
	ASMMAC_MMC_STAT(mmc_tx_broadcastframe_g),
	ASMMAC_MMC_STAT(mmc_tx_multicastframe_g),
	ASMMAC_MMC_STAT(mmc_tx_64_octets_gb),
	ASMMAC_MMC_STAT(mmc_tx_65_to_127_octets_gb),
	ASMMAC_MMC_STAT(mmc_tx_128_to_255_octets_gb),
	ASMMAC_MMC_STAT(mmc_tx_256_to_511_octets_gb),
	ASMMAC_MMC_STAT(mmc_tx_512_to_1023_octets_gb),
	ASMMAC_MMC_STAT(mmc_tx_1024_to_max_octets_gb),
	ASMMAC_MMC_STAT(mmc_tx_unicast_gb),
	ASMMAC_MMC_STAT(mmc_tx_multicast_gb),
	ASMMAC_MMC_STAT(mmc_tx_broadcast_gb),
	ASMMAC_MMC_STAT(mmc_tx_underflow_error),
	ASMMAC_MMC_STAT(mmc_tx_singlecol_g),
	ASMMAC_MMC_STAT(mmc_tx_multicol_g),
	ASMMAC_MMC_STAT(mmc_tx_deferred),
	ASMMAC_MMC_STAT(mmc_tx_latecol),
	ASMMAC_MMC_STAT(mmc_tx_exesscol),
	ASMMAC_MMC_STAT(mmc_tx_carrier_error),
	ASMMAC_MMC_STAT(mmc_tx_octetcount_g),
	ASMMAC_MMC_STAT(mmc_tx_framecount_g),
	ASMMAC_MMC_STAT(mmc_tx_excessdef),
	ASMMAC_MMC_STAT(mmc_tx_pause_frame),
	ASMMAC_MMC_STAT(mmc_tx_vlan_frame_g),
	ASMMAC_MMC_STAT(mmc_rx_framecount_gb),
	ASMMAC_MMC_STAT(mmc_rx_octetcount_gb),
	ASMMAC_MMC_STAT(mmc_rx_octetcount_g),
	ASMMAC_MMC_STAT(mmc_rx_broadcastframe_g),
	ASMMAC_MMC_STAT(mmc_rx_multicastframe_g),
	ASMMAC_MMC_STAT(mmc_rx_crc_errror),
	ASMMAC_MMC_STAT(mmc_rx_align_error),
	ASMMAC_MMC_STAT(mmc_rx_run_error),
	ASMMAC_MMC_STAT(mmc_rx_jabber_error),
	ASMMAC_MMC_STAT(mmc_rx_undersize_g),
	ASMMAC_MMC_STAT(mmc_rx_oversize_g),
	ASMMAC_MMC_STAT(mmc_rx_64_octets_gb),
	ASMMAC_MMC_STAT(mmc_rx_65_to_127_octets_gb),
	ASMMAC_MMC_STAT(mmc_rx_128_to_255_octets_gb),
	ASMMAC_MMC_STAT(mmc_rx_256_to_511_octets_gb),
	ASMMAC_MMC_STAT(mmc_rx_512_to_1023_octets_gb),
	ASMMAC_MMC_STAT(mmc_rx_1024_to_max_octets_gb),
	ASMMAC_MMC_STAT(mmc_rx_unicast_g),
	ASMMAC_MMC_STAT(mmc_rx_length_error),
	ASMMAC_MMC_STAT(mmc_rx_autofrangetype),
	ASMMAC_MMC_STAT(mmc_rx_pause_frames),
	ASMMAC_MMC_STAT(mmc_rx_fifo_overflow),
	ASMMAC_MMC_STAT(mmc_rx_vlan_frames_gb),
	ASMMAC_MMC_STAT(mmc_rx_watchdog_error),
	ASMMAC_MMC_STAT(mmc_rx_ipc_intr_mask),
	ASMMAC_MMC_STAT(mmc_rx_ipc_intr),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_gd),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_hderr),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_nopay),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_frag),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_udsbl),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_gd_octets),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_hderr_octets),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_nopay_octets),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_frag_octets),
	ASMMAC_MMC_STAT(mmc_rx_ipv4_udsbl_octets),
	ASMMAC_MMC_STAT(mmc_rx_ipv6_gd_octets),
	ASMMAC_MMC_STAT(mmc_rx_ipv6_hderr_octets),
	ASMMAC_MMC_STAT(mmc_rx_ipv6_nopay_octets),
	ASMMAC_MMC_STAT(mmc_rx_ipv6_gd),
	ASMMAC_MMC_STAT(mmc_rx_ipv6_hderr),
	ASMMAC_MMC_STAT(mmc_rx_ipv6_nopay),
	ASMMAC_MMC_STAT(mmc_rx_udp_gd),
	ASMMAC_MMC_STAT(mmc_rx_udp_err),
	ASMMAC_MMC_STAT(mmc_rx_tcp_gd),
	ASMMAC_MMC_STAT(mmc_rx_tcp_err),
	ASMMAC_MMC_STAT(mmc_rx_icmp_gd),
	ASMMAC_MMC_STAT(mmc_rx_icmp_err),
	ASMMAC_MMC_STAT(mmc_rx_udp_gd_octets),
	ASMMAC_MMC_STAT(mmc_rx_udp_err_octets),
	ASMMAC_MMC_STAT(mmc_rx_tcp_gd_octets),
	ASMMAC_MMC_STAT(mmc_rx_tcp_err_octets),
	ASMMAC_MMC_STAT(mmc_rx_icmp_gd_octets),
	ASMMAC_MMC_STAT(mmc_rx_icmp_err_octets),
};
#define ASMMAC_MMC_STATS_LEN ARRAY_SIZE(asmmac_mmc)

static void mac9260_get_drvinfo(struct net_device *dev,struct ethtool_drvinfo *info)
{
	strcpy(info->driver, CARDNAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->bus_info, "RMII");
}

/*
 * get basic network information and working mode
 */
static int mac9260_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
        int phy_bcr_val,phy_auto_val,macval;
        struct asmmac_priv *priv = netdev_priv(dev);
        phy_bcr_val = priv->mii->read(0,PHY_ADDRESS,PHY_BCR);
        macval=as3310_readl(HW_ETH_MACCR);
        phy_auto_val = priv->mii->read(0,PHY_ADDRESS,PHY_AUTO_RESULT);
        phy_auto_val = phy_auto_val & 0x03;

        cmd->supported	= SUPPORTED_10baseT_Half
			| SUPPORTED_10baseT_Full
			| SUPPORTED_100baseT_Half
                        | SUPPORTED_100baseT_Full
                        | SUPPORTED_Autoneg;
        cmd->port	= PORT_MII;
        cmd->transceiver = XCVR_INTERNAL;

        if (macval & ETH_SPEED_100M){
           cmd->speed=100;
        }

        if (!(macval & ETH_SPEED_100M)) {
           cmd->speed=10;
        }

        if (macval & ETH_DUPLEX_FULL){
           cmd->duplex=1;
        }

        if (!(macval & ETH_DUPLEX_FULL)) {
           cmd->duplex=0;
        }

        if (phy_bcr_val & ETH_AUTONEG) {
           cmd->autoneg=1;
        }

        if (!(phy_bcr_val & ETH_AUTONEG)) {
           cmd->autoneg=0;
        }
        
	return 0;
}

/*
 * config network speed,duplex and autonegotiate mode 
 */
static int mac9260_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
        int num=0;
        struct asmmac_priv *priv = netdev_priv(dev);

        switch (cmd->speed) {
            case 10:
               as3310_writel(MAC_10MBPS & as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);		/*10mbps*/      
               priv->mii->write(0, PHY_ADDRESS, PHY_BCR,(num & PHY_10MBPS));
               break;
            case 100:
               as3310_writel(MAC_100MBPS | as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);		/*100mbps*/
               priv->mii->write(0, PHY_ADDRESS, PHY_BCR,(num | PHY_100MBPS));
               break;
        }

        num = priv->mii->read(0,PHY_ADDRESS,PHY_BCR);

        switch (cmd->duplex) {
            case 0:
               /*half duplex*/
               as3310_writel(MAC_HD & as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);		
               priv->mii->write(0, PHY_ADDRESS, PHY_BCR,(num & PHY_HD));
               break;
            case 1:        
               /*full duplex*/
               as3310_writel(MAC_FD | as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);		
               priv->mii->write(0, PHY_ADDRESS, PHY_BCR,(num | PHY_FD));                                                                    
               break;
        }

        num = priv->mii->read(0,PHY_ADDRESS,PHY_BCR);

        switch (cmd->autoneg) {
            case 0:
               priv->mii->write(0, PHY_ADDRESS, PHY_BCR,(num & PHY_AUTONEG_OFF));
               break;
            case 1:
               priv->mii->write(0, PHY_ADDRESS, PHY_BCR,(num | PHY_AUTONEG_ON));
               break;
        }

        return 0;
}

/*
 * restart network autonegotiation
 */
static int mac9260_nway_reset(struct net_device *dev)
{
        int value;
        struct asmmac_priv *priv = netdev_priv(dev);
        
        value = priv->mii->read(0,PHY_ADDRESS,PHY_BCR);
        value |= PHY_AUTONEG_ENABLE;
        priv->mii->write(0, PHY_ADDRESS, PHY_BCR,value);

        /* wait until autonegotiation completed */
        mdelay(5000);       

        value = priv->mii->read(0,PHY_ADDRESS,PHY_AUTO_RESULT);
        value = value & 0x07;
        
        if (value == PHY_AUTORES_10Base_half) {                          //10Base-half
           as3310_writel(MAC_10MBPS & as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);	
           as3310_writel(MAC_HD & as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);
        }
        else if (value == PHY_AUTORES_100Base_half) {                    //100Base-half
           as3310_writel(MAC_100MBPS | as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);
           as3310_writel(MAC_HD & as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);
        }
        else if (value == PHY_AUTORES_10Base_full) {                     //10Base-full
           as3310_writel(MAC_10MBPS & as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);	
           as3310_writel(MAC_FD | as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);
        }
        else if (value == PHY_AUTORES_100Base_full) {                    //100Base-full
           as3310_writel(MAC_100MBPS | as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);
           as3310_writel(MAC_FD | as3310_readl(HW_ETH_MACCR),HW_ETH_MACCR);
        }else{
        }

	
        return 0;
}

/*
 * get network link status
 */
static u32 mac9260_get_link(struct net_device *dev)
{
        int value;
        int ret;
        struct asmmac_priv *priv = netdev_priv(dev);
        value = priv->mii->read(0,PHY_ADDRESS,PHY_BSR);
        value = priv->mii->read(0,PHY_ADDRESS,PHY_BSR);

        if (value & PHY_LINK) {
           ret=1;
        }else{
           ret=0;
        }
	return ret;
}

static void asmmac_ethtool_getdrvinfo(struct net_device *dev,
				      struct ethtool_drvinfo *info)
{
	struct asmmac_priv *priv = netdev_priv(dev);

	if (priv->plat->has_gmac)
		strlcpy(info->driver, GMAC_ETHTOOL_NAME, sizeof(info->driver));
	else
		strlcpy(info->driver, MAC100_ETHTOOL_NAME,
			sizeof(info->driver));

	strcpy(info->version, DRV_MODULE_VERSION);
	info->fw_version[0] = '\0';
}

static int asmmac_ethtool_getsettings(struct net_device *dev,
				      struct ethtool_cmd *cmd)
{
	struct asmmac_priv *priv = netdev_priv(dev);
	struct phy_device *phy = priv->phydev;
	int rc;
	if (phy == NULL) {
		pr_err("%s: %s: PHY is not registered\n",
		       __func__, dev->name);
		return -ENODEV;
	}
	if (!netif_running(dev)) {
		pr_err("%s: interface is disabled: we cannot track "
		"link speed / duplex setting\n", dev->name);
		return -EBUSY;
	}
	cmd->transceiver = XCVR_INTERNAL;
	spin_lock_irq(&priv->lock);
	rc = phy_ethtool_gset(phy, cmd);
	spin_unlock_irq(&priv->lock);
	return rc;
}

static int asmmac_ethtool_setsettings(struct net_device *dev,
				      struct ethtool_cmd *cmd)
{
	struct asmmac_priv *priv = netdev_priv(dev);
	struct phy_device *phy = priv->phydev;
	int rc;

	spin_lock(&priv->lock);
	rc = phy_ethtool_sset(phy, cmd);
	spin_unlock(&priv->lock);

	return rc;
}

static u32 asmmac_ethtool_getmsglevel(struct net_device *dev)
{
	struct asmmac_priv *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void asmmac_ethtool_setmsglevel(struct net_device *dev, u32 level)
{
	struct asmmac_priv *priv = netdev_priv(dev);
	priv->msg_enable = level;

}

static int asmmac_check_if_running(struct net_device *dev)
{
	if (!netif_running(dev))
		return -EBUSY;
	return 0;
}

static int asmmac_ethtool_get_regs_len(struct net_device *dev)
{
	return REG_SPACE_SIZE;
}

static void asmmac_ethtool_gregs(struct net_device *dev,
			  struct ethtool_regs *regs, void *space)
{
	int i;
	u32 *reg_space = (u32 *) space;

	struct asmmac_priv *priv = netdev_priv(dev);

	memset(reg_space, 0x0, REG_SPACE_SIZE);

	if (!priv->plat->has_gmac) {
		/* MAC registers */
		for (i = 0; i < 12; i++)
			reg_space[i] = readl(priv->ioaddr + (i * 4));
		/* DMA registers */
		for (i = 0; i < 9; i++)
			reg_space[i + 12] =
			    readl(priv->ioaddr + (DMA_BUS_MODE + (i * 4)));
		reg_space[22] = readl(priv->ioaddr + DMA_CUR_TX_BUF_ADDR);
		reg_space[23] = readl(priv->ioaddr + DMA_CUR_RX_BUF_ADDR);
	} else {
		/* MAC registers */
		for (i = 0; i < 55; i++)
			reg_space[i] = readl(priv->ioaddr + (i * 4));
		/* DMA registers */
		for (i = 0; i < 22; i++)
			reg_space[i + 55] =
			    readl(priv->ioaddr + (DMA_BUS_MODE + (i * 4)));
	}
}

static void
asmmac_get_pauseparam(struct net_device *netdev,
		      struct ethtool_pauseparam *pause)
{
	struct asmmac_priv *priv = netdev_priv(netdev);

	spin_lock(&priv->lock);

	pause->rx_pause = 0;
	pause->tx_pause = 0;
	pause->autoneg = priv->phydev->autoneg;

	if (priv->flow_ctrl & FLOW_RX)
		pause->rx_pause = 1;
	if (priv->flow_ctrl & FLOW_TX)
		pause->tx_pause = 1;

	spin_unlock(&priv->lock);
}

static int
asmmac_set_pauseparam(struct net_device *netdev,
		      struct ethtool_pauseparam *pause)
{
	struct asmmac_priv *priv = netdev_priv(netdev);
	struct phy_device *phy = priv->phydev;
	int new_pause = FLOW_OFF;
	int ret = 0;

	spin_lock(&priv->lock);

	if (pause->rx_pause)
		new_pause |= FLOW_RX;
	if (pause->tx_pause)
		new_pause |= FLOW_TX;

	priv->flow_ctrl = new_pause;
	phy->autoneg = pause->autoneg;

	if (phy->autoneg) {
		if (netif_running(netdev))
			ret = phy_start_aneg(phy);
	} else
		priv->hw->mac->flow_ctrl(priv->ioaddr, phy->duplex,
					 priv->flow_ctrl, priv->pause);
	spin_unlock(&priv->lock);
	return ret;
}

static void asmmac_get_ethtool_stats(struct net_device *dev,
				 struct ethtool_stats *dummy, u64 *data)
{
	struct asmmac_priv *priv = netdev_priv(dev);
	int i, j = 0;

	/* Update the DMA HW counters for dwmac10/100 */
	if (!priv->plat->has_gmac)
		priv->hw->dma->dma_diagnostic_fr(&dev->stats,
						 (void *) &priv->xstats,
						 priv->ioaddr);
	else {
		/* If supported, for new GMAC chips expose the MMC counters */
		if (priv->dma_cap.rmon) {
			dwmac_mmc_read(priv->ioaddr, &priv->mmc);

			for (i = 0; i < ASMMAC_MMC_STATS_LEN; i++) {
				char *p;
				p = (char *)priv + asmmac_mmc[i].stat_offset;

				data[j++] = (asmmac_mmc[i].sizeof_stat ==
					     sizeof(u64)) ? (*(u64 *)p) :
					     (*(u32 *)p);
			}
		}
	}
	for (i = 0; i < ASMMAC_STATS_LEN; i++) {
		char *p = (char *)priv + asmmac_gstrings_stats[i].stat_offset;
		data[j++] = (asmmac_gstrings_stats[i].sizeof_stat ==
			     sizeof(u64)) ? (*(u64 *)p) : (*(u32 *)p);
	}
}

static int asmmac_get_sset_count(struct net_device *netdev, int sset)
{
	struct asmmac_priv *priv = netdev_priv(netdev);
	int len;

	switch (sset) {
	case ETH_SS_STATS:
		len = ASMMAC_STATS_LEN;

		if (priv->dma_cap.rmon)
			len += ASMMAC_MMC_STATS_LEN;

		return len;
	default:
		return -EOPNOTSUPP;
	}
}

static void asmmac_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	int i;
	u8 *p = data;
	struct asmmac_priv *priv = netdev_priv(dev);

	switch (stringset) {
	case ETH_SS_STATS:
		if (priv->dma_cap.rmon)
			for (i = 0; i < ASMMAC_MMC_STATS_LEN; i++) {
				memcpy(p, asmmac_mmc[i].stat_string,
				       ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
		for (i = 0; i < ASMMAC_STATS_LEN; i++) {
			memcpy(p, asmmac_gstrings_stats[i].stat_string,
				ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		WARN_ON(1);
		break;
	}
}

/* Currently only support WOL through Magic packet. */
static void asmmac_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct asmmac_priv *priv = netdev_priv(dev);

	spin_lock_irq(&priv->lock);
	if (device_can_wakeup(priv->device)) {
		wol->supported = WAKE_MAGIC | WAKE_UCAST;
		wol->wolopts = priv->wolopts;
	}
	spin_unlock_irq(&priv->lock);
}

static int asmmac_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct asmmac_priv *priv = netdev_priv(dev);
	u32 support = WAKE_MAGIC | WAKE_UCAST;

	/* By default almost all GMAC devices support the WoL via
	 * magic frame but we can disable it if the HW capability
	 * register shows no support for pmt_magic_frame. */
	if ((priv->hw_cap_support) && (!priv->dma_cap.pmt_magic_frame))
		wol->wolopts &= ~WAKE_MAGIC;

	if (!device_can_wakeup(priv->device))
		return -EINVAL;

	if (wol->wolopts & ~support)
		return -EINVAL;

	if (wol->wolopts) {
		pr_info("asmmac: wakeup enable\n");
		device_set_wakeup_enable(priv->device, 1);
		enable_irq_wake(priv->wol_irq);
	} else {
		device_set_wakeup_enable(priv->device, 0);
		disable_irq_wake(priv->wol_irq);
	}

	spin_lock_irq(&priv->lock);
	priv->wolopts = wol->wolopts;
	spin_unlock_irq(&priv->lock);

	return 0;
}

static const struct ethtool_ops asmmac_ethtool_ops = {
	.begin = asmmac_check_if_running,
	.get_drvinfo = mac9260_get_drvinfo,
	.get_settings = mac9260_get_settings,
	.set_settings = mac9260_set_settings,
        .nway_reset   = mac9260_nway_reset,
	.get_msglevel = asmmac_ethtool_getmsglevel,
	.set_msglevel = asmmac_ethtool_setmsglevel,
	.get_regs = asmmac_ethtool_gregs,
	.get_regs_len = asmmac_ethtool_get_regs_len,
	.get_link = mac9260_get_link,
	.get_pauseparam = asmmac_get_pauseparam,
	.set_pauseparam = asmmac_set_pauseparam,
	.get_ethtool_stats = asmmac_get_ethtool_stats,
	.get_strings = asmmac_get_strings,
	.get_wol = asmmac_get_wol,
	.set_wol = asmmac_set_wol,
	.get_sset_count	= asmmac_get_sset_count,
};

void asmmac_set_ethtool_ops(struct net_device *netdev)
{
	SET_ETHTOOL_OPS(netdev, &asmmac_ethtool_ops);
}
