/*******************************************************************************
  DWMAC Management Counters

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/io.h>
#include <mach/mac.h>
#include <mach/io.h>
#include "mmc.h"

/* MAC Management Counters register offset */

#define MMC_CNTRL		0x00000100	/* MMC Control */
#define MMC_RX_INTR		0x00000104	/* MMC RX Interrupt */
#define MMC_TX_INTR		0x00000108	/* MMC TX Interrupt */
#define MMC_RX_INTR_MASK	0x0000010c	/* MMC Interrupt Mask */
#define MMC_TX_INTR_MASK	0x00000110	/* MMC Interrupt Mask */
#define MMC_DEFAULT_MASK		0xffffffff

/* MMC TX counter registers */

/* Note:
 * _GB register stands for good and bad frames
 * _G is for good only.
 */
#define MMC_TX_OCTETCOUNT_GB		0x00000114
#define MMC_TX_FRAMECOUNT_GB		0x00000118
#define MMC_TX_BROADCASTFRAME_G		0x0000011c
#define MMC_TX_MULTICASTFRAME_G		0x00000120
#define MMC_TX_64_OCTETS_GB		0x00000124
#define MMC_TX_65_TO_127_OCTETS_GB	0x00000128
#define MMC_TX_128_TO_255_OCTETS_GB	0x0000012c
#define MMC_TX_256_TO_511_OCTETS_GB	0x00000130
#define MMC_TX_512_TO_1023_OCTETS_GB	0x00000134
#define MMC_TX_1024_TO_MAX_OCTETS_GB	0x00000138
#define MMC_TX_UNICAST_GB		0x0000013c
#define MMC_TX_MULTICAST_GB		0x00000140
#define MMC_TX_BROADCAST_GB		0x00000144
#define MMC_TX_UNDERFLOW_ERROR		0x00000148
#define MMC_TX_SINGLECOL_G		0x0000014c
#define MMC_TX_MULTICOL_G		0x00000150
#define MMC_TX_DEFERRED			0x00000154
#define MMC_TX_LATECOL			0x00000158
#define MMC_TX_EXESSCOL			0x0000015c
#define MMC_TX_CARRIER_ERROR		0x00000160
#define MMC_TX_OCTETCOUNT_G		0x00000164
#define MMC_TX_FRAMECOUNT_G		0x00000168
#define MMC_TX_EXCESSDEF		0x0000016c
#define MMC_TX_PAUSE_FRAME		0x00000170
#define MMC_TX_VLAN_FRAME_G		0x00000174

/* MMC RX counter registers */
#define MMC_RX_FRAMECOUNT_GB		0x00000180
#define MMC_RX_OCTETCOUNT_GB		0x00000184
#define MMC_RX_OCTETCOUNT_G		0x00000188
#define MMC_RX_BROADCASTFRAME_G		0x0000018c
#define MMC_RX_MULTICASTFRAME_G		0x00000190
#define MMC_RX_CRC_ERRROR		0x00000194
#define MMC_RX_ALIGN_ERROR		0x00000198
#define MMC_RX_RUN_ERROR		0x0000019C
#define MMC_RX_JABBER_ERROR		0x000001A0
#define MMC_RX_UNDERSIZE_G		0x000001A4
#define MMC_RX_OVERSIZE_G		0x000001A8
#define MMC_RX_64_OCTETS_GB		0x000001AC
#define MMC_RX_65_TO_127_OCTETS_GB	0x000001b0
#define MMC_RX_128_TO_255_OCTETS_GB	0x000001b4
#define MMC_RX_256_TO_511_OCTETS_GB	0x000001b8
#define MMC_RX_512_TO_1023_OCTETS_GB	0x000001bc
#define MMC_RX_1024_TO_MAX_OCTETS_GB	0x000001c0
#define MMC_RX_UNICAST_G		0x000001c4
#define MMC_RX_LENGTH_ERROR		0x000001c8
#define MMC_RX_AUTOFRANGETYPE		0x000001cc
#define MMC_RX_PAUSE_FRAMES		0x000001d0
#define MMC_RX_FIFO_OVERFLOW		0x000001d4
#define MMC_RX_VLAN_FRAMES_GB		0x000001d8
#define MMC_RX_WATCHDOG_ERROR		0x000001dc
/* IPC*/
#define MMC_RX_IPC_INTR_MASK		0x00000200
#define MMC_RX_IPC_INTR			0x00000208
/* IPv4*/
#define MMC_RX_IPV4_GD			0x00000210
#define MMC_RX_IPV4_HDERR		0x00000214
#define MMC_RX_IPV4_NOPAY		0x00000218
#define MMC_RX_IPV4_FRAG		0x0000021C
#define MMC_RX_IPV4_UDSBL		0x00000220

#define MMC_RX_IPV4_GD_OCTETS		0x00000250
#define MMC_RX_IPV4_HDERR_OCTETS	0x00000254
#define MMC_RX_IPV4_NOPAY_OCTETS	0x00000258
#define MMC_RX_IPV4_FRAG_OCTETS		0x0000025c
#define MMC_RX_IPV4_UDSBL_OCTETS	0x00000260

/* IPV6*/
#define MMC_RX_IPV6_GD_OCTETS		0x00000264
#define MMC_RX_IPV6_HDERR_OCTETS	0x00000268
#define MMC_RX_IPV6_NOPAY_OCTETS	0x0000026c

#define MMC_RX_IPV6_GD			0x00000224
#define MMC_RX_IPV6_HDERR		0x00000228
#define MMC_RX_IPV6_NOPAY		0x0000022c

/* Protocols*/
#define MMC_RX_UDP_GD			0x00000230
#define MMC_RX_UDP_ERR			0x00000234
#define MMC_RX_TCP_GD			0x00000238
#define MMC_RX_TCP_ERR			0x0000023c
#define MMC_RX_ICMP_GD			0x00000240
#define MMC_RX_ICMP_ERR			0x00000244

#define MMC_RX_UDP_GD_OCTETS		0x00000270
#define MMC_RX_UDP_ERR_OCTETS		0x00000274
#define MMC_RX_TCP_GD_OCTETS		0x00000278
#define MMC_RX_TCP_ERR_OCTETS		0x0000027c
#define MMC_RX_ICMP_GD_OCTETS		0x00000280
#define MMC_RX_ICMP_ERR_OCTETS		0x00000284

void dwmac_mmc_ctrl(void __iomem *ioaddr, unsigned int mode)
{
	u32 value = as3310_readl(HW_ETH_BASE_ADDR + MMC_CNTRL);

	value |= (mode & 0x3F);

	as3310_writel(value, HW_ETH_BASE_ADDR + MMC_CNTRL);

	pr_debug("asmmac: MMC ctrl register (offset 0x%x): 0x%08x\n",
		 MMC_CNTRL, value);
}

/* To mask all all interrupts.*/
void dwmac_mmc_intr_all_mask(void __iomem *ioaddr)
{
	as3310_writel(MMC_DEFAULT_MASK, HW_ETH_BASE_ADDR + MMC_RX_INTR_MASK);
	as3310_writel(MMC_DEFAULT_MASK, HW_ETH_BASE_ADDR + MMC_TX_INTR_MASK);
}

/* This reads the MAC core counters (if actaully supported).
 * by default the MMC core is programmed to reset each
 * counter after a read. So all the field of the mmc struct
 * have to be incremented.
 */
void dwmac_mmc_read(void __iomem *ioaddr, struct asmmac_counters *mmc)
{
	mmc->mmc_tx_octetcount_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_OCTETCOUNT_GB);
	mmc->mmc_tx_framecount_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_FRAMECOUNT_GB);
	mmc->mmc_tx_broadcastframe_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_BROADCASTFRAME_G);
	mmc->mmc_tx_multicastframe_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_MULTICASTFRAME_G);
	mmc->mmc_tx_64_octets_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_64_OCTETS_GB);
	mmc->mmc_tx_65_to_127_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_65_TO_127_OCTETS_GB);
	mmc->mmc_tx_128_to_255_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_128_TO_255_OCTETS_GB);
	mmc->mmc_tx_256_to_511_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_256_TO_511_OCTETS_GB);
	mmc->mmc_tx_512_to_1023_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_512_TO_1023_OCTETS_GB);
	mmc->mmc_tx_1024_to_max_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_1024_TO_MAX_OCTETS_GB);
	mmc->mmc_tx_unicast_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_UNICAST_GB);
	mmc->mmc_tx_multicast_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_MULTICAST_GB);
	mmc->mmc_tx_broadcast_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_BROADCAST_GB);
	mmc->mmc_tx_underflow_error += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_UNDERFLOW_ERROR);
	mmc->mmc_tx_singlecol_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_SINGLECOL_G);
	mmc->mmc_tx_multicol_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_MULTICOL_G);
	mmc->mmc_tx_deferred += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_DEFERRED);
	mmc->mmc_tx_latecol += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_LATECOL);
	mmc->mmc_tx_exesscol += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_EXESSCOL);
	mmc->mmc_tx_carrier_error += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_CARRIER_ERROR);
	mmc->mmc_tx_octetcount_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_OCTETCOUNT_G);
	mmc->mmc_tx_framecount_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_FRAMECOUNT_G);
	mmc->mmc_tx_excessdef += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_EXCESSDEF);
	mmc->mmc_tx_pause_frame += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_PAUSE_FRAME);
	mmc->mmc_tx_vlan_frame_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_TX_VLAN_FRAME_G);

	/* MMC RX counter registers */
	mmc->mmc_rx_framecount_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_FRAMECOUNT_GB);
	mmc->mmc_rx_octetcount_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_OCTETCOUNT_GB);
	mmc->mmc_rx_octetcount_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_OCTETCOUNT_G);
	mmc->mmc_rx_broadcastframe_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_BROADCASTFRAME_G);
	mmc->mmc_rx_multicastframe_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_MULTICASTFRAME_G);
	mmc->mmc_rx_crc_errror += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_CRC_ERRROR);
	mmc->mmc_rx_align_error += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_ALIGN_ERROR);
	mmc->mmc_rx_run_error += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_RUN_ERROR);
	mmc->mmc_rx_jabber_error += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_JABBER_ERROR);
	mmc->mmc_rx_undersize_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_UNDERSIZE_G);
	mmc->mmc_rx_oversize_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_OVERSIZE_G);
	mmc->mmc_rx_64_octets_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_64_OCTETS_GB);
	mmc->mmc_rx_65_to_127_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_65_TO_127_OCTETS_GB);
	mmc->mmc_rx_128_to_255_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_128_TO_255_OCTETS_GB);
	mmc->mmc_rx_256_to_511_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_256_TO_511_OCTETS_GB);
	mmc->mmc_rx_512_to_1023_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_512_TO_1023_OCTETS_GB);
	mmc->mmc_rx_1024_to_max_octets_gb +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_1024_TO_MAX_OCTETS_GB);
	mmc->mmc_rx_unicast_g += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_UNICAST_G);
	mmc->mmc_rx_length_error += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_LENGTH_ERROR);
	mmc->mmc_rx_autofrangetype += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_AUTOFRANGETYPE);
	mmc->mmc_rx_pause_frames += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_PAUSE_FRAMES);
	mmc->mmc_rx_fifo_overflow += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_FIFO_OVERFLOW);
	mmc->mmc_rx_vlan_frames_gb += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_VLAN_FRAMES_GB);
	mmc->mmc_rx_watchdog_error += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_WATCHDOG_ERROR);
	/* IPC */
	mmc->mmc_rx_ipc_intr_mask += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPC_INTR_MASK);
	mmc->mmc_rx_ipc_intr += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPC_INTR);
	/* IPv4 */
	mmc->mmc_rx_ipv4_gd += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_GD);
	mmc->mmc_rx_ipv4_hderr += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_HDERR);
	mmc->mmc_rx_ipv4_nopay += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_NOPAY);
	mmc->mmc_rx_ipv4_frag += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_FRAG);
	mmc->mmc_rx_ipv4_udsbl += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_UDSBL);

	mmc->mmc_rx_ipv4_gd_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_GD_OCTETS);
	mmc->mmc_rx_ipv4_hderr_octets +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_HDERR_OCTETS);
	mmc->mmc_rx_ipv4_nopay_octets +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_NOPAY_OCTETS);
	mmc->mmc_rx_ipv4_frag_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_FRAG_OCTETS);
	mmc->mmc_rx_ipv4_udsbl_octets +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV4_UDSBL_OCTETS);

	/* IPV6 */
	mmc->mmc_rx_ipv6_gd_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV6_GD_OCTETS);
	mmc->mmc_rx_ipv6_hderr_octets +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV6_HDERR_OCTETS);
	mmc->mmc_rx_ipv6_nopay_octets +=
	    as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV6_NOPAY_OCTETS);

	mmc->mmc_rx_ipv6_gd += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV6_GD);
	mmc->mmc_rx_ipv6_hderr += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV6_HDERR);
	mmc->mmc_rx_ipv6_nopay += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_IPV6_NOPAY);

	/* Protocols */
	mmc->mmc_rx_udp_gd += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_UDP_GD);
	mmc->mmc_rx_udp_err += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_UDP_ERR);
	mmc->mmc_rx_tcp_gd += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_TCP_GD);
	mmc->mmc_rx_tcp_err += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_TCP_ERR);
	mmc->mmc_rx_icmp_gd += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_ICMP_GD);
	mmc->mmc_rx_icmp_err += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_ICMP_ERR);

	mmc->mmc_rx_udp_gd_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_UDP_GD_OCTETS);
	mmc->mmc_rx_udp_err_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_UDP_ERR_OCTETS);
	mmc->mmc_rx_tcp_gd_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_TCP_GD_OCTETS);
	mmc->mmc_rx_tcp_err_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_TCP_ERR_OCTETS);
	mmc->mmc_rx_icmp_gd_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_ICMP_GD_OCTETS);
	mmc->mmc_rx_icmp_err_octets += as3310_readl(HW_ETH_BASE_ADDR + MMC_RX_ICMP_ERR_OCTETS);
}
