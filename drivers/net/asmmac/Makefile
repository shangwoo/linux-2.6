obj-$(CONFIG_ASM9260_MAC) += asmmac.o
asmmac-$(CONFIG_ASM9260_MAC_TIMER) += asmmac_timer.o
asmmac-$(CONFIG_ASM9260_MAC_RING) += ring_mode.o
asmmac-$(CONFIG_ASM9260_MAC_CHAINED) += chain_mode.o
asmmac-$(CONFIG_ASM9260_MAC_PLATFORM) += asmmac_platform.o
asmmac-objs:= asmmac_main.o asmmac_ethtool.o asmmac_mdio.o	\
	      dwmac_lib.o dwmac1000_core.o  dwmac1000_dma.o	\
	      dwmac100_core.o dwmac100_dma.o enh_desc.o  norm_desc.o \
	      mmc_core.o $(asmmac-y)
