/***********************************************
 *  linux/arch/arm/mach-as9260/core.c
 *  Copyright (C) 2011-2014 Alpscale
 *
 */

#include <linux/clk-provider.h>
#include <linux/of_platform.h>

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/serial_8250.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
//#include <linux/i2c/at24.h>
//#include <linux/dma-mapping.h>
#include <linux/spi/flash.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/as9260_ts.h>
#include <mach/lradc.h>
#include <mach/pwm.h>
#include <mach/pincontrol.h>
//#include <mach/dma.h>
#include <mach/mac.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include <asm/hardware/iomd.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/uart_reg.h>
#include <mach/mci.h>
#include <mach/rtc.h>
#include <mach/keypad.h>
#include <mach/i2c.h>
#include <mach/spi.h>
#include <mach/hardware.h>
#include <mach/asm9260_uart.h>
#include <mach/asm9260_canserial.h>
#include <linux/usb/musb.h>
#include <linux/usb/otg.h>
//#include <linux/asmnet.h>
#include "irq.h"
#include "timer.h"
#include <mach/asm9260_nand.h>
#include <mach/xpt2046_ts.h>

/* RTC RESOURCES*/
#ifdef CONFIG_RTC_ASM9260
static struct resource asm9260_rtc_resources[] = {
	{
		.start		= IO_ADDRESS(HW_RTC_ILR),
		.end		= IO_ADDRESS(HW_RTC_ILR) + 0x80-1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_RTC,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device asm9260_rtc_device = {
	.name           = "asm9260_rtc",
	.id             = -1,
	.num_resources	= ARRAY_SIZE(asm9260_rtc_resources),
	.resource	= asm9260_rtc_resources,
};
EXPORT_SYMBOL(asm9260_rtc_device);

void __init asm9260_add_rtc(void)
{
	platform_device_register(&asm9260_rtc_device);
}
#endif

/* MAC RESOURCES */
#if defined(CONFIG_ASM9260_MAC) || defined(CONFIG_ASM9260_MAC_MODULE)
static struct asmmac_mdio_bus_data phy_private_data = {
	.bus_id = 0,
	.phy_mask = 1,
};

static struct plat_asmmacenet_data eth_private_data = {
	.bus_id   = 0,
	.clk_csr  = ASMMAC_CSR_35_60M,
	.enh_desc = 0,
	.phy_addr = 1,
	.mdio_bus_data = &phy_private_data,
};

static u64 eth_dmamask = DMA_BIT_MASK(32);
static struct resource as9260_mac_resources[] = {
	[0] = {
		.start  = HW_ETH_BASE_ADDR ,
		.end    = HW_ETH_BASE_ADDR + 0x1054,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = INT_MAC,
		.end    = INT_MAC,
		.flags  = (IORESOURCE_IRQ
				| IORESOURCE_IRQ_HIGHLEVEL
				| IORESOURCE_IRQ_SHAREABLE),
	}
};

static struct platform_device as9260_mac_device = {
	.name = "mac9260",
	.id = -1,
	.dev = {
		.dma_mask		= &eth_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.power.can_wakeup	= 1,
		.platform_data		= &eth_private_data,
	},
	.resource = as9260_mac_resources,
	.num_resources = ARRAY_SIZE(as9260_mac_resources),
};
#endif

/* mcpwm RESOURCES */
#if defined(CONFIG_MCPWM_ASM9260) || defined(CONFIG_MCPWM_ASM9260_MODULE)
static struct resource as9260_mcpwm_resources[] = {
	[0] = {
		.start	= MCPWM_BASE_ADDRESS ,
		.end	= MCPWM_BASE_ADDRESS  + 0x8c,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_MCPWM,
		.end	= INT_MCPWM,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device as9260_mcpwm_device = {
	.name	= "asm9260-mcpwm",
	.id	= -1,
	.resource = as9260_mcpwm_resources,
	.num_resources = ARRAY_SIZE(as9260_mcpwm_resources),
};
#endif

/*timer1 RESOURCES*/
#if defined(CONFIG_TIMER_ASM9260) || defined(CONFIG_TIMER_ASM9260_MODULE)
static struct resource as9260_timer1_resources[] = {
	[0] = {
		.start  = TIMER1_BASE_ADDRESS,
		.end    = TIMER1_BASE_ADDRESS + 0x190,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = INT_TIMER1,
		.end    = INT_TIMER1,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct platform_device as9260_timer_device = {
	.name   = "asm9260-timer",
	.id     = -1,
	.resource = as9260_timer1_resources,
	.num_resources = ARRAY_SIZE(as9260_timer1_resources),
};
#endif

/* Watchdog RESOURCES*/
#ifdef CONFIG_ASM9260_WATCHDOG
static struct resource asm9260_wdt_resource[] = {
	[0] = {
		.start = HW_WATCHDOG_WDMOD,
		.end   = HW_WATCHDOG_WDMOD + 0xc - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_WATCHDOG,
		.end   = INT_WATCHDOG,
		.flags = IORESOURCE_IRQ,
	}

};

static struct platform_device asm9260_device_wdt = {
	.name		= "asm9260-wdt",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(asm9260_wdt_resource),
	.resource	= asm9260_wdt_resource,
};
EXPORT_SYMBOL(asm9260_device_wdt);

void __init asm9260_add_wdt(void)
{
	/* do we need config pin muxl? */
	platform_device_register(&asm9260_device_wdt);

}
#endif

/* NAND Controller RESOURCES*/
#ifdef CONFIG_MTD_NAND_ASM9260
static struct resource asm9260_nand_resource[] = {
	[0] = {
		.start = 0x80600000,
		.end   = 0x806000C8 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_NAND,
		.end   = INT_NAND,
		.flags = IORESOURCE_IRQ,
	}
};


struct mtd_partition asm9260_default_nand_part[] = {
	[0] = {
		.name   = "bootloader",
		.offset	= 0,
		.size   = 0x00100000,
	},
	[1] = {
		.name   = "kernel",
		.offset = MTDPART_OFS_APPEND,
		.size   = 0x00500000,
	},
	[2] = {
		.name   = "rootfs",
		.offset = MTDPART_OFS_APPEND,
		.size   = 0x01800000,
	},
	[3] = {
		.name   = "yaffs2",
		.offset = MTDPART_OFS_APPEND,
		.size   = 0x03200000,
	},
	[4] = {
		.name   = "ubifs",
		.offset = MTDPART_OFS_APPEND,
		.size   = MTDPART_SIZ_FULL,
	},
};

struct asm9260_nand_data asm9260_default_mtd_part = {
	.mtd_part_num = ARRAY_SIZE(asm9260_default_nand_part),
	.asm9260_mtd_part = asm9260_default_nand_part,
};

struct platform_device asm9260_device_nand = {
	.name = "asm9260-nand",
	.id = -1,
	.dev = {
		.platform_data = &asm9260_default_mtd_part,
	},
	.num_resources = ARRAY_SIZE(asm9260_nand_resource),
	.resource = asm9260_nand_resource,
};

#endif

/*Touch Screen RESOUCES*/
#ifdef CONFIG_TOUCHSCREEN_AS9260
static struct lradc_config ts_lradc_conf = {
	TS_LRADC_CH,
	0
};


static struct as9260_ts_mach_info as9260ts_info = {
	.ts_lradc_config = &ts_lradc_conf,
	.sample_ratio_HZ = TS_SAMPLE_RATIO,
};

struct platform_device as_device_ts = {
	.name = "as9260ts",
	.id = -1,
	.dev = {
		.platform_data = &as9260ts_info,
	}
};
EXPORT_SYMBOL(as_device_ts);

void __init as9260_add_ts(void)
{
	platform_device_register(&as_device_ts);
}
#endif

/* Touch Screen xpt2046 RESOUCES */
#ifdef CONFIG_TOUCHSCREEN_XPT2046
struct xpt2046_hw_info xpt2046_hw_conf = {
	1,
	2000000,
	10,
	2,
	10
};

struct platform_device xpt2046_device_ts = {
	.name = "xpt2046ts",
	.id = -1,
	.dev = {
		.platform_data = &xpt2046_hw_conf,
	}
};

void __init as9260_add_xpt2046_ts(void)
{
	platform_device_register(&xpt2046_device_ts);
}
#endif

/* Camera RESOURCES */
#ifdef CONFIG_SOC_CAMERA_ASM9260
static struct resource asm9260_cam_resource[] = {
	[0] = {
		.start = HW_DCMI_CR_ADDR,
		.end   = HW_DCMI_CR_ADDR + 0xA0-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_CAMIF,
		.end   = INT_CAMIF,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device asm9260_device_cam = {
	.name		  = "asm9260-camif",
	.id		  = -1,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources	  = ARRAY_SIZE(asm9260_cam_resource),
	.resource	  = asm9260_cam_resource,
};
EXPORT_SYMBOL(asm9260_device_cam);

void __init asm9260_add_camif(void)
{
	/* do we need config pin muxl?*/
	platform_device_register(&asm9260_device_cam);
}
#endif


/* MMC RESOURCES */
#if defined(CONFIG_MMC_AS9260)
static struct as9260_mmc_data __initdata as9260c_mmc_data = {
	.det_pin = 1,
	.slot_b  = 0,
	.wire4   = 1,
	.vcc_pin = 0,
};

static u64 mmc_dmamask = 0xffffffffUL;
static struct as9260_mmc_data mmc_data;

static struct resource mmc_resources[] = {
	[0] = {
		.start  = ALPAS9260_SSP_BASE,
		.end    = ALPAS9260_SSP_BASE + HW_SSP_XFER + 0x10,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = 63,
		.end    = 63,
		.flags  = IORESOURCE_IRQ,
    }
};

static struct platform_device as9260_mmc_device = {
	.name   = "as9260_sdi",
	.id     = -1,
	.dev    = {
		.dma_mask   = &mmc_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = &mmc_data,
	},
	.resource   = mmc_resources,
	.num_resources   = ARRAY_SIZE(mmc_resources),
};

void __init as9260_add_device_mmc(struct as9260_mmc_data *data)
{
	mmc_data = *data;
	platform_device_register(&as9260_mmc_device);
}
#else
void __init as9260_add_device_mmc(struct as9260_mmc_data *data) {}
#endif


/* i2c RESOURCES */
/***********************************************************
 * I2C0 and I2C1 share the I2C adapter(bus) driver method,
 * but not the chip method *
 ***********************************************************/
#ifdef CONFIG_I2C_ASM9260
static struct resource as9260_i2c0_resources[] = {
	[0] = {
		.start  = I2C0_BASE_ADDRESS,
		.end    = HW_I2C0_COMP_TYPE + 0x04,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = 26,
		.end    = 26,
		.flags  = IORESOURCE_IRQ,
	}
};

struct as9260_i2c_hw_data as9260_i2c0_support = {
	.class = I2C_CLASS_SPD,
	.speedmode = I2C_Speed_Standard,
};

static struct platform_device as9260_i2c0_device = {
	.name = "as9260_i2c",
	.id = 0,
	.dev = {
		.platform_data = &as9260_i2c0_support,
	},
	.resource = as9260_i2c0_resources,
	.num_resources = ARRAY_SIZE(as9260_i2c0_resources),
};

static struct resource as9260_i2c1_resources[] = {
	[0] = {
		.start  = I2C1_BASE_ADDRESS,
		.end    = HW_I2C1_COMP_TYPE + 0x04,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = 27,
		.end    = 27,
		.flags  = IORESOURCE_IRQ,
	}
};

struct as9260_i2c_hw_data as9260_i2c1_support = {
	/* i2c1-eeprom/i2c1-ov7670(hardware line setting) */
	.class = I2C_CLASS_SPD | I2C_CLASS_CAM_DIGITAL | I2C_CLASS_SOUND,
	.speedmode = I2C_Speed_Standard,
};

static struct platform_device as9260_i2c1_device = {
	.name = "as9260_i2c",
	.id = 1,
	.dev = {
		.platform_data = &as9260_i2c1_support,
	},
	.resource   = as9260_i2c1_resources,
	.num_resources   = ARRAY_SIZE(as9260_i2c1_resources),
};
#endif

#if defined(CONFIG_I2C_ASM9260) && defined(CONFIG_TOUCHSCREEN_FT5X06)
struct i2c_board_info i2c_ft5x06_board_info = {
	.type = "FT5X06_ts",
	.flags = 0,
	.addr = 0x38,
};
const struct i2c_board_info *i2c_ft5x06_board_info_pointer =
					&i2c_ft5x06_board_info;
#endif

/* STD SPI RESOURCES */
/**************************************************
  STD SPI0/SPI1
 **************************************************/
#ifdef CONFIG_SPI_ASM9260

/* spi 1 */
static struct resource as9260_spi1_resources[] = {
	[0] = {
		.start  = ALPAS9260_SPI1_BASE,
		.end    = HW_SPI1_XFER + 0x10,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = 61,
		.end    = 61,
		.flags  = IORESOURCE_IRQ,
	}
};

struct alp_spi_info as9260_spi1_support = {
	.num_cs = 1,
	.bus_num = 1,
		.dma_module = 1,
		.dma_channel = 5,
	.tx_handshake_interface = 14,
	.rx_handshake_interface = 15,
	.quadSupport = NO_QUAD,
};

static struct platform_device as9260_spi1_device = {
	.name = "as9260_std_spi",
	.id = 1,
	.dev = {
		.platform_data = &as9260_spi1_support,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource   = as9260_spi1_resources,
	.num_resources   = ARRAY_SIZE(as9260_spi1_resources),
};

/* Used with drivers/spi/spidev.c, this is used for SPI controller
 * raw ioctl function, ie the /dev node implementation. */
struct spi_board_info spi_controller_dummy_board_info = {
	.modalias = "spidev",
	.max_speed_hz = 33000000,
	.bus_num = 1,
	.chip_select = 0,
	.mode = 0,
};

const struct spi_board_info *spi_controller_dummy_board_info_pointer =
					&spi_controller_dummy_board_info;
#endif

/* QSPI RESOURCES */
#ifdef CONFIG_QSPI_ASM9260
static struct resource as9260_qspi_resources[] = {
	[0] = {
		.start  = ALPAS9260_QUAD_SPI_BASE,
		.end    = HW_QUAD_SPI_XFER+0x10,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = 62,
		.end    = 62,
		.flags  = IORESOURCE_IRQ,
	}
};

struct alp_spi_info as9260_qspi_support = {
	.num_cs = 1,
	.bus_num = 2,
		.dma_module = 1,
		.dma_channel = 6,
		.tx_handshake_interface = 2,
		.rx_handshake_interface = 3,
	.quadSupport = DO_QUAD,
};

static struct platform_device as9260_qspi_device = {
	.name = "as9260_quad_spi",
	.id = 0,
	.dev = {
		.platform_data = &as9260_qspi_support,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource   = as9260_qspi_resources,
	.num_resources   = ARRAY_SIZE(as9260_qspi_resources),
};

/* Used with drivers/spi/spidev.c,
 * this is used for SPI controller raw ioctl function,
 * ie the /dev node implementation. */
struct spi_board_info qspi_controller_dummy_board_info = {
	.modalias = "spidev",
	.max_speed_hz = 33000000,
	.bus_num = 2, /* QUAD SPI BUS! */
	.chip_select = 0,
	.mode = 0,
};

const struct spi_board_info *qspi_controller_dummy_board_info_pointer =
					&qspi_controller_dummy_board_info;
#endif

/* USB RESOURCES */
/* --------------------------------------------------------------------
 *  USB Device (Gadget)
 * -------------------------------------------------------------------- */

#ifdef CONFIG_USB_GADGET_AS9260
#define AS9260_USB0_BASE_UDP_PHY 0x80300000

static struct resource udc_resources[] = {
	[0] = {
		.start	= AS9260_USB0_BASE_UDP_PHY,
		.end	= AS9260_USB0_BASE_UDP_PHY + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_USB0,
		.end	= INT_USB0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device as9260_udc_device = {
	.name		= "as9260_udc",
	.id		= -1,
	.dev		= {
				.platform_data		= NULL,
	},
	.resource	= udc_resources,
	.num_resources	= ARRAY_SIZE(udc_resources),
};

void __init as9260_add_device_udc()
{
	platform_device_register(&as9260_udc_device);
}
#else
void __init as9260_add_device_udc(void) {}
#endif


/* --------------------------------------------------------------------
 *  USB HOST (musb)
 * -------------------------------------------------------------------- */

#ifdef CONFIG_USB_MUSB_HOST

#define AS9260_USB1_BASE_UDP_PHY 0x80400000

static struct musb_hdrc_eps_bits musb_eps[] = {
	{ "ep1_tx", 8, },
	{ "ep1_rx", 8, },
	{ "ep2_tx", 8, },
	{ "ep2_rx", 8, },
	{ "ep3_tx", 5, },
	{ "ep3_rx", 5, },
	{ "ep4_tx", 5, },
	{ "ep4_rx", 5, },
};

static struct musb_hdrc_config musb_config = {
	.multipoint	= true,
	.dyn_fifo	= true,
	.soft_con	= true,
	.dma		= true,

	.num_eps	= 5,
	.dma_channels	= 8,
	.ram_bits	= 10,
	.eps_bits	= musb_eps,
};

struct musb_hdrc_platform_data usb_data = {
#if defined(CONFIG_USB_MUSB_OTG)
	/* OTG requires a Mini-AB connector */
	.mode	= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_PERIPHERAL)
	.mode	= MUSB_PERIPHERAL,
#elif defined(CONFIG_USB_MUSB_HOST)
	.mode	= MUSB_HOST,
#endif
	.config	= &musb_config,
	.clock	= NULL,
};



static struct resource usb_resources[] = {
	[0] = {
		.start	= AS9260_USB1_BASE_UDP_PHY,
		.end	= AS9260_USB1_BASE_UDP_PHY + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_USB1,
		.end	= INT_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device as9260_host_device = {
	.name		= "musb_hdrc",
	.id		= -1,
	.dev		= {
				.platform_data		= &usb_data,
	},
	.resource	= usb_resources,
	.num_resources	= ARRAY_SIZE(usb_resources),
};

void __init as9260_add_device_musb(void)
{
	platform_device_register(&as9260_host_device);
}
#else
void __init as9260_add_device_musb(void) {}
#endif


/* IIS RESOURCES*/
#ifdef CONFIG_SOUND_ASM9260_SND_OSS
static struct resource asm9260_iis_resource[] = {
	[0] = {
		.start = I2S0_BASE_ADDRESS,
		.end   = (HW_I2S0_COMP_TYPE - 1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_I2S1,
		.end	= INT_I2S1,
		.flags	= IORESOURCE_IRQ,
	}
};


struct platform_device asm9260_device_iis = {
	.name		  = "asm9260-iis",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(asm9260_iis_resource),
	.resource	  = asm9260_iis_resource,
};
#endif

#ifdef CONFIG_SENSORS_ADC_ASM9260
static struct platform_device asm9260_adc_device = {
	.name           = "as9260-adc",
	.id             = -1,
	.num_resources	= 0,
	.resource	= NULL, /* not used for the momment. */
};
#endif

/* we map the io address to virtual address here*/
static struct map_desc as9260_io_desc[] __initdata = {
	{	/* IO space	*/
		.virtual	= (unsigned long)IO_BASE,
		.pfn		= __phys_to_pfn(IO_PHYS),
		.length		= IO_SIZE,
		.type		= MT_DEVICE
	},
	{	/*LCD IO space	*/
		.virtual	= (unsigned long)0xF0A00000,
		.pfn		= __phys_to_pfn(0x80800000),
		.length		= 0x9000,
		.type		= MT_DEVICE
	},
	{	/*GPIO IO space	*/
		.virtual	= (unsigned long)0xF0800000,
		.pfn		= __phys_to_pfn(0x50000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	},
	{	/* SRAM space Cacheable  */
		.virtual	= AS9260_SRAM_VIRT_BASE,
		.pfn		= __phys_to_pfn(AS9260_SRAM_PHY_BASE),
		.length		= 0x00100000,
#ifdef CONFIG_SRAM_MEM_CACHED
		.type		= MT_MEMORY
#else
		.type		= MT_DEVICE
#endif
	},
};


static void __init as9260_map_io(void)
{
	/* we remap our io to high address 0xe0000000 */
	iotable_init(as9260_io_desc, ARRAY_SIZE(as9260_io_desc));

	/* UART4 on ttyS4. (Rx, Tx) */

#ifdef CONFIG_ENABLE_UART0
	asm9260_register_uart(0, 0, 0);
#endif

#ifdef CONFIG_ENABLE_UART1
	asm9260_register_uart(1, 1, 0);
#endif

#ifdef CONFIG_ENABLE_UART2
	asm9260_register_uart(2, 2, 0);
#endif

#ifdef CONFIG_ENABLE_UART3
	asm9260_register_uart(3, 3, 0);
#endif

#ifdef CONFIG_ENABLE_UART4
	asm9260_register_uart(4, 4, 0);
#endif

#ifdef CONFIG_ENABLE_UART5
	asm9260_register_uart(5, 5, 0);
#endif

#ifdef CONFIG_ENABLE_UART6
	asm9260_register_uart(6, 6, ASM9260_UART_RTS_PIN);
#endif

#ifdef CONFIG_ENABLE_UART7
	asm9260_register_uart(7, 7, 0);
#endif

#ifdef CONFIG_ENABLE_UART8
	asm9260_register_uart(8, 8, 0);
#endif

#ifdef CONFIG_ENABLE_UART9
	asm9260_register_uart(9, 9, 0);
#endif

#ifdef CONFIG_CAN0_ASM9260_PLATFORM
	asm9260_register_can(0, 0, 0);
#endif

#ifdef CONFIG_CAN1_ASM9260_PLATFORM
	asm9260_register_can(1, 1, 0);
#endif
	/* set serial console to ttyS4 (UART4) */
	asm9260_set_serial_console(4);
}

extern void ioctime_init(void);
extern unsigned long ioc_timer_gettimeoffset(void);
extern void __init setup_usb(void);


static void __init as9260_init(void)
{
	/* must init before any other
	 * devices which may use gpio pins */
	asm9260_gpio_init();

	asm9260_add_device_serial();
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

#ifdef CONFIG_CAN1_ASM9260_PLATFORM
	asm9260_add_device_can();
#endif

#ifdef CONFIG_MTD_NAND_ASM9260
	platform_device_register(&asm9260_device_nand);
#endif

#ifdef CONFIG_USB_MUSB_HOST
	setup_usb(); /* open USB Controller Power and CLK */
	as9260_add_device_udc();
	as9260_add_device_musb();
#endif


#ifdef CONFIG_TOUCHSCREEN_AS9260
	as9260_add_ts();
#endif

#ifdef CONFIG_MMC_AS9260
	as9260_add_device_mmc(&as9260c_mmc_data);
#endif

#ifdef CONFIG_I2C_ASM9260
#ifdef CONFIG_TOUCHSCREEN_FT5X06
	/* FocalTech's touchScreen */
	i2c_register_board_info(1, i2c_ft5x06_board_info_pointer, 1);
#endif
	/* Register I2C1 adpter */
	platform_device_register(&as9260_i2c1_device);
	/* Register I2C0 adpter */
	platform_device_register(&as9260_i2c0_device);
#endif


#ifdef CONFIG_SPI_ASM9260
	/* used for spi controller raw /dev node operation */
	spi_register_board_info(spi_controller_dummy_board_info_pointer, 1);
	platform_device_register(&as9260_spi1_device);
#endif

#ifdef CONFIG_TOUCHSCREEN_XPT2046
	as9260_add_xpt2046_ts();
#endif


#ifdef CONFIG_QSPI_ASM9260
	/* used for spi controller raw /dev node operation*/
	spi_register_board_info(qspi_controller_dummy_board_info_pointer, 1);
	platform_device_register(&as9260_qspi_device);
#endif

#ifdef CONFIG_ASM9260_WATCHDOG
	asm9260_add_wdt();
#endif

#ifdef CONFIG_RTC_ASM9260
	asm9260_add_rtc();
#endif

#ifdef CONFIG_SENSORS_ADC_ASM9260
	platform_device_register(&asm9260_adc_device);
#endif

#ifdef CONFIG_SOC_CAMERA_ASM9260
	asm9260_add_camif();
#endif

#ifdef CONFIG_SOUND_ASM9260_SND_OSS
	platform_device_register(&asm9260_device_iis);
#endif

#if defined(CONFIG_ASM9260_MAC) || defined(CONFIG_ASM9260_MAC_MODULE)
	platform_device_register(&as9260_mac_device);
#endif

#if defined(CONFIG_MCPWM_ASM9260) || defined(CONFIG_MCPWM_ASM9260_MODULE)
	platform_device_register(&as9260_mcpwm_device);
#endif

#if defined(CONFIG_TIMER_ASM9260) || defined(CONFIG_TIMER_ASM9260_MODULE)
	platform_device_register(&as9260_timer_device);
#endif
}

static const char * const asm9260_dt_board_compat[] __initconst = {
	"alpscale,asm9260",
	NULL
};

DT_MACHINE_START(ASM9260, "Alpscale ASM9260 (Device Tree Support)")
//	.boot_params	= 0x20000100,
//	.phys_io	= 0x80000000,   /* the perih address */
//	.io_pg_offst	= ((0xfef00000) >> 18) & 0xfffc,
	.map_io		= as9260_map_io,    /*MAP_IO*/
//	.handle_irq	= icoll_handle_irq,
//	.init_irq	= as9260_init_irq,
	.init_machine	= as9260_init,
	.dt_compat	= asm9260_dt_board_compat,
//	.timer		= &as9260_timer,
	.init_time	= as9260_timer_init,
MACHINE_END
