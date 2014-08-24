#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <mach/system.h>
#include <asm/hardware/iomd.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/uart_reg.h>
#include <mach/irqs.h>
#include <mach/asm9260_uart.h>
#include <mach/hardware.h>
#include <mach/pincontrol.h>

static struct platform_device *__initdata asm9260_uarts[ASM9260_MAX_UART];
/* the serial console device */
struct platform_device *asm9260_default_console_device;

/*************************************************************/



static struct resource uart0_resources[] = {
	[0] = {
		.start	= HW_UART0_CTRL0,
		.end	= HW_UART0_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART0,
		.end	= INT_UART0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart0_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART0_CTRL0),
};

static struct platform_device asm9260_uart0_device = {
	.name		= "asm9260_uart",
	.id			= 0,
	.dev		= {
				.platform_data		= &uart0_data,
	},
	.resource	= uart0_resources,
	.num_resources	= ARRAY_SIZE(uart0_resources),
};

static inline void configure_uart0_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(6, 0, 2);
	set_pin_mux(6, 1, 2);

	if (pins & ASM9260_UART_RTS_PIN)
		set_pin_mux(6, 2, 2);
	if (pins & ASM9260_UART_CTS_PIN)
		set_pin_mux(6, 3, 2);
	if (pins & ASM9260_UART_DTR_PIN)
		set_pin_mux(6, 4, 2);
	if (pins & ASM9260_UART_DSR_PIN)
		set_pin_mux(6, 5, 2);
	if (pins & ASM9260_UART_DCD_PIN)
		set_pin_mux(6, 6, 2);
	if (pins & ASM9260_UART_RI_PIN)
		set_pin_mux(6, 7, 2);
}

static struct resource uart1_resources[] = {
	[0] = {
		.start	= HW_UART1_CTRL0,
		.end	= HW_UART1_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart1_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART1_CTRL0),
};


static struct platform_device asm9260_uart1_device = {
	.name		= "asm9260_uart",
	.id			= 1,
	.dev		= {
				.platform_data		= &uart1_data,
	},
	.resource	= uart1_resources,
	.num_resources	= ARRAY_SIZE(uart1_resources),
};

static inline void configure_uart1_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(0, 1, 2);
	set_pin_mux(0, 2, 2);

	if (pins & ASM9260_UART_CLK_PIN)
		set_pin_mux(0, 0, 2);
	if (pins & ASM9260_UART_RTS_PIN)
		set_pin_mux(0, 3, 2);
	if (pins & ASM9260_UART_CTS_PIN)
		set_pin_mux(0, 4, 2);
}

static struct resource uart2_resources[] = {
	[0] = {
		.start	= HW_UART2_CTRL0,
		.end	= HW_UART2_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart2_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART2_CTRL0),
};

static struct platform_device asm9260_uart2_device = {
	.name		= "asm9260_uart",
	.id			= 2,
	.dev		= {
				.platform_data		= &uart2_data,
	},
	.resource	= uart2_resources,
	.num_resources	= ARRAY_SIZE(uart2_resources),
};

static inline void configure_uart2_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(0, 6, 2);
	set_pin_mux(0, 7, 2);

	if (pins & ASM9260_UART_CLK_PIN)
		set_pin_mux(0, 5, 2);
	if (pins & ASM9260_UART_RTS_PIN)
		set_pin_mux(2, 0, 2);
	if (pins & ASM9260_UART_CTS_PIN)
		set_pin_mux(2, 1, 2);

}

static struct resource uart3_resources[] = {
	[0] = {
		.start	= HW_UART3_CTRL0,
		.end	= HW_UART3_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart3_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART3_CTRL0),
};

static struct platform_device asm9260_uart3_device = {
	.name		= "asm9260_uart",
	.id			= 3,
	.dev		= {
				.platform_data		= &uart3_data,
	},
	.resource	= uart3_resources,
	.num_resources	= ARRAY_SIZE(uart3_resources),
};

static inline void configure_uart3_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(2, 3, 2);
	set_pin_mux(2, 4, 2);

	if (pins & ASM9260_UART_CLK_PIN)
		set_pin_mux(2, 2, 2);
	if (pins & ASM9260_UART_RTS_PIN)
		set_pin_mux(2, 5, 2);
	if (pins & ASM9260_UART_CTS_PIN)
		set_pin_mux(2, 6, 2);
}

static struct resource uart4_resources[] = {
	[0] = {
		.start	= HW_UART4_CTRL0,
		.end	= HW_UART4_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART4,
		.end	= INT_UART4,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart4_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART4_CTRL0),
};

static struct platform_device asm9260_uart4_device = {
	.name		= "asm9260_uart",
	.id			= 4,
	.dev		= {
				.platform_data		= &uart4_data,
	},
	.resource	= uart4_resources,
	.num_resources	= ARRAY_SIZE(uart4_resources),
};

static inline void configure_uart4_pins(unsigned pins)
{
#if 1
	/* TXD & RXD */
	set_pin_mux(3, 0, 2);
	set_pin_mux(3, 1, 2);

	if (pins & ASM9260_UART_CLK_PIN)
		set_pin_mux(2, 7, 2);
	if (pins & ASM9260_UART_RTS_PIN)
		set_pin_mux(3, 2, 2);
	if (pins & ASM9260_UART_CTS_PIN)
		set_pin_mux(3, 3, 2);
#else
	/* TXD & RXD */
	set_pin_mux(16, 0, 2);
	set_pin_mux(16, 1, 2);

	/* set pin3-0 and pin3-1 to gpio */
	set_pin_mux(3, 0, 0);
	set_pin_mux(3, 1, 0);
#endif
}

static struct resource uart5_resources[] = {
	[0] = {
		.start	= HW_UART5_CTRL0,
		.end	= HW_UART5_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART5,
		.end	= INT_UART5,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart5_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART5_CTRL0),
};

static struct platform_device asm9260_uart5_device = {
	.name		= "asm9260_uart",
	.id			= 5,
	.dev		= {
				.platform_data		= &uart5_data,
	},
	.resource	= uart5_resources,
	.num_resources	= ARRAY_SIZE(uart5_resources),
};

static inline void configure_uart5_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(16, 2, 2);
	set_pin_mux(16, 3, 2);
}

static struct resource uart6_resources[] = {
	[0] = {
		.start	= HW_UART6_CTRL0,
		.end	= HW_UART6_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART6,
		.end	= INT_UART6,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart6_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART6_CTRL0),
};

static struct platform_device asm9260_uart6_device = {
	.name		= "asm9260_uart",
	.id			= 6,
	.dev		= {
				.platform_data		= &uart6_data,
	},
	.resource	= uart6_resources,
	.num_resources	= ARRAY_SIZE(uart6_resources),
};

static inline void configure_uart6_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(16, 4, 2);
	set_pin_mux(16, 5, 2);

	if (pins & ASM9260_UART_RTS_PIN)
		set_pin_mux(16, 6, 2);

}

static struct resource uart7_resources[] = {
	[0] = {
		.start	= HW_UART7_CTRL0,
		.end	= HW_UART7_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART7,
		.end	= INT_UART7,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart7_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART7_CTRL0),
};

static struct platform_device asm9260_uart7_device = {
	.name		= "asm9260_uart",
	.id			= 7,
	.dev		= {
				.platform_data		= &uart7_data,
	},
	.resource	= uart7_resources,
	.num_resources	= ARRAY_SIZE(uart7_resources),
};

static inline void configure_uart7_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(17, 0, 2);
	set_pin_mux(17, 1, 2);
}

static struct resource uart8_resources[] = {
	[0] = {
		.start	= HW_UART8_CTRL0,
		.end	= HW_UART8_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART8,
		.end	= INT_UART8,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart8_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART8_CTRL0),
};

static struct platform_device asm9260_uart8_device = {
	.name		= "asm9260_uart",
	.id			= 8,
	.dev		= {
				.platform_data		= &uart8_data,
	},
	.resource	= uart8_resources,
	.num_resources	= ARRAY_SIZE(uart8_resources),
};

static inline void configure_uart8_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(17, 4, 2);
	set_pin_mux(17, 5, 2);
}

static struct resource uart9_resources[] = {
	[0] = {
		.start	= HW_UART9_CTRL0,
		.end	= HW_UART9_CTRL0 + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_UART9,
		.end	= INT_UART9,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct asm9260_uart_data uart9_data = {
	.regs = (void __iomem *)IO_ADDRESS(HW_UART9_CTRL0),
};


static struct platform_device asm9260_uart9_device = {
	.name		= "asm9260_uart",
	.id			= 9,
	.dev		= {
				.platform_data		= &uart9_data,
	},
	.resource	= uart9_resources,
	.num_resources	= ARRAY_SIZE(uart9_resources),
};

static inline void configure_uart9_pins(unsigned pins)
{
	/* TXD & RXD */
	set_pin_mux(17, 6, 2);
	set_pin_mux(17, 7, 2);
}

#define	CLOCK_DIV_MASK		0xFF
void __init asm9260_register_uart(unsigned id, unsigned portnr, unsigned pins)
{
	struct platform_device *pdev;
	unsigned int uartclk_div, cpuclk_div, hclk_div;

	hclk_div = as3310_readl(HW_SYSAHBCLKDIV) & CLOCK_DIV_MASK;
	cpuclk_div = as3310_readl(HW_CPUCLKDIV) & CLOCK_DIV_MASK;

	/* uartclk must be equal to hclk because of RS485 delay register */
	uartclk_div = hclk_div * cpuclk_div;

	/* UART clock select pll */
	if (as3310_readl(HW_PDRUNCFG) & 0x4) {
		/* pllclk power down */
		as3310_writel(0, HW_UARTCLKSEL);
		cpuclk_div = 1;
	} else
		as3310_writel(1, HW_UARTCLKSEL);

	as3310_writel(0, HW_UARTCLKUEN);
	as3310_writel(1, HW_UARTCLKUEN);

	switch (id) {
	case 0:
		as3310_writel(1 << 11, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART0CLKDIV);
		pdev = &asm9260_uart0_device;
		configure_uart0_pins(pins);
		break;
	case 1:
		as3310_writel(1 << 12, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART1CLKDIV);
		pdev = &asm9260_uart1_device;
		configure_uart1_pins(pins);
		break;
	case 2:
		as3310_writel(1 << 13, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART2CLKDIV);
		pdev = &asm9260_uart2_device;
		configure_uart2_pins(pins);
		break;
	case 3:
		as3310_writel(1 << 14, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART3CLKDIV);
		pdev = &asm9260_uart3_device;
		configure_uart3_pins(pins);
		break;
	case 4:
		as3310_writel(1 << 15, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART4CLKDIV);
		pdev = &asm9260_uart4_device;
		configure_uart4_pins(pins);
		break;
	case 5:
		as3310_writel(1 << 16, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART5CLKDIV);
		pdev = &asm9260_uart5_device;
		configure_uart5_pins(pins);
		break;
	case 6:
		as3310_writel(1 << 17, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART6CLKDIV);
		pdev = &asm9260_uart6_device;
		configure_uart6_pins(pins);
		break;
	case 7:
		as3310_writel(1 << 18, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART7CLKDIV);
		pdev = &asm9260_uart7_device;
		configure_uart7_pins(pins);
		break;
	case 8:
		as3310_writel(1 << 19, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART8CLKDIV);
		pdev = &asm9260_uart8_device;
		configure_uart8_pins(pins);
		break;
	case 9:
		as3310_writel(1 << 20, HW_AHBCLKCTRL0 + 4);
		as3310_writel(uartclk_div, HW_UART9CLKDIV);
		pdev = &asm9260_uart9_device;
		configure_uart9_pins(pins);
		break;
	default:
		return;
	}

	pdev->id = portnr;

	if (portnr < ASM9260_MAX_UART)
		asm9260_uarts[portnr] = pdev;
}

void __init asm9260_set_serial_console(unsigned portnr)
{
	if (portnr < ASM9260_MAX_UART)
		asm9260_default_console_device = asm9260_uarts[portnr];
}

void __init asm9260_add_device_serial(void)
{
	int i;

#if 0
	for (i = 0; i < ASM9260_MAX_UART; i++) {
		if (asm9260_uarts[i])
			platform_device_register(asm9260_uarts[i]);
	}
#endif

	if (!asm9260_default_console_device)
		printk(KERN_INFO "ASM9260: No default serial console defined.\n");
}
