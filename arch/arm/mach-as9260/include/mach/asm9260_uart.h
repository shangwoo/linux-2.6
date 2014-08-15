#ifndef __ASM_ARCH_ASM9260_UART_H
#define __ASM_ARCH_ASM9260_UART_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/serial.h>

/* Serial */
#define ASM9260_UART_CTS_PIN	0x01
#define ASM9260_UART_RTS_PIN	0x02
#define ASM9260_UART_DSR_PIN	0x04
#define ASM9260_UART_DTR_PIN	0x08
#define ASM9260_UART_DCD_PIN	0x10
#define ASM9260_UART_RI_PIN		0x20
#define ASM9260_UART_CLK_PIN	0x40

struct asm9260_uart_data {
	short		use_dma_tx;	/* use transmit DMA? */
	short		use_dma_rx;	/* use receive DMA? */
	void __iomem	*regs;		/* virtual base address, if any */
	struct serial_rs485	rs485;		/* rs485 settings */
};

extern struct platform_device *asm9260_default_console_device;
extern void __init asm9260_add_device_serial(void);
extern void __init asm9260_set_serial_console(unsigned portnr);
extern void __init asm9260_register_uart(unsigned id, unsigned portnr, unsigned pins);



#endif

