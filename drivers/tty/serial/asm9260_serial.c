/*
 *  linux/drivers/char/asm9260_serial.c
 *
 *  Driver for ALPSCALE ASM9260 Serial ports
 *  Copyright (C) 2013
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#include <linux/io.h>
#include <linux/slab.h>
#include <mach/hardware.h>
#include <mach/asm9260_uart.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#define SERIAL_ASM9260_MAJOR	204
#define MINOR_START		64
#define ASM9260_DEVICENAME	"ttyS"


#ifdef CONFIG_ASM9260_UART_DEBUG
#define dbg(format, arg...) printk("ASM9260_UART_DBG: " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

#define ASM9260_UART_FIFOSIZE	16
#define ASM9260_ISR_PASS_LIMIT	256

#define ASM9260_BUS_RATE	100000000

#define SOURCE_CLOCK_EXT12M				0
#define SOURCE_CLOCK_SYSPLL				1
#define	PLL_POWER_DOWN					(1<<2)
#define	SYSPLL_MASK						0x1FF

#define UART_BAUD_DIVINT_MASK			((unsigned int)0x003FFFC0)
#define UART_BAUD_DIVFRAC_MASK			((unsigned int)0x0000003F)
#define	UART_BAUD_DIV_MAX				0x3FFFFF
#define ASM9260_UART_RXTIMEOUT			(0xFF<<16)
#define ASM9260_UART_RXTO_ENABLE		(1<<24)
#define ASM9260_UART_RXTO_SOURCE_DATA	(0<<25)
#define ASM9260_UART_RXTO_SOURCE_STATUS	(1<<25)
#define ASM9260_UART_DEFAULT_RXTIMEOUT	(20<<16) /* TIMEOUT = (100*7+1)*(1/BAUD) */

#define ASM9260_UART_ENABLE				(1<<0)
#define ASM9260_UART_LBE				(1<<7)
#define ASM9260_UART_TXE				(1<<8)
#define ASM9260_UART_RXE				(1<<9)
#define ASM9260_UART_RTSE				(1<<14)
#define ASM9260_UART_CTSE				(1<<15)
#define ASM9260_UART_TXIFLSEL				(7<<16)
#define ASM9260_UART_RXIFLSEL				(7<<20)
#define ASM9260_UART_DEFAULT_TXIFLSEL			(2<<16)
#define ASM9260_UART_DEFAULT_RXIFLSEL			(3<<20)


#define ASM9260_UART_FRAMEERR				(1<<16)
#define ASM9260_UART_PARITYERR				(1<<17)
#define ASM9260_UART_BREAKERR				(1<<18)
#define ASM9260_UART_OVERRUNERR				(1<<19)
#define ASM9260_UART_RXEMPTY				(1<<24)
#define ASM9260_UART_TXFULL				(1<<25)
#define ASM9260_UART_RXFULL				(1<<26)
#define ASM9260_UART_TXEMPTY				(1<<27)
#define ASM9260_UART_CTS				(1<<28)
#define ASM9260_UART_BUSY				(1<<29)

#define ASM9260_UART_BREAK				(1<<0)
#define ASM9260_UART_PEN				(1<<1)
#define ASM9260_UART_EPS				(1<<2)
#define ASM9260_UART_STP2				(1<<3)
#define ASM9260_UART_FEN				(1<<4)
#define ASM9260_UART_WLEN				(3<<5)
#define ASM9260_UART_SPS				(1<<7)
#define ASM9260_UART_BAUD_DIVFRA			(0x3F<<8)
#define ASM9260_UART_BAUD_DIVINT			(0xFFFF<<16)
#define ASM9260_US_CHRL_5				(0<<5)
#define ASM9260_US_CHRL_6				(1<<5)
#define ASM9260_US_CHRL_7				(2<<5)
#define ASM9260_US_CHRL_8				(3<<5)
#define ASM9260_US_NBSTOP_1				(0<<3)
#define ASM9260_US_NBSTOP_2				(1<<3)
#define ASM9260_US_PAR_MARK				((3<<1) | (1<<7))
#define ASM9260_US_PAR_SPACE				((1<<1) | (1<<7))
#define ASM9260_US_PAR_ODD				((1<<1) | (0<<7))
#define ASM9260_US_PAR_EVEN				((3<<1) | (0<<7))
#define ASM9260_US_PAR_NONE				(0<<1)

#define ASM9260_UART_RIMIS				(1<<0)
#define ASM9260_UART_CTSMIS				(1<<1)
#define ASM9260_UART_DCDMIS				(1<<2)
#define ASM9260_UART_DSRMIS				(1<<3)
#define ASM9260_UART_RXIS				(1<<4)
#define ASM9260_UART_TXIS				(1<<5)
#define ASM9260_UART_RTIS				(1<<6)
#define ASM9260_UART_FEIS				(1<<7)
#define ASM9260_UART_PEIS				(1<<8)
#define ASM9260_UART_BEIS				(1<<9)
#define ASM9260_UART_OEIS				(1<<10)
#define ASM9260_UART_TFEIS				(1<<11)
#define ASM9260_UART_ABEO				(1<<12)
#define ASM9260_UART_ABTO				(1<<13)
#define ASM9260_UART_RIMIEN				(1<<16)
#define ASM9260_UART_CTSMIEN				(1<<17)
#define ASM9260_UART_DCDMIEN				(1<<18)
#define ASM9260_UART_DSRMIEN				(1<<19)
#define ASM9260_UART_RXIEN				(1<<20)
#define ASM9260_UART_TXIEN				(1<<21)
#define ASM9260_UART_RTIEN				(1<<22)
#define ASM9260_UART_FEIEN				(1<<23)
#define ASM9260_UART_PEIEN				(1<<24)
#define ASM9260_UART_BEIEN				(1<<25)
#define ASM9260_UART_OEIEN				(1<<26)
#define ASM9260_UART_TFEIEN				(1<<27)
#define ASM9260_UART_INTREN				(0x3fff0000)
#define ASM9260_UART_INTRIS				(0x00003fff)

#define ASM9260_UART_CTRL0				0x00
#define ASM9260_UART_CTRL1				0x10
#define ASM9260_UART_CTRL2				0x20
#define ASM9260_UART_CTRL3				0xD0
#define ASM9260_UART_LINECTRL				0x30
#define ASM9260_UART_INTR				0x40
#define ASM9260_UART_DATA				0x50
#define ASM9260_UART_STAT				0x60
#define ASM9260_UART_ILPR				0x80
#define ASM9260_UART_RS485CTRL				0x90
#define ASM9260_UART_RS485ADRMATCH			0xA0
#define ASM9260_UART_RS485DLY				0xB0
#define ASM9260_UART_AUTOBAUD				0xC0

#define	ASM9260_UART_RS485EN				0x01
#define	ASM9260_UART_RS485_RXDIS			0x02
#define	ASM9260_UART_RS485_AADEN			0x04
#define	ASM9260_UART_RS485_PINSEL			0x08
#define	ASM9260_UART_RS485_DIR_CTRL			0x10
#define	ASM9260_UART_RS485_ONIV				0x20

#define UART_PUT_CTRL0(port, v)				iowrite32(v, (port)->membase + ASM9260_UART_CTRL0)
#define UART_PUT_CTRL0_SET(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_CTRL0 + SET_OFFSET)
#define UART_PUT_CTRL0_CLR(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_CTRL0 + CLR_OFFSET)

#define UART_PUT_CTRL1(port, v)				iowrite32(v, (port)->membase + ASM9260_UART_CTRL1)
#define UART_PUT_CTRL1_SET(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_CTRL1 + SET_OFFSET)
#define UART_PUT_CTRL1_CLR(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_CTRL1 + CLR_OFFSET)

#define UART_PUT_CTRL2(port, v)				iowrite32(v, (port)->membase + ASM9260_UART_CTRL2)
#define UART_PUT_CTRL2_SET(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_CTRL2 + SET_OFFSET)
#define UART_PUT_CTRL2_CLR(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_CTRL2 + CLR_OFFSET)

#define UART_PUT_CTRL3(port, v)				iowrite32(v, (port)->membase + ASM9260_UART_CTRL3)
#define UART_PUT_CTRL3_SET(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_CTRL3 + SET_OFFSET)
#define UART_PUT_CTRL3_CLR(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_CTRL3 + CLR_OFFSET)

#define UART_PUT_LINECTRL(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_LINECTRL)
#define UART_PUT_LINECTRL_SET(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_LINECTRL + SET_OFFSET)
#define UART_PUT_LINECTRL_CLR(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_LINECTRL + CLR_OFFSET)

#define UART_PUT_INTR(port, v)				iowrite32(v, (port)->membase + ASM9260_UART_INTR)
#define UART_PUT_INTR_SET(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_INTR + SET_OFFSET)
#define UART_PUT_INTR_CLR(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_INTR + CLR_OFFSET)

#define UART_PUT_DATA(port, v)				iowrite32(v, (port)->membase + ASM9260_UART_DATA)

#define UART_PUT_STAT(port, v)				iowrite32(v, (port)->membase + ASM9260_UART_STAT)
#define UART_PUT_STAT_SET(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_STAT + SET_OFFSET)
#define UART_PUT_STAT_CLR(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_STAT + CLR_OFFSET)

#define UART_PUT_ILPR(port, v)				iowrite32(v, (port)->membase + ASM9260_UART_ILPR)
#define UART_PUT_ILPR_SET(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_ILPR + SET_OFFSET)
#define UART_PUT_ILPR_CLR(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_ILPR + CLR_OFFSET)

#define UART_PUT_RS485CTRL(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_RS485CTRL)
#define UART_PUT_RS485ADRMATCH(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_RS485ADRMATCH)
#define UART_PUT_RS485DLY(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_RS485DLY)
#define UART_PUT_AUTOBAUD(port, v)			iowrite32(v, (port)->membase + ASM9260_UART_AUTOBAUD)
#define UART_GET_CTRL0(port)				ioread32((port)->membase + ASM9260_UART_CTRL0)
#define UART_GET_CTRL1(port)				ioread32((port)->membase + ASM9260_UART_CTRL1)
#define UART_GET_CTRL2(port)				ioread32((port)->membase + ASM9260_UART_CTRL2)
#define UART_GET_CTRL3(port)				ioread32((port)->membase + ASM9260_UART_CTRL3)
#define UART_GET_LINECTRL(port)				ioread32((port)->membase + ASM9260_UART_LINECTRL)
#define UART_GET_INTR(port)				ioread32((port)->membase + ASM9260_UART_INTR)
#define UART_GET_DATA(port)				ioread32((port)->membase + ASM9260_UART_DATA)
#define UART_GET_STAT(port)				ioread32((port)->membase + ASM9260_UART_STAT)
#define UART_GET_ILPR(port)				ioread32((port)->membase + ASM9260_UART_ILPR)
#define UART_GET_RS485CTRL(port)			ioread32((port)->membase + ASM9260_UART_RS485CTRL)
#define UART_GET_RS485ADRMATCH(port)			ioread32((port)->membase + ASM9260_UART_RS485ADRMATCH)
#define UART_GET_RS485DLY(port)				ioread32((port)->membase + ASM9260_UART_RS485DLY)
#define UART_GET_AUTOBAUD(port)				ioread32((port)->membase + ASM9260_UART_AUTOBAUD)

struct asm9260_uart_char {
	u16		status;
	u16		ch;
};

#define ASM9260_SERIAL_RINGSIZE 4096

/*
 * We wrap our port structure around the generic uart_port.
 */
struct asm9260_uart_port {
	struct uart_port	uart;		/* uart */
	struct device_node	*np;
	struct clk		*clk;		/* uart clock */
	struct clk		*clk_ahb;
	int			break_active;	/* break being received */

	struct tasklet_struct	tasklet;

	struct circ_buf		rx_ring;

	struct serial_rs485	rs485;		/* rs485 settings */

	uint32_t intmask;
	int init_ok;
};

//static struct asm9260_uart_port asm9260_ports[ASM9260_MAX_UART];
static void asm9260_start_rx(struct uart_port *port);
static struct asm9260_uart_port *asm9260_ports;
static int asm9260_ports_num;


static inline struct asm9260_uart_port *
to_asm9260_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct asm9260_uart_port, uart);
}

static void asm9260_intr_mask_set(struct uart_port *port, uint32_t val)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);

	WARN_ON(val & ~ASM9260_UART_INTREN);

	if (val && (asm9260_port->intmask & val) != val) {
		UART_PUT_INTR_SET(port, val);
		asm9260_port->intmask |= val;
	}
}

static void asm9260_intr_mask_clr(struct uart_port *port, uint32_t val)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);

	WARN_ON(val & ~ASM9260_UART_INTREN);

	if (val && (asm9260_port->intmask & val) != 0) {
		UART_PUT_INTR_CLR(port, val);
		asm9260_port->intmask &= ~val;
	}
}

/*
 * Return TIOCSER_TEMT when transmitter FIFO and Shift register is empty.
 */
static u_int asm9260_tx_empty(struct uart_port *port)
{
	dbg("asm9260_tx_empty");
	return (UART_GET_STAT(port) & ASM9260_UART_TXEMPTY) ? TIOCSER_TEMT : 0;
}

/*
 * Set state of the modem control output lines
 */
static void asm9260_set_mctrl(struct uart_port *port, u_int mctrl)
{
	dbg("asm9260_set_mctrl:0x%x", mctrl);
}

/*
 * Get state of the modem control input lines
 */
static u_int asm9260_get_mctrl(struct uart_port *port)
{
	dbg("asm9260_get_mctrl");
	/*The driver doesn't support modem control*/

	return 0;
}

/*
 * Stop transmitting.
 */
static void asm9260_stop_tx(struct uart_port *port)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);

	asm9260_intr_mask_clr(port, ASM9260_UART_TXIEN);

	if ((asm9260_port->rs485.flags & SER_RS485_ENABLED) &&
	    !(asm9260_port->rs485.flags & SER_RS485_RX_DURING_TX))
		asm9260_start_rx(port);
}

static void asm9260_tx_chars(struct uart_port *port);
/*
 * Start transmitting.
 */
static void asm9260_start_tx(struct uart_port *port)
{
	asm9260_intr_mask_set(port, ASM9260_UART_TXIEN);
	asm9260_tx_chars(port);
}

/*
 * start receiving - port is in process of being opened.
 */
static void asm9260_start_rx(struct uart_port *port)
{
	dbg("asm9260_start_rx");

	UART_PUT_INTR_CLR(port, ASM9260_UART_RXIS | ASM9260_UART_RTIS);

	/* enable receive */
	UART_PUT_CTRL2_SET(port, ASM9260_UART_RXE);
	UART_PUT_INTR_SET(port, ASM9260_UART_RXIEN | ASM9260_UART_RTIEN);
}

/*
 * Stop receiving - port is in process of being closed.
 */
static void asm9260_stop_rx(struct uart_port *port)
{
	dbg("asm9260_stop_rx");

	/* disable receive */
	UART_PUT_CTRL2_CLR(port, ASM9260_UART_RXE);
	asm9260_intr_mask_clr(port, ASM9260_UART_RXIEN | ASM9260_UART_RTIEN);
}

/*
 * Enable modem status interrupts
 */
static void asm9260_enable_ms(struct uart_port *port)
{
	dbg("asm9260_enable_ms");
	/*The driver doesn't support modem control*/
}

/*
 * Control the transmission of a break signal
 */
static void asm9260_break_ctl(struct uart_port *port, int break_state)
{
	dbg("asm9260_break_ctl");
	if (break_state != 0)
		UART_PUT_LINECTRL_SET(port, ASM9260_UART_BREAK);	/* start break */
	else
		UART_PUT_LINECTRL_CLR(port, ASM9260_UART_BREAK);	/* stop break */
}

/*
 * Stores the incoming character in the ring buffer
 */
static void
asm9260_buffer_rx_char(struct uart_port *port, unsigned int status,
		     unsigned int ch)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);
	struct circ_buf *ring = &asm9260_port->rx_ring;
	struct asm9260_uart_char *c;

	dbg("asm9260_buffer_rx_char");

	if (!CIRC_SPACE(ring->head, ring->tail, ASM9260_SERIAL_RINGSIZE))
		/* Buffer overflow, ignore char */
		return;

	c = &((struct asm9260_uart_char *)ring->buf)[ring->head];
	c->status	= status;
	c->ch		= ch;

	/* Make sure the character is stored before we update head. */
	smp_wmb();

	ring->head = (ring->head + 1) & (ASM9260_SERIAL_RINGSIZE - 1);
}

/*
 * Characters received (called from interrupt handler)
 */
static void asm9260_rx_chars(struct uart_port *port)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);
	unsigned int status, intr, ch;

	dbg("asm9260_rx_chars");

	status = UART_GET_STAT(port);
	while (!(status & ASM9260_UART_RXEMPTY)) {
		ch = UART_GET_DATA(port);
		intr = UART_GET_INTR(port);
		/*
		 * note that the error handling code is
		 * out of the main execution path
		 */
		if (unlikely(intr & (ASM9260_UART_PEIS | ASM9260_UART_FEIS
				       | ASM9260_UART_OEIS | ASM9260_UART_BEIS)
			     || asm9260_port->break_active)) {

			/* clear error */
			UART_PUT_STAT(port, 0);
			UART_PUT_INTR_CLR(port, ASM9260_UART_PEIS
					| ASM9260_UART_FEIS  | ASM9260_UART_OEIS
					| ASM9260_UART_BEIS);

			if (intr & ASM9260_UART_BEIS
			    && !asm9260_port->break_active) {
				asm9260_port->break_active = 1;
				asm9260_intr_mask_set(port, ASM9260_UART_BEIEN);
			} else {
				asm9260_intr_mask_clr(port, ASM9260_UART_BEIEN);
				intr &= ~ASM9260_UART_BEIS;
				asm9260_port->break_active = 0;
			}
		}

		asm9260_buffer_rx_char(port, intr, ch);
		status = UART_GET_STAT(port);
	}

	tasklet_schedule(&asm9260_port->tasklet);
}

/*
 * Transmit characters (called from tasklet with TXRDY interrupt
 * disabled)
 */
static void asm9260_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	dbg("asm9260_tx_chars");

	if (port->x_char && !(UART_GET_STAT(port) & ASM9260_UART_TXFULL)) {
		UART_PUT_DATA(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;

	while (!uart_circ_empty(xmit)) {
		if (UART_GET_STAT(port) & ASM9260_UART_TXFULL) {
			break;;
		}
		UART_PUT_DATA(port, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		asm9260_intr_mask_clr(port, ASM9260_UART_TXIEN);
}

/*
 * receive interrupt handler.
 */
static void
asm9260_handle_receive(struct uart_port *port, unsigned int pending)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);

	dbg("asm9260_handle_receive");

	/* Interrupt receive */
	if ((pending & ASM9260_UART_RXIS) || (pending & ASM9260_UART_RTIS)) {

		if (pending & ASM9260_UART_RXIS)
			UART_PUT_INTR_CLR(port, ASM9260_UART_RXIS);
		if (pending & ASM9260_UART_RTIS)
			UART_PUT_INTR_CLR(port, ASM9260_UART_RTIS);

		asm9260_rx_chars(port);
	} else if (pending & ASM9260_UART_BEIS) {
		/*
		 * End of break detected. If it came along with a
		 * character, asm9260_rx_chars will handle it.
		 */
		UART_PUT_STAT(port, 0);
		asm9260_intr_mask_clr(port, ASM9260_UART_BEIEN);
		asm9260_port->break_active = 0;
	}
}

/*
 * transmit interrupt handler. (Transmit is IRQF_NODELAY safe)
 */
static void
asm9260_handle_transmit(struct uart_port *port, unsigned int pending)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);

    /* Interrupt transmit */

	if (pending & ASM9260_UART_TXIS) {
		UART_PUT_INTR_CLR(port, ASM9260_UART_TXIS);
		tasklet_schedule(&asm9260_port->tasklet);
	}
}

/*
 * Interrupt handler
 */
static irqreturn_t asm9260_interrupt(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned int status, pending, pass_counter = 0;

	do {
		status = UART_GET_INTR(port);
		pending = (status & (status >> 16)) & 0xFFF;
		if (!pending)
			break;
		asm9260_handle_receive(port, pending);
		asm9260_handle_transmit(port, pending);
	} while (pass_counter++ < ASM9260_ISR_PASS_LIMIT);

	return pass_counter ? IRQ_HANDLED : IRQ_NONE;
}

static void asm9260_rx_from_ring(struct uart_port *port)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);
	struct circ_buf *ring = &asm9260_port->rx_ring;
	unsigned int flg;
	unsigned int status;

	dbg("asm9260_rx_from_ring");

	while (ring->head != ring->tail) {
		struct asm9260_uart_char c;

		/* Make sure c is loaded after head. */
		smp_rmb();

		c = ((struct asm9260_uart_char *)ring->buf)[ring->tail];

		ring->tail = (ring->tail + 1) & (ASM9260_SERIAL_RINGSIZE - 1);

		port->icount.rx++;
		status = c.status;
		flg = TTY_NORMAL;

		/*
		 * note that the error handling code is
		 * out of the main execution path
		 */
		if (unlikely(status & (ASM9260_UART_PEIS | ASM9260_UART_FEIS
				       | ASM9260_UART_OEIS | ASM9260_UART_BEIS))) {
			if (status & ASM9260_UART_BEIS) {
				status &= ~(ASM9260_UART_PEIS | ASM9260_UART_FEIS);

				port->icount.brk++;
				if (uart_handle_break(port))
					continue;
			}
			if (status & ASM9260_UART_PEIS)
				port->icount.parity++;
			if (status & ASM9260_UART_FEIS)
				port->icount.frame++;
			if (status & ASM9260_UART_OEIS)
				port->icount.overrun++;

			status &= port->read_status_mask;

			if (status & ASM9260_UART_BEIS)
				flg = TTY_BREAK;
			else if (status & ASM9260_UART_PEIS)
				flg = TTY_PARITY;
			else if (status & ASM9260_UART_FEIS)
				flg = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, c.ch))
			continue;

		uart_insert_char(port, status, ASM9260_UART_OEIS, c.ch, flg);
	}

	/*
	 * Drop the lock here since it might end up calling
	 * uart_start(), which takes the lock.
	 */
	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
	spin_lock(&port->lock);
}

/*
 * tasklet handling tty stuff outside the interrupt handler.
 */
static void asm9260_tasklet_func(unsigned long data)
{
	struct uart_port *port = (struct uart_port *)data;

	dbg("asm9260_tasklet_func");

	/* The interrupt handler does not take the lock */
	spin_lock(&port->lock);

	asm9260_tx_chars(port);
	asm9260_rx_from_ring(port);

	spin_unlock(&port->lock);
}

/*
 * Perform initialization and enable port for reception
 */
static int asm9260_startup(struct uart_port *port)
{
	struct tty_struct *tty = port->state->port.tty;
	int retval;

	dbg("asm9260_startup");

	/*
	 * Ensure that no interrupts are enabled otherwise when
	 * request_irq() is called we could get stuck trying to
	 * handle an unexpected interrupt
	 */
	UART_PUT_INTR(port, 0); /* disable all interrupt */

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(port->irq, asm9260_interrupt, IRQF_SHARED,
			tty ? tty->name : "asm9260_serial", port);
	if (retval) {
		printk("asm9260_serial : asm9260_startup - Can't get irq\n");
		return retval;
	}

	/*enable rx timeout*/
	UART_PUT_CTRL0_CLR(port, ASM9260_UART_RXTIMEOUT
			| ASM9260_UART_RXTO_SOURCE_STATUS);
	UART_PUT_CTRL0_SET(port, ASM9260_UART_DEFAULT_RXTIMEOUT
			| ASM9260_UART_RXTO_ENABLE);

	/* disable txfifo empty interrupt, enable rx and rxto interrupt */
	asm9260_intr_mask_clr(port, ASM9260_UART_TFEIEN);
	asm9260_intr_mask_set(port, ASM9260_UART_RXIEN | ASM9260_UART_RTIEN);

	/*
	 * Finally, enable the serial port
	 * enable tx & rx
	 */
	UART_PUT_CTRL2_CLR(port, ASM9260_UART_RXIFLSEL | ASM9260_UART_TXIFLSEL);
	UART_PUT_CTRL2(port, ASM9260_UART_ENABLE | ASM9260_UART_TXE
			| ASM9260_UART_RXE | ASM9260_UART_DEFAULT_TXIFLSEL
			| ASM9260_UART_DEFAULT_RXIFLSEL);

	return 0;
}

/*
 * Disable the port
 */
static void asm9260_shutdown(struct uart_port *port)
{
	int timeout = 10000;

	dbg("asm9260_shutdown");

	/*wait for controller finish tx*/
	while (!(UART_GET_STAT(port) & ASM9260_UART_TXEMPTY)) {
		if (--timeout < 0)
			break;
	}

	/*
	 * Ensure everything is stopped.
	 */
	asm9260_stop_tx(port);
	asm9260_stop_rx(port);

	/*
	 * Free the interrupt
	 */
	free_irq(port->irq, port);
}

/*
 * Flush any TX data submitted for DMA. Called when the TX circular
 * buffer is reset.
 */
static void asm9260_flush_buffer(struct uart_port *port)
{
	dbg("asm9260_flush_buffer");
}

/*
 * Power / Clock management.
 */
static void asm9260_serial_pm(struct uart_port *port, unsigned int state,
			    unsigned int oldstate)
{
	dbg("asm9260_serial_pm");
}

/*
 * Change the port parameters
 */
static void asm9260_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	unsigned long flags;
	unsigned int mode, baud, rs485_ctrl;
	unsigned int bauddivint, bauddivfrac;
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);

	dbg("set_termios start");

	/*
	 * We don't support modem control lines.
	*/
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/* Get current mode register */
	mode = UART_GET_LINECTRL(port) & ~(ASM9260_UART_PEN | ASM9260_UART_EPS
			| ASM9260_UART_STP2 | ASM9260_UART_FEN
			| ASM9260_UART_WLEN | ASM9260_UART_SPS
			| ASM9260_UART_BAUD_DIVFRA | ASM9260_UART_BAUD_DIVINT);

	baud = uart_get_baud_rate(port, termios, old,
			port->uartclk * 4 / UART_BAUD_DIV_MAX, port->uartclk / 16);
	bauddivint =
		(((port->uartclk << 2) / baud) & UART_BAUD_DIVINT_MASK) << 10;
	bauddivfrac =
		(((port->uartclk << 2) / baud) & UART_BAUD_DIVFRAC_MASK) << 8;
	/* byte size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		mode |= ASM9260_US_CHRL_5;
		break;
	case CS6:
		mode |= ASM9260_US_CHRL_6;
		break;
	case CS7:
		mode |= ASM9260_US_CHRL_7;
		break;
	default:
		mode |= ASM9260_US_CHRL_8;
		break;
	}

	/* disable fifo */
	mode |= ASM9260_UART_FEN;

	/* stop bits */
	if (termios->c_cflag & CSTOPB)
		mode |= ASM9260_US_NBSTOP_2;
	else
		mode |= ASM9260_US_NBSTOP_1;

	/* parity */
	if (termios->c_cflag & PARENB) {
		/* Mark or Space parity */
		if (termios->c_cflag & CMSPAR) {
			if (termios->c_cflag & PARODD)
				mode |= ASM9260_US_PAR_MARK;
			else
				mode |= ASM9260_US_PAR_SPACE;
		} else if (termios->c_cflag & PARODD)
			mode |= ASM9260_US_PAR_ODD;
		else
			mode |= ASM9260_US_PAR_EVEN;
	} else
		mode |= ASM9260_US_PAR_NONE;

	spin_lock_irqsave(&port->lock, flags);

	port->read_status_mask = ASM9260_UART_OEIS;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= (ASM9260_UART_FEIS | ASM9260_UART_PEIS);
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= ASM9260_UART_BEIS;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |=
			(ASM9260_UART_FEIS  | ASM9260_UART_PEIS);
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= ASM9260_UART_BEIS;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= ASM9260_UART_OEIS;
	}

	/* update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	/* drain transmitter */
	while (!(UART_GET_STAT(port) & ASM9260_UART_TXEMPTY))
		cpu_relax();

	while (!(UART_GET_STAT(port) & ASM9260_UART_RXEMPTY))
		UART_GET_DATA(port);

	/* set RS485 */
	rs485_ctrl = UART_GET_RS485CTRL(port);

	/* Resetting serial mode to RS232 (0x0) */
	rs485_ctrl &= ~ASM9260_UART_RS485EN;

	if (asm9260_port->rs485.flags & SER_RS485_ENABLED) {
		dev_dbg(port->dev, "Setting UART to RS485\n");
		if ((asm9260_port->rs485.delay_rts_after_send) > 0) {
			/* delay is (rs485conf->delay_rts_after_send * Bit Period * 1/16) */
			UART_PUT_RS485DLY(port, asm9260_port->rs485.delay_rts_after_send);
		}

		if ((asm9260_port->rs485.flags & SER_RS485_RTS_ON_SEND) &&
			!(asm9260_port->rs485.flags & SER_RS485_RTS_AFTER_SEND)) {
			/*
			 * Set logical level for RTS pin equal to 1 when sending,
			 * and set logical level for RTS pin equal to 0 after sending
			*/
			rs485_ctrl |= ASM9260_UART_RS485_ONIV;
		} else if (!(asm9260_port->rs485.flags & SER_RS485_RTS_ON_SEND) &&
			(asm9260_port->rs485.flags & SER_RS485_RTS_AFTER_SEND)) {
			/*
			 * Set logical level for RTS pin equal to 0 when sending,
			 * and set logical level for RTS pin equal to 1 after sending
			*/
			rs485_ctrl &= ~ASM9260_UART_RS485_ONIV;
		} else{
			printk("Please view RS485CTRL register in datasheet for more details.\n");
		}

		/* Enable RS485 and RTS is used to control direction automatically,  */
		rs485_ctrl |= ASM9260_UART_RS485EN | ASM9260_UART_RS485_DIR_CTRL;
		rs485_ctrl &= ~ASM9260_UART_RS485_PINSEL;

		if (asm9260_port->rs485.flags & SER_RS485_RX_DURING_TX)
			dev_dbg(port->dev, "hardware should support SER_RS485_RX_DURING_TX.\n");
	} else {
		dev_dbg(port->dev, "Setting UART to RS232\n");
	}

	UART_PUT_RS485CTRL(port, rs485_ctrl);

	/* set hardware flow control */
	if (termios->c_cflag & CRTSCTS)
		UART_PUT_CTRL2_SET(port, ASM9260_UART_CTSE | ASM9260_UART_RTSE);
	else
		UART_PUT_CTRL2_CLR(port, ASM9260_UART_CTSE | ASM9260_UART_RTSE);

	/* set the parity, stop bits, data size and baud rate*/
	UART_PUT_LINECTRL(port, mode | bauddivint | bauddivfrac);

	/* CTS flow-control and modem-status interrupts */
	if (UART_ENABLE_MS(port, termios->c_cflag))
		port->ops->enable_ms(port);

	spin_unlock_irqrestore(&port->lock, flags);

	dbg("mode:0x%x, baud:%d, bauddivint:0x%x, bauddivfrac:0x%x, ctrl2:0x%x\n",
			mode, baud, bauddivint, bauddivfrac, UART_GET_CTRL2(port));
}

/*
 * Return string describing the specified port
 */
static const char *asm9260_type(struct uart_port *port)
{
	dbg("asm9260_type");
	return (port->type == PORT_ATMEL) ? "ASM9260_SERIAL" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void asm9260_release_port(struct uart_port *port)
{
	dbg("asm9260_release_port");
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int asm9260_request_port(struct uart_port *port)
{

	dbg("asm9260_request_port");

	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void asm9260_config_port(struct uart_port *port, int flags)
{

	dbg("asm9260_config_port");

	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_ATMEL;
		asm9260_request_port(port);
	}
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 */
static int asm9260_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;

	dbg("asm9260_verify_port");

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_ATMEL)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != SERIAL_IO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if ((void *)port->mapbase != ser->iomem_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

/* Enable or disable the rs485 support */
void asm9260_config_rs485(struct uart_port *port, struct serial_rs485 *rs485conf)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);
	unsigned int rs485_ctrl;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Disable interrupts */
	asm9260_intr_mask_clr(port, ASM9260_UART_TXIEN);

	rs485_ctrl = UART_GET_RS485CTRL(port);

	/* Resetting serial mode to RS232 (0x0) */
	rs485_ctrl &= ~ASM9260_UART_RS485EN;

	asm9260_port->rs485 = *rs485conf;

	if (rs485conf->flags & SER_RS485_ENABLED) {
		dev_dbg(port->dev, "Setting UART to RS485\n");
		if ((rs485conf->delay_rts_after_send) > 0) {
			/* delay is (rs485conf->delay_rts_after_send * Bit Period * 1/16) */
			UART_PUT_RS485DLY(port, rs485conf->delay_rts_after_send);
		}

		if ((rs485conf->flags & SER_RS485_RTS_ON_SEND) &&
				!(rs485conf->flags & SER_RS485_RTS_AFTER_SEND)) {
			/*
			 * Set logical level for RTS pin equal to 1 when sending,
			 * and set logical level for RTS pin equal to 0 after sending
			*/
			rs485_ctrl |= ASM9260_UART_RS485_ONIV;
		} else if (!(rs485conf->flags & SER_RS485_RTS_ON_SEND) &&
			(rs485conf->flags & SER_RS485_RTS_AFTER_SEND)) {
			/*
			 * Set logical level for RTS pin equal to 0 when sending,
			 * and set logical level for RTS pin equal to 1 after sending
			 */
			rs485_ctrl &= ~ASM9260_UART_RS485_ONIV;
		} else{
			printk(KERN_INFO "Please view RS485CTRL register in datasheet for more details.\n");
		}
		/* Enable RS485 and RTS is used to control direction automatically,  */
		rs485_ctrl |= ASM9260_UART_RS485EN | ASM9260_UART_RS485_DIR_CTRL;
		rs485_ctrl &= ~ASM9260_UART_RS485_PINSEL;

		if (rs485conf->flags & SER_RS485_RX_DURING_TX)
			printk(KERN_INFO "Hardware should support SER_RS485_RX_DURING_TX.\n");
	} else {
		dev_dbg(port->dev, "Setting UART to RS232\n");
	}

	UART_PUT_RS485CTRL(port, rs485_ctrl);

	/* Enable tx interrupts */
	asm9260_intr_mask_set(port, ASM9260_UART_TXIEN);

	spin_unlock_irqrestore(&port->lock, flags);

}

static int asm9260_ioctl(struct uart_port *port,
		unsigned int cmd, unsigned long arg)
{
	struct serial_rs485 rs485conf;

	switch (cmd) {
	case TIOCSRS485:
		if (copy_from_user(&rs485conf, (struct serial_rs485 *) arg,
					sizeof(rs485conf)))
			return -EFAULT;

		asm9260_config_rs485(port, &rs485conf);
		break;

	case TIOCGRS485:
		if (copy_to_user((struct serial_rs485 *) arg,
					&(to_asm9260_uart_port(port)->rs485),
					sizeof(rs485conf)))
			return -EFAULT;
		break;

	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static struct uart_ops asm9260_pops = {
	.tx_empty	= asm9260_tx_empty,
	.set_mctrl	= asm9260_set_mctrl,
	.get_mctrl	= asm9260_get_mctrl,
	.stop_tx	= asm9260_stop_tx,
	.start_tx	= asm9260_start_tx,
	.stop_rx	= asm9260_stop_rx,
	.enable_ms	= asm9260_enable_ms,
	.break_ctl	= asm9260_break_ctl,
	.startup	= asm9260_startup,
	.shutdown	= asm9260_shutdown,
	.flush_buffer	= asm9260_flush_buffer,
	.set_termios	= asm9260_set_termios,
	.type		= asm9260_type,
	.release_port	= asm9260_release_port,
	.request_port	= asm9260_request_port,
	.config_port	= asm9260_config_port,
	.verify_port	= asm9260_verify_port,
	.pm		= asm9260_serial_pm,
	.ioctl	= asm9260_ioctl,
};

#ifdef CONFIG_SERIAL_ASM9260_CONSOLE

static struct asm9260_uart_port *get_asm9260_uart_port(int line);
static struct console asm9260_console;

static void asm9260_console_putchar(struct uart_port *uport, int ch)
{
	while (UART_GET_STAT(uport) & ASM9260_UART_TXFULL)
		cpu_relax();
	UART_PUT_DATA(uport, ch);
}

/*
 * Interrupts are disabled on entering
 */
static void asm9260_console_write(struct console *co, const char *s, u_int count)
{
	struct uart_port *uport;
	struct asm9260_uart_port *port;
	unsigned int status, intr;
	unsigned long flags;
	int locked = 1;

	port = get_asm9260_uart_port(co->index);
	uport = &port->uart;

	if (oops_in_progress)
		locked = spin_trylock_irqsave(&uport->lock, flags);
	else
		spin_lock_irqsave(&uport->lock, flags);

	/*
	 * First, save IMR and then disable interrupts
	 */
	intr = UART_GET_INTR(uport) & (ASM9260_UART_RXIEN | ASM9260_UART_TXIEN);
	asm9260_intr_mask_clr(uport, intr);

	uart_console_write(uport, s, count, asm9260_console_putchar);

	/*
	 * Finally, wait for transmitter to become empty
	 * and restore IMR
	 */
	do {
		status = UART_GET_STAT(uport);
	} while (!(status & ASM9260_UART_TXEMPTY));

	asm9260_intr_mask_set(uport, intr);

	if (locked)
		spin_unlock_irqrestore(&uport->lock, flags);
}

/*
 * If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init asm9260_console_get_options(struct uart_port *port, int *baud,
					     int *parity, int *bits)
{
	unsigned int mr, quot, linectrl, bauddivint, bauddivfrc;

	/*
	 * If the baud rate generator isn't running, the port wasn't
	 * initialized by the boot loader.
	 */
	linectrl = UART_GET_LINECTRL(port);
	bauddivint = (linectrl & ASM9260_UART_BAUD_DIVINT) >> 16;
	bauddivfrc = (linectrl & ASM9260_UART_BAUD_DIVFRA) >> 8;
	quot = (bauddivint << 6) | bauddivfrc;

	if (!quot)
		return;

	mr = UART_GET_LINECTRL(port) & ASM9260_UART_WLEN;
	if (mr == ASM9260_US_CHRL_8)
		*bits = 8;
	else
		*bits = 7;

	mr = UART_GET_LINECTRL(port) &
		(ASM9260_UART_PEN | ASM9260_UART_EPS | ASM9260_UART_SPS);
	if (mr == ASM9260_US_PAR_EVEN)
		*parity = 'e';
	else if (mr == ASM9260_US_PAR_ODD)
		*parity = 'o';

	/*
	 * The serial core only rounds down when matching this to a
	 * supported baud rate. Make sure we don't end up slightly
	 * lower than one of those, as it would make us fall through
	 * to a much lower baud rate than we really want.
	 */
	*baud = (port->uartclk * 4) / quot;
}

static void asm9260_uart_of_enumerate(void);
static int __init asm9260_console_setup(struct console *co, char *options)
{
	struct uart_port *uport;
	struct asm9260_uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	asm9260_uart_of_enumerate();

	port = get_asm9260_uart_port(co->index);
	uport = &port->uart;

	/* TODO: need correct init */
	uport->membase = 0xf0010000;
	uport->uartclk = 100000000;

	UART_PUT_CTRL2_SET(uport, ASM9260_UART_TXE
			| ASM9260_UART_RXE | ASM9260_UART_ENABLE);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		asm9260_console_get_options(uport, &baud, &parity, &bits);

	return uart_set_options(uport, co, baud, parity, bits, flow);
}

static struct uart_driver asm9260_uart;

static struct console asm9260_console = {
	.name		= ASM9260_DEVICENAME,
	.write		= asm9260_console_write,
	.device		= uart_console_device,
	.setup		= asm9260_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &asm9260_uart,
};

#define ASM9260_CONSOLE_DEVICE	(&asm9260_console)

#if 1
/*
 * Early console initialization (before VM subsystem initialized).
 */
static int __init asm9260_console_init(void)
{
	register_console(&asm9260_console);
	return 0;
}

console_initcall(asm9260_console_init);
#endif

static inline bool asm9260_is_console_port(struct uart_port *port)
{
	return port->cons && port->cons->index == port->line;
}

#else
#define ASM9260_CONSOLE_DEVICE	NULL

static inline bool asm9260_is_console_port(struct uart_port *port)
{
	return false;
}
#endif

static struct uart_driver asm9260_uart = {
	.owner			= THIS_MODULE,
	.driver_name		= "asm9260_serial",
	.dev_name		= ASM9260_DEVICENAME,
	.nr			= ASM9260_MAX_UART,
	.cons			= ASM9260_CONSOLE_DEVICE,
};

#if defined(CONFIG_OF)
/* Match table for of_platform binding */
static struct of_device_id asm9260_of_match[] = {
	{ .compatible = "alpscale,asm9260-uart", },
	{}
};
MODULE_DEVICE_TABLE(of, asm9260_of_match);
#endif /* CONFIG_OF */


static void asm9260_enable_clks(struct asm9260_uart_port *port)
{
	struct uart_port *uport = &port->uart;
	int err;

	err = clk_set_rate(port->clk, ASM9260_BUS_RATE);
	if (err) {
		dev_err(uport->dev, "Failed to set rate!\n");
	}

	err = clk_prepare_enable(port->clk);
	if (err) {
		dev_err(uport->dev, "Failed to enable clk!\n");
	}

	err = clk_prepare_enable(port->clk_ahb);
	if (err) {
		dev_err(uport->dev, "Failed to enable ahb_clk!\n");
	}
}


/* get devicetree clocks, if some thing wrong, warn about it */
static int asm9260_get_of_clks(struct asm9260_uart_port *port,
		struct device_node *np)
{
	int clk_idx = 0;

	port->clk = of_clk_get(np, clk_idx);
	if (IS_ERR(port->clk))
		goto out_err;

	/* configure AHB clock */
	clk_idx = 1;
	port->clk_ahb = of_clk_get(np, clk_idx);
	if (IS_ERR(port->clk_ahb))
		goto out_err;

	return 0;
out_err:
	pr_err("%s: Failed to get clk (%i)\n", __func__, clk_idx);
	return 1;
}

static int asm9260_get_count_of_nodes(const struct of_device_id *matches)
{
	int count = 0;
	struct device_node *np;

	for_each_matching_node(np, matches)
		count++;

	return count;
}

static struct asm9260_uart_port *get_asm9260_uart_port(int line)
{
	if (line >= asm9260_ports_num) {
		pr_err("%s: Line number overflow. Check DeviceTree!!",
				__func__);
		return NULL;
	}

	return &asm9260_ports[line];
}

static void asm9260_uart_of_enumerate(void)
{
	static int enum_done;
	struct device_node *np;

	if (enum_done)
		return;

	asm9260_ports_num = asm9260_get_count_of_nodes(asm9260_of_match);
	asm9260_ports = kcalloc(asm9260_ports_num,
				sizeof(struct asm9260_uart_port), GFP_KERNEL);

	for_each_matching_node(np, asm9260_of_match) {
		struct uart_port *uport;
		struct asm9260_uart_port *port;
		int line;

		line = of_alias_get_id(np, "serial");

		port = get_asm9260_uart_port(line);
		if (!port)
			continue;

		uport = &port->uart;
		uport->iotype	= UPIO_MEM;
		uport->flags	= UPF_BOOT_AUTOCONF;
		uport->ops	= &asm9260_pops;
		uport->fifosize	= ASM9260_UART_FIFOSIZE;
		uport->line	= line;
		port->init_ok = 1;
	}

	enum_done = 1;
}

static void asm9260_release_clk(struct uart_port *port)
{
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);
	if (asm9260_port->clk) {
		clk_disable_unprepare(asm9260_port->clk);
		asm9260_port->clk = NULL;
	}
	if (asm9260_port->clk_ahb) {
		clk_disable_unprepare(asm9260_port->clk_ahb);
		asm9260_port->clk_ahb = NULL;
	}
}

/*
 * Configure the port from the platform device resource info.
 */
static void asm9260_init_port(struct asm9260_uart_port *asm9260_port,
				      struct platform_device *pdev)
{
	struct uart_port *uport = &asm9260_port->uart;
	struct asm9260_uart_data *data = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	int locked = 0;
	unsigned int flags;

	printk("asm9260_init_port\n");
	if (!(uart_console(uport) && (uport->cons->flags & CON_ENABLED))) {
		spin_lock_irqsave(&uport->lock, flags);
		locked = 1;
	}

	if (data)
		asm9260_port->rs485	= data->rs485;

	printk("%s:%i\n", __func__, __LINE__);
	if (np) {
		uport->membase = of_iomap(np, 0);
		printk("iomap: %p\n", uport->membase);
		if (!uport->membase) {
			dev_err(uport->dev, "Unable to map registers\n");
			return;
		}
	} else if (data && data->regs)
		/* Already mapped by setup code */
		uport->membase = data->regs;
	else {
		uport->flags	|= UPF_IOREMAP;
		uport->membase	= NULL;
	}

	if(asm9260_get_of_clks(asm9260_port, np))
		return;

	asm9260_enable_clks(asm9260_port);

	uport->uartclk = clk_get_rate(asm9260_port->clk);
	printk("clk = %li\n", clk_get_rate(asm9260_port->clk));

	if (np) {
		/* todo: need working DT irq infrastructure */
		//uport->irq	= of_irq_get(np, 0);
		of_property_read_u32_index(np, "interrupts", 0, &uport->irq);
		uport->mapbase	= uport->membase;
	} else {
		uport->line	= pdev->id;
		uport->mapbase	= pdev->resource[0].start;
		uport->irq	= pdev->resource[1].start;
	}

	if (locked)
		spin_unlock_irqrestore(&uport->lock, flags);

		printk("uport->line == %x; irq == %x\n", uport->line, uport->irq);
	tasklet_init(&asm9260_port->tasklet, asm9260_tasklet_func,
			(unsigned long)uport);/*setp 2*/

	memset(&asm9260_port->rx_ring, 0, sizeof(asm9260_port->rx_ring));
}



static int asm9260_serial_probe(struct platform_device *pdev)
{
	struct asm9260_uart_port *port;
	struct device_node *np = pdev->dev.of_node;
	void *data;
	int ret, line;

	printk("asm9260_serial_probe: %x\n", pdev->id);

	BUILD_BUG_ON(!is_power_of_2(ASM9260_SERIAL_RINGSIZE));

	asm9260_uart_of_enumerate();

	if (np)
		line = of_alias_get_id(np, "serial");
	else
		line = pdev->id;

	port = get_asm9260_uart_port(line);

	if (!port->init_ok) {
		dev_err(&pdev->dev, "Bad init!\n");
	}

	asm9260_init_port(port, pdev);

	data = devm_kmalloc(&pdev->dev, sizeof(struct asm9260_uart_char)
			* ASM9260_SERIAL_RINGSIZE, GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Filed to allocate ring\n");
		ret = -ENOMEM;
		goto err_alloc_ring;
	}
	port->rx_ring.buf = data;

	ret = uart_add_one_port(&asm9260_uart, &port->uart);
	if (ret) {
		dev_err(&pdev->dev, "Filed to add uart port\n");
		goto err_add_port;
	}

	platform_set_drvdata(pdev, port);

	return 0;

err_add_port:
//	port->rx_ring.buf = NULL;
err_alloc_ring:
	if (!asm9260_is_console_port(&port->uart)) {
		clk_put(port->clk);
		port->clk = NULL;
	}
	dev_err(&pdev->dev, "Filed to probe device\n");
	return ret;
}

static int asm9260_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct asm9260_uart_port *asm9260_port = to_asm9260_uart_port(port);
	int ret = 0;

	tasklet_kill(&asm9260_port->tasklet);

	uart_remove_one_port(&asm9260_uart, port);
	uart_unregister_driver(&asm9260_uart);

	clk_put(asm9260_port->clk);

	return ret;
}

static struct platform_driver asm9260_serial_driver = {
	.driver		= {
		.name	= "asm9260_uart",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(asm9260_of_match),
	},
	.probe		= asm9260_serial_probe,
	.remove		= asm9260_serial_remove,
};

static int __init asm9260_serial_init(void)
{
	int ret;
	dbg("asm9260_serial_init");
	ret = uart_register_driver(&asm9260_uart);
	if (ret)
		return ret;

	ret = platform_driver_register(&asm9260_serial_driver);
	if (ret)
		uart_unregister_driver(&asm9260_uart);

	return ret;
}

static void __exit asm9260_serial_exit(void)
{
	dbg("asm9260_serial_exit");
	platform_driver_unregister(&asm9260_serial_driver);
	uart_unregister_driver(&asm9260_uart);
}

module_init(asm9260_serial_init);
module_exit(asm9260_serial_exit);

MODULE_AUTHOR("Chen Dongdong");
MODULE_DESCRIPTION("ASM9260 serial port driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:asm9260_uart");
