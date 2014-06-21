#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>


#include "musb_core.h"


#define MUSB_TIMEOUT_A_WAIT_BCON	1100


extern int musb_core_init(u16 musb_type, struct musb *musb);

static irqreturn_t arch_interrupt(int irq, void *__hci)
{
	unsigned long	flags;
	irqreturn_t	retval = IRQ_NONE;
	struct musb	*musb = __hci;
	struct musb_hw_ep	*hw_ep = musb->control_ep;
	void __iomem		*epio = hw_ep->regs;
	
	u32		tmp,i;
	u8 tmp_intusbe,tmp_power,tmp_devctrl;
	u16 tmp_csr0;

	spin_lock_irqsave(&musb->lock, flags);

	
	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);

	
		void __iomem *mregs = musb->mregs;
		u8	devctl = musb_readb(mregs, MUSB_DEVCTL);
		int	err = musb->int_usb & MUSB_INTR_VBUSERROR;

		err = is_host_enabled(musb)
				&& (musb->int_usb & MUSB_INTR_VBUSERROR);

		if (err) {
			/* The Mentor core doesn't debounce VBUS as needed
			 * to cope with device connect current spikes. This
			 * means it's not uncommon for bus-powered devices
			 * to get VBUS errors during enumeration.
			 *
			 * This is a workaround.
			 */
			musb->int_usb &= ~MUSB_INTR_VBUSERROR;
			musb->xceiv.state = OTG_STATE_A_WAIT_VFALL;
			
			tmp_intusbe = musb_readb(musb->mregs, MUSB_INTRUSBE);
			tmp_csr0 = musb_readw(epio, MUSB_CSR0);
			tmp_power = musb_readb(musb->mregs, MUSB_POWER);
			tmp_devctrl = musb_readb(musb->mregs, MUSB_DEVCTL);


			as3310_writel(1<<8,HW_AHBCLKCTRL0+4);//open usb1 clk
	
			as3310_writel(1<<8,HW_PRESETCTRL0+8);//reset usb1
			for(i=0;i<0x10000;i++);
			as3310_writel(1<<8,HW_PRESETCTRL0+4);
			for(i=0;i<0x1000000;i++);

			musb_writeb(musb->mregs, MUSB_INTRUSBE	,tmp_intusbe);
			musb_writew(epio, MUSB_CSR0	,tmp_csr0);
			musb_writeb(musb->mregs, MUSB_POWER	,tmp_power);
			musb_writeb(musb->mregs, MUSB_DEVCTL	,1);


			musb_core_init(0, musb);//0:MUSB_CONTROLLER_MHDRC
								//1: MUSB_CONTROLLER_HDRC

			WARNING("VBUS error workaround (delay&reset coming)\n");
		} else if (is_host_enabled(musb)) {
			musb->is_active = 1;
			MUSB_HST_MODE(musb);
			musb->xceiv.default_a = 1;
			musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		} else {
			musb->is_active = 0;
			MUSB_DEV_MODE(musb);
			musb->xceiv.default_a = 0;
			musb->xceiv.state = OTG_STATE_B_IDLE;
		}

	
		retval = IRQ_HANDLED;


	if (musb->int_tx || musb->int_rx || musb->int_usb)
	{
		retval |= musb_interrupt(musb);
	}


	spin_unlock_irqrestore(&musb->lock, flags);

	/* REVISIT we sometimes get unhandled IRQs
	 * (e.g. ep0).  not clear why...
	 */
	if (retval != IRQ_HANDLED)
		DBG(5, "unhandled? %08x\n", tmp);
	return IRQ_HANDLED;
}

void musb_platform_enable(struct musb *musb)
{
}
void musb_platform_disable(struct musb *musb)
{
}
static void arch_vbus_power(struct musb *musb, int is_on, int sleeping)
{
}

static void arch_set_vbus(struct musb *musb, int is_on)
{
	u8		devctl;

    /* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */


	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv.default_a = 1;
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv.default_a = 0;
		musb->xceiv.state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}
static int arch_set_power(struct otg_transceiver *x, unsigned mA)
{
    //we have no power set mode.so we supply the max 500 mA output.
	return 0;
}

static int musb_platform_resume(struct musb *musb);

void musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	switch (musb_mode) {
	case MUSB_HOST:
		otg_set_host(&musb->xceiv, musb->xceiv.host);
		break;
	case MUSB_PERIPHERAL:
		otg_set_peripheral(&musb->xceiv, musb->xceiv.gadget);
		break;
	case MUSB_OTG:
		break;
	}
}


int __init musb_platform_init(struct musb *musb)
{
	//musb_platform_resume(musb);

	arch_vbus_power(musb, musb->board_mode == MUSB_HOST, 1);

	if (is_host_enabled(musb))
	{	
		musb->board_set_vbus = arch_set_vbus;
	}
	if (is_peripheral_enabled(musb))
	{	
		musb->xceiv.set_power = arch_set_power;
	}

	musb->a_wait_bcon = MUSB_TIMEOUT_A_WAIT_BCON;

	musb->isr = arch_interrupt;
	return 0;
}

int musb_platform_suspend(struct musb *musb)
{
	if (!musb->clock)
		return 0;
	if (musb->xceiv.set_suspend)
		musb->xceiv.set_suspend(&musb->xceiv, 1);

	if (musb->set_clock)
		musb->set_clock(musb->clock, 0);
	else
		clk_disable(musb->clock);

	return 0;
}

static int musb_platform_resume(struct musb *musb)
{
	if (!musb->clock)
		return 0;

	if (musb->xceiv.set_suspend)
		musb->xceiv.set_suspend(&musb->xceiv, 0);

	if (musb->set_clock)
		musb->set_clock(musb->clock, 1);
	else
		clk_enable(musb->clock);

	return 0;
}


int musb_platform_exit(struct musb *musb)
{

	/* delay, to avoid problems with module reload */
	if (is_host_enabled(musb) && musb->xceiv.default_a) {
		int	maxdelay = 30;
		u8	devctl, warn = 0;

		/* if there's no peripheral connected, this can take a
		 * long time to fall, especially on EVM with huge C133.
		 */
		do {
			devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
			if (!(devctl & MUSB_DEVCTL_VBUS))
				break;
			if ((devctl & MUSB_DEVCTL_VBUS) != warn) {
				warn = devctl & MUSB_DEVCTL_VBUS;
				DBG(1, "VBUS %d\n",
					warn >> MUSB_DEVCTL_VBUS_SHIFT);
			}
			msleep(1000);
			maxdelay--;
		} while (maxdelay > 0);

		/* in OTG mode, another host might be connected */
		if (devctl & MUSB_DEVCTL_VBUS)
			DBG(1, "VBUS off timeout (devctl %02x)\n", devctl);
	}

	arch_vbus_power(musb, 0 /*off*/, 1);

	clk_put(musb->clock);
	musb->clock = 0;

	return 0;
}
