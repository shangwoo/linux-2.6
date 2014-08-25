
/*
 * linux/arch/arm/mach-as9260/time.c
 *
 */

#include <linux/clk-provider.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

//#include <asm/system.h>
//#include <asm/hardware.h>
#include <asm/io.h>
//#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/timex.h>

#include <mach/hardware.h>

static __init void as9260_timer0_setup(void)
{
	unsigned int pllclk ,hclkdiv, cpuclkdiv, hclk;

	hclkdiv = as3310_readl(HW_SYSAHBCLKDIV);
	cpuclkdiv = as3310_readl(HW_CPUCLKDIV);
	pllclk = as3310_readl(HW_SYSPLLCTRL);
	hclk = ((pllclk / cpuclkdiv) / hclkdiv);

	as3310_writel(1<<4, HW_AHBCLKCTRL1 + SET_OFFSET);
	as3310_writel(1<<4, HW_PRESETCTRL1 + CLR_OFFSET);
	as3310_writel(1<<4, HW_PRESETCTRL1 + SET_OFFSET);
	as3310_writel(1<<4, HW_AHBCLKCTRL1 + SET_OFFSET);

	as3310_writel(0x3,HW_TIMER0_DIR + CLR_OFFSET);      // timer0 count-Up

	as3310_writel(hclk, HW_TIMER0_MR0);                   // 
	as3310_writel(9999, HW_TIMER0_PR);                  // MR0 * (PR + 1) = hclk * 10000----100HZ-->hclk M
	as3310_writel(0x3, HW_TIMER0_CTCR + CLR_OFFSET);    // timer mode
	as3310_writel(0x3, HW_TIMER0_MCR + SET_OFFSET);     // if (tc == mr0) then reset tc and interrupt

	as3310_writel(1<<0, HW_TIMER0_TCR + SET_OFFSET);    // enable timer0
}

#if 0
void as3310_timer2_init(ushort counter){

    printk("Timer 2 Inited (%d)\n",counter);
    as3310_writel(0x0000407f, HW_TIMROT_TIMCTRL2); // 24MHz devided by 8
    as3310_writel(0xffff0000+counter, HW_TIMROT_TIMCOUNTER2); 

}

void as3310_timer2_release(void){
    printk("Timer 2 Released\n");
    as3310_writel(0x0000003f, HW_TIMROT_TIMCTRL2); // stop Timer
}
#endif
// static unsigned int dp_i;
/*
 * IRQ handler for the timer 0
 */
static irqreturn_t
as9260_timer0_interrupt(int irq, void *dev_id)
{   
	as3310_writel(0x00000001, HW_TIMER0_IR);
	timer_tick();

	return IRQ_HANDLED;
}

static struct irqaction as9260_timer0_irq = {
	.name		= "AS9260 Timer 0 - Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= as9260_timer0_interrupt,
};

void __init as9260_timer_init(void)
{
	printk("%s\n", __func__);
	//setup_irq(INT_TIMER0, &as9260_timer0_irq);
	request_irq(INT_TIMER0, as9260_timer0_interrupt, IRQF_DISABLED |
			IRQF_TIMER | IRQF_IRQPOLL, "AS9260 Timer 0 - Tick", NULL);
	as9260_timer0_setup();
	of_clk_init(NULL);
}

#if 0
struct sys_timer as9260_timer = {
	.init		= as9260_timer_init,
	.offset		= NULL,		/* Initialized later */
};
#endif
