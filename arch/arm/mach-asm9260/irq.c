/*****************************************
 * linux/arch/arm/mach-as9260/irq.c
 * copyright (c) 2005-2014 Alpscale
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
//#include <linux/sysdev.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>
#include <mach/mac.h>




char * get_irq_reg(int irq) {
	if (irq < 0 || irq >= 64)
		return NULL;

	return (char *)(IO_ADDRESS(HW_ICOLL_PRIORITY0) + ((irq>>2)*0x10) + (irq&3));
}


char is_enabled_irq(int irq){
	volatile char * priority_reg = get_irq_reg(irq);
	return (*(priority_reg) & 0x4);  // is irq enabled
}

void hw_enable_irq(int irq){
	volatile char * priority_reg = get_irq_reg(irq);
	*(priority_reg+4) = 0x4;  // enable irq
}

void hw_disable_irq(int irq){
	volatile char * priority_reg = get_irq_reg(irq);
	*(priority_reg+8) = 0x4;  // disable irq
}

void clear_soft_irq(int irq){
	volatile char * priority_reg = get_irq_reg(irq);
	*(priority_reg+8) = 0x8;  // clear softirq
}

void set_soft_irq(int irq){
	volatile char * priority_reg = get_irq_reg(irq);
	*(priority_reg+4) = 0x8;  // set softirq
}

void irq_set_level(int irq,int level){
	volatile char * priority_reg = get_irq_reg(irq);
	*(priority_reg+8) = 0x3;  // clear level
	*(priority_reg+4) = (level&0x3);  // set new level
}

int irq_get_level(int irq){
	volatile char * priority_reg = get_irq_reg(irq);
	return (*priority_reg & 0x3);  // get level
}

static void as9260_mask_irq(struct irq_data *data)
{
	unsigned int	irq = data->irq;
 	unsigned long mask;
	unsigned int  reg_addr = HW_ICOLL_PRIORITY0 + ((irq>>2)<<4);	
	mask = as3310_readl(reg_addr);
	mask &= ~(1<<(2 + ((irq&0x3)<<3)));
	as3310_writel(mask, reg_addr);
}

static void as9260_unmask_irq(struct irq_data *data)
{
	unsigned int	irq = data->irq;
	unsigned long	mask;
	unsigned int	reg_addr = HW_ICOLL_PRIORITY0 + ((irq>>2)<<4);	
	unsigned long	level;

	/* ========= clear irq controller ================*/
	as3310_writel((0x00000001 << (irq&0x1f)), HW_ICOLL_CLEAR0 + ((irq>>5)*0x10) + 4 );

	/* Get current Level before IRQ enabled */
	level = irq_get_level(irq);

	/* ========= clear irq level  ================*/
	as3310_writel((1<<level), HW_ICOLL_LEVELACK);

	mask = as3310_readl(reg_addr);
	mask |= (1<<(2 + ((irq&0x3)<<3)));
	as3310_writel(mask, reg_addr);
}


static void as9260_ack_irq(struct irq_data *data)
{
	as3310_readl(HW_ICOLL_VECTOR);
	return;
}

int as9260_irq_wake (unsigned int irqno, unsigned int state)
{
	return 0;
}

static struct irq_chip as9260_irq_chip = {
	.name       = "as9260_irq",
	.irq_ack		= as9260_ack_irq,
	.irq_mask		= as9260_mask_irq,
	.irq_unmask		= as9260_unmask_irq,
	.irq_mask_ack   = NULL,
};



void __init as9260_init_irq(void)
{
	int irq_no;
	int delay;

	as3310_writel(0x1<<30, HW_ICOLL_CTRL+8);//clear bit 30 open clk

	/*reset controller*/
	as3310_writel(0x1<<31, HW_ICOLL_CTRL+4);//set bit 31
	delay = 0x1000;
        while(delay--);//wait

	as3310_writel(0x1<<31, HW_ICOLL_CTRL+8); //clear bit 31
	delay = 0x1000;
        while(delay--);//wait

	as3310_writel(0x5<<16, HW_ICOLL_CTRL); //IRQ enable,RISC32_RSE_MODE


	as3310_writel(0x0, HW_ICOLL_VBASE);   // set irq_table addr
	as3310_writel(0xffff0000, HW_ICOLL_UNDEF_VECTOR);   // set irq_table addr
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY0);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY1);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY2);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY3);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY4);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY5);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY6);
	as3310_writel(0x00000300, HW_ICOLL_PRIORITY7);//set timer 0 priority level high
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY8);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY9);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY10);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY11);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY12);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY13);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY14);
	as3310_writel(0x00000000, HW_ICOLL_PRIORITY15);

	for (irq_no=0; irq_no<NR_IRQS; irq_no++){
		irq_set_chip_and_handler(irq_no, &as9260_irq_chip, handle_level_irq);
		set_irq_flags(irq_no, IRQF_VALID);
	}

	return;
}
