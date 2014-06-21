/*******************************************************************************
* Copyright (C) 2014 AlphaScale												   											   
* file:asm9260_timer.c                                                                                                                                                                                                                                        
* ASM9260 timer driver	
*******************************************************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/irq.h>
#include <mach/io.h>
#include <mach/pincontrol.h>

#include "asm9260_timer.h"

static struct device    *timer_dev;	/* platform device attached to */
static struct resource	*timer_mem;
static struct resource	*timer_irq;

static DECLARE_WAIT_QUEUE_HEAD(timer_waitq);
static volatile int ev_timer = 0;
static char flag=0;
static struct timer_config *reg;
static struct timer_reg dval;

/********************************************************************* 
 * get default value of timer register 
 */
static void read_dval(void)
{
	dval.timer_pr=as3310_readl(HW_TIMER1_PR);
	dval.timer_mcr=as3310_readl(HW_TIMER1_MCR);
	dval.timer_tcr=as3310_readl(HW_TIMER1_TCR);
        dval.timer_ir=as3310_readl(HW_TIMER1_IR);
	dval.timer_pwmc=as3310_readl(HW_TIMER1_PWMC);
	dval.timer_ctcr=as3310_readl(HW_TIMER1_CTCR);
        dval.timer_emr=as3310_readl(HW_TIMER1_EMR);
	dval.timer_ccr=as3310_readl(HW_TIMER1_CCR);
}

/************************************************************************ 
 * write default value to timer register 
 */
static void write_dval(void)
{
	as3310_writel(dval.timer_pr,HW_TIMER1_PR);
	as3310_writel(dval.timer_mcr,HW_TIMER1_MCR);
	as3310_writel(dval.timer_tcr,HW_TIMER1_TCR);
        as3310_writel(dval.timer_ir,HW_TIMER1_IR);
	as3310_writel(dval.timer_pwmc,HW_TIMER1_PWMC);
	as3310_writel(dval.timer_ctcr,HW_TIMER1_CTCR);
        as3310_writel(dval.timer_emr,HW_TIMER1_EMR);
	as3310_writel(dval.timer_ccr,HW_TIMER1_CCR);
}

/*******************************************************************************
 * reset and start timer
 * channel:timer channel(TIMER_CHANNEL0 ~ TIMER_CHANNEL3)
 */
void timer_start(u8 channel)
{
	u32 rstval = 0;
	u32 chlen = as3310_readl(HW_TIMER1_TCR);
        /* reset and start timer1 channel0 */
	if(channel & TIMER1_CHANNEL0)
	{
		rstval |= TIMER1_CHANNEL0_RESET; 
		chlen |= TIMER1_CHANNEL0_START;
	}
        /* reset and start timer1 channel1 */
	if(channel & TIMER1_CHANNEL1)
	{
		rstval |= TIMER1_CHANNEL1_RESET; 
		chlen |= TIMER1_CHANNEL1_START;
	}
        /* reset and start timer1 channel2 */
	if(channel & TIMER1_CHANNEL2)
	{
		rstval |= TIMER1_CHANNEL2_RESET; 
		chlen |= TIMER1_CHANNEL2_START;
	}
        /* reset and start timer1 channel3 */
	if(channel & TIMER1_CHANNEL3)
	{
		rstval |= TIMER1_CHANNEL3_RESET; 
		chlen |= TIMER1_CHANNEL3_START;
	}

        /* write rstval and chlen to tcr register */
	as3310_writel(rstval,HW_TIMER1_TCR+4);		  
	as3310_writel(rstval,HW_TIMER1_TCR+8);
	as3310_writel(chlen,HW_TIMER1_TCR);

        /* waiting for init operation completed */
	mdelay(100);
}

/****************************************************************************** 
 * timer works in interrupt mode 
 * chan: timer channel 
 * mrval: delay value between each interrupt  
 */
void timer_match(int chan,unsigned long *mrval)
{	
   u32 mcrval = 0;
   write_dval();
   if (chan & TIMER1_CHANNEL0) {
         /* write channel0 match val to MR0 */
         as3310_writel(mrval[0],HW_TIMER1_MR0); 
         /* enable channel0 match interrupt */                     
         mcrval |= TIMER1_CHANNEL0_INT_SET;		
   }
   if (chan & TIMER1_CHANNEL1) 
   {
         /* write channel1 match val to MR1 */
         as3310_writel(mrval[1],HW_TIMER1_MR1);   
         /* enable channel1 match interrupt */                   
         mcrval |= TIMER1_CHANNEL1_INT_SET; 		
   }
   if (chan & TIMER1_CHANNEL2) 
   {
         /* write channel2 match val to MR2 */
         as3310_writel(mrval[2],HW_TIMER1_MR2);  
         /* enable channel2 match interrupt */  
         mcrval |= TIMER1_CHANNEL2_INT_SET;                  		
   }
   if (chan & TIMER1_CHANNEL3) 
   {
         /* write channel3 match val to MR1 */
         as3310_writel(mrval[3],HW_TIMER1_MR3);    
         /* enable channel3 match interrupt */                   
         mcrval |= TIMER1_CHANNEL3_INT_SET;		
   }
                    
   /* write mcrval to MCR register */                                              
   as3310_writel(mcrval,HW_TIMER1_MCR);
   /* start timer1 channel */
   timer_start(chan);
}

/***************************************************************************** 
 * timer works in pwm mode 
 * chan: pwm channel 
 * mr: register HW_TIMER1_MR value
 * th: register HW_TIMER1_PWMTH value
 */
void pwm_output(int chan,unsigned long *mr,unsigned long *th)
{
   u32 pwmcval = 0;
   write_dval();
   if (chan & TIMER1_CHANNEL0) {
        /* config channel0 pwm period and duty cycle */ 
        as3310_writel(mr[0],HW_TIMER1_MR0); 
        as3310_writel(th[0],HW_TIMER1_PWMTH0);
        /* enable channel0 pwm function */ 
        pwmcval |= TIMER1_CHANNEL0_PWM; 
        /* CT1_MAT0 */	  
        set_pin_mux(10,0,PIN_FUNCTION_1);
   }
   if (chan & TIMER1_CHANNEL1) 
   {
        /* config channel1 pwm period and duty cycle*/
        as3310_writel(mr[1],HW_TIMER1_MR1);
        as3310_writel(th[1],HW_TIMER1_PWMTH1); 
        /* enable channel1 pwm function */
        pwmcval |= TIMER1_CHANNEL1_PWM;	
        /* CT1_MAT1 */	  
        set_pin_mux(10,1,PIN_FUNCTION_1);
   }
   if (chan & TIMER1_CHANNEL2) 
   {
        /* config channel2 pwm period and duty cycle*/
        as3310_writel(mr[2],HW_TIMER1_MR2);	
        as3310_writel(th[2],HW_TIMER1_PWMTH2);
        /* enable channel2 pwm function */
        pwmcval |= TIMER1_CHANNEL2_PWM; 
        /* CT1_MAT2 */		  
        set_pin_mux(10,2,PIN_FUNCTION_1);
   }
   if (chan & TIMER1_CHANNEL3) 
   {
        /* config channel3 pwm period and duty cycle*/
        as3310_writel(mr[3],HW_TIMER1_MR3);	
        as3310_writel(th[3],HW_TIMER1_PWMTH3);
        /* enable channel3 pwm function */
        pwmcval |= TIMER1_CHANNEL3_PWM; 
        /* CT1_MAT3 */		  
        set_pin_mux(10,3,PIN_FUNCTION_1);
   }
	 
   /* write pwmcval to PWMC register */
   as3310_writel(pwmcval,HW_TIMER1_PWMC);
   /* start timer1 channel */	 
   timer_start(chan);
}

/******************************************************************************** 
 * timer works in pwm mode,only output single wave 
 * chan: timer channel 
 * mr: register HW_TIMER1_MR value
 * th: register HW_TIMER1_PWMTH value
 */
void pwm_single_output(int chan,unsigned long *mr,unsigned long *th)
{
   u32 pwmcval = 0;
   write_dval();
   if (chan & TIMER1_CHANNEL0) {
        /* config channel0 pwm period and duty cycle */ 
        as3310_writel(mr[0],HW_TIMER1_MR0); 
        as3310_writel(th[0],HW_TIMER1_PWMTH0); 
        /* enable channel0 single pwm function */
        pwmcval |= TIMER1_CHANNEL0_SINGLE_PWM; 	
        /* CT1_MAT0 */	  
        set_pin_mux(10,0,PIN_FUNCTION_1);
   }
   if (chan & TIMER1_CHANNEL1) 
   {
        /* config channel1 pwm period and duty cycle */ 
        as3310_writel(mr[1],HW_TIMER1_MR1);
        as3310_writel(th[1],HW_TIMER1_PWMTH1); 
        /* enable channel1 single pwm function */		  
        pwmcval |= TIMER1_CHANNEL1_SINGLE_PWM; 
        /* CT1_MAT1 */
        set_pin_mux(10,1,PIN_FUNCTION_1);
   }
   if (chan & TIMER1_CHANNEL2) 
   {
        /* config channel2 pwm period and duty cycle */ 
        as3310_writel(mr[2],HW_TIMER1_MR2);	
        as3310_writel(th[2],HW_TIMER1_PWMTH2);
        /* enable channel2 single pwm function */
        pwmcval |= TIMER1_CHANNEL2_SINGLE_PWM;
        /* CT1_MAT2 */  		  
        set_pin_mux(10,2,PIN_FUNCTION_1);
   }
   if (chan & TIMER1_CHANNEL3) 
   {
        /* config channel3 pwm period and duty cycle */ 
        as3310_writel(mr[3],HW_TIMER1_MR3);	
        as3310_writel(th[3],HW_TIMER1_PWMTH3); 
        /* enable channel3 single pwm function */
        pwmcval |= TIMER1_CHANNEL3_SINGLE_PWM;
        /* CT1_MAT3 */ 		  
        set_pin_mux(10,3,PIN_FUNCTION_1);
   }
	 
   /* write pwmcval to PWMC register */
   as3310_writel(pwmcval,HW_TIMER1_PWMC);
   /* start timer1 channel */
   timer_start(chan);
}

/********************************************************************************* 
 * timer works in capture mode,CT1_CAP should connect 
 * with an output signal 
 */
void capture_irq(int chan)
{
        write_dval();                                         

	/******capture mode********/
	as3310_writel(TIMER1_CAPTURE_UP,HW_TIMER1_CCR);	
        /* CT1_CAP */	
	set_pin_mux(10,4,PIN_FUNCTION_1);	    
	
        /* start timer1 channel */
        timer_start(chan);

}

/**************************************************************************** 
 * timer works in counter mode,CT1_CAP should connect 
 * with an output signal 
 * chan: timer channel 
 * count_num: register HW_TIMER1_MR value
 */
void counter_irq(int chan,unsigned long *count_num)
{ 
        u32 ctcrval=0,mcrval=0;
        write_dval();                                         

        /********CT1_CAP********/
        set_pin_mux(10,4,PIN_FUNCTION_1);	                  
	/*****counter mode********/
        if (chan & TIMER1_CHANNEL0) {
           /* timer1 channel0 count at rising pulse */
           ctcrval |= TIMER1_COUNTER_CHAN0_UP;	
           /* write count num to MR0 */  	
           as3310_writel(count_num[0],HW_TIMER1_MR0);
           /* enable channel0 interrupt */
           mcrval |= TIMER1_CHANNEL0_INT_SET; 	  	  	
        }
        if (chan & TIMER1_CHANNEL1) {
           /* timer1 channel1 count at rising pulse */
           ctcrval |= TIMER1_COUNTER_CHAN1_UP;	 
           /* write count num to MR1 */ 
           as3310_writel(count_num[1],HW_TIMER1_MR1); 
           /* enable channel1 interrupt */
           mcrval |= TIMER1_CHANNEL1_INT_SET; 	  	  	
        }
        if (chan & TIMER1_CHANNEL2) {
           /* timer1 channel2 count at rising pulse */
           ctcrval |= TIMER1_COUNTER_CHAN2_UP;     
           /* write count num to MR2 */  
           as3310_writel(count_num[2],HW_TIMER1_MR2);
           /* enable channel2 interrupt */
           mcrval |= TIMER1_CHANNEL2_INT_SET;  	  	  
        }
        if (chan & TIMER1_CHANNEL3) {
           /* timer1 channel3 count at rising pulse */
           ctcrval |= TIMER1_COUNTER_CHAN3_UP;	
           /* write count num to MR3 */
           as3310_writel(count_num[3],HW_TIMER1_MR3);
           /* enable channel3 interrupt */
           mcrval |= TIMER1_CHANNEL3_INT_SET;  	  	  
        }
        
        /* write ctcrval and mcrval to register */
        as3310_writel(ctcrval,HW_TIMER1_CTCR);
        as3310_writel(mcrval,HW_TIMER1_MCR);

        /* start timer1 channel */
        timer_start(chan);
	
}

/******************************************************************************* 
 * timer works in emr mode 
 * chan: timer channel
 * emr: output mode,0:none,1:output '1',2:output '0',3:toggle
 * mrval: register HW_TIMER1_MR value
 */
void emr_mode(int chan,unsigned long emr,unsigned long *mrval)
{
   u32 emrval=0;
   write_dval();
   /*
   emr mode: 
   TIMER1_EMR_CHAN0_NONE: none
   TIMER1_EMR_CHAN0_HIGH: output high 
   TIMER1_EMR_CHAN0_LOW: output low
   TIMER1_EMR_CHAN0_TOGGLE: output toggle waves
   */

   if (chan & TIMER1_CHANNEL0) {
         /* CT1_MAT0 */
         set_pin_mux(10,0,PIN_FUNCTION_1);	               
         switch (emr) {
         case TIMER1_EMR_NONE:
            emrval |= TIMER1_EMR_CHAN0_NONE;
               break;
         case TIMER1_EMR_HIGH:
            emrval |= TIMER1_EMR_CHAN0_HIGH;
               break;
         case TIMER1_EMR_LOW:
            emrval |= TIMER1_EMR_CHAN0_LOW;
               break;
         case TIMER1_EMR_TOGGLE:
            emrval |= TIMER1_EMR_CHAN0_TOGGLE;
               break;
         default:
            dev_err(timer_dev,"input emr error!\n");
               break;
         }
         /* write mrval to MR0 register */
         as3310_writel(mrval[0],HW_TIMER1_MR0);
   }
   if (chan & TIMER1_CHANNEL1) 
   {
         /* CT1_MAT1 */
	 set_pin_mux(10,1,PIN_FUNCTION_1);	              
           switch (emr) {
           case TIMER1_EMR_NONE:
              emrval |= TIMER1_EMR_CHAN1_NONE;
               break;
           case TIMER1_EMR_HIGH:
              emrval |= TIMER1_EMR_CHAN1_HIGH;
               break;
           case TIMER1_EMR_LOW:
              emrval |= TIMER1_EMR_CHAN1_LOW;
               break;
           case TIMER1_EMR_TOGGLE:
              emrval |= TIMER1_EMR_CHAN1_TOGGLE;
               break;
         default:
            dev_err(timer_dev,"input emr error!\n");
               break;
         }
           /* write mrval to MR1 register */
         as3310_writel(mrval[1],HW_TIMER1_MR1);
   }
   if (chan & TIMER1_CHANNEL2) 
   {
         /* CT1_MAT2 */
         set_pin_mux(10,2,PIN_FUNCTION_1);	               
            switch (emr) {
            case TIMER1_EMR_NONE:
               emrval |= TIMER1_EMR_CHAN2_NONE;
               break;
            case TIMER1_EMR_HIGH:
               emrval |= TIMER1_EMR_CHAN2_HIGH;
               break;
            case TIMER1_EMR_LOW:
               emrval |= TIMER1_EMR_CHAN2_LOW;
               break;
            case TIMER1_EMR_TOGGLE:
               emrval |= TIMER1_EMR_CHAN2_TOGGLE;
               break;
         default:
            dev_err(timer_dev,"input emr error!\n");
               break;
         }
            /* write mrval to MR2 register */
         as3310_writel(mrval[2],HW_TIMER1_MR2);
   }
   if (chan & TIMER1_CHANNEL3) 
   {
         /* CT1_MAT3 */
	 set_pin_mux(10,3,PIN_FUNCTION_1);	                  
            switch (emr) {
            case TIMER1_EMR_NONE:
               emrval |= TIMER1_EMR_CHAN3_NONE;
               break;
            case TIMER1_EMR_HIGH:
               emrval |= TIMER1_EMR_CHAN3_HIGH;
               break;
            case TIMER1_EMR_LOW:
               emrval |= TIMER1_EMR_CHAN3_LOW;
               break;
            case TIMER1_EMR_TOGGLE:
               emrval |= TIMER1_EMR_CHAN3_TOGGLE;
               break;
         default:
            dev_err(timer_dev,"input emr error!\n");
               break;
         }
            /* write mrval to MR3 register */
         as3310_writel(mrval[3],HW_TIMER1_MR3);
   }

   as3310_writel(emrval,HW_TIMER1_EMR);
   timer_start(chan);
}

static void asm9260_timer_close(void)
{
        /* close timer1 */
	as3310_writel(TIMER1_CLOCK_ENABLE,HW_AHBCLKCTRL1+8);	 
}

static void asm9260_timer_start(void)
{
        /* start timer1 */
	as3310_writel(TIMER1_CLOCK_ENABLE,HW_AHBCLKCTRL1+4);            
}

static int asm9260_timer_open(struct inode *inode, struct file *file)
{
        /* open timer1 device */
        asm9260_timer_start();
        return 0;
}

static int asm9260_timer_release(struct inode *inode, struct file *file)
{
        /* close timer1 device */
	asm9260_timer_close();
	return 0;
}
  
static int asm9260_timer_read(struct file *filp,char __user *buff,size_t count,loff_t *offp)
{
    /* wait until interrupt on */
    wait_event_interruptible(timer_waitq,ev_timer);
    ev_timer = TIMER1_EVENT_NONE;
    /* copy flag to user space */
    copy_to_user(buff,&flag,sizeof(flag));
    flag = TIMER1_INTERRUPT_NONE;
    return 0;
}


/****************************************************** 
 * timer works in various modes based on cmd value
 */
static long asm9260_timer_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        memset((struct timer_config *)reg,0,sizeof(struct timer_config));
        /* copy regval from user space */
	copy_from_user((struct timer_config *)reg,(struct timer_config *)arg,sizeof(struct timer_config));
	switch(cmd)
	{
                /* timer operations according to cmd */
		case TIMER1_MODE_PWM:
			pwm_output(reg->chan,reg->mrval,reg->thval);
			break;
                case TIMER1_MODE_SINGLE_PWM:
                        pwm_single_output(reg->chan,reg->mrval,reg->thval);
                        break;
		case TIMER1_MODE_EMR:
			emr_mode(reg->chan,reg->emr,reg->mrval);
			break;
                case TIMER1_MODE_MATCH_INTERRUPT:
			timer_match(reg->chan,reg->mrval);
			break;
		case TIMER1_MODE_COUNTER:
			counter_irq(reg->chan,reg->countval);
			break;
		case TIMER1_MODE_CAPTURE:
			capture_irq(reg->chan);
			break;
		default:
			break;
	}
	
	return 0;
}

/* kernel interface */
static const struct file_operations asm9260_timer_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= asm9260_timer_ioctl,
	.open		= asm9260_timer_open,
	.release	= asm9260_timer_release,
        .read           = asm9260_timer_read,
};

static struct miscdevice asm9260_timer_miscdev = {
	.minor		= TIMER1_MINOR,
	.name		= "timer1",
	.fops		= &asm9260_timer_fops,
};

/* interrupt handler code */
void asm9260_timer_interrupt(void)
{
	int ir_tmp;
	
        /* read val of interrupt flag register */
	ir_tmp=as3310_readl(HW_TIMER1_IR);
	
        /* channel0 interrupt */
	if((ir_tmp & TIMER1_CHAN0_INT_FLAG) != 0)
	{
                flag |= TIMER1_CHAN0_INT;
                /* set ev_timer and wake up waiting event */
                ev_timer = TIMER1_EVENT_ON; 
                wake_up_interruptible(&timer_waitq);
                /* clear channel0 interrupt flag */
		as3310_writel(TIMER1_CHAN0_INT_FLAG,HW_TIMER1_IR);
	}
        /* channel1 interrupt */
	if((ir_tmp & TIMER1_CHAN1_INT_FLAG) != 0)
	{
                flag |= TIMER1_CHAN1_INT;
                ev_timer = TIMER1_EVENT_ON; 
                wake_up_interruptible(&timer_waitq);
		as3310_writel(TIMER1_CHAN1_INT_FLAG,HW_TIMER1_IR);
	}

        /* channel2 interrupt */
	if((ir_tmp & TIMER1_CHAN2_INT_FLAG) != 0)
	{
                flag |= TIMER1_CHAN2_INT;
                ev_timer = TIMER1_EVENT_ON; 
                wake_up_interruptible(&timer_waitq);
		as3310_writel(TIMER1_CHAN2_INT_FLAG,HW_TIMER1_IR);
	}

        /* channel3 interrupt */
	if((ir_tmp & TIMER1_CHAN3_INT_FLAG) != 0)
	{
                flag |= TIMER1_CHAN3_INT;
                ev_timer = TIMER1_EVENT_ON; 
                wake_up_interruptible(&timer_waitq);
		as3310_writel(TIMER1_CHAN3_INT_FLAG,HW_TIMER1_IR);
	}

        /* capture interrupt */
	if((ir_tmp & TIMER1_CAPTURE_INT_FLAG) !=0)
	{
                flag |= TIMER1_CAPTURE_INT;
                ev_timer = TIMER1_EVENT_ON; 
                wake_up_interruptible(&timer_waitq);
		as3310_writel(TIMER1_CAPTURE_INT_FLAG,HW_TIMER1_IR);
	}

}

static irqreturn_t asm9260timer_irq(int irqno, void *param)
{
        /* hanndle interrupt event */
	asm9260_timer_interrupt();

	return IRQ_HANDLED;
}

/* device interface */
static int asm9260_timer_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev;
	int ret;
	int size;
    
	dev = &pdev->dev;
	timer_dev = &pdev->dev;

        /* assign memory for struct timer_config */
        reg=(struct timer_config *)kmalloc(sizeof(struct timer_config),GFP_KERNEL);
        if (reg == NULL) 
        {
           dev_err(dev,"assign mem failed!\n");
        }

        /* get platform resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no memory resource specified\n");
		return -ENOENT;
	}
	
        /* request memory */
	size = (res->end - res->start);
	timer_mem = request_mem_region(res->start, size, pdev->name);
	if (timer_mem == NULL) {
		dev_err(dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

        /* get irq resources */
	timer_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (timer_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
	}

        /* request irq */
	ret = request_irq(timer_irq->start, &asm9260timer_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_irq;
	}

        /* register timer1 device */
	ret = misc_register(&asm9260_timer_miscdev);
	if (ret) {
		dev_err(dev, "cannot register miscdev on minor=%d (%d)\n", TIMER1_MINOR, ret);
	}

        /* timer init operation */
        asm9260_timer_start();
        read_dval();


	return 0;

 err_irq:
	free_irq(timer_irq->start, pdev);

 err_req:
	release_resource(timer_mem);

	return ret;
}

static int asm9260_timer_remove(struct platform_device *dev)
{
        /* release resources and free irq */
	release_resource(timer_mem);
	timer_mem = NULL;

	free_irq(timer_irq->start, dev);
	timer_irq = NULL;

        kfree(reg);
	misc_deregister(&asm9260_timer_miscdev);
        asm9260_timer_close();
	return 0;
}

static void asm9260_timer_shutdown(struct platform_device *dev)
{
        /* shutdown mcpwm device */
	asm9260_timer_close();
}

static struct platform_driver asm9260_timer_driver = {
	.probe		= asm9260_timer_probe,
	.remove		= asm9260_timer_remove,
	.shutdown	= asm9260_timer_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "asm9260-timer",
	},
};

static int __init timer_init(void)
{
	return platform_driver_register(&asm9260_timer_driver);
}

static void __exit timer_exit(void)
{
	platform_driver_unregister(&asm9260_timer_driver);
}

module_init(timer_init);
module_exit(timer_exit);

MODULE_AUTHOR("alphascale");
MODULE_DESCRIPTION("Asm9260 Timer Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(TIMER_MINOR);
MODULE_ALIAS("platform:asm9260-timer");

