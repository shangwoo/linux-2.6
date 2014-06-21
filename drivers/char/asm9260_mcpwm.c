/*******************************************************************************
* Copyright (C) 2014 AlphaScale												   											   
* file:asm9260_mcpwm.c                                                                                                                                                                                                                                        
* ASM9260 mcpwm driver	
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
#include <asm/uaccess.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/irq.h>
#include <mach/io.h>
#include <mach/pincontrol.h>

#include "asm9260_mcpwm.h"

struct mcpwm_config *reg_val;
static struct device    *mcpwm_dev;	/* platform device attached to */
static struct resource	*mcpwm_mem;
static struct resource	*mcpwm_irq;

static DECLARE_WAIT_QUEUE_HEAD(mcpwm_waitq);
static volatile int ev_mcpwm = 0;
static char flag=0;

static void asm9260_mcpwm_close(void)
{
        /* close mcpwm clock */
	as3310_writel(MCPWM_CLOCK_ENABLE,HW_AHBCLKCTRL0+8);
}

static void asm9260_mcpwm_start(void)
{
        /* enable mcpwm clock */
	as3310_writel(MCPWM_CLOCK_ENABLE,HW_AHBCLKCTRL0+4);    
}

static void asm9260_mcpwm_chan_pinmux(int chan)
{
   if (chan & MCPWM_CHANNEL0) {    
      set_pin_mux(5,1,PIN_FUNCTION_1);       /* MCI0 */     
      set_pin_mux(5,2,PIN_FUNCTION_1);       /* MCOA0 */   
      set_pin_mux(5,3,PIN_FUNCTION_1);       /* MCOB0 */
   }
   if (chan & MCPWM_CHANNEL1) {
      set_pin_mux(5,4,PIN_FUNCTION_1);       /* MCI1 */
      set_pin_mux(13,5,PIN_FUNCTION_1);      /* MCOA1 */ 
      set_pin_mux(13,6,PIN_FUNCTION_1);      /* MCOB1 */
   }
   if (chan & MCPWM_CHANNEL2) {
      set_pin_mux(13,7,PIN_FUNCTION_1);      /* MCI2 */
      set_pin_mux(14,0,PIN_FUNCTION_1);      /* MCOA2 */
      set_pin_mux(14,1,PIN_FUNCTION_1);      /* MCOB2 */  
   }
}

static int asm9260_mcpwm_open(struct inode *inode, struct file *file)
{
        /* start mcpwm */
	asm9260_mcpwm_start();
	return 0;
}

static int asm9260_mcpwm_release(struct inode *inode, struct file *file)
{
        /* close mcpwm */
	asm9260_mcpwm_close();
	return 0;
}

/********************************************************************* 
 * config mcpwm register as default value
 */
static void write_dval(void)
{
	as3310_writel(MCPWM_CON_DEFAULT_VAL,HW_MCPWM_CON_CLR);               
        as3310_writel(MCPWM_INT_DAFAULT_VAL,HW_MCPWM_INTEN_CLR);  	                 
        as3310_writel(MCPWM_INTF_DAFAULT_VAL,HW_MCPWM_INTF_CLR);  	          
	as3310_writel(MCPWM_CNTCON_DAFAULT_VAL,HW_MCPWM_CNTCON_CLR);
	as3310_writel(MCPWM_CAPCON_DAFAULT_VAL,HW_MCPWM_CAPCON_CLR);
}

/************************************************************************ 
 * mcpwm work in pwm mode 
 * chan: mcpwm channel 
 * limval: register HW_MCPWM_LIM value 
 * matval: register HW_MCPWM_MAT value 
 */
static int mcpwm_out(int chan,unsigned long *limval,unsigned long *matval)
{
        u32 cntconval=0,conval=0;
	write_dval();
        /* config mcpwm pinmux */
        asm9260_mcpwm_chan_pinmux(chan);
	
	if(chan & MCPWM_CHANNEL0)
	{
           /* set channel0 to timer mode */
           cntconval |= MCPWM_MODE_TIMER_CHAN0;  
           /* config pwm period and duty cycle*/      
           as3310_writel(limval[0],HW_MCPWM_LIM0);
           as3310_writel(matval[0],HW_MCPWM_MAT0);
           /* channel0 run in edge */
           conval |= MCPWM_CHAN0_RUN_EDGE;            
	}
        if (chan & MCPWM_CHANNEL1) 
        {
           /* set channel1 to timer mode */
           cntconval |= MCPWM_MODE_TIMER_CHAN1; 
           /* config pwm period and duty cycle*/      
           as3310_writel(limval[1],HW_MCPWM_LIM1);
           as3310_writel(matval[1],HW_MCPWM_MAT1); 
           /* channel1 run in edge */
           conval |= MCPWM_CHAN1_RUN_EDGE;             
        }
        if (chan & MCPWM_CHANNEL2) 
        {
           /* set channel2 to timer mode */
           cntconval |= MCPWM_MODE_TIMER_CHAN2;  
           /* config pwm period and duty cycle*/      
           as3310_writel(limval[2],HW_MCPWM_LIM2);
           as3310_writel(matval[2],HW_MCPWM_MAT2);  
           /* channel2 run in edge */
           conval |= MCPWM_CHAN2_RUN_EDGE;             
        }
	
        /* write val to timer cntcon and con register */
        as3310_writel(cntconval,HW_MCPWM_CNTCON_CLR);  
        as3310_writel(conval,HW_MCPWM_CON_SET);	
        		
	return 0;
}

/*********************************************************************** 
 * mcpwm work in counter mode 
 * chan: mcpwm channel 
 * limval: register HW_MCPWM_LIM value 
 * matval: register HW_MCPWM_MAT value 
 */
static int counter(int chan,unsigned long *limval,unsigned long *matval)
{
      u32 cntconval=0,conval=0;
      /* config related register as default value */
      write_dval();
      /* set pinmux according to channel num */
      asm9260_mcpwm_chan_pinmux(chan);
      
      if(chan & MCPWM_CHANNEL0)
      {
         /* set channel0 to counter mode */
         cntconval |= MCPWM_MODE_COUNTER_CHAN0;
         /* config pwm period and duty cycle*/
         as3310_writel(limval[0],HW_MCPWM_LIM0);
         as3310_writel(matval[0],HW_MCPWM_MAT0);
         /* channel0 run in edge */
         conval |= MCPWM_CHAN0_RUN_EDGE;
      }
      if (chan & MCPWM_CHANNEL1) 
      {
         /* set channel1 to counter mode */
         cntconval |= MCPWM_MODE_COUNTER_CHAN1;
         /* config pwm period and duty cycle*/
         as3310_writel(limval[1],HW_MCPWM_LIM1);
         as3310_writel(matval[1],HW_MCPWM_MAT1);
         /* channel1 run in edge */
         conval |= MCPWM_CHAN1_RUN_EDGE;  
      }
      if (chan & MCPWM_CHANNEL2) 
      {
         /* set channel2 to counter mode */
         cntconval |= MCPWM_MODE_COUNTER_CHAN2;
         /* config pwm period and duty cycle*/
         as3310_writel(limval[2],HW_MCPWM_LIM2);
         as3310_writel(matval[2],HW_MCPWM_MAT2);
         /* channel2 run in edge */
         conval |= MCPWM_CHAN2_RUN_EDGE; 
      }

      /* write val to timer cntcon and con register */
      as3310_writel(cntconval,HW_MCPWM_CNTCON_SET);
      as3310_writel(conval,HW_MCPWM_CON_SET);
      return 0;
}

/********************************************************************** 
 * mcpwm work in capture mode 
 * capnum: mcpwm capture channel  
 */
static int capture(int cap)
{
   u32 capconval=0,intval=0,conval=0;

   /* config related register as default value */
   write_dval();
   /* set pinmux according to channel num */
   asm9260_mcpwm_chan_pinmux(cap);

   if (cap & MCPWM_CHANNEL0) 
   {
      /* cap0 capture on the trailing edge */
      capconval |= MCPWM_CAP0_DOWN;
      /* enable cap0 interrupt */
      intval |= MCPWM_CAP0_INTEN;
      /* channel0 run in edge */
      conval |= MCPWM_CHAN0_RUN_EDGE;
   }
   if (cap & MCPWM_CHANNEL1) 
   {
      /* cap1 capture on the trailing edge */
      capconval |= MCPWM_CAP1_DOWN;
      /* enable cap1 interrupt */
      intval |= MCPWM_CAP1_INTEN;
      /* channel1 run in edge */
      conval |= MCPWM_CHAN1_RUN_EDGE;
   }
   if (cap & MCPWM_CHANNEL2) 
   {
      /* cap2 capture on the trailing edge */
      capconval |= MCPWM_CAP2_DOWN;
      /* enable cap2 interrupt */
      intval |= MCPWM_CAP2_INTEN;
      /* channel2 run in edge */
      conval |= MCPWM_CHAN2_RUN_EDGE;
   }
   
   /* write val to timer capcon inten con register */
   as3310_writel(capconval,HW_MCPWM_CAPCON_SET);		   
   as3310_writel(intval,HW_MCPWM_INTEN_SET);  	                  
   as3310_writel(conval,HW_MCPWM_CON_SET);  

   return 0;
}

/************************************************************************ 
 * mcpwm work in capture with dt mode 
 * capnum: mcpwm capture channel 
 * dtval: register  HW_MCPWM_DT  value 
 */
static int capture_dt(int cap,unsigned long *dtval)
{
   u32 capconval=0,intval=0,conval=0;

   /* config related register as default value */
   write_dval();
   /* set pinmux according to channel num */
   asm9260_mcpwm_chan_pinmux(cap);

   if (cap & MCPWM_CHANNEL0) 
   {
      /* cap0 capture on the trailing edge with dt */
      capconval |= MCPWM_CAP0_WITH_DT_DOWN;
      /* enable cap0 interrupt */
      intval |= MCPWM_CAP0_INTEN;
      /* channel0 run in edge with dten */
      conval |= MCPWM_CHAN0_RUN_DTEN_EDGE;                                                                                     
   }
   if (cap & MCPWM_CHANNEL1) 
   {
      /* cap1 capture on the trailing edge with dt */
      capconval |= MCPWM_CAP1_WITH_DT_DOWN;
      /* enable cap1 interrupt */
      intval |= MCPWM_CAP1_INTEN;
      /* channel1 run in edge with dten */
      conval |= MCPWM_CHAN1_RUN_DTEN_EDGE;                                                                                                                                        
   }
   if (cap & MCPWM_CHANNEL2) 
   {
      /* cap2 capture on the trailing edge with dt */
      capconval |= MCPWM_CAP2_WITH_DT_DOWN;
      /* enable cap2 interrupt */
      intval |= MCPWM_CAP2_INTEN;
      /* channel2 run in edge with dten */
      conval |= MCPWM_CHAN2_RUN_DTEN_EDGE;                                                                                                       
   }
   
   /* write val to timer capcon inten dt and con register */
   as3310_writel(capconval,HW_MCPWM_CAPCON_SET);	
   as3310_writel(intval,HW_MCPWM_INTEN_SET);  
   /* bit 0-9: channel0 dt num; bit 10-19: channel1 dt num; bit 20-29: channel2 dt num */
   as3310_writel((dtval[0]|(dtval[1]<<10)|(dtval[2]<<20)),HW_MCPWM_DT);     
   as3310_writel(conval,HW_MCPWM_CON_SET);    
   return 0;
}

static int asm9260_mcpwm_read(struct file *filp,char __user *buff,size_t count,loff_t *offp)
{
    /* wait until interrupt on */
    wait_event_interruptible(mcpwm_waitq,ev_mcpwm);
    ev_mcpwm = MCPWM_EVENT_NONE;
    /* copy flag to user space */
    copy_to_user(buff,&flag,sizeof(flag));
    flag = MCPWM_INTERRUPT_NONE;
    return 0;
}

/************************************************************************* 
 * mcpwm works in various modes based on cmd value
 */
static long asm9260_mcpwm_ioctl(struct file *file,	unsigned int cmd, unsigned long arg)
{
    int ret;
    memset((struct mcpwm_config *)reg_val,0,sizeof(struct mcpwm_config));
        /* copy regval from user space */
	ret=copy_from_user((struct mcpwm_config *)reg_val,(struct mcpwm_config *)arg,sizeof(struct mcpwm_config));
	switch (cmd) {
        /* mcpwm operations according to cmd */
    	case MCPWM_MODE_OUTPUT:
                mcpwm_out(reg_val->chan,reg_val->lim,reg_val->mat);
                break;
	case MCPWM_MODE_COUNTER:
                counter(reg_val->chan,reg_val->lim,reg_val->mat);
		break;
	case MCPWM_MODE_CAPTURE:
                capture(reg_val->chan);
		break;
        case MCPWM_MODE_CAPTURE_WITH_DT:
                capture_dt(reg_val->chan,reg_val->dt);
                break;
	default:
		dev_err(mcpwm_dev,"input error!\n");
		break;
	}
	return 0;
}

/* kernel interface */
static const struct file_operations asm9260_mcpwm_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= asm9260_mcpwm_ioctl,
	.open		= asm9260_mcpwm_open,
	.release	= asm9260_mcpwm_release,
        .read           = asm9260_mcpwm_read,
};

static struct miscdevice asm9260_mcpwm_miscdev = {
	.minor		= MCPWM_MINOR,
	.name		= "mcpwm",
	.fops		= &asm9260_mcpwm_fops,
};

/* interrupt handler code */
void asm9260_mcpwm_interrupt(void)
{
	unsigned int ir_tmp;

        /* read val of interrupt flag register */
 	ir_tmp=as3310_readl(HW_MCPWM_INTF);

        /* channel0 interrupt */
	if((ir_tmp & MCPWM_CAP0_INT_FLAG) !=0)
	{
                flag |= MCPWM_CHAN0_INT;
                /* set ev_mcpwm and wake up waiting event */
                ev_mcpwm = MCPWM_EVENT_ON; 
                wake_up_interruptible(&mcpwm_waitq);
                /* clear channel0 interrupt flag */
		as3310_writel(MCPWM_CAP0_INT_FLAG,HW_MCPWM_INTF_CLR);
	}

        /* channel1 interrupt */
	if((ir_tmp & MCPWM_CAP1_INT_FLAG) !=0)
	{
                flag |= MCPWM_CHAN1_INT;
                /* set ev_mcpwm and wake up waiting event */
                ev_mcpwm = MCPWM_EVENT_ON; 
                wake_up_interruptible(&mcpwm_waitq);
                /* clear channel1 interrupt flag */
		as3310_writel(MCPWM_CAP1_INT_FLAG,HW_MCPWM_INTF_CLR);
	}
	
        /* channel1 interrupt */
	if((ir_tmp & MCPWM_CAP2_INT_FLAG) !=0)
	{
                flag |= MCPWM_CHAN2_INT;
                /* set ev_mcpwm and wake up waiting event */
                ev_mcpwm = MCPWM_EVENT_ON; 
                wake_up_interruptible(&mcpwm_waitq);
                /* clear channel2 interrupt flag */
		as3310_writel(MCPWM_CAP2_INT_FLAG,HW_MCPWM_INTF_CLR);
	}

}

static irqreturn_t asm9260_mcpwm_irq(int irqno, void *param)
{
        /* hanndle interrupt event */
	asm9260_mcpwm_interrupt();

	return IRQ_HANDLED;
}

/* device interface */
static int asm9260_mcpwm_probe(struct platform_device *pdev)
{
       
	struct resource *res;
	struct device *dev;
	int ret;
	int size;
    
	dev = &pdev->dev;
	mcpwm_dev = &pdev->dev;

        /* assign memory for struct mcpwm_config */
        reg_val=(struct mcpwm_config *)kmalloc(sizeof(struct mcpwm_config),GFP_KERNEL);
        if (reg_val == NULL) 
        {
                dev_err(mcpwm_dev,"assign mem failed!\n");
                return -1;
        }

        /* get platform resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(mcpwm_dev, "no memory resource specified\n");
		return -ENOENT;
	}
	
        /* request memory */
	size = (res->end - res->start);
	mcpwm_mem = request_mem_region(res->start, size, pdev->name);
	if (mcpwm_mem == NULL) {
		dev_err(mcpwm_dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

        /* get irq resources */
	mcpwm_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (mcpwm_irq == NULL) {
		dev_err(mcpwm_dev, "no irq resource specified\n");
		ret = -ENOENT;
	}

        /* request irq */
	ret = request_irq(mcpwm_irq->start, &asm9260_mcpwm_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(mcpwm_dev, "failed to install irq (%d)\n", ret);
		goto err_irq;
	}

        /* register mcpwm device */
	ret = misc_register(&asm9260_mcpwm_miscdev);
	if (ret) {
		dev_err(mcpwm_dev, "cannot register miscdev on minor=%d (%d)\n", MCPWM_MINOR, ret);
	}

        /* mcpwm init operation */
        asm9260_mcpwm_start();

	return 0;


 err_irq:
	free_irq(mcpwm_irq->start, pdev);

 err_req:
	release_resource(mcpwm_mem);

	return ret;
}

static int asm9260_mcpwm_remove(struct platform_device *dev)
{
        /* release resources and free irq */
	release_resource(mcpwm_mem);
	mcpwm_mem = NULL;

	free_irq(mcpwm_irq->start, dev);
	mcpwm_irq = NULL;

        kfree(reg_val);
	misc_deregister(&asm9260_mcpwm_miscdev);
        asm9260_mcpwm_close();
	return 0;
}

static void asm9260_mcpwm_shutdown(struct platform_device *dev)
{
        /* shutdown mcpwm device */
	asm9260_mcpwm_close();
}


static struct platform_driver asm9260_mcpwm_driver = {
	.probe		= asm9260_mcpwm_probe,
	.remove		= asm9260_mcpwm_remove,
	.shutdown	= asm9260_mcpwm_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "asm9260-mcpwm",
	},
};


static int __init mcpwm_init(void)
{
	return platform_driver_register(&asm9260_mcpwm_driver);
}

static void __exit mcpwm_exit(void)
{
	platform_driver_unregister(&asm9260_mcpwm_driver);
}

module_init(mcpwm_init);
module_exit(mcpwm_exit);

MODULE_AUTHOR("alphascale");
MODULE_DESCRIPTION("Asm9260 Mcpwm Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(MCPWM_MINOR);
MODULE_ALIAS("platform:asm9260-mcpwm");

