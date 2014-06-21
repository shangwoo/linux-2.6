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

static struct device    *timer_dev;	/* platform device attached to */
static struct resource	*timer_mem;
static struct resource	*timer_irq;
static void __iomem	*timer_base;

static DECLARE_WAIT_QUEUE_HEAD(timer_waitq);
static volatile int ev_timer = 0;
static char flag=0;

/********************************************************** 
config timer register 
chan: timer channel
mrval: register HW_TIMER1_MR value
emr: register HW_TIMER1_EMR value
countval: when work in counter mode, set counter value
**********************************************************/
struct timer_config
{
        int chan;
	unsigned long mrval;
	unsigned long thval;
	unsigned long emr;
	unsigned long countval;
};

struct timer_reg
{
   unsigned long timer_pr;
   unsigned long timer_mcr;
   unsigned long timer_tcr;
   unsigned long timer_mr[4];
   unsigned long timer_pwmth[4];
   unsigned long timer_ir;
   unsigned long timer_pwmc;
   unsigned long timer_ctcr;
   unsigned long timer_emr;
   unsigned long timer_ccr;
};

static struct timer_config *reg;
static struct timer_reg dval;

/********************************************** 
get default value of timer register 
**********************************************/
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

/********************************************* 
write default value to timer register 
*********************************************/
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

/******************************************************** 
timer works in interrupt mode 
chan: timer channel 
mrval: delay value between each interrupt  
********************************************************/
void timer_irq_test(int chan,unsigned long mrval)
{	
   write_dval();
   if (chan==0) {
         as3310_writel(mrval,HW_TIMER1_MR0);                      //timer1   HWHW_TIMERx_MR0
         as3310_writel(0X00000000,HW_TIMER1_PR);		  //timer1   HWHW_TIMERx_PR
         as3310_writel(0X00000001,HW_TIMER1_MCR);		  //timer1   HWHW_TIMERx_MCR  249:intr 492:reset 924:stop
   }
   else if (chan==1) 
   {
         as3310_writel(mrval,HW_TIMER1_MR1);                      //timer1   HWHW_TIMERx_MR0
         as3310_writel(0X00000000,HW_TIMER1_PR);		  //timer1   HWHW_TIMERx_PR
         as3310_writel(0X00000008,HW_TIMER1_MCR);		
   }
   else if (chan==2) 
   {
         as3310_writel(mrval,HW_TIMER1_MR2);                      //timer1   HWHW_TIMERx_MR0
         as3310_writel(0X00000000,HW_TIMER1_PR);		  //timer1   HWHW_TIMERx_PR
         as3310_writel(0X00000040,HW_TIMER1_MCR);		
   }
   else if (chan==3) 
   {
         as3310_writel(mrval,HW_TIMER1_MR3);                      //timer1   HWHW_TIMERx_MR0
         as3310_writel(0X00000000,HW_TIMER1_PR);		  //timer1   HWHW_TIMERx_PR
         as3310_writel(0X00000200,HW_TIMER1_MCR);		
   }
   else
   {
         dev_err(timer_dev,"input chan error!\n");
   }
                                                                  
   as3310_writel(0X000000F0,HW_TIMER1_TCR+4);		          //timer1   HWHW_TIMERx_TCR
   as3310_writel(0X000000F0,HW_TIMER1_TCR+8);
   as3310_writel(0X0000000F,HW_TIMER1_TCR);
	
   mdelay(100);
}

/******************************************************** 
timer works in pwm mode 
chan: pwm channel 
mr: register HW_TIMER1_MR value
th: register HW_TIMER1_PWMTH value
********************************************************/
void pwm_output(int chan,unsigned long mr,unsigned long th)
{
   write_dval();
   if (chan==0) {
        as3310_writel(mr,HW_TIMER1_MR0); 
        as3310_writel(th,HW_TIMER1_PWMTH0); 
        as3310_writel(0X00000000,HW_TIMER1_IR);		  
  	as3310_writel(0X00000000,HW_TIMER1_PR);		  
	as3310_writel(0x00000001,HW_TIMER1_PWMC);
        set_pin_mux(10,0,PIN_FUNCTION_1);
   }
   else if (chan==1) 
   {
        as3310_writel(mr,HW_TIMER1_MR1);
        as3310_writel(th,HW_TIMER1_PWMTH1); 
        as3310_writel(0X00000000,HW_TIMER1_IR);		  
  	as3310_writel(0X00000000,HW_TIMER1_PR);		  
	as3310_writel(0x00000002,HW_TIMER1_PWMC);
        set_pin_mux(10,1,PIN_FUNCTION_1);
   }
   else if (chan==2) 
   {
        as3310_writel(mr,HW_TIMER1_MR2);	
        as3310_writel(th,HW_TIMER1_PWMTH2); 
        as3310_writel(0X00000000,HW_TIMER1_IR);		  
  	as3310_writel(0X00000000,HW_TIMER1_PR);		  
	as3310_writel(0x00000004,HW_TIMER1_PWMC);
        set_pin_mux(10,2,PIN_FUNCTION_1);
   }
   else if (chan==3) 
   {
        as3310_writel(mr,HW_TIMER1_MR3);	
        as3310_writel(th,HW_TIMER1_PWMTH3); 
        as3310_writel(0X00000000,HW_TIMER1_IR);		  
  	as3310_writel(0X00000000,HW_TIMER1_PR);		  
	as3310_writel(0x00000008,HW_TIMER1_PWMC);
        set_pin_mux(10,3,PIN_FUNCTION_1);
   }
   else
   {
         dev_err(timer_dev,"input chan error!\n");
   }
	 
   as3310_writel(0X000000f0,HW_TIMER1_TCR+4);		  
   as3310_writel(0X000000f0,HW_TIMER1_TCR+8);
   as3310_writel(0x0000000f,HW_TIMER1_TCR);
}

/******************************************************** 
timer works in pwm mode,only output single wave 
chan: timer channel 
mr: register HW_TIMER1_MR value
th: register HW_TIMER1_PWMTH value
********************************************************/
void pwm_single_output(int chan,unsigned long mr,unsigned long th)
{
   write_dval();
   if (chan==0) {
        as3310_writel(mr,HW_TIMER1_MR0); 
        as3310_writel(th,HW_TIMER1_PWMTH0); 
        as3310_writel(0X00000000,HW_TIMER1_IR);		  
  	as3310_writel(0X00000000,HW_TIMER1_PR);		  
	as3310_writel(0x00000011,HW_TIMER1_PWMC);
        set_pin_mux(10,0,PIN_FUNCTION_1);
   }
   else if (chan==1) 
   {
        as3310_writel(mr,HW_TIMER1_MR1);
        as3310_writel(th,HW_TIMER1_PWMTH1); 
        as3310_writel(0X00000000,HW_TIMER1_IR);		  
  	as3310_writel(0X00000000,HW_TIMER1_PR);		  
	as3310_writel(0x00000022,HW_TIMER1_PWMC);
        set_pin_mux(10,1,PIN_FUNCTION_1);
   }
   else if (chan==2) 
   {
        as3310_writel(mr,HW_TIMER1_MR2);	
        as3310_writel(th,HW_TIMER1_PWMTH2); 
        as3310_writel(0X00000000,HW_TIMER1_IR);		  
  	as3310_writel(0X00000000,HW_TIMER1_PR);		  
	as3310_writel(0x00000044,HW_TIMER1_PWMC);
        set_pin_mux(10,2,PIN_FUNCTION_1);
   }
   else if (chan==3) 
   {
        as3310_writel(mr,HW_TIMER1_MR3);	
        as3310_writel(th,HW_TIMER1_PWMTH3); 
        as3310_writel(0X00000000,HW_TIMER1_IR);		  
  	as3310_writel(0X00000000,HW_TIMER1_PR);		  
	as3310_writel(0x00000088,HW_TIMER1_PWMC);
        set_pin_mux(10,3,PIN_FUNCTION_1);
   }
   else
   {
         dev_err(timer_dev,"input chan error!\n");
   }
	 
   as3310_writel(0X000000f0,HW_TIMER1_TCR+4);		  
   as3310_writel(0X000000f0,HW_TIMER1_TCR+8);
   as3310_writel(0x0000000f,HW_TIMER1_TCR);
}

/******************************************************** 
timer works in capture mode,CT1_CAP should connect 
with an output signal 
********************************************************/
void capture_irq(void)
{
        write_dval();
/*test*/
#if 0
	as3310_writel(100000000,HW_TIMER1_MR2);      //timer0~3   HWHW_TIMERx_MR0  1s
	as3310_writel(50000000,HW_TIMER1_PWMTH2);  
	as3310_writel(0x00000004,HW_TIMER1_PWMC);	  //timer0~3   HWHW_TIMERx_PWMC
	set_pin_mux(10,2,PIN_FUNCTION_1);	  //CT1_MAT2
#endif                                                    

	/******capture mode********/
	as3310_writel(0x00000005,HW_TIMER1_CCR);		//up
	set_pin_mux(10,4,PIN_FUNCTION_1);	   //CT1_CAP 
	
	as3310_writel(0X000000f0,HW_TIMER1_TCR+4);		  //timer0~3   HWHW_TIMERx_TCR
	as3310_writel(0X000000f0,HW_TIMER1_TCR+8);
	as3310_writel(0x0000000f,HW_TIMER1_TCR);

}

/******************************************************** 
timer works in counter mode,CT1_CAP should connect 
with an output signal 
chan: timer channel 
count_num: register HW_TIMER1_MR value
********************************************************/
void counter_irq(int chan,unsigned long count_num)
{ 
        write_dval();
/*test*/
#if 0
	as3310_writel(100000,HW_TIMER1_MR2);                      //timer1   HWHW_TIMERx_MR0
	as3310_writel(50000,HW_TIMER1_PWMTH2);  
	as3310_writel(0x00000004,HW_TIMER1_PWMC);	          //timer1   HWHW_TIMERx_PWMC
	set_pin_mux(10,2,PIN_FUNCTION_1);	                  //CT1_MAT2
#endif                                                    

        set_pin_mux(10,4,PIN_FUNCTION_1);	                  //CT1_CAP 
	/*****counter mode********/
        if (chan==0) {
           as3310_writel(0x00000002,HW_TIMER1_CTCR);	  	
           as3310_writel(count_num,HW_TIMER1_MR0); 	  	  //timer1   HWHW_TIMERx_MR1
           as3310_writel(0X00000001,HW_TIMER1_MCR);		  //timer1   HWHW_TIMERx_MCR  249:intr 492:reset 924:stop
        }
        else if (chan==1) {
           as3310_writel(0x00000008,HW_TIMER1_CTCR);	  
           as3310_writel(count_num,HW_TIMER1_MR1); 	  	  //timer1   HWHW_TIMERx_MR1
           as3310_writel(0X00000008,HW_TIMER1_MCR);		  //timer1   HWHW_TIMERx_MCR  249:intr 492:reset 924:stop
          
        }
        else if (chan==2) {
           as3310_writel(0x00000020,HW_TIMER1_CTCR);	  	  //TC1ÔÚCAP up count++
           as3310_writel(count_num,HW_TIMER1_MR2); 	  	  //timer1   HWHW_TIMERx_MR1
           as3310_writel(0X00000040,HW_TIMER1_MCR);		  //timer1   HWHW_TIMERx_MCR  249:intr 492:reset 924:stop
         
        }
        else if (chan==3) {
           as3310_writel(0x00000080,HW_TIMER1_CTCR);	  	  //TC1ÔÚCAP up count++
           as3310_writel(count_num,HW_TIMER1_MR3); 	  	  //timer1   HWHW_TIMERx_MR1
           as3310_writel(0X00000200,HW_TIMER1_MCR);		  //timer1   HWHW_TIMERx_MCR  249:intr 492:reset 924:stop
         
        }
        else {
           dev_err(timer_dev,"input chan error!\n");
        }

           as3310_writel(0X000000f0,HW_TIMER1_TCR+4);		  //timer1   HWHW_TIMERx_TCR
           as3310_writel(0X000000f0,HW_TIMER1_TCR+8);
           as3310_writel(0x0000000f,HW_TIMER1_TCR);
	
}

/************************************************************ 
timer works in emr mode 
chan: timer channel
emr: output mode,0:none,1:output '1',2:output '0',3:toggle
mrval: register HW_TIMER1_MR value
************************************************************/
void emr_mode(int chan,unsigned long emr,unsigned long mrval)
{
   write_dval();
   as3310_writel(0X00000000,HW_TIMER1_PR);		       //timer0~3   HWHW_TIMERx_PR

   if (chan==0) {
         set_pin_mux(10,0,PIN_FUNCTION_1);	               //CT1_MAT0
         switch (emr) {
         case 0:
            as3310_writel(0x00000000,HW_TIMER1_EMR);           //none
               break;
         case 1:
            as3310_writel(0x00000020,HW_TIMER1_EMR);           //set high
               break;
         case 2:
            as3310_writel(0x00000010,HW_TIMER1_EMR);           //set low
               break;
         case 3:
            as3310_writel(0x00000030,HW_TIMER1_EMR);           //toggle
               break;
         default:
            dev_err(timer_dev,"input emr error!\n");
               break;
         }
         as3310_writel(mrval,HW_TIMER1_MR0);
   }
   else if (chan==1) 
   {
	 set_pin_mux(10,1,PIN_FUNCTION_1);	               //CT1_MAT1  
           switch (emr) {
         case 0:
            as3310_writel(0x00000000,HW_TIMER1_EMR);           //none
               break;
         case 1:
            as3310_writel(0x00000080,HW_TIMER1_EMR);           //high
               break;
         case 2:
            as3310_writel(0x00000040,HW_TIMER1_EMR);           //low
               break;
         case 3:
            as3310_writel(0x000000c0,HW_TIMER1_EMR);           //toggle
               break;
         default:
            dev_err(timer_dev,"input emr error!\n");
               break;
         }
         as3310_writel(mrval,HW_TIMER1_MR1);
   }
   else if (chan==2) 
   {
         set_pin_mux(10,2,PIN_FUNCTION_1);	               //CT1_MAT2  
            switch (emr) {
         case 0:
            as3310_writel(0x00000000,HW_TIMER1_EMR);
               break;
         case 1:
            as3310_writel(0x00000200,HW_TIMER1_EMR);
               break;
         case 2:
            as3310_writel(0x00000100,HW_TIMER1_EMR);
               break;
         case 3:
            as3310_writel(0x00000300,HW_TIMER1_EMR);
               break;
         default:
            dev_err(timer_dev,"input emr error!\n");
               break;
         }
         as3310_writel(mrval,HW_TIMER1_MR2);
   }
   else if (chan==3) 
   {
	 set_pin_mux(10,3,PIN_FUNCTION_1);	                  //CT1_MAT3  
            switch (emr) {
         case 0:
            as3310_writel(0x00000000,HW_TIMER1_EMR);
               break;
         case 1:
            as3310_writel(0x00000800,HW_TIMER1_EMR);
               break;
         case 2:
            as3310_writel(0x00000400,HW_TIMER1_EMR);
               break;
         case 3:
            as3310_writel(0x00000c00,HW_TIMER1_EMR);
               break;
         default:
            dev_err(timer_dev,"input emr error!\n");
               break;
         }
         as3310_writel(mrval,HW_TIMER1_MR3);
   }
   else
   {
         dev_err(timer_dev,"input chan error!\n");
   }

  	as3310_writel(0X000000f0,HW_TIMER1_TCR+4);		  //timer1   HWHW_TIMERx_TCR
	as3310_writel(0X000000f0,HW_TIMER1_TCR+8);
	as3310_writel(0X0000000f,HW_TIMER1_TCR);		  //timer1   HWHW_TIMERx_TCR
}

static void asm9260_timer_close(void)
{
	as3310_writel(0x00000020,HW_AHBCLKCTRL1+8);	 
}

static void asm9260_timer_start(void)
{
	as3310_writel(0x00000020,HW_AHBCLKCTRL1+4);            
}

static int asm9260_timer_open(struct inode *inode, struct file *file)
{
        asm9260_timer_start();
	//return nonseekable_open(inode, file);
        return 0;
}

static int asm9260_timer_release(struct inode *inode, struct file *file)
{
	asm9260_timer_close();
	return 0;
}
  
static int asm9260_timer_read(struct file *filp,char __user *buff,size_t count,loff_t *offp)
{
    wait_event_interruptible(timer_waitq,ev_timer);
    ev_timer = 0;
    copy_to_user(buff,&flag,sizeof(flag));
    flag=0;
    return 0;
}


/****************************************************** 
timer works in various modes based on cmd value
******************************************************/
static long asm9260_timer_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        memset((struct timer_config *)reg,0,sizeof(struct timer_config));
	copy_from_user((struct timer_config *)reg,(struct timer_config *)arg,sizeof(struct timer_config));
	switch(cmd)
	{
		case 1://pwm output
			pwm_output(reg->chan,reg->mrval,reg->thval);
			break;
                case 2://pwm single output
                        pwm_single_output(reg->chan,reg->mrval,reg->thval);
                        break;
		case 3://emr mode
			emr_mode(reg->chan,reg->emr,reg->mrval);
			break;
		case 4://match interrupt
			timer_irq_test(reg->chan,reg->mrval);
			break;
		case 5://counter mode
			counter_irq(reg->chan,reg->countval);
			break;
		case 6://capture mode
			capture_irq();
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
	unsigned long data;
	
	ir_tmp=as3310_readl(HW_TIMER1_IR);
	
	if((ir_tmp & 0x1) != 0)
	{
                data=as3310_readl(HW_TIMER1_TC0);
                flag = 1;
                ev_timer = 1; 
                wake_up_interruptible(&timer_waitq);
                //printk("TC0_data:%8x\n",data);
		as3310_writel(0x1,HW_TIMER1_IR);
	}
	if((ir_tmp & 0x2) != 0)
	{
                data=as3310_readl(HW_TIMER1_TC1);
                flag = 2;
                ev_timer = 1; 
                wake_up_interruptible(&timer_waitq);
                //printk("TC1_data:%8x\n",data);
		as3310_writel(0x2,HW_TIMER1_IR);
	}
	if((ir_tmp & 0x4) != 0)
	{
                data=as3310_readl(HW_TIMER1_TC2);
                flag = 3;
                ev_timer = 1; 
                wake_up_interruptible(&timer_waitq);
                //printk("TC2_data:%8x\n",data);
		as3310_writel(0x4,HW_TIMER1_IR);
	}
	if((ir_tmp & 0x8) != 0)
	{
                data=as3310_readl(HW_TIMER1_TC3);
                flag = 4;
                ev_timer = 1; 
                wake_up_interruptible(&timer_waitq);
                //printk("TC3_data:%8x\n",data);
		as3310_writel(0x8,HW_TIMER1_IR);
	}
	if((ir_tmp & 0x10) !=0)
	{
		data=as3310_readl(HW_TIMER1_CR2);
                flag = 5;
                ev_timer = 1; 
                wake_up_interruptible(&timer_waitq);
		//printk("CR2_data:%8x\n",data);
		as3310_writel(0x10,HW_TIMER1_IR);
	}

}

static irqreturn_t asm9260timer_irq(int irqno, void *param)
{
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

        reg=(struct timer_config *)kmalloc(sizeof(struct timer_config),GFP_KERNEL);
        if (reg == NULL) 
        {
           dev_err(dev,"assign mem failed!\n");
        }

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no memory resource specified\n");
		return -ENOENT;
	}
	
	size = (res->end - res->start);
	timer_mem = request_mem_region(res->start, size, pdev->name);
	if (timer_mem == NULL) {
		dev_err(dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	timer_base = ioremap(res->start, size);
	if (timer_base == NULL) {
		dev_err(dev, "failed to ioremap() region\n");
		ret = -EINVAL;
		goto err_req;
	}

	dev_dbg(dev,"probe: mapped timer_base=%8x\n", timer_base);

	timer_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (timer_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_map;
	}

	ret = request_irq(timer_irq->start, &asm9260timer_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_irq;
	}

	ret = misc_register(&asm9260_timer_miscdev);
	if (ret) {
		dev_err(dev, "cannot register miscdev on minor=%d (%d)\n", TIMER1_MINOR, ret);
	}

        asm9260_timer_start();
        read_dval();


	return 0;

 err_irq:
	free_irq(timer_irq->start, pdev);

 err_map:
	iounmap(timer_base);

 err_req:
	release_resource(timer_mem);

	return ret;
}

static int asm9260_timer_remove(struct platform_device *dev)
{
	release_resource(timer_mem);
	timer_mem = NULL;

	free_irq(timer_irq->start, dev);
	timer_irq = NULL;

	iounmap(timer_base);
        kfree(reg);
	misc_deregister(&asm9260_timer_miscdev);
        asm9260_timer_close();
	return 0;
}

static void asm9260_timer_shutdown(struct platform_device *dev)
{
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

