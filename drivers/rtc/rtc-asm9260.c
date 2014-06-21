
/*/drivers/rtc/rtc-asm9260.c
 *alpscale asm9260 realtime alarm driver
 *Created by shanjs 1/12/2012
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <mach/rtc.h>
#include <mach/io.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/time.h>


/* platform_bus isn't hotpluggable, so for static linkage it'd be safe
 * to get rid of probe() and remove() code ... too bad the driver struct
 * remembers probe(), that's about 25% of the runtime footprint!!
 */
#ifndef	MODULE
#undef	__devexit
#undef	__devexit_p
#define	__devexit	__exit
#define	__devexit_p	__exit_p
#endif

#define CTIME2_DOY 0xFFF
#define CTIME1_YEAR 0xFFF
#define CTIME1_MON 0xF
#define CTIME1_DOM 0x1F
#define CTIME0_HOUR 0x1F
#define CTIME0_MIN 0x3F
#define CTIME0_SEC 0x3F
#define CTIME0_WEEK 0x7
#define SHIFT_8 8
#define SHIFT_16 16
#define SHIFT_24 24
#define AMR_OFF 0
#define ALARM_OFF 0
#define ALARM_SET 0x2
#define START_RTC_32K 0x1
#define START_RTC_12M 0x5
#define CLOSE_RTC 0x0
#define CIIR_OFF 0
#define CAL_ON (1<<4)
#define TICK_IRQ 0x1
#define ALARM_IRQ 0x2
#define CLEAR_TICK_IRQ 0x1
#define CLEAR_ALARM_IRQ 0x2
#define RTC_IRQ_NUM 0x1
#define RTC_CCR_RESET0 2
#define RTC_CCR_RESET1 0
#define  RTC_IT_SEC             0x00000001      
#define  RTC_IT_MIN             0x00000002            
#define  RTC_IT_HOUR           0x00000004        
#define  RTC_IT_DOM            0x00000008         
#define  RTC_IT_DOW            0x00000010           
#define  RTC_IT_DOY             0x00000020            
#define  RTC_IT_MON            0x00000040            
#define  RTC_IT_YEAR            0x00000080
#define  RTC_AMR_SEC           0x00000001            
#define  RTC_AMR_MIN           0x00000002          
#define  RTC_AMR_HOUR         0x00000004            
#define  RTC_AMR_DOM           0x00000008            
#define  RTC_AMR_DOW           0x00000010           
#define  RTC_AMR_DOY            0x00000020           
#define  RTC_AMR_MON           0x00000040            
#define  RTC_AMR_YEAR           0x00000080
#define	RTC_CAL_FORWARD      0x00000000									   
#define	RTC_CAL_BACKWARD     0x00000001									   

struct rtc_cal{
	int value;
	int direction;
};


#if 0
#define rtc_dbg(x...) dbg_printf(x);
#else
#define rtc_dbg(x...) do{}while(0);
#endif

static int asm9260_rtc_alarm;

static void delay(int delay)
{
	while (delay--);
}

/* 
  *  Print the real time.
  */
static void showtime(void)
{
	printk("%d-%d-%d %d:%d:%d ",
		((as3310_readl(HW_RTC_CTIME1)>>SHIFT_16)&CTIME1_YEAR)+1900,
		((as3310_readl(HW_RTC_CTIME1)>>SHIFT_8)&CTIME1_MON)+1,
		as3310_readl(HW_RTC_CTIME1)&CTIME1_DOM,
		(as3310_readl(HW_RTC_CTIME0)>>SHIFT_16)&CTIME0_HOUR,
		(as3310_readl(HW_RTC_CTIME0)>>SHIFT_8)&CTIME0_MIN,
		as3310_readl(HW_RTC_CTIME0)&(CTIME0_SEC));
	printk("\n");
}

/* 
  *  Read the real time.
  */
static int asm9260_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	tm->tm_sec=as3310_readl(HW_RTC_CTIME0)&(CTIME0_SEC);
	tm->tm_min=(as3310_readl(HW_RTC_CTIME0)>>SHIFT_8)&CTIME0_MIN;
	tm->tm_hour=(as3310_readl(HW_RTC_CTIME0)>>SHIFT_16)&CTIME0_HOUR;
	tm->tm_wday=(as3310_readl(HW_RTC_CTIME0)>>SHIFT_24)&CTIME0_WEEK;
	tm->tm_mday=as3310_readl(HW_RTC_CTIME1)&CTIME1_DOM;
	tm->tm_mon=(as3310_readl(HW_RTC_CTIME1)>>SHIFT_8)&CTIME1_MON;
	tm->tm_year=(as3310_readl(HW_RTC_CTIME1)>>SHIFT_16)&CTIME1_YEAR;
	tm->tm_yday=as3310_readl(HW_RTC_CTIME2)&CTIME2_DOY;
	return 0; 
}

/* 
  *  Set the real time.
  */
static int asm9260_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	as3310_writel(tm->tm_sec , HW_RTC_SEC);
	as3310_writel(tm->tm_min,  HW_RTC_MIN);
	as3310_writel(tm->tm_hour,  HW_RTC_HOUR);
	as3310_writel(tm->tm_wday,  HW_RTC_DOW);
	as3310_writel(tm->tm_mday,  HW_RTC_DOM);
	as3310_writel(tm->tm_yday,  HW_RTC_DOY);
	as3310_writel(tm->tm_mon,  HW_RTC_MONTH);
	as3310_writel(tm->tm_year,  HW_RTC_YEAR);
	return 0;
}

/* 
  *  Read the alram time.
  */
static int asm9260_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
     alm->time.tm_sec= as3310_readl(HW_RTC_ALSEC);
     alm->time.tm_min= as3310_readl(HW_RTC_ALMIN);
     alm->time.tm_hour=as3310_readl(HW_RTC_ALHOUR);
	alm->time.tm_mday=as3310_readl(HW_RTC_ALDOM);
	alm->time.tm_mon= as3310_readl(HW_RTC_ALMONTH);
	alm->time.tm_year=as3310_readl(HW_RTC_ALYEAR);
	alm->time.tm_wday=as3310_readl(HW_RTC_ALDOW);
	alm->time.tm_yday=as3310_readl(HW_RTC_ALDOY);
	return 0;
}

/* 
  *  Set the alram time.
  */
static int asm9260_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	as3310_writel(alm->time.tm_sec, HW_RTC_ALSEC);
	as3310_writel(alm->time.tm_min, HW_RTC_ALMIN);
	as3310_writel(alm->time.tm_hour,HW_RTC_ALHOUR);
	as3310_writel(alm->time.tm_mday,HW_RTC_ALDOM);
	as3310_writel(alm->time.tm_mon,HW_RTC_ALMONTH);
	as3310_writel(alm->time.tm_year,HW_RTC_ALYEAR);
	as3310_writel(alm->time.tm_yday,HW_RTC_ALDOY);
	as3310_writel(alm->time.tm_wday,HW_RTC_ALDOW);
	return 0;
}

/*
 *	/dev/rtc0 handling.
 */
static int asm9260_rtc_ioctl(struct device *dev, unsigned int cmd,
			      unsigned long arg)
{        
     struct rtc_time tm;
	struct rtc_wkalrm alm;
	struct rtc_cal rtccal;
	void __user *argp = (void __user *)arg;
	switch (cmd) {
		
	case RTC_SET_TIME:
		copy_from_user(&tm,argp,sizeof(struct rtc_time));
		asm9260_rtc_set_time(dev, &tm);
		break;

	case RTC_RD_TIME:
		asm9260_rtc_read_time(dev, &tm);
		copy_to_user(argp,&tm,sizeof(struct rtc_time));
		break;
		
	case RTC_ALM_SET:
		copy_from_user(&alm,argp,sizeof(struct rtc_wkalrm));
		asm9260_rtc_set_alarm(dev, &alm);
		break;

	case RTC_ALM_READ:		
		asm9260_rtc_read_alarm(dev, &alm);
		copy_to_user(argp,&alm,sizeof(struct rtc_wkalrm));
		break;
	
	case RTC_AIE_OFF:		
		as3310_writel(AMR_OFF, HW_RTC_AMR); 
		as3310_writel(ALARM_OFF, HW_RTC_ALARM);
		break;
		
     case RTC_AIE_ON: 
		as3310_writel(RTC_AMR_SEC|RTC_AMR_MIN|RTC_AMR_HOUR|RTC_AMR_DOM|RTC_AMR_MON|RTC_AMR_YEAR, HW_RTC_AMR); 
		as3310_writel(ALARM_SET, HW_RTC_ALARM);
		break;
		
	case RTC_UIE_OFF:
		as3310_writel(CIIR_OFF, HW_RTC_CIIR);
		break;
		
     case RTC_UIE_ON:
		as3310_writel(RTC_IT_SEC, HW_RTC_CIIR);
		as3310_writel( (as3310_readl(HW_RTC_CCR)|START_RTC_32K), HW_RTC_CCR);
		break;
		
	case RTC_PLL_SET:
		copy_from_user(&rtccal,argp,sizeof(struct rtc_cal));
		as3310_writel( (rtccal.value|(rtccal.direction)<<17), HW_RTC_CAL );  
		as3310_writel( (as3310_readl(HW_RTC_CCR)|CAL_ON), HW_RTC_CCR);
		break;
		
	default:
		return -ENOIOCTLCMD;
	} 

	return 0;
}

/* 
  *  RTC Interrupt Handle.
  */
static irqreturn_t rtc_irq(int irq, void *dev_id)
{
	u32 ILRreg;
	u32 events = 0;
	struct platform_device *pdev = to_platform_device(dev_id);
     struct rtc_device *rtc = platform_get_drvdata(pdev);
	ILRreg = as3310_readl(HW_RTC_ILR);

	if(ILRreg&TICK_IRQ)
	{
		printk(KERN_INFO"#");
		showtime();
		as3310_writel( CLEAR_TICK_IRQ, HW_RTC_ILR);
		events |= RTC_IRQF | RTC_UF;	 
	}
	
	if(ILRreg&ALARM_IRQ)
	{
		printk("\r\nalarm!\n");
		printk( "HW_RTC_ALARM: %d\n",as3310_readl(HW_RTC_ALARM));
		as3310_writel(CLEAR_ALARM_IRQ, HW_RTC_ILR);
		as3310_writel(as3310_readl(HW_RTC_ALARM) & CLEAR_ALARM_IRQ, HW_RTC_ALARM);
		events |= RTC_IRQF | RTC_AF;
	}
	
	if(events) {			   
		rtc_update_irq(rtc, RTC_IRQ_NUM, events);
	}
		
	return IRQ_HANDLED;
}

/*
 *    Open the RTC device.
 */
static int asm9260_rtc_open(struct device *dev)
{
	int r;  
	r = request_irq(asm9260_rtc_alarm, rtc_irq,
			IRQF_DISABLED, "ASM9260-RTC ", dev);
	if (r) {
		dev_err(dev, "Cannot claim IRQ%d\n", asm9260_rtc_alarm);
		goto fail;
	}
	return 0;
fail:
	free_irq(asm9260_rtc_alarm, dev);
	return r;
}


/*
 *	Initialize the RTC. 
 */
static int __devinit asm9260_rtc_enable(void)
{
	as3310_writel(1<<9, HW_AHBCLKCTRL1+4);
     as3310_writel(RTC_CCR_RESET0, HW_RTC_CCR); 
     delay(0x1000); 
     as3310_writel(RTC_CCR_RESET1, HW_RTC_CCR); 
	as3310_writel(AMR_OFF, HW_RTC_AMR);
	as3310_writel(CIIR_OFF, HW_RTC_CIIR);
     return 0;
}

static void asm9260_rtc_release(struct device *dev)
{
	free_irq(asm9260_rtc_alarm, dev);
}

/* Kernel interface. */
static struct rtc_class_ops asm9260_rtc_ops = {
	.open       = asm9260_rtc_open,
	.release    = asm9260_rtc_release,
	.ioctl		= asm9260_rtc_ioctl,
	.read_time	= asm9260_rtc_read_time,
	.set_time	= asm9260_rtc_set_time,
	.read_alarm	= asm9260_rtc_read_alarm,
	.set_alarm	= asm9260_rtc_set_alarm,
};

static int __devinit asm9260_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device	*rtc;

	asm9260_rtc_alarm = platform_get_irq(pdev, 0);
	if (asm9260_rtc_alarm <= 0) {
		rtc_dbg("%s: no alarm irq?\n", pdev->name);
		return -ENOENT;
	}
	
     asm9260_rtc_enable();
	as3310_writel(START_RTC_32K, HW_RTC_CCR);
	rtc = rtc_device_register(pdev->name, &pdev->dev,
			&asm9260_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		rtc_dbg("%s: can't register RTC device, err %ld\n",
			pdev->name, PTR_ERR(rtc));
		goto fail;
	}
	platform_set_drvdata(pdev, rtc);

	device_init_wakeup(&pdev->dev, 0);

	return 0;

fail :
	rtc_device_unregister(rtc);
	return -EIO;
}

static int __devexit asm9260_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device	*rtc = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);

	as3310_writel(CLOSE_RTC,HW_RTC_CCR);

	free_irq(asm9260_rtc_alarm, rtc);

	rtc_device_unregister(rtc);
	return 0;
}


#define asm9260_rtc_suspend NULL
#define asm9260_rtc_resume  NULL


static void asm9260_rtc_shutdown(struct platform_device *pdev)
{
	//nothing to do
}

MODULE_ALIAS("asm9260_rtc");
static struct platform_driver asm9260_rtc_driver = {
	.probe		= asm9260_rtc_probe,
	.remove		= __devexit_p(asm9260_rtc_remove),
	.suspend	= asm9260_rtc_suspend,
	.resume		= asm9260_rtc_resume,
	.shutdown	= asm9260_rtc_shutdown,
	.driver		= {
		.name	= "asm9260_rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init rtc_init(void)
{
	return platform_driver_register(&asm9260_rtc_driver);
}
module_init(rtc_init);

static void __exit rtc_exit(void)
{
	platform_driver_unregister(&asm9260_rtc_driver);
}
module_exit(rtc_exit);

MODULE_AUTHOR("AlpScale Inc");
MODULE_LICENSE("GPL");

