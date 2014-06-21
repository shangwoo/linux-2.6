/* linux/drivers/watchdog/asm9260_wdt.c
 *
 *
 * asm9260 Watchdog Support
 *
 * Based on, softdog.c by Alan Cox,
 * Revised by Shanjs2/12/2012
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

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

#define PFX "asm9260-wdt: "
#define WDMOD_WDINT 0x8
#define ICOLL_LEVELACK 0x1
#define WD_ICOLL_CLEAR1 0x00800000
#define WDMOD_INT 0x1
#define WDMOD_RESET 0x3
#define WDTCLKDIV 240
#define WDTCLK 2000000
#define WDFEED0 0xaa
#define WDFEED1 0x55

#define CONFIG_asm9260_WATCHDOG_ATBOOT		(0)
#define CONFIG_asm9260_WATCHDOG_DEFAULT_TIME	(15)
#define	WDIOC_SETMSTIMER	0xfedcba//_IOWR(WATCHDOG_IOCTL_BASE, 101, int)
#define	WDIOC_GETMSTIMER	0xfedcb9//_IOR(WATCHDOG_IOCTL_BASE, 102, int)

static int nowayout	= WATCHDOG_NOWAYOUT;
static int tmr_margin	= CONFIG_asm9260_WATCHDOG_DEFAULT_TIME;
static int tmr_atboot	= CONFIG_asm9260_WATCHDOG_ATBOOT;
static int soft_noboot = 0;/*watchdog mode  0:reset   1:irq*/
static int debug;

module_param(tmr_margin,  int, 0);
module_param(tmr_atboot,  int, 0);
module_param(nowayout,    int, 0);
module_param(soft_noboot, int, 0);
module_param(debug,	  int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. default="
		__MODULE_STRING(CONFIG_asm9260_WATCHDOG_DEFAULT_TIME) ")");
MODULE_PARM_DESC(tmr_atboot,
		"Watchdog is started at boot time if set to 1, default="
			__MODULE_STRING(CONFIG_asm9260_WATCHDOG_ATBOOT));
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
MODULE_PARM_DESC(soft_noboot, "Watchdog action, set to 1 to ignore reboots, 0 to reboot (default depends on ONLY_TESTING)");
MODULE_PARM_DESC(debug, "Watchdog debug, set to >1 for debug, (default 0)");


typedef enum close_state {
	CLOSE_STATE_NOT,
	CLOSE_STATE_ALLOW = 0x4021
} close_state_t;

static unsigned long open_lock;
static struct device    *wdt_dev;	/* platform device attached to */
static struct resource	*wdt_mem;
static struct resource	*wdt_irq;
static struct clk	*wdt_clock;
static void __iomem	*wdt_base;
static unsigned int	 wdt_count;
static close_state_t	 allow_close;
static struct timer_list wdt_timer;
static unsigned long asm9260wdt_mstimer = 0;
static DEFINE_SPINLOCK(wdt_lock);


/* Watchdog control routines. */

#define DBG(msg...) do { \
	if (debug) \
		printk(KERN_INFO msg); \
	} while (0)

static void delay(int delay)
{
	while (delay--);
}

/* 
  *  Keep the watchdog alive.
  */
static void asm9260wdt_keepalive(void)
{
     spin_lock(&wdt_lock);
	as3310_writel(wdt_count,HW_WATCHDOG_WDTC);
	as3310_writel(WDFEED0,HW_WATCHDOG_WDFEED);
	as3310_writel(WDFEED1,HW_WATCHDOG_WDFEED);
	spin_unlock(&wdt_lock);
}

static void __asm9260wdt_stop(void)
{
	as3310_writel(1<<26,HW_AHBCLKCTRL0+8);
	as3310_writel(1<<26,HW_PRESETCTRL0+8);  //reset watchdog
	delay(200);
	as3310_writel(1<<26,HW_PRESETCTRL0+4);  //clear watchdog reset
	delay(200);
     //   printk(KERN_INFO "STOP\n");
}

/* 
  *  Stop the watchdog. 
  */
static void asm9260wdt_stop(void)
{
	spin_lock(&wdt_lock);
	__asm9260wdt_stop();
	spin_unlock(&wdt_lock);
}

/* 
  *  Start the watchdog. 
  */
static void asm9260wdt_start(void)
{
	spin_lock(&wdt_lock);
	if (soft_noboot) {
		as3310_writel(WDMOD_INT,HW_WATCHDOG_WDMOD);//interrupt
	} else {
		as3310_writel(WDMOD_RESET,HW_WATCHDOG_WDMOD);//reset
	}
	
	DBG("%s: wdt_count=0x%08x\n",__func__, wdt_count);
	
	as3310_writel(wdt_count,HW_WATCHDOG_WDTC);
	delay(2000);
	as3310_writel(WDFEED0,HW_WATCHDOG_WDFEED);
	as3310_writel(WDFEED1,HW_WATCHDOG_WDFEED);
        	
	spin_unlock(&wdt_lock);
        
}

/*
  *  asm9260wdt_set_heartbeat:
  *  Initialize watchdog. 
  */
static int asm9260wdt_set_heartbeat(int timeout)
{
	if (timeout < 1)
		return -EINVAL;
     tmr_margin = timeout;
	wdt_count = timeout * WDTCLK;

	/* update the pre-scaler */
     as3310_writel(1<<26,HW_AHBCLKCTRL0+4);  
     delay(200);
     as3310_writel(1<<26,HW_PRESETCTRL0+8);  //reset watchdog
	delay(200);
	as3310_writel(1<<26,HW_PRESETCTRL0+4);  //clear watchdog reset
	delay(200);
       
	as3310_writel(1,HW_WDTCLKSEL);
	as3310_writel(0,HW_WDTCLKUEN);
	as3310_writel(1,HW_WDTCLKUEN);

	as3310_writel(WDTCLKDIV,HW_WDTCLKDIV);
	if (soft_noboot) {
		as3310_writel(WDMOD_INT,HW_WATCHDOG_WDMOD);//interrupt
	} else {
		as3310_writel(WDMOD_RESET,HW_WATCHDOG_WDMOD);//reset
	}
	delay(1000);

	return 0;
}

/*
 *    Open the watchdog device.
 */

static int asm9260wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &open_lock))
		return -EBUSY;

	if (nowayout)
		__module_get(THIS_MODULE);

	allow_close = CLOSE_STATE_NOT;
	
	/* start the timer */
	
	return nonseekable_open(inode, file);
}

static int asm9260wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *	Shut off the timer.
	 * 	Lock it in if it's a module and we set nowayout
	 */

	if (allow_close == CLOSE_STATE_ALLOW)
		asm9260wdt_stop();
	else {
		dev_err(wdt_dev, "Unexpected close, not stopping watchdog\n");
		asm9260wdt_keepalive();
	}
	
	if(asm9260wdt_mstimer!=0)
        del_timer(&wdt_timer);
	
	allow_close = CLOSE_STATE_NOT;
	clear_bit(0, &open_lock);
	return 0;
}

static ssize_t asm9260wdt_write(struct file *file, const char __user *data,
				size_t len, loff_t *ppos)
{
	/*
	 *	Refresh the timer.   nowayout
	 */
	if (len) {
		if (!0) {
			size_t i;

			/* In case it was set long ago */
			allow_close = CLOSE_STATE_NOT;

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					allow_close = CLOSE_STATE_ALLOW;
			}
		}
		asm9260wdt_keepalive();
	}
	return len;
}

#define OPTIONS WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE

static const struct watchdog_info asm9260_wdt_ident = {
	.options          =     OPTIONS,
	.firmware_version =	0,
	.identity         =	"asm9260 Watchdog",
};

/*
 *	/dev/watchdog handling.
 */
static long asm9260wdt_ioctl(struct file *file,	unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_margin;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &asm9260_wdt_ident,
			sizeof(asm9260_wdt_ident)) ? -EFAULT : 0;
	
	case WDIOC_SETMSTIMER:
		if(get_user(asm9260wdt_mstimer,p))
			return -EFAULT;
		if(asm9260wdt_mstimer==0){
			del_timer(&wdt_timer);
			asm9260wdt_keepalive();
		}
		else
		{
			/*asm9260wdt_mstimer always an integer multiple of 10ms*/
			wdt_timer.expires = jiffies + (asm9260wdt_mstimer*HZ)/1000;
			asm9260wdt_keepalive();
			add_timer(&wdt_timer);
		}
		return put_user(asm9260wdt_mstimer,p);
	
	case WDIOC_GETMSTIMER:
		return put_user(asm9260wdt_mstimer/1000,p);
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_SETOPTIONS:
		if (get_user(new_margin, p))
			return -EFAULT;
		if (new_margin & WDIOS_DISABLECARD)
			asm9260wdt_stop();
		if (new_margin & WDIOS_ENABLECARD)
			asm9260wdt_start();
		return 0;
	case WDIOC_KEEPALIVE:
		asm9260wdt_keepalive();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, p))
			return -EFAULT;
		if (asm9260wdt_set_heartbeat(new_margin))
			return -EINVAL;
		asm9260wdt_keepalive();
		return put_user(tmr_margin, p);
	case WDIOC_GETTIMEOUT:
		return put_user(tmr_margin, p);
	default:
		return -ENOTTY;
	}
}

/* Kernel interface. */

static const struct file_operations asm9260wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= asm9260wdt_write,
	.unlocked_ioctl	= asm9260wdt_ioctl,
	.open		= asm9260wdt_open,
	.release	= asm9260wdt_release,
};

static struct miscdevice asm9260wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &asm9260wdt_fops,
};

/* Interrupt handler code. */

static irqreturn_t asm9260wdt_irq(int irqno, void *param)
{
	dev_info(wdt_dev, "watchdog timer expired (irq)\n");
	u32 data;
	data = as3310_readl(HW_WATCHDOG_WDMOD);
	if(data&WDMOD_WDINT)
	{
		as3310_writel(WDMOD_WDINT,HW_WATCHDOG_WDMOD+8);
		as3310_writel(WD_ICOLL_CLEAR1,HW_ICOLL_CLEAR1);
		as3310_writel(0x00000000,HW_ICOLL_CLEAR1);
	}
	as3310_writel(ICOLL_LEVELACK,HW_ICOLL_LEVELACK);
//	asm9260wdt_keepalive();
	return IRQ_HANDLED;
}
/* Device interface. */

static void asm9260wdt_timer_function(ulong data)
{
	asm9260wdt_keepalive();
	mod_timer(&wdt_timer,jiffies + (asm9260wdt_mstimer*HZ)/1000);
}

static int asm9260wdt_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev;
	int started = 0;
	int ret;
	int size;
	DBG("%s: probe=%p\n", __func__, pdev);
    
	dev = &pdev->dev;
	wdt_dev = &pdev->dev;

	/* get the mem resource for the watchdog */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no memory resource specified\n");
		return -ENOENT;
	}
	
	size = (res->end - res->start) + 1;
	wdt_mem = request_mem_region(res->start, size, pdev->name);
	if (wdt_mem == NULL) {
		dev_err(dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	wdt_base = ioremap(res->start, size);
	if (wdt_base == NULL) {
		dev_err(dev, "failed to ioremap() region\n");
		ret = -EINVAL;
		goto err_req;
	}

	DBG("probe: mapped wdt_base=%p\n", wdt_base);

	/* get the irq resource for the watchdog */
	wdt_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (wdt_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_map;
	}

	ret = request_irq(wdt_irq->start, asm9260wdt_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_map;
	}

	/* see if we can actually set the requested timer margin, and if
	 * not, try the default value */

	if (asm9260wdt_set_heartbeat(tmr_margin)) {
		started = asm9260wdt_set_heartbeat(
					CONFIG_asm9260_WATCHDOG_DEFAULT_TIME);

		if (started == 0)
			dev_info(dev,
			   "tmr_margin value out of range, default %d used\n",
			       CONFIG_asm9260_WATCHDOG_DEFAULT_TIME);
		else
			dev_info(dev, "default timer value is out of range, cannot start\n");
	}

	ret = misc_register(&asm9260wdt_miscdev);
	if (ret) {
		dev_err(dev, "cannot register miscdev on minor=%d (%d)\n",
			WATCHDOG_MINOR, ret);
		goto err_clk;
	}

	init_timer(&wdt_timer);
	wdt_timer.function = asm9260wdt_timer_function;
	
	if (tmr_atboot && started == 0) {
		dev_info(dev, "starting watchdog timer\n");
		asm9260wdt_start();
	} else if (!tmr_atboot) {
		/* if we're not enabling the watchdog, then ensure it is
		 * disabled if it has been left running from the bootloader
		 * or other source */

		asm9260wdt_stop();
	}


	return 0;

 err_clk:
	clk_disable(wdt_clock);
	clk_put(wdt_clock);

 err_irq:
	free_irq(wdt_irq->start, pdev);

 err_map:
	iounmap(wdt_base);

 err_req:
	release_resource(wdt_mem);
	kfree(wdt_mem);

	return ret;
}

static int asm9260wdt_remove(struct platform_device *dev)
{
	release_resource(wdt_mem);
	kfree(wdt_mem);
	wdt_mem = NULL;

	free_irq(wdt_irq->start, dev);
	wdt_irq = NULL;

	clk_disable(wdt_clock);
	clk_put(wdt_clock);
	wdt_clock = NULL;

	iounmap(wdt_base);
	misc_deregister(&asm9260wdt_miscdev);
	return 0;
}

static void asm9260wdt_shutdown(struct platform_device *dev)
{
	asm9260wdt_stop();
}

#ifdef CONFIG_PM


static int asm9260wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int asm9260wdt_resume(struct platform_device *dev)
{
	/* Restore watchdog state. */
	return 0;
}

#else
#define asm9260wdt_suspend NULL
#define asm9260wdt_resume  NULL
#endif /* CONFIG_PM */


static struct platform_driver asm9260wdt_driver = {
	.probe		= asm9260wdt_probe,
	.remove		= asm9260wdt_remove,
	.shutdown	= asm9260wdt_shutdown,
	.suspend	= asm9260wdt_suspend,
	.resume		= asm9260wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "asm9260-wdt",
	},
};


static char banner[] __initdata =
	KERN_INFO "asm9260 Watchdog , (c) 2012 Simtec Electronics\n";

static int __init watchdog_init(void)
{
	printk(banner);
	return platform_driver_register(&asm9260wdt_driver);
}

static void __exit watchdog_exit(void)
{
	platform_driver_unregister(&asm9260wdt_driver);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("AlpScale Inc");
MODULE_DESCRIPTION("Asm9260 Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:asm9260-wdt");
