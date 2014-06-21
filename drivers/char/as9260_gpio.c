/*
 * GPIO functions support for ALPSCALE 9260 chip
 *
 * Copyright (c) 2014 ALPSCALE Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License, version
 * 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 *Note: This demo driver support GPIO output drive and GPIO Falling-Edge or 
 *      Rising-Edge interrupt trigger, but no more for the momnent. 
 *
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/utsname.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/times.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/major.h>
#include <asm/uaccess.h>

#include <mach/pincontrol.h>


#define DRIVER_NAME "alpscale-gpio"
#define DEV_NAME "gpio"


#define GPIO_IOC_MAGIC   'G'
#define IOCTL_GPIO_SETPINUX               _IOW(GPIO_IOC_MAGIC, 0, int)                   
#define IOCTL_GPIO_REVPINMUX              _IOW(GPIO_IOC_MAGIC, 1, int)
#define IOCTL_GPIO_SETVALUE               _IOW(GPIO_IOC_MAGIC, 2, int) 
#define IOCTL_GPIO_GETVALUE    		      _IOR(GPIO_IOC_MAGIC, 3, int)

#define IOCTL_GPIO_SETIRQ                 _IOW(GPIO_IOC_MAGIC, 4, int) 
#define IOCTL_GPIO_CLRIRQ    		      _IOR(GPIO_IOC_MAGIC, 5, int)


struct as9260_gpio_arg {
	int port;
    int pin;
	int data;
	int pinmuxback;
};


static dev_t devid;
static struct cdev cdev;
static struct class *class;
struct device *gpio_dev;


struct as9260_gpio_arg pargIRQ = {0xff, 0xff, 0, 0}; 

/********************************************************************/
static int device_open(struct inode *inode,struct file *filp)
{
    	return 0;
}

/********************************************************************/

static int device_release(struct inode *inode,struct file *filp)
{
	return 0;
}

static ssize_t device_read (struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 1;
}

static ssize_t device_write (struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 1;
}


static irqreturn_t gpio_irq_isr(int irq, void *Ptr)
{
    struct as9260_gpio_arg *pargPtr; 

    pargPtr = Ptr;
    if( (get_io_irq_status(pargPtr->port, pargPtr->pin)!=0)&&(pargPtr->port!=0xff)&&(pargPtr->pin!=0xff) ) {
        printk("pin[%d][%d] %s IRQ detected!\n", pargPtr->port, pargPtr->pin, (pargPtr->data==0)?"EDGE_RISING":"EDGE_FALLING" );
        io_irq_clr(pargPtr->port, pargPtr->pin);
        return IRQ_HANDLED;
    }else{
        printk("pin[%d][%d] suspicious IRQ detected!\n", pargPtr->port, pargPtr->pin);
		return IRQ_NONE;
    }
}


static int gpio_ioctl( struct inode *inode, struct file *file,
                unsigned int cmd, unsigned long arg)
{   
    struct as9260_gpio_arg parg; 
    int ret=0;

    if ( copy_from_user(&parg,
			   (struct as9260_gpio_arg __user *)arg,
			   sizeof(struct as9260_gpio_arg)) )
		return -EFAULT;

    if( !(parg.port>=0 && parg.port<=20)|| !(parg.pin>=0 && parg.pin<=7) ) {
        printk("Pin index invalid!\n");
        return -EINVAL;
    }


	switch (cmd){
	case IOCTL_GPIO_SETPINUX:
		parg.pinmuxback = get_pin_mux_val(parg.port, parg.pin);
        set_pin_mux(parg.port, parg.pin, 0);
		break;	
	case IOCTL_GPIO_REVPINMUX:
		set_pin_mux(parg.port, parg.pin, parg.pinmuxback);	
		break;
	case IOCTL_GPIO_SETVALUE:
        write_GPIO(parg.port, parg.pin, parg.data);
		break;
	case IOCTL_GPIO_GETVALUE:
		parg.data = read_GPIO(parg.port, parg.pin);
		break;
    case IOCTL_GPIO_SETIRQ:
        if( (pargIRQ.port==0xff)&&(pargIRQ.pin==0xff) ) 
        {
        
            pargIRQ = parg;
            ret = request_irq((INT_GPIO0+pargIRQ.port/4), gpio_irq_isr, 0, "GPIO-IRQ", &pargIRQ);     
            if(ret) {
                printk("IRQ-line-request failed!\n");
                pargIRQ.port = 0xff;            
                pargIRQ.pin = 0xff;
                return -EBUSY;
            }
            io_irq_enable_edge(pargIRQ.port, pargIRQ.pin, (pargIRQ.data==0)?GPIO_IRQ_EDGE_RISING:GPIO_IRQ_EDGE_FALLING);
        }
        else
        {
            printk("pin[%d][%d] IRQ pinned, please clear first!\n", pargIRQ.port, pargIRQ.pin);
            return -EACCES;
        }
        break;
    case IOCTL_GPIO_CLRIRQ:
        
        if( (pargIRQ.port==parg.port)&&(pargIRQ.pin==parg.pin) ){                       
            io_irq_disable(pargIRQ.port, pargIRQ.pin);
            free_irq( (INT_GPIO0+pargIRQ.port/4), &pargIRQ);
            pargIRQ.port = 0xff;
            pargIRQ.pin = 0xff;
        }
        break;

	default :
		printk("Unsuported Ioctl\n");
        return -ENOTTY;
	}

    if( (cmd == IOCTL_GPIO_SETPINUX)||(cmd == IOCTL_GPIO_GETVALUE ) ) {
        if( copy_to_user((struct as9260_gpio_arg __user *)arg, &parg, sizeof(struct as9260_gpio_arg)) )
            return -EFAULT;
    }

	return 0;
}

/*****************************File operations define************************/
static struct file_operations Fops = {
    .open		= device_open,			//open
    .release	= device_release,		//release
	.read		= device_read,
	.write		= device_write,
	.ioctl		= gpio_ioctl
};

/****************************Module initialize******************************/
static int __init gpio_init_module(void)
{
        int ret;

        /* Register the character device (atleast try) */
	    ret = alloc_chrdev_region(&devid, 0, 1, DRIVER_NAME);
 
    	if (ret < 0) {
        	printk("register char device region (%d:%d) failed (ret = %d)\n", MAJOR(devid), MINOR(devid), ret);
       		return ret;
    	}
 
    	cdev_init(&cdev, &Fops);
    	cdev.owner = THIS_MODULE;
    	ret = cdev_add(&cdev, devid, 1);
    	if (ret < 0) {
        	printk("register char device failed (ret = %d)\n", ret);
        	unregister_chrdev_region(devid, 1); 
    		return ret;
	    }

	class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(class)){
        	printk("class_create faild\n");
		cdev_del(&cdev);
		unregister_chrdev_region(devid, 1);
		return ret;
	}
        
	gpio_dev= device_create(class, NULL, devid, 0, DEV_NAME);

	return 0;
}

/****************************Module release********************************/
static void __exit gpio_cleanup_module(void)
{
    device_destroy(class, devid);
	class_destroy(class);
    cdev_del(&cdev);
	unregister_chrdev_region(devid, 1);     	
}

module_init(gpio_init_module);
module_exit(gpio_cleanup_module);

MODULE_DESCRIPTION("GPIO func-driver for Alpscale 9260 Platform");
MODULE_LICENSE("GPL");
