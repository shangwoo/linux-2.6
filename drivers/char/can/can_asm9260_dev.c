#include <linux/uaccess.h> 
#include <linux/io.h> 
#include <linux/kernel.h>
#include <linux/miscdevice.h> 
#include <linux/interrupt.h> 
#include <mach/irqs.h> 
#include <linux/fs.h>
//#include <linux/platform.h>
#include <mach/pincontrol.h>
#include "config.h"
#include "print_DEBUG.h"

stcASM9260_BufInfo ASM9260_RxBuf;
static DECLARE_WAIT_QUEUE_HEAD(can_wait);

static int can_open(struct inode *p_inode, struct file *p_file)
{
	if(TRUE != ASM9260Init(PELI_CAN, 0x00037f00, 0x00000000, 0xFFFFFFFF))
	{
		printk("<0>ASM9260 Init Error!\n");
		return -ENODEV;
	}
      
       printk("<0>" DEV_NAME " Opened!\n");
	return(0);
}

static int can_release(struct inode *p_inode, struct file *p_flie)
{
	printk("ASM9260 released!\n");
	return(0);
}

static ssize_t can_read(struct file *p_flie, char *p_buf, size_t count, loff_t *f_pos)
{
	interruptible_sleep_on(&can_wait);
	if(count != sizeof(stcASM9260_BufInfo))
	{
		return (-EINVAL);
	}
	if(copy_to_user(p_buf, (void*)(&ASM9260_RxBuf), sizeof(stcASM9260_BufInfo)))
	{
		printk("Copy to usr Error!\n");
		return -EFAULT;
	}
	return(sizeof(ASM9260_RxBuf));
}

static ssize_t can_write(struct file *p_flie, const char *p_buf, size_t count, loff_t *f_pos)
{
	stcASM9260_BufInfo TxBuf;
	if(count != sizeof(stcASM9260_BufInfo))
	{
		printk("count = %d not %d\n", count, sizeof(stcASM9260_BufInfo));
		return (-EINVAL);
	}
	if(copy_from_user((void*)&TxBuf, (void*)p_buf, sizeof(stcASM9260_BufInfo)))
	{
		printk("copy_from_user Error!\n");
		return -EFAULT;
	}
	SetTxBuf(&TxBuf);
	SetCommand(CMR_NOM_SD);
        as3310_writeb(0,0x80050000+IR);

    return(sizeof(ASM9260_RxBuf));
}

 uint32 param;
static int can_ioctl(struct inode *p_inode, struct file *p_flie, unsigned int cmd, unsigned long arg)
{
	int val = 0;
        void __user *argp = (void __user *)arg;
	int __user *p = argp;
        get_user(param ,p);

    switch(cmd)
	{
		case IOCTL_BAUD:
			asm_SoftRst(TRUE);
                        Write_ASM9260(0x01,MOD);
			if(TRUE != SetBaudRate((uint32)param)) {
				val = -ENOTTY;

			}
			asm_SoftRst(FALSE);
			break;

		case IOCTL_ACR:
			asm_SoftRst(TRUE);
			if(TRUE != ACRCode(param)) {
				val = -ENOTTY;
			}
			asm_SoftRst(FALSE);
			break;

		case IOCTL_AMR:
			asm_SoftRst(TRUE);
			if(TRUE != AMRCode(param)) {
				val = -ENOTTY;
			}
			asm_SoftRst(FALSE);
			break;

		default:
			val = -ENOTTY;
			break;
	}

    return(val);
}

static irqreturn_t can_interrupt(int irq , void* dev_id, struct pt_regs *regs)
{
       
         INT8U Temp = Read_ASM9260(IR);
         as3310_writeb(0,0x80050000+IR); 
	 IntEntry(Temp);
	 wake_up_interruptible(&can_wait);

     return IRQ_HANDLED;
}

static const struct file_operations asm9260_can_fops = 
{
	.owner	 = THIS_MODULE,
	.ioctl	 = can_ioctl,
	.open	 = can_open,
	.write	 = can_write,
	.read	 = can_read,
	.release = can_release,
		
};

static struct miscdevice can_miscdev =
{
	 .minor	= MISC_DYNAMIC_MINOR,
	 .name	= DEV_NAME,
	 .fops	= &asm9260_can_fops
};

static int can_probe(struct platform_device *dev)
{
	int ret;

	printk("probing "DEV_NAME"!\n");
	ret = misc_register(&can_miscdev);
	if (ret)
		printk("Failed to register miscdev for "DEV_NAME".\n");

	return ret;
}

static int can_remove(struct platform_device *dev)
{
	misc_deregister(&can_miscdev);
	printk(DEV_NAME " removed!\n");
	
	return 0;
}

struct platform_device *can_device;

static struct platform_driver can_driver = {
    .driver = {
        .name    = DEV_NAME,
        .owner   = THIS_MODULE,
    },
    .probe   = can_probe,
    .remove  = can_remove,
};

static int __init can_init(void)
{
	int i, rc;
    set_pin_mux(10,5,7);
    set_pin_mux(10,6,7);

	printk(DEV_NAME" init......\n");

        if (request_irq(57, can_interrupt, IRQF_DISABLED|IRQF_TRIGGER_FALLING,	"asm9260", NULL))
		printk(KERN_WARNING "SmartARM3250: Failed to get 57 IRQ!\n");
        as3310_writel(1<<28,HW_PRESETCTRL0+8);      //reset CAN1 module;
        as3310_writel(1<<27, HW_PRESETCTRL0+4); 
        as3310_writel(1<<28, HW_PRESETCTRL0+4);
        as3310_writel(1<<27, HW_AHBCLKCTRL0+4);
        as3310_writel(1<<28, HW_AHBCLKCTRL0+4);  
        as3310_writel(0x20000038, HW_ICOLL_VBASE);
        as3310_writel(0x00070000, HW_ICOLL_CTRL); 
        as3310_writel(0X00000400, HW_ICOLL_PRIORITY14);
      
       can_device = platform_device_alloc(DEV_NAME, -1);
    if (!can_device)
        return -ENOMEM;


	rc = platform_device_add(can_device);
    if (rc < 0) {
        platform_device_put(can_device);
        return rc;
    }

	rc = platform_driver_register(&can_driver);
    if (rc < 0)
        platform_device_unregister(can_device);

         Write_ASM9260(0x4f,IER);   
          Write_ASM9260(0x01,MOD); 
          Write_ASM9260(0x02,OCR);
          Write_ASM9260(0x04,MOD);
    return rc;	
}

static void __exit can_exit(void)
{
	//关闭CAN设备控制器     
	if(TRUE != asm_SoftRst(TRUE)) {
		printk("<0>" "SJA_SoftRst Failed!.\n");
	}   
	
	free_irq(57,NULL);
    platform_driver_unregister(&can_driver);
    platform_device_unregister(can_device);
    printk(DEV_NAME" exit!\n");
}

module_init(can_init);
module_exit(can_exit);

MODULE_AUTHOR("zixin");
MODULE_DESCRIPTION(" asm9260 can Driver");
MODULE_LICENSE("GPL");

