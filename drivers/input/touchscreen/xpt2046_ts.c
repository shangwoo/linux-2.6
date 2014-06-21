/*
 * Support for ALPSCALE XPT2046 Touch-Screen interface.
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
 */ 
 
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/kthread.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/pincontrol.h>
#include <mach/xpt2046_ts.h>


/*AXIS SMAPLE value range*/
#define XPT2046_ABS_X_MIN 300
#define XPT2046_ABS_X_MAX 3800
#define XPT2046_ABS_Y_MIN 300
#define XPT2046_ABS_Y_MAX 3700
#define XPT2046_PRESSURE_MIN		0
#define XPT2046_PRESSURE_MAX		300



static char *as9260xpt2046ts_name = "XPT2046";

struct as9260_xpt2046_ts {
    struct input_dev *dev;
    struct timer_list xpt2046_timer;
    wait_queue_head_t waiter;
    struct task_struct *thread;
    int tpd_flag; /* 0, not position processing; 1, do position processing.*/
    struct spi_device *SPI_device;
    struct xpt2046_hw_info *info;
    int xp;
    int yp;
    int pressure;
};


union {
    u8 val[4];
    u32 dummy[CACHE_LINE_BYTES/sizeof(u32)];
} __attribute__ ((aligned(CACHE_LINE_BYTES)))  xy_val;

static int xpt2046_ts_raw_xy(struct as9260_xpt2046_ts *as9260xptptr, int xycmd)
{
	ssize_t retval;
	u8 code = xycmd;

    
    /*must be 4 bytes aligned.*/
    if( (u32)(&xy_val.val)&0x03 ) {
        dev_err(&as9260xptptr->dev->dev, "%s not 4 bytes aligned.\n", __func__);
        return -EINVAL;
    }

	retval = spi_write_then_read(as9260xptptr->SPI_device, &code, 1, &xy_val.val[0], 4);

	if (retval < 0) {
		dev_err(&as9260xptptr->dev->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return (((xy_val.val[0]<<8)|xy_val.val[1])>>3);
}

int xpt2046_ts_get_pos(struct as9260_xpt2046_ts *as9260xptptr)
{
   int x_val,y_val;
   int z1,z2,pressure;
   const int Rxpanel = 100, pressureCoe = 300;
    

   y_val = xpt2046_ts_raw_xy(as9260xptptr, 0x90);

   if(y_val<0) {
       return -EIO;
   }
   as9260xptptr->yp = y_val;

   x_val = xpt2046_ts_raw_xy(as9260xptptr, 0xd0);

   if(x_val<0) {
       return -EIO;
   }
   as9260xptptr->xp = x_val;
   
   z1 = xpt2046_ts_raw_xy(as9260xptptr, 0xb0);
   if(z1<=0) {
       return -EAGAIN;
   }

   z2 = xpt2046_ts_raw_xy(as9260xptptr, 0xc0);
   if(z2<=0) {
       return -EAGAIN;
   }

   pressure = ((int)(z2*Rxpanel/z1-Rxpanel)*x_val)/4096;
    
   if(pressure > pressureCoe) {
       pressure = 299;
   }

   as9260xptptr->pressure = pressureCoe - pressure;

   return 0;
}

void xpt2046_report_value(struct as9260_xpt2046_ts *as9260xptptr)
{
            input_report_key(as9260xptptr->dev, BTN_TOUCH, 1);
            input_report_abs(as9260xptptr->dev, ABS_X, as9260xptptr->xp);
            input_report_abs(as9260xptptr->dev, ABS_Y, as9260xptptr->yp);              
            input_report_abs(as9260xptptr->dev, ABS_PRESSURE, as9260xptptr->pressure);
            input_sync(as9260xptptr->dev);

            //printk("x: %d, y: %d, p:%d\n", as9260xptptr->xp, as9260xptptr->yp, as9260xptptr->pressure);
            //printk("xpt2046_report_value %p -----------------\n", as9260xptptr->dev);
}

static int xpt2046_touch_event_handler(void *used)
{  
     int ret = -1; 
	 struct as9260_xpt2046_ts *as9260xptptr = (struct as9260_xpt2046_ts *)used;

	 
     struct sched_param param = { .sched_priority = 1 };
	 sched_setscheduler(current, SCHED_FIFO, &param);

     do{
		 set_current_state(TASK_INTERRUPTIBLE); 
		 wait_event_interruptible(as9260xptptr->waiter, as9260xptptr->tpd_flag!=0);
						 
		 as9260xptptr->tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);

         ret = xpt2046_ts_get_pos(as9260xptptr);	
		 if (ret == 0) {	
			xpt2046_report_value(as9260xptptr);
		 }else if(ret == -EAGAIN ){
             ;//just ignore
         }else{
             printk("XPT2046 SPI IO error!\n");
         }
     }while(!kthread_should_stop());

     return 0;
}


static void xpt2046_ts(ulong data)
{
  static int TouchDetect = 0;
  struct as9260_xpt2046_ts *as9260xptptr = (struct as9260_xpt2046_ts *)data;

  if( (read_GPIO(as9260xptptr->info->penIRQport, as9260xptptr->info->penIRQpin) == 0)&&(as9260xptptr->tpd_flag == 0) ) {
      TouchDetect = 1;
      as9260xptptr->tpd_flag = 1;
      wake_up_interruptible(&as9260xptptr->waiter);

  }else if( (read_GPIO(as9260xptptr->info->penIRQport, as9260xptptr->info->penIRQpin) == 1)&&(TouchDetect == 1) ){
          TouchDetect = 0;  
          as9260xptptr->tpd_flag = 0;
          input_report_key(as9260xptptr->dev, BTN_TOUCH, 0);
          input_report_abs(as9260xptptr->dev, ABS_PRESSURE, 0);
          input_sync(as9260xptptr->dev); 
          //printk("touch over!\n\n");    


  }

  mod_timer(&as9260xptptr->xpt2046_timer, jiffies + HZ/as9260xptptr->info->sample_ratio_HZ);
  return;
}

static void __init xpt2046ts_init_timer(struct timer_list *timer)
{
    struct as9260_xpt2046_ts *as9260xptptr;

    as9260xptptr =  container_of(timer, struct as9260_xpt2046_ts, xpt2046_timer);
    init_timer(timer);
    timer->function = xpt2046_ts;
    timer->data = (ulong)as9260xptptr;
    timer->expires = jiffies + HZ/as9260xptptr->info->sample_ratio_HZ;
    add_timer(timer);
}



#if 0
static irqreturn_t ts_irq(int irq, void *dev_id)
{
     u32 lradc_ctrl0_reg, lradc_ctrl1_reg;

     return IRQ_HANDLED;
}
#endif


static int __init xpt2046ts_probe(struct device *dev)
{     
     struct as9260_xpt2046_ts *as9260xptptr;
     struct xpt2046_hw_info *xpt2046_spi_info_ptr;
     struct spi_master *spi_master_ptr;
     struct spi_device *spi_device_ptr;           
     int err = 0;
   

     as9260xptptr = kzalloc(sizeof(struct as9260_xpt2046_ts), GFP_KERNEL);
	 if (!as9260xptptr)	{
        printk(KERN_ERR "as9260xptptr alloc failed!\n");
		err = -ENOMEM;
        goto exit_as9260xpt_failed;	
	 }

     xpt2046_spi_info_ptr = ( struct xpt2046_hw_info *)dev->platform_data;
     if(xpt2046_spi_info_ptr == NULL) {
         printk(KERN_ERR "Hm... too bad : no platform data for xpt2046\n");
         err = -EINVAL;
         goto exit_hwinfo_failed;
     }else{  
         as9260xptptr->info = xpt2046_spi_info_ptr;
         dev_set_drvdata(dev, as9260xptptr);
     }

     /*get SPI master*/
    spi_master_ptr = spi_busnum_to_master(as9260xptptr->info->SPI_bus_num);       
    if(spi_master_ptr == NULL) {
        printk(KERN_ERR "xpt2046 SPI bus not bound!\n");
        err = -EINVAL;
        goto exit_getmaster_failed;
    }else{
        spi_device_ptr = spi_alloc_device(spi_master_ptr);

        if(spi_device_ptr == NULL) {
            printk(KERN_ERR "xpt2046 SPI device failed!\n");
            err = -ENOMEM;
            goto exit_spialloc_failed;
        }else{
             as9260xptptr->SPI_device = spi_device_ptr;
             as9260xptptr->SPI_device->max_speed_hz = as9260xptptr->info->SPI_clk;
             as9260xptptr->SPI_device->bits_per_word = 8;
             strlcpy(as9260xptptr->SPI_device->modalias, as9260xpt2046ts_name, sizeof(as9260xptptr->SPI_device->modalias));
        }       
    }
   
    as9260xptptr->dev = input_allocate_device();
    if (!as9260xptptr->dev)
    {
        printk(KERN_ERR "%s:%s allocate_device\n", __FUNCTION__, __FILE__);
        err = -ENOMEM;
        goto exit_inputalloc_failed;
    }    
    as9260xptptr->dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    as9260xptptr->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    input_set_abs_params(as9260xptptr->dev, ABS_X, XPT2046_ABS_X_MIN, XPT2046_ABS_X_MAX, 0, 0);
    input_set_abs_params(as9260xptptr->dev, ABS_Y, XPT2046_ABS_Y_MIN, XPT2046_ABS_Y_MAX, 0, 0);
    input_set_abs_params(as9260xptptr->dev, ABS_PRESSURE, XPT2046_PRESSURE_MIN, XPT2046_PRESSURE_MAX, 0, 0);
    as9260xptptr->dev->name = as9260xpt2046ts_name;
    as9260xptptr->tpd_flag = 0;

    /*PEN IRQ pin setting*/
    set_pin_mux(as9260xptptr->info->penIRQport, as9260xptptr->info->penIRQpin, 0);
    #if 0
    if(request_irq(ADC_IRQ, ts_irq, 0, "as9260_action", ts.dev))
    {
        printk(KERN_ERR "as9260_ts.c:can't get assigned irq %d\n",ADC_IRQ);
        free_irq(ADC_IRQ, ts.dev);
        return -EIO;
    }
    #endif
    

    /* All went ok, so register to the input system */
    input_register_device(as9260xptptr->dev);

    //printk("xpt2046ts_probe %p --------------\n", as9260xptptr->dev);

    //the deamons
    init_waitqueue_head(&as9260xptptr->waiter);
	as9260xptptr->thread = kthread_run(xpt2046_touch_event_handler, as9260xptptr, as9260xpt2046ts_name);
	if (IS_ERR(as9260xptptr->thread))
	{ 
   	  err = PTR_ERR(as9260xptptr->thread);
   	  printk("XPT2046 failed to create kernel thread.\n");	
      goto exit_kthreadrun_failed;  
	} 

    xpt2046ts_init_timer(&as9260xptptr->xpt2046_timer);

    printk(KERN_INFO "%s successfully registered\n", as9260xpt2046ts_name);

    return 0;

exit_kthreadrun_failed:
        input_unregister_device(as9260xptptr->dev);

exit_inputalloc_failed:
        spi_dev_put(as9260xptptr->SPI_device);

exit_spialloc_failed:
        ;
exit_getmaster_failed:
        dev_set_drvdata(dev,NULL);

exit_hwinfo_failed:
        kfree(as9260xptptr);

exit_as9260xpt_failed:
        ;

    return err; 
}

static int xpt2046ts_remove(struct device *dev)
{
    struct as9260_xpt2046_ts *as9260xptptr;


    as9260xptptr = dev_get_drvdata(dev);
    kthread_stop(as9260xptptr->thread);
    input_unregister_device(as9260xptptr->dev);
    spi_dev_put(as9260xptptr->SPI_device);
    dev_set_drvdata(dev,NULL);
    kfree(as9260xptptr);
    #if 0
	free_irq(ADC_IRQ, ts.dev);
    #endif


    return 0;
}

static struct device_driver xpt2046ts_driver = {
    .name = "xpt2046ts",
    .bus = &platform_bus_type,
    .probe = xpt2046ts_probe,
    .remove = xpt2046ts_remove,
};


int __init xpt2046ts_init(void)
{
    return driver_register(&xpt2046ts_driver);
}

void __exit xpt2046ts_exit(void)
{
    driver_unregister(&xpt2046ts_driver);
}

late_initcall(xpt2046ts_init);
module_exit(xpt2046ts_exit);


MODULE_DESCRIPTION("as9260 touchscreen driver");
MODULE_LICENSE("GPL");
