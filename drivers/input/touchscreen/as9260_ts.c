/*
 * Support for ALPSCALE 9260 LRADC Touch-Screen interface.
 *
 * Copyright (c) 2013 ALPSCALE Corporation.
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

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/lradc.h>
#include <mach/as9260_ts.h>


/* For ts.dev.id.version */
#define as9260tsVERSION 0x0101


#define DEBUG_LVL KERN_INFO

#ifdef CONFIG_TOUCHSCREEN_as9260_DEBUG
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

/*
* Definitions & global arrays.input_event
*/


static char *as9260ts_name = TS_NAME;

/*
* Per-touchscreen data.
*/
static struct timer_list ts_timer;
struct as9260_ts_mach_info *as9260_ts_config;

struct as9260ts {
    struct input_dev *dev;
    long xp;    /* ch 2  */
    long yp;    /* ch 3  */
    long xn;    /* ch 4  */
    long yn;    /* ch 5  */
    int state;
    int in_process; /* 0 or 1 */
    int start_sample; /*0 or 1*/
};


static struct as9260ts ts;


void disable_irq_ts(void)
{
	    as3310_writel(0x013c0000, HW_LRADC_CTRL1+8);   /* disable the interrupt */
        as3310_writel(0x001f0000, HW_LRADC_CTRL0+8); 
}  



void start_touch_detect(void)
{
            as3310_writel(0x3c0000, HW_LRADC_CTRL1 + 8);
            as3310_writel(0xf0000, HW_LRADC_CTRL0 + 8);            
            as3310_writel(0x00100000,HW_LRADC_CTRL0+4);//enable touch_detect

            mdelay(2);
            as3310_clear_bit(HW_LRADC_CTRL1, LRADC_CTRL1_TD_IRQ);
            as3310_set_bit(HW_LRADC_CTRL1, LRADC_CTRL1_TD_IRQ_EN);//clr irq_en and hw_en   
}

void start_axis_sample(void)
{
           /*trigger the next conversion!*/
           as3310_writel(0x00100000,HW_LRADC_CTRL0 + 8);//clr ts_detect_en
           as3310_writel(LRADC_CTRL0_X_EN, HW_LRADC_CTRL0 + 8);
           as3310_writel(LRADC_CTRL0_Y_EN, HW_LRADC_CTRL0 + 4);//set y+,y- drive

           as3310_writel(0x40000, HW_LRADC_CTRL1+4);
           as3310_writel(0x80000, HW_LRADC_CTRL1+8);
           
           as3310_clear_bit(HW_LRADC_CTRL1, LRADC_CTRL1_TD_IRQ_EN);//clr irq_en and hw_en

           mdelay(2);
           as3310_writel(0, HW_LRADC_CH2);
           as9260_ts_config->ts_lradc_config->sample_ch = TS_LRADC_X_CH;
           as3310_writel(0x04, HW_LRADC_CTRL0+4);
}

void lradc_ts_init(void)
{       
        

        as3310_writel(1<<11, HW_AHBCLKCTRL1+4);
        as3310_writel(20, HW_ADCCLKDIV);

        as3310_writel(as3310_readl(HW_PDRUNCFG)&0xffffffaf, HW_PDRUNCFG);


        as3310_writel(0xc0000000, HW_LRADC_CTRL0_CLR);
        as3310_writel(0x00a00000, HW_LRADC_CTRL3_SET);
        
        as3310_writel(1<<2, HW_LRADC_CTRL4_SET);
        mdelay(1);
        as3310_writel(1<<1, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<14, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<14, HW_LRADC_CTRL4_CLR);
        as3310_writel(1<<15, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<15, HW_LRADC_CTRL4_CLR);
        as3310_writel(1<<11, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<11, HW_LRADC_CTRL4_CLR);

        while( as3310_readl(HW_LRADC_STATUS)&(1<<10) ) {
            ;
        }

        as3310_writel(0x40<<16, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<13, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<13, HW_LRADC_CTRL4_CLR);

        mdelay(1);
        as3310_writel(3<<8, HW_LRADC_CTRL4_SET);
        as3310_writel(1<<10, HW_LRADC_CTRL4_SET);


        /*start analog touch detect*/
        start_touch_detect();
}

static void as9260_ts(ulong data){

if(ts.start_sample == 1) {

  if(ts.state == STATE_DOWN){  
     if( ts.in_process == 0 ) {
         //sanity check
         if( !(ts.xp <TS_ABS_X_MIN || ts.xp>TS_ABS_X_MAX || ts.yp<TS_ABS_Y_MIN|| ts.yp>TS_ABS_Y_MAX) ) {
         
             input_report_key(ts.dev, BTN_TOUCH, 1);
             input_report_abs(ts.dev, ABS_X, ts.xp);
             input_report_abs(ts.dev, ABS_Y, ts.yp);  
             input_sync(ts.dev);
             //printk("x: %d, y:%d\n", ts.xp, ts.yp);

             start_axis_sample();
         }
     }else{
         ;//wait for the current conversion finish
     }
  }else{
      ;//wait for first conversion finish or next touch;
  }

}else{
    ;//wait for first touch
}

  mod_timer(&ts_timer, jiffies + HZ/as9260_ts_config->sample_ratio_HZ);

}

static void __init as9260_ts_init_timer(void){

    init_timer(&ts_timer);
    ts_timer.function = as9260_ts;
    ts_timer.expires = jiffies + HZ/as9260_ts_config->sample_ratio_HZ;
    ts.state = STATE_UP;
    add_timer(&ts_timer);
}


static irqreturn_t ts_action(int irq, void *dev_id)
{
//printk("2ts_irq ctrl0: %8x ctrl1: %8x status: %8x\n", as3310_readl(HW_LRADC_CTRL0), as3310_readl(HW_LRADC_CTRL1), as3310_readl(HW_LRADC_STATUS));
       
as3310_clear_bit(HW_LRADC_CTRL1, LRADC_CTRL1_TD_IRQ); 

      if(as3310_readl(HW_LRADC_STATUS) & (1 << LRADC_STATUS_TD_RAW))
       {         
            start_axis_sample();
       }else{
            ts.state = STATE_UP;

            input_report_key(ts.dev, BTN_TOUCH, 0);
            input_sync(ts.dev);

            //printk("4ts_irq ctrl0: %8x ctrl1: %8x status: %8x\n", as3310_readl(HW_LRADC_CTRL0), as3310_readl(HW_LRADC_CTRL1), as3310_readl(HW_LRADC_STATUS));
            start_touch_detect();
       }

       
       return IRQ_HANDLED;    
}


static irqreturn_t ts_getlable(int irq, void *dev_id)
{
        int ch, positive = 0, negative = 0 ;
        int tmp;
        u32 lradc_ctrl1_reg, lradc_irq_status;



        lradc_ctrl1_reg = as3310_readl(HW_LRADC_CTRL1);
        lradc_irq_status = as3310_readl(HW_LRADC_CTRL1)&0x1ff;
        
        as3310_writel(lradc_irq_status, HW_LRADC_CTRL1+8); 

//printk("5ts_irq ctrl0: %8x ctrl1: %8x status: %8x\n", as3310_readl(HW_LRADC_CTRL0), as3310_readl(HW_LRADC_CTRL1), as3310_readl(HW_LRADC_STATUS));

        ts.in_process = 1;
        for(ch = 2;ch < 6;ch++){
            if((1 << ch) & as9260_ts_config->ts_lradc_config->sample_ch)
            {
               // printk("ch: %d\n", ch);

                tmp = lradc_get_value(ch);
                if (ch >= 4) 
                    negative = tmp;
                else
                    positive = tmp;
               
            }
        }
          switch( (lradc_ctrl1_reg&0xc0000)>>16 ){
        
            case 4:
               
                        ts.yp = positive;

                        //printk("ts.yp: %d, Ts.xp: %d\n", ts.yp, lradc_get_value(3));
               
                        as3310_writel(LRADC_CTRL0_Y_EN, HW_LRADC_CTRL0 + 8);
                        as3310_writel(LRADC_CTRL0_X_EN, HW_LRADC_CTRL0 + 4);//set x+,x- drive                              
                        

                        as3310_clear_bit(HW_LRADC_CTRL1, LRADC_CTRL1_TD_IRQ_EN);//clr irq_en and hw_en
                        as3310_writel(0x40000, HW_LRADC_CTRL1+8);
                        as3310_writel(0x80000, HW_LRADC_CTRL1+4);

                                             
                        mdelay(2);
                        as9260_ts_config->ts_lradc_config->sample_ch = TS_LRADC_Y_CH;
                        as3310_writel(0, HW_LRADC_CH3);
                        as3310_writel(0x08, HW_LRADC_CTRL0+4);
                        break;            
            case 8:
               
                    ts.xp = positive;

                    //printk("ts.xp: %d, Ts.yp: %d\n", ts.xp, lradc_get_value(2));

                   
                    if(ts.state == STATE_UP)
                    {
                        //printk("ts.x:y=%d:%d\n",ts.xp,ts.yp);
                       
                        ts.state = STATE_DOWN;
                    }

                    goto samplefinish;

            default:
                printk(KERN_ERR "error!\n");
                return 1;
        }

        
          
        return IRQ_HANDLED;

        samplefinish:
            start_touch_detect();           
            ts.in_process = 0;

        return IRQ_HANDLED;       
}



static irqreturn_t ts_irq(int irq, void *dev_id)
{
     u32 lradc_ctrl0_reg, lradc_ctrl1_reg;


     as3310_writel(as3310_readl(HW_LRADC_CTRL1)&0x1ff, HW_LRADC_CTRL1+8);
     as3310_writel(1<<24, HW_LRADC_CTRL1+8);


     lradc_ctrl0_reg = as3310_readl(HW_LRADC_CTRL0);
     lradc_ctrl1_reg = as3310_readl(HW_LRADC_CTRL1);

//     printk("1ts_irq ctrl0: %8x ctrl1: %8x status: %8x\n", as3310_readl(HW_LRADC_CTRL0), lradc_ctrl1_reg, as3310_readl(HW_LRADC_STATUS));

     if(ts.start_sample == 0 ) {
         ts.start_sample = 1 ;
     }

     if( (lradc_ctrl1_reg&(1<<LRADC_CTRL1_TD_IRQ_EN))||(lradc_ctrl0_reg&(1<<LRADC_CTRL0_TD_ENABLE)) ) {
            ts_action(irq, dev_id);
     }else if( lradc_ctrl1_reg&(0x0c<<LRADC_CTRL1_CH_IRQ_EN) ){
            ts_getlable(irq, dev_id);
     }
     else
         ;
     return IRQ_HANDLED;
}



static int __init as9260ts_probe(struct device *dev)
{   

    as9260_ts_config = ( struct as9260_ts_mach_info *)dev->platform_data;

    if (!as9260_ts_config)
    {
       printk(KERN_ERR "Hm... too bad : no platform data for ts\n");
       return -EINVAL;
    }

    #ifdef CONFIG_TOUCHSCREEN_as9260_DEBUG
        printk(DEBUG_LVL "Entering as9260ts_init\n");
    #endif
        
    memset(&ts, 0, sizeof(struct as9260ts));
    ts.dev = input_allocate_device();
    if (!ts.dev)
    {
        printk(KERN_ERR "%s:%s allocate_device\n", __FUNCTION__, __FILE__);
        return -ENOMEM;
    }   

    

    ts.dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    ts.dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    input_set_abs_params(ts.dev, ABS_X, TS_ABS_X_MIN, TS_ABS_X_MAX, 0, 0);
    input_set_abs_params(ts.dev, ABS_Y, TS_ABS_Y_MIN, TS_ABS_Y_MAX, 0, 0);
    
//    ts.dev->private = &ts;
    ts.dev->name = as9260ts_name;
    ts.dev->id.bustype = BUS_RS232;
    ts.dev->id.vendor = 0xDEAD;
    ts.dev->id.product = 0xBEEF;
    ts.dev->id.version = as9260tsVERSION;
    ts.state = STATE_UP;
    ts.in_process = 0;
    ts.start_sample = 0;

    if(request_irq(ADC_IRQ, ts_irq, 0, "as9260_action", ts.dev))
    {
        printk(KERN_ERR "as9260_ts.c:can't get assigned irq %d\n",ADC_IRQ);
        free_irq(ADC_IRQ, ts.dev);
        return -EIO;
    }
   

    printk(KERN_INFO "%s successfully loaded\n", as9260ts_name);
   

    /* All went ok, so register to the input system */
    input_register_device(ts.dev);


    as9260_ts_init_timer();
    lradc_ts_init(); 

    return 0;
}

static int as9260ts_remove(struct device *dev)
{
	disable_irq_ts();
	free_irq(ADC_IRQ, ts.dev);
    input_unregister_device(ts.dev);

    return 0;
}

static struct device_driver as9260ts_driver = {
    .name = TS_NAME,
    .bus = &platform_bus_type,
    .probe = as9260ts_probe,
    .remove = as9260ts_remove,
};


int __init as9260ts_init(void)
{
    return driver_register(&as9260ts_driver);
}

void __exit as9260ts_exit(void)
{
    driver_unregister(&as9260ts_driver);
}

module_init(as9260ts_init);
module_exit(as9260ts_exit);


MODULE_DESCRIPTION("as9260 touchscreen driver");
MODULE_LICENSE("GPL");
