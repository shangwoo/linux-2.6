
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <mach/lradc.h>
#include <asm/io.h>


static int delay_reg_now;


int lradc_set_timer(struct  timer_list* lradc_timer, int jif_num, void* func)
{
    if (jif_num < 0 || func == NULL)
        return LRADC_SET_TIMER_ERROR;
    init_timer(lradc_timer);
    lradc_timer->expires = jiffies + jif_num;
    lradc_timer->function = (void*)func;     
    add_timer(lradc_timer);
    return LRADC_OK;
}


void lradc_channel_setting(struct lradc_config * l)
{
    int n;
    char ch,d_open;

    ch = l->sample_ch;
    d_open = l->divide_open;


    for (n = 0;n < 8;n++)
    {
        if (ch & (1 << n))
        {

            as3310_clear_bit(HW_LRADC_CTRL1, n);//clr irq 
            as3310_writel(0x0003ffff, HW_LRADC_CH0 + n*0x10 + 8);      //clr result
            as3310_clear_bit(HW_LRADC_CH0 + n*0x10, CHANNEL_ACCUMULATE);   //clr acc

            if (d_open & (1 << n))
            {
                as3310_writel(1 << (n + LRADC_CONTRL2_DIVIDE_BY_2), HW_LRADC_CTRL2 + 4);
            }

            if (n == 7) // if battery channel, divide_by_2 should be set
            {

                as3310_writel(1 << (7 + LRADC_CONTRL2_DIVIDE_BY_2), HW_LRADC_CTRL2 + 4);
            }

        }
    }  

}

int request_lradc_ch(struct lradc_config* conf, int flag)
{

    lradc_channel_setting(conf);


    switch (flag)
    {
    
    case LRADC_USE_IRQ:
        as3310_writel((conf->sample_ch) << LRADC_CONTRL1_IRQ_EN_BIT, HW_LRADC_CTRL1 + 4);
        break;
    case LRADC_NOT_USE_IRQ:
        as3310_writel((conf->sample_ch) << LRADC_CONTRL1_IRQ_EN_BIT, HW_LRADC_CTRL1 + 8);
        break;
    default:
        return LRADC_REQ_FLAG_ERROR;

    }
    return LRADC_OK;

}



/* arg channel is a concrete number for lradc channel, such as 6, 7 , 
different from sample_ch of struct lradc_config */

int lradc_get_value(char channel)
{
    int value;
    value = as3310_readl(HW_LRADC_CH0 + 0x10 * channel);
    value &= 0x0003ffff;
    return value;
}




void lradc_init(void)
{
    delay_reg_now = 0;  // USE delay0 register at first

}





