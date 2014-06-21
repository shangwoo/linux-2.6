

#ifndef __AS9260_TS_H__
#define __AS9260_TS_H__



/* register bit fields */
#define LRADC_CTRL0_TD_ENABLE       20

#define LRADC_CTRL0_X_EN            0xa0000
#define LRADC_CTRL0_Y_EN            0x50000

#define LRADC_CTRL1_TD_IRQ          8
#define LRADC_CTRL1_CH_IRQ_EN       16
#define LRADC_CTRL1_TD_IRQ_EN       24

#define LRADC_STATUS_TD_RAW         0




#define _set_bit(addr,n)     as3310_writel((0x00000001<<n),addr+4)
#define _clear_bit(addr,n)   as3310_writel((0x00000001<<n),addr+8)

//#define _check_bit(addr,n)   ((*(volatile unsigned long*)IO_ADDRESS(addr)&(0x00000001<<n))!=0)

/* irq number */
#define ADC_IRQ 33



#define ERR -1

#define TS_MAJOR 119
#define TS_MINOR 1

#define TS_NAME "as9260ts"

/* Lradc config */
#define TS_LRADC_X_CH      0x4
#define TS_LRADC_Y_CH      0x8

#define TS_LRADC_CH                     0x3c    
#define TS_LRADC_SAMPLE_NUMBER          16
#define TS_LRADC_SAPMLE_DELAY_TIME      1      // 1 * 0.5 = 0.5 ms

/* sample frequency, which is TS_SAMPLE_RATIO per HZ*/
#define TS_SAMPLE_RATIO 10

/*AXIS SMAPLE value range*/
#define TS_ABS_X_MIN 50
#define TS_ABS_X_MAX 2000
#define TS_ABS_Y_MIN 100
#define TS_ABS_Y_MAX 1950

/* touch screen state */
#define STATE_UP    0
#define STATE_DOWN  1

#define COMPARE_TWO_TS_VALUE(x, y)  ((((x) - (y))>0?((x) - (y)):((y) - (x))) < 0x100)

void lradc_ts_init(void);
void disable_irq_ts(void);





struct as9260_ts_mach_info {
    struct lradc_config* ts_lradc_config;
    int sample_ratio_HZ; //jiffy period = HZ/sample_ratio_HZ
};

void __init set_as3310ts_info(struct as9260_ts_mach_info *hard_as3310ts_info);

#endif

