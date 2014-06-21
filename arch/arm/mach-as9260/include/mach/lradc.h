
#ifndef     __LRADC__H__
#define     __LRADC__H__

#define CLKCTRL_XTAL_LRADC  24

#define LRADC_CONTRL0_TD_ENABLE 20
#define LRADC_CONTRL1_TD_IRQ_EN 24

#define LRADC_CONTRL1_IRQ_EN_BIT    16

#define CHANNEL_ACCUMULATE  29
#define NUM_SAMPLES_BIT     24

#define DELAY_LOOPCNT_BIT   11
#define DELAY_KICK_BIT      20
#define DELAY_TRIGGER_DELAY_BIT 16
#define DELAY_TRIGGER_LRADC_BIT 24

#define LRADC_STATUS_TD_RAW 0

#define LRADC_CONTRL2_CH7_SEL   16
#define LRADC_CONTRL2_CH6_SEL   20
#define LRADC_CONTRL2_DIVIDE_BY_2   24

/* state machine */
//#define FINGER_PRESS_DOWN       0
#define FINGER_SLIDE_AROUND     1
#define FINGER_LEAVE_OFF        2

/* sample settings */
#define SAMPLE_NUMBER           4// sample number
#define SAPMLE_DELAY_TIME       0x50// delay time*0.5 ms times and do a sample

#define LRADC_USE_IRQ     0
#define LRADC_NOT_USE_IRQ   1
#define LRADC_STR_MAX_SIZE  16

#define LAUNCH_LRADC_CHANNEL(SCHEDULE_BIT)  do{ as3310_writel(SCHEDULE_BIT, HW_LRADC_CTRL0 + 4); } while(0)

/* abnormal return */
#define LRADC_OK                    6   // in china '6' stands for lucky
#define LRADC_REQ_CH_ERROR          -1
#define LRADC_REQ_SAMPLE_ERROR      -2
#define LRADC_REQ_DELAY_ERROR       -3
#define LRADC_REQ_FLAG_ERROR        -4
#define LRADC_SET_TIMER_ERROR       -5

#define DEVIDE_OPENED       1
#define DEVIDE_CLOSED       0

struct lradc_config
{
    char    sample_ch;  // exp: 00011100 means trigger ch 2~4 at the same time 
    char    divide_open; // exp: 00011000 means ch 2,3 open divide, ch 4 close divede.
};

void lradc_channel_setting(struct lradc_config * l);
int lradc_set_timer(struct  timer_list* lradc_timer, int jif_num, void* func);
int request_lradc_ch(struct lradc_config* conf, int flag);
int lradc_get_value(char channel);
void lradc_init(void);

#endif

