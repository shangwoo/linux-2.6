#ifndef __LRADC_KEYPAD__H
#define __LRADC_KEYPAD__H

/* lradc keypad settings */
#define KP_LRADC_SAMPLE_NUMBER      16
#define KP_LRADC_SAPMLE_DELAY_TIME  0x1// delay 1*0.5 = 0.5ms times and do a sample



#define CHANNEL_N   8
#define COLUM_N     11

#define COMPARE_TIMES   5   // compare 2 times

#define DELAY_BETWEEN_SAMP     2   // 2 * 10 = 10 ms
/**************** state flag *****************/
#define NON_TOUCHED           0
#define ALREAD_PRESSED        1

/* in which status, the key message is sent to app */
#define ON_BOTH_STATUS  3
#define ON_PRESS_DOWN   2
#define ON_RELEASE_OFF  1    

/****  sample value ******/
#define NO_PRESS    0x3ff
#define KEY_AD2_CH  2
#define KEY_AD3_CH  3
#define KEY_AD4_CH  4

#ifdef CONFIG_BOARD_AS3310_DEV  
#define KEY3_6_9_VL     0x42    
#define KEY4_7_10_VL    0x109   
#define KEY5_8_12_VL    0x1b1   
#endif

#ifdef CONFIG_EVB_0612 // 1820 develop board
//#define KEY3_6_9_VL     0x175
//#define KEY4_7_10_VL    0x5ef
//#define KEY5_8_12_VL    0x9a8
#define KEY_1_VL 0x556
#define KEY_2_VL 0x150
#define KEY_3_VL 0x657
#define KEY_4_VL 0x350
#define KEY_5_VL 0x4c3
#define KEY_6_VL 0x5e0
#define KEY_7_VL 0x6ca
#define KEY_8_VL 0x720
#define KEY_9_VL 0x775
#define KEY_10_VL 0x417
#define KEY_11_VL  0x262
#endif 

#ifdef CONFIG_EVB_0601 // 1826 develop board
#define KEY_1_VL 0xa
#define KEY_2_VL 0x70
#define KEY_3_VL 0xe4
#define KEY_4_VL 0x147
#define KEY_5_VL 0x1d5
#define KEY_6_VL 0x20f
#define KEY_7_VL 0x293
#endif 

#ifdef CONFIG_AS2808 // as 2808
#define KEY3_6_9_VL     0x135   
#define KEY4_7_10_VL    0x5c0  
#define KEY5_8_12_VL    0x9f0  
#endif 

#ifdef CONFIG_AS3008 // as 3008
#define KEY3_6_9_VL     0x190  
#define KEY4_7_10_VL    0x610  
#define KEY5_8_12_VL    0x9f0  
#endif 

#define LRADC_KEY_CMP_SHORT(x, y)  ( ( ((x)-(y))>=0?((x)-(y)):((y)-(x)) ) < 0x20 )

struct as3310_lradc_kp_platform_data {

	int *keymap;
    int delay;
	unsigned int keymapsize;
	struct lradc_config* lradc_config;
    int *channel_colum_num;
    int *colum_vol;
};

struct as3310_lradc_kp{
   struct input_dev *input;
   int delay;
   char keypad_state[CHANNEL_N][COLUM_N];
   char comp_times[CHANNEL_N]; // for eliminate Doudong
   int former_key[CHANNEL_N];
     
   short keypad_vol[CHANNEL_N];
   int *keymap;
   int *channel_colum_num;
   int *colum_vol;
   struct lradc_config* lradc_config;
   struct  timer_list  kp_lradc_timer;
};




#endif

