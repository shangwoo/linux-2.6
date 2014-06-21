
/****************************************************************
 *  linux/include/asm-arm/arch-as3310/keypad.h
 *
 *  Copyright (C) 2007 Alpscale Corporation.
 *  Created by zhy
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *****************************************************************/
#ifdef CONFIG_KEYBOARD_AS3310

#ifdef CONFIG_KEYBOARD_AS3310_10
#define KEYBOARD_AS3310_INPUT 3
#define KEYBOARD_AS3310_OUTPUT 3
#endif

#ifdef CONFIG_KEYBOARD_AS3310_64
#define KEYBOARD_AS3310_INPUT 8
#define KEYBOARD_AS3310_OUTPUT 8
#endif

#define COL KEYBOARD_AS3310_INPUT
#define ROW KEYBOARD_AS3310_OUTPUT
struct as3310_kp_gpios_row{
    int port;
    int pin;
};

struct as3310_kp_gpios_col{
    int port;
    int pin;
};

struct as3310_kp_special_key{
    int port;
    int pin;
    int value;
    int state;
    int state_new;
    int state_count;
};

struct as3310_kp_platform_data {

	int *keymap;
	unsigned int keymapsize;
	unsigned long delay;
	struct as3310_kp_gpios_row row_gpios[ROW];
	struct as3310_kp_gpios_col col_gpios[COL];
    struct as3310_kp_special_key special_key;
};

void update_state_new(void);
void scan_state(void);
void init_as3310_kp_gpio_irq(void);
void kp_io_irq_unmask(void);
void kp_io_irq_clr(void);



#endif


//core.c
///*****************************************************************
// * as3310_keypad
// *****************************************************************/
//
//
////key position
////row0(O0)----0-------1-------2-------3
////            |       |       |       |  
////            |       |       |       |
////row1(O1)----4-------5-------6-------7
////            |       |       |       |
////            |       |       |       |
////row2(O2)----8-------9-------10------11
////            |       |       |       |
////            |       |       |       |
////      col0(I0) col1(I1) col2(I2)   col3(I3)
//     
//#ifdef CONFIG_KEYBOARD_AD3310
//static struct resource kp_resources[] = {
//	[0] = {
//		.start	= INT_AS3310_GPIO0,
//		.end	= INT_AS3310_GPIO3,
//		.flags	= IORESOURCE_IRQ,
//	},
//};
//
////static int keypad_keycode[] = {
////    KEY_UP         ,KEY_LEFT            ,KEY_RESTART        ,
////    KEY_RIGHT      ,KEY_DOWN            ,KEY_F2             ,
////    KEY_A          ,KEY_B               ,KEY_F1
////};
////
//
//static int keypad_keycode[] = {
//KEY_ESC             ,KEY_1               ,KEY_2               ,KEY_3               ,
//KEY_4               ,KEY_5               ,KEY_6               ,KEY_7               ,
//KEY_8               ,KEY_9               ,KEY_0               ,KEY_MINUS           ,
//KEY_EQUAL           ,KEY_BACKSPACE       ,KEY_TAB             ,KEY_Q               ,
//KEY_W               ,KEY_E               ,KEY_R               ,KEY_T               ,
//KEY_Y               ,KEY_U               ,KEY_I               ,KEY_O               ,
//KEY_P               ,KEY_LEFTBRACE       ,KEY_RIGHTBRACE      ,KEY_ENTER           ,
//KEY_LEFTCTRL        ,KEY_A               ,KEY_S               ,KEY_D               ,
//KEY_F               ,KEY_G               ,KEY_H               ,KEY_J               ,
//KEY_K               ,KEY_L               ,KEY_SEMICOLON       ,KEY_APOSTROPHE      ,
//KEY_GRAVE           ,KEY_LEFTSHIFT       ,KEY_BACKSLASH       ,KEY_Z               ,
//KEY_X               ,KEY_C               ,KEY_V               ,KEY_B               ,
//KEY_N               ,KEY_M               ,KEY_COMMA           ,KEY_DOT             ,
//KEY_SLASH           ,KEY_RIGHTSHIFT      ,KEY_KPASTERISK      ,KEY_LEFTALT         ,
//KEY_SPACE           ,KEY_CAPSLOCK        ,KEY_F1              ,KEY_F2              ,
//KEY_F3              ,KEY_F4              ,KEY_F5              ,KEY_F6              
//};
//
//
//static struct as3310_kp_platform_data kp_data = {
//
//	.keymap		= keypad_keycode,
//	.keymapsize	= ARRAY_SIZE(keypad_keycode),
//	.delay		= 5,
//    .row_gpios  = {{0,8}, {0,9},{0,10},{0,11},{0,12},{0,13},{0,14},{0,15}},
//    .col_gpios  = {{3,15}, {3,16}, {3,14},{0,13},{1,28},{1,29},{1,30},{1,31}},
//};
//
//
//static struct platform_device kp_device = {
//    .name		= "as3310-keypad",
//	.id		= -1,
//	.dev		= {
//		.platform_data = &kp_data,
//	},
//	.num_resources	= ARRAY_SIZE(kp_resources),
//	.resource	= kp_resources,
//};
//
//#endif
///*****************************************************************
// * as3310_keypad
// *****************************************************************/
