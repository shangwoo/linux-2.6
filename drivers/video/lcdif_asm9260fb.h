/*
 * linux/drivers/asm9260fb.h
 * Copyright (c) He Yong
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    ASM9260 LCD Controller Frame Buffer Driver
 *	    based on s3c2410fb.c skeletonfb.c, sa1100fb.h
 *
 * ChangeLog
 *
 * 2004-12-04: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Moved dprintk to s3c2410fb.c
 *2008-08-25:zhangyb
 *          clean up
 */

#ifndef __as3310fb_H
#define __as3310fb_H
#define  CONFIG_FB_EVK_480_272 

#define _rgb_max_x 240
#define _rgb_max_y 320

#ifdef CONFIG_FB_SAMSUNG_LVT350Q_F04
    #define SCREEN_SIZE_COL 320
    #define SCREEN_SIZE_ROW 240
#endif

#ifdef CONFIG_FB_EVK_480_272
   #define SCREEN_SIZE_COL _rgb_max_x
   #define SCREEN_SIZE_ROW _rgb_max_y 
#endif

#ifdef CONFIG_FB_FTG500C06Z_800_480
    #define SCREEN_SIZE_COL 800
    #define SCREEN_SIZE_ROW 480
#endif

#ifdef CONFIG_FB_ILI9320RGB
    #ifdef CONFIG_ILI9320RGB_ROTATE_TO_240_320
        #define SCREEN_SIZE_COL 240
        #define SCREEN_SIZE_ROW 320
    #else
        #define SCREEN_SIZE_COL 320
        #define SCREEN_SIZE_ROW 240
    #endif
#endif


#ifdef CONFIG_FB_VIRTUAL_VGA
    #define START_COL   0  /*start point for direct copy*/
    #define START_ROW   0
    #define SCREEN_SIZE_COL 320
    #define SCREEN_SIZE_ROW 240
    #define AS3310_LCD_XRES 640
    #define AS3310_LCD_YRES 480
#else

    #ifdef CONFIG_FB_EVK_480_272
        #define AS3310_LCD_XRES 240
        #define AS3310_LCD_YRES 320
    #else
        #ifdef CONFIG_FB_FTG500C06Z_800_480
            #define AS3310_LCD_XRES 800
            #define AS3310_LCD_YRES 480
        #else
            #define AS3310_LCD_XRES SCREEN_SIZE_COL
            #define AS3310_LCD_YRES SCREEN_SIZE_ROW
        #endif
    #endif

#endif

#define LCD_MODE_RGB  		0
#define LCD_MODE_YUV  		1
#define LCD_FIFO_PRIV_HIGH  16
#define LCD_FIFO_PRIV_LOW   17

#define LCD_CH_NUM_Y		(3)//APBH-LCD CHANNEL 3
#define LCD_CH_NUM_U		(4)//APBH-LCD CHANNEL 4
#define LCD_CH_NUM_V		(5)//APBH-LCD CHANNEL 5

#define AS3310_DEFAULT_HCLK     60    /* 90 MHz */
#define AS3310_DEFAULT_FRAME_RATE  30    /* 60 Hz */
#define LCD_DEFAULT_PIXCLK_DIVIDER ((int)(AS3310_DEFAULT_HCLK*1000000 / (AS3310_DEFAULT_FRAME_RATE*AS3310_LCD_XRES*AS3310_LCD_YRES*1.1)))

#define FRAMEBUFFER_PHY_ADDR 0x20f00000
#define FRAMEBUFFER_VIRT_ADDR 0xe0f00000
#define FRAMEBUFFER_SIZE (AS3310_LCD_XRES*AS3310_LCD_YRES*2)

#define LCD_PKG_NUM  24
#define LCD_TIME_OUT 24  // 12 for 48Hz
#define LCD_PKG_NUM_Y		(12)
#define LCD_PKG_NUM_U		(12)
#define LCD_PKG_NUM_V		(12)


#define AS3310_LCD_PALETTE_NUM 256


#define FBIOBLANK		0x4611		/* arg: 0 or vesa level + 1 */
#define FBIO_RGB		0x3301	   
#define FBIO_YUV		0x3302	   
#define FBIO_SYNC	            0x3303	   
#define FBIO_TIMER_FLUSH_ON     0x3304	   
#define FBIO_TIMER_FLUSH_OFF    0x3305	
#define FBIO_OFF		0x3306	   
#define FBIO_ON		    0x3307 
#define FBIO_GET_DISP_MODE      0x3308    // FBIO_RGB  , FBIO_YUV   


//define RGB mode parameter
#define AS3310_LCD_R_OFFSET 11 /*565 mode R at bit 5*/
#define AS3310_LCD_G_OFFSET 5  /*565 mode G at bit 5*/
#define AS3310_LCD_B_OFFSET 0  /*565 mode B at bit 0*/
#define AS3310_LCD_R_LEN    5  /*565  red length*/
#define AS3310_LCD_G_LEN    6  /*565  green length*/                                
#define AS3310_LCD_B_LEN    5  /*565  blue length*/


#define ASAP1826_LCD_R_OFFSET 16 /*888 mode R at bit 5*/
#define ASAP1826_LCD_G_OFFSET 8 /*888 mode G at bit 5*/
#define ASAP1826_LCD_B_OFFSET 0  /*888 mode B at bit 0*/
#define ASAP1826_LCD_R_LEN    8  /*888  red length*/
#define ASAP1826_LCD_G_LEN    8  /*888  green length*/                                
#define ASAP1826_LCD_B_LEN    8  /*888  blue length*/

#define SZ_2K 0x800

/*TIMING2*/
#define LCD_IPC_EDGE_FALL 22   /*reverse pixel clock*/
#define LCD_HSYNC_ACTIVE_LOW 21    /*reverse HSYNC*/
#define LCD_VSYNC_ACTIVE_LOW 20    /*reverse VSYNC*/
/*CTRL1*/
#define LCD_CTRL_LCDEN 0       /*LCD Controller enable*/
#define LCD_CTRL_LCDTFT 7      /*LCD TFT*/
#define LCD_PALETTE_LOAD 1
#define LCD_DATA_LOAD 2

struct as3310fb_info {
    struct fb_info      *fb;
    struct device       *dev;
    struct clk      *clk;

    struct as3310fb_mach_info *mach_info;
     /* 1, auto refresh screen by timer; 0, manually refresh screen */
    volatile int timer_flushing;
    volatile int frame_delay_output;
    /* auto refresh timer */
    struct timer_list lcdif_timer;
    /* raw memory addresses */
    dma_addr_t      map_dma;    /* physical */
    u_char *        map_cpu;    /* virtual */
    u_int           map_size;
    dma_addr_t      map_dma1;   /* physical */
    u_char *        map_cpu1;   /* virtual */
    u_int           map_size1;
    int             framerate;  /* 60-75 Hz */

    struct as3310_dma_chain * dmachain;
    struct as3310_dma_chain * dmachain_y;
    struct as3310_dma_chain * dmachain_u;
    struct as3310_dma_chain * dmachain_v;

    struct as3310fb_hw  regs;

    /* addresses of pieces placed in raw buffer */
    u_char *        screen_cpu; /* virtual address of buffer */
    dma_addr_t      screen_dma; /* physical address of buffer */
    unsigned int        palette_ready;

    /* keep these registers in case we need to re-write palette */
    u32         palette_buffer[AS3310_LCD_PALETTE_NUM];
    u32         pseudo_pal[16];

};


struct as3310_lcd_table {
    u32     table[15];
    u32     dma_addr;
};

#define DMA_XFER_COUNT          0x9600   /*half QVGA*/
#define LCDIF_CTRLC_VALUE       0xc0000000
#define LCDIF_CTRL_VALUE        (0x21170000)|((int)DMA_XFER_COUNT)
#define LCDIF_TIMING_VALUE      0x0a030a03
#define HW_PRESETCTRL0        0x80040000
#define HW_PRESETCTRL1        0x80040010
#define HW_AHBCLKCTRL0        0x80040020
#define HW_AHBCLKCTRL1        0x80040030
#define HW_LCDCLKDIV          0x800401fc

#define PALETTE_BUFF_CLEAR (0x80000000)	/* entry is clear/invalid */

#define AS3310_TFTPAL(x) IO_ADDRESS(HW_LCDIF_CTRL + (x<<4))

int as3310fb_init(void);
void  as3310_lcd_refresh(struct as3310fb_info *fbi);

#endif

