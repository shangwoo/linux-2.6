/*
 * linux/drivers/asm9260fb.h
 * Copyright (c) Shanjs
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    ASM9260 LCD Controller Frame Buffer Driver
 *	    based on skeletonfb.c, sa1100fb.h
 *
 * ChangeLog
 *
 * 2012-12-05: Shanjs <shanjs@alpscale.cn>
 *        Create File
 *
 */

#ifndef __ASM9260FB_H
#define __ASM9260FB_H

#ifdef CONFIG_FB_SAMSUNG_LVT350Q_F04
    #define SCREEN_SIZE_COL 320
    #define SCREEN_SIZE_ROW 240
#endif

#ifdef CONFIG_FB_EVK_480_272
   #define SCREEN_SIZE_COL 480
   #define SCREEN_SIZE_ROW 272 
   #define LCD_TIMING0_RGB 0x400b1000
   #define LCD_TIMING1_RGB 0x090d1000
   #define LCD_TIMING2_RGB (0xFF000000 + 10)
   #define LCD_TIMING0_YUV 0x20091000
   #define LCD_TIMING1_YUV 0x203d1000
   #define LCD_TIMING2_YUV (0xFF000000 + 10)
#endif

#ifdef CONFIG_FB_FTG500C06Z_800_480
    #define SCREEN_SIZE_COL 800
    #define SCREEN_SIZE_ROW 480
    #define LCD_TIMING0_RGB 0x52521000
    #define LCD_TIMING1_RGB 0x121d1000
    #define LCD_TIMING2_RGB (0xFE000000 + 4)
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
    #define ASM9260_LCD_XRES 640
    #define ASM9260_LCD_YRES 480
#else

    #ifdef CONFIG_FB_EVK_480_272
        #define ASM9260_LCD_XRES 480
        #define ASM9260_LCD_YRES 272
    #else
        #ifdef CONFIG_FB_FTG500C06Z_800_480
            #define ASM9260_LCD_XRES 800
            #define ASM9260_LCD_YRES 480
        #else
            #define ASM9260_LCD_XRES SCREEN_SIZE_COL
            #define ASM9260_LCD_YRES SCREEN_SIZE_ROW
        #endif
    #endif

#endif
#ifdef CONFIG_FB_LCD_BPP_16
	#define LCD_BPP 2
	#define LCD_CTRL1 0xfc800000
	#define LCD_TABLE 0x00004000
#else
	#define LCD_BPP 4
	#define LCD_CTRL1 0xfc000000
	#define LCD_TABLE 0x00005000
#endif

#define _rgb_max_x 480
#define _rgb_max_y 272
#define LCD_MODE_RGB  	0
#define LCD_MODE_YUV  	1
#define LCD_FIFO_PRIV_HIGH  16
#define LCD_FIFO_PRIV_LOW   17

#define LCD_CH_NUM_Y		(3)//APBH-LCD CHANNEL 3
#define LCD_CH_NUM_U		(4)//APBH-LCD CHANNEL 4
#define LCD_CH_NUM_V		(5)//APBH-LCD CHANNEL 5

#define ASM9260_DEFAULT_HCLK     60    /* 90 MHz */
#define ASM9260_DEFAULT_FRAME_RATE  30    /* 60 Hz */
#define LCD_DEFAULT_PIXCLK_DIVIDER ((int)(ASM9260_DEFAULT_HCLK*1000000 / (ASM9260_DEFAULT_FRAME_RATE*ASM9260_LCD_XRES*ASM9260_LCD_YRES*1.1)))

#define FRAMEBUFFER_PHY_ADDR 0x20f00000
#define FRAMEBUFFER_VIRT_ADDR 0xe0f00000
#define FRAMEBUFFER_SIZE (ASM9260_LCD_XRES*ASM9260_LCD_YRES*2)

#define LCD_PKG_NUM  481
#define LCD_TIME_OUT 24  // 12 for 48Hz
#define LCD_PKG_NUM_Y		(480)
#define LCD_PKG_NUM_U		(240)
#define LCD_PKG_NUM_V		(240)
#define LCD_ON  0
#define LCD_OFF 1
#define LCD_DBG_ON  1
#define LCD_DBG_OFF 0
#define RGB_BLANK 0xFF
#define Y_BLANK 0x10
#define UV_BLANK 0x80
#define LCD_CH0_SEMA 1
#define LCD_CH1_SEMA 1
#define LCD_CH2_SEMA 1
#define LCD_BURST_CLEAR 0x03000000
#define LCD_BURST_MODE 0x02000000
#define CYCLE_MODE_RGB 0x04000000
#define CYCLE_MODE_YUV 0x3C000000
#define LCD_CTRL0_GATE 0x80000000
#define LCD_CTRL0_RESET 0x40000000
#define APBH_CTRL0_GATE 0x40000000
#define APBH_CTRL0_RESET 0x80000000
#define APBH_CHAN_GATE 0x00000700
#define LOWTHESHOLD 128
#define UPTHESHOLD 384
#define LCD_CTRL0 0x30000010
#define LCD_DMA_CYCLE_EN 0x20000000
#define DMA_CH_CYCLE 0x1C000000 
#define APBH_CH0_CMD_TABLE 0x002000c2 
#define LCD_CH0_CTRL0_TABLE 0x40000000 
#define APBH_CH0_CMD 0x00001086
#define APBH_CH1_CMD 0x00001086
#define APBH_CH2_CMD 0x00001086
#define APBH_CH0_CMD_LAST 0x000010c2
#define APBH_CH1_CMD_LAST 0x000010c2
#define APBH_CH2_CMD_LAST 0x000010c2
#define LCD_CH0_CTRL0 0x2c030000
#define LCD_CH0_CTRL0_Y 0x25830000 
#define LCD_CH_STARTF 0x08000000

#define LCD_CTRL_STAT 0x00000040
#define LCD_DMA_TIMEOUT 0x800000
#define LCDCLKDIV 4
#define ASM9260_LCD_PALETTE_NUM 256
#define LCD_REQ_MEM 0xc0100000


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
#define ASM9260_565_R_OFFSET 11 /*565 mode R at bit 5*/
#define ASM9260_565_G_OFFSET 5  /*565 mode G at bit 5*/
#define ASM9260_565_B_OFFSET 0  /*565 mode B at bit 0*/
#define ASM9260_565_R_LEN    5  /*565  red length*/
#define ASM9260_565_G_LEN    6  /*565  green length*/                                
#define ASM9260_565_B_LEN    5  /*565  blue length*/


#define ASM9260_888_R_OFFSET 16 /*888 mode R at bit 5*/
#define ASM9260_888_G_OFFSET 8 /*888 mode G at bit 5*/
#define ASM9260_888_B_OFFSET 0  /*888 mode B at bit 0*/
#define ASM9260_888_R_LEN    8  /*888  red length*/
#define ASM9260_888_G_LEN    8  /*888  green length*/                                
#define ASM9260_888_B_LEN    8  /*888  blue length*/

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

#define	RGB888(r,g,b)	((u32)( \
								(((u8)(r)) << 16) \
							|	(((u8)(g)) << 8) \
							|	((u8)(b)) \
						))
#define	RGB565(r, g, b)			((u16)(	\
									(((r) & 0xF8) << 8) \
									| (((g) & 0xFC) << 3) \
									| (((b) & 0xF8) >> 3) \
									))
#define rgb2yuv(R,G,B,Y,U,V) {\
	Y = (unsigned char) (( (   66 * R + 129 * G +	25 * B + 128) >> 8) +	16); \
	U = (unsigned char) (( ( -38 * R -	 74 * G + 112 * B + 128) >> 8) + 128); \
	V = (unsigned char) (( ( 112 * R -	 94 * G -	18 * B + 128) >> 8) + 128); \
	}

struct asm9260fb_info {
    struct fb_info      *fb;
    struct device       *dev;
    struct clk      *clk;

    struct asm9260fb_mach_info *mach_info;

    /* raw memory addresses */
    dma_addr_t      map_dma;    /* physical */
    u_char *        map_cpu;    /* virtual */
    u_int           map_size;
    dma_addr_t      map_dma1;   /* physical */
    u_char *        map_cpu1;   /* virtual */
    u_int           map_size1;
    int             framerate;  /* 60-75 Hz */

    struct asm9260_dma_chain * dmachain;
    struct asm9260_dma_chain * dmachain_y;
    struct asm9260_dma_chain * dmachain_u;
    struct asm9260_dma_chain * dmachain_v;

    struct asm9260fb_hw  regs;

    /* addresses of pieces placed in raw buffer */
    u_char *        screen_cpu; /* virtual address of buffer */
    dma_addr_t      screen_dma; /* physical address of buffer */
    unsigned int        palette_ready;

    /* keep these registers in case we need to re-write palette */
    u32         palette_buffer[ASM9260_LCD_PALETTE_NUM];
    u32         pseudo_pal[16];

};


struct asm9260_lcd_table {
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
#define HW_LCD_CTRL_BASE			(0x80800000+0x4000) 
#define	HW_LCD_CTRL_CTRL0			(HW_LCD_CTRL_BASE)
#define	HW_LCD_CTRL_CTRL1			(HW_LCD_CTRL_BASE+0x10)
#define	HW_LCD_CTRL_Y_RGB_DATA		(HW_LCD_CTRL_BASE+0x20)
#define	HW_LCD_CTRL_TIMING0			(HW_LCD_CTRL_BASE+0x30)
#define	HW_LCD_CTRL_TIMING1			(HW_LCD_CTRL_BASE+0x40)
#define	HW_LCD_CTRL_TIMING2			(HW_LCD_CTRL_BASE+0x50)
#define	HW_LCD_CTRL_FIFO_STAT		(HW_LCD_CTRL_BASE+0x60)
#define	HW_LCD_CTRL_STAT			(HW_LCD_CTRL_BASE+0x70)
#define	HW_LCD_CTRL_SUBPANEL		(HW_LCD_CTRL_BASE+0x80)
#define	HW_LCD_CTRL_LINEINT			(HW_LCD_CTRL_BASE+0x90)
#define	HW_LCD_CTRL_DISPLAYSTATUS	(HW_LCD_CTRL_BASE+0xa0)
#define	HW_LCD_CTRL_MONITOR_CTRL	(HW_LCD_CTRL_BASE+0xb0)
#define	HW_LCD_CTRL_SWITCH_CTRL		(HW_LCD_CTRL_BASE+0xc0)
#define	HW_LCD_CTRL_CHECK_CTRL		(HW_LCD_CTRL_BASE+0xd0)
#define	HW_LCD_CTRL_SECURE_CTRL		(HW_LCD_CTRL_BASE+0xe0)
#define HW_LCD_CTRL_TIMING3			(HW_LCD_CTRL_BASE+0x120)
#define HW_LCD_LOWTHESHOLD			(HW_LCD_CTRL_BASE+0x130)
#define HW_LCD_UPTHESHOLD			(HW_LCD_CTRL_BASE+0x140)
#define PALETTE_BUFF_CLEAR (0x80000000)	/* entry is clear/invalid */

#define ASM9260_TFTPAL(x) IO_ADDRESS(HW_LCDIF_CTRL + (x<<4))

int asm9260fb_init(void);
void  asm9260_lcd_refresh(struct asm9260fb_info *fbi);

#endif

