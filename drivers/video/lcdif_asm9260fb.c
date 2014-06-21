/*
 * linux/drivers/video/as3310bfb.c
 *	Copyright (c) He Yong
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    AS3310B LCD Controller Frame Buffer Driver
 *	    based on s3c2410fb.c, skeletonfb.c, sa1100fb.c and others
 *
 * ChangeLog
 * 2006-09-05: He Yong <hoffer@sjtu.org>
 *        Create File
 *
 *2008-08-25:zhangyb
 *          clean up
 * Revised by Shan Jiasheng 7/3/2013
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <mach/pwm.h>

#include <mach/system.h>
#include <mach/pincontrol.h>
#include <mach/hwecc.h>
#include <mach/nand.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <asm/hardware/iomd.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/uart_reg.h>
#include <mach/fb.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include "lcdif_asm9260fb.h"

static struct as3310fb_mach_info *mach_info;
static struct timer_list lcd_temer;
static dma_addr_t irq_lcd_pkg_addr;
static dma_addr_t irq_lcd_pkg_addr_y;
static dma_addr_t irq_lcd_pkg_addr_u;
static dma_addr_t irq_lcd_pkg_addr_v;
struct as3310_dma_pkg_s * chain_head_p;
static int is_lcd_on;
static int _rgb_mode = 0;
extern void dbg_putc(unsigned char c);
extern void set_pin_mux(int port,int pin,int mux_type);
/* Debugging stuff */
#ifdef CONFIG_FB_AS3310_DEBUG
static int lcd_debug	   = 1;//on
#else
static int lcd_debug	   = 0;//off
#endif

#define dprintk(msg...)	if (lcd_debug) { /*dbg_printf(KERN_DEBUG "as3310fb: " msg)*/; }


/******************************
*input:ms delay time(msecond)
*return :null
*/
void delayms(int ms){
    mdelay(ms);
}

void lcd_write_reg(unsigned short n){
        while ((0x01000000 & as3310_readl_lcd(HW_LCDIF_YUV_CTRL)) ==0 ) {    
           // printk("HW_LCDIF_CTRL = %x\n",as3310_readl(HW_LCDIF_CTRL)); 
            }

        as3310_writew_lcd(n,HW_LCDIF_YUV_DATA);
        //printk("HW_LCDIF_DATA is 0x%p\n",as3310_readl(HW_LCDIF_DATA));

}


void writec(unsigned short index)
{
     as3310_writel_lcd( 0x10000 ,HW_LCDIF_YUV_CTRL+8); // STOP LCD 
     as3310_writel_lcd( 0x40000 ,HW_LCDIF_YUV_CTRL+8); // SET cmd MODE
     as3310_writel_lcd( 0x1 ,HW_LCDIF_YUV_CTRL+4); // SET Count 
     as3310_writel_lcd( 0x10000 ,HW_LCDIF_YUV_CTRL+4); // run LCD 
     lcd_write_reg(index);

}

void writed(unsigned short instruction)
{
     as3310_writel_lcd( 0x10000 ,HW_LCDIF_YUV_CTRL+8); // STOP LCD 
     as3310_writel_lcd( 0x40000 ,HW_LCDIF_YUV_CTRL+4); // SET data MODE 
     as3310_writel_lcd( 0x1,HW_LCDIF_YUV_CTRL+4); // SET Count 
     as3310_writel_lcd( 0x10000 ,HW_LCDIF_YUV_CTRL+4); // run LCD 
     lcd_write_reg(instruction);

}

void change_to_data_mode(void)
{
    as3310_writel_lcd( 0x10000, HW_LCDIF_YUV_CTRL+8); // STOP LCD 
    as3310_writel_lcd( 0x40000, HW_LCDIF_YUV_CTRL+4); // SET data MODE 
    as3310_writel_lcd( 0xffff, HW_LCDIF_YUV_CTRL+8); // clear Count 
    as3310_writel_lcd( 0x10000 ,HW_LCDIF_YUV_CTRL+4); // run LCD
}

void Init_data(unsigned short index,unsigned short instruction)
 {
      //printf("in init_data\n");
        writec(index);
        writed(instruction);
 }
void LCD_SetDispAddr(unsigned int x, unsigned int y)
{
    Init_data(0x004e, (u16)x);
    Init_data(0x004f, (u16)y);
    writec(0x0022);
}

 void LCDDEV_SetWindow(int x0, int y0, int x1, int y1)
{
    x1=320-1;y1=240-1;//alwayes 320*240
    Init_data(0x50, (u16)y0); // Horizontal GRAM Start Address-----HSA[7:0]
    Init_data(0x51, (u16)y1); // Horizontal GRAM End Address-----HEA[7:0]
    Init_data(0x52, (u16)x0); // Vertical GRAM Start Address-----VSA[8:0]
    Init_data(0x53, (u16)x1); // Vertical GRAM Start Address-----VEA[8:0]
     
    LCD_SetDispAddr(x0, y0);
}

void PowerOnLcdinit_ili9320(void)
{
	 Init_data(0x0000,0x0001);	 delayms(50);	
	 Init_data(0x0003,0xA8A4);	 delayms(50);	 
	 Init_data(0x000C,0x0000);	 delayms(50);	 
	 Init_data(0x000D,0x080C);	 delayms(50);	 
	 Init_data(0x000E,0x2B00);	 delayms(50);	 
	 Init_data(0x001E,0x00B0);	 delayms(50);	 
	 Init_data(0x0001,0x2B3F);	 delayms(50);	
	 Init_data(0x0002,0x0600);	 delayms(50);
	 Init_data(0x0010,0x0000);	 delayms(50);
	 Init_data(0x0011,0x6070);	 delayms(50);	 
	 Init_data(0x0005,0x0000);	 delayms(50);
	 Init_data(0x0006,0x0000);	 delayms(50);
	 Init_data(0x0016,0xEF1C);	 delayms(50);
	 Init_data(0x0017,0x0003);	 delayms(50);
	 Init_data(0x0007,0x0133);	 delayms(50);		   
	 Init_data(0x000B,0x0000);	 delayms(50);
	 Init_data(0x000F,0x0000);	 delayms(50);	 
	 Init_data(0x0041,0x0000);	 delayms(50);
	 Init_data(0x0042,0x0000);	 delayms(50);
	 Init_data(0x0048,0x0000);	 delayms(50);
	 Init_data(0x0049,0x013F);	 delayms(50);
	 Init_data(0x004A,0x0000);	 delayms(50);
	 Init_data(0x004B,0x0000);	 delayms(50);
	 Init_data(0x0044,0xEF00);	 delayms(50);
	 Init_data(0x0045,0x0000);	 delayms(50);
	 Init_data(0x0046,0x013F);	 delayms(50);
	 Init_data(0x0030,0x0707);	 delayms(50);
	 Init_data(0x0031,0x0204);	 delayms(50);
	 Init_data(0x0032,0x0204);	 delayms(50);
	 Init_data(0x0033,0x0502);	 delayms(50);
	 Init_data(0x0034,0x0507);	 delayms(50);
	 Init_data(0x0035,0x0204);	 delayms(50);
	 Init_data(0x0036,0x0204);	 delayms(50);
	 Init_data(0x0037,0x0502);	 delayms(50);
	 Init_data(0x003A,0x0302);	 delayms(50);
	 Init_data(0x003B,0x0302);	 delayms(50);
	 Init_data(0x0023,0x0000);	 delayms(50);
	 Init_data(0x0024,0x0000);	 delayms(50);
	 Init_data(0x0025,0x8000);	 delayms(50);
	 Init_data(0x004f,0);		
	 Init_data(0x004e,0);		
	 writec(0x0022);
 
	 change_to_data_mode();
}

int dma_apbh_lcd_init(void)
{
	as3310_writel_lcd(0x40000000, HW_APBH_LCD_CTRL0+8);//clear the clk gate
        as3310_writel_lcd(0xff100000,HW_APBH_LCD_CTRL0+8);//clear the clk gate   
        as3310_writel_lcd(0x7800,HW_APBH_LCD_CTRL0+8);//open clkgate
        return 0;
}

int lcdif_dmargb_mode(void)
{
	as3310_writel_lcd(0x04000000, HW_LCDIF_YUV_CTRL1+8);//rgb mode
	return 0;
}

int lcdif_dmayuv_mode(void)
{
	as3310_writel_lcd(0x04000000, HW_LCDIF_YUV_CTRL1+4);//yuv mode
	return 0;
}
#define is_apbh_lcd_complete(ch) \
    ((as3310_readl_lcd(HW_APBH_LCD_CH0_SEMA + ((ch)*0x70))&0x00ff0000)==0)

int  lcd_set_mode(int mode)
{
	unsigned int count=1000000;
	
    	while (!is_apbh_lcd_complete(LCD_CH_NUM_Y) && count>0)
    	{
		//mode change must after DMA finish
		count--;
	}
	if(count == 0)
	{
		printk("apbh_lcd_not_complete!\n");
		return 0x101;
	}
	if (mode == LCD_MODE_RGB)//RGB
	{
		lcdif_dmargb_mode();
	}
	else	//YUV
	{
		lcdif_dmayuv_mode();
	}
	return 0;
}

static int dma_start_lcd_apbh(ulong pkg_addr,int ch_num)
{
	as3310_writel_lcd((u8*)pkg_addr,HW_APBH_LCD_CH0_NXTCMDAR + (0x70 * ch_num));
	as3310_writel_lcd(1,HW_APBH_LCD_CH0_SEMA  + (0x70 * ch_num));
	return 0;
}


void lcdif_start_dma_yuv(void)
{
	dma_start_lcd_apbh((dma_addr_t)irq_lcd_pkg_addr_u, 
	 LCD_CH_NUM_U);//U channel
	dma_start_lcd_apbh((dma_addr_t)irq_lcd_pkg_addr_v, 
	 LCD_CH_NUM_V);//V channel
	dma_start_lcd_apbh((dma_addr_t)irq_lcd_pkg_addr_y, 
	 LCD_CH_NUM_Y);//Y channel
}

void lcdif_start_dma(void)
{
	dma_start_lcd_apbh((dma_addr_t)irq_lcd_pkg_addr, 
	LCD_CH_NUM_Y);//rgb channel   
}

int refresh_lcd(void)
{

	if (_rgb_mode == LCD_MODE_RGB)
		{
			lcd_set_mode(LCD_MODE_RGB);
			printk("lcdif_dma_rgb_start!\r\n");
			lcdif_start_dma();
		}
	else 
		{
			lcd_set_mode(LCD_MODE_YUV);
			printk("lcdif_dma_yuv_start!\r\n");
			lcdif_start_dma_yuv();
		}
	return 0;
}

void Display_cam(int colour_mode)
{		
	dma_apbh_lcd_init();	
	as3310_writel_lcd( 0x10000, HW_LCDIF_YUV_CTRL+8); // STOP LCD 	
	as3310_writel_lcd( _rgb_max_x*_rgb_max_y, HW_LCDIF_YUV_CTRL1+8); // clear Count 	
	as3310_writel_lcd( 0x10000 ,HW_LCDIF_YUV_CTRL+4); // run LCD	HY320lcd_set_mode(0);
	if (colour_mode == LCD_MODE_RGB)
	{
      	lcd_set_mode(LCD_MODE_RGB);
		lcdif_start_dma();
	}
	else 
     {
     		lcd_set_mode(LCD_MODE_YUV);
		lcdif_start_dma_yuv();
	}
	return 0;	
}
EXPORT_SYMBOL_GPL(Display_cam);

/*
 *	as3310fb_check_var():
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 */
static int as3310fb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct as3310fb_info *fbi = info->par;

  	// validate x/y resolution //

	if (var->yres > fbi->mach_info->yres.max)
		var->yres = fbi->mach_info->yres.max;
	else if (var->yres < fbi->mach_info->yres.min)
		var->yres = fbi->mach_info->yres.min;

	if (var->xres > fbi->mach_info->xres.max)
		var->xres = fbi->mach_info->xres.max;
	else if (var->xres < fbi->mach_info->xres.min)
		var->xres = fbi->mach_info->xres.min;

	// validate bpp //

	if (var->bits_per_pixel > fbi->mach_info->bpp.max)
		var->bits_per_pixel = fbi->mach_info->bpp.max;
	else if (var->bits_per_pixel < fbi->mach_info->bpp.min)
		var->bits_per_pixel = fbi->mach_info->bpp.min;

	// set r/g/b positions //

    if (var->bits_per_pixel == 16) {
        var->red.offset     = AS3310_LCD_R_OFFSET;
        var->green.offset   = AS3310_LCD_G_OFFSET;
        var->blue.offset    = AS3310_LCD_B_OFFSET;
        var->red.length     = AS3310_LCD_R_LEN;
        var->green.length   = AS3310_LCD_G_LEN;
        var->blue.length    = AS3310_LCD_B_LEN;
        var->transp.length  = 0;
    } else if (var->bits_per_pixel == 32) {
        var->red.offset     = ASAP1826_LCD_R_OFFSET;
        var->green.offset   = ASAP1826_LCD_G_OFFSET;
        var->blue.offset    = ASAP1826_LCD_B_OFFSET;
        var->red.length     = ASAP1826_LCD_R_LEN;
        var->green.length   = ASAP1826_LCD_G_LEN;
        var->blue.length    = ASAP1826_LCD_B_LEN;
        var->transp.length  = 0; 
    } else {
        var->red.length     = var->bits_per_pixel;
        var->red.offset     = 0;
        var->green.length   = var->bits_per_pixel;
        var->green.offset   = 0;
        var->blue.length    = var->bits_per_pixel;
        var->blue.offset    = 0;
        var->transp.length  = 0;
    }
    

	return 0;
}

/* as3310fb_activate_var
 *
 * activate (set) the controller from the given framebuffer
 * information
*/

static void as3310fb_activate_var(struct as3310fb_info *fbi,
				   struct fb_var_screeninfo *var)
{
    dprintk("Entered as3310fb_activate_var()\n");
}


/*
 *      as3310fb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 */
static int as3310fb_set_par(struct fb_info *info)
{
	struct as3310fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;

    dprintk("as3310fb_set_par()\n");
	if (var->bits_per_pixel == 16)
		fbi->fb->fix.visual = FB_VISUAL_TRUECOLOR;
	else
		fbi->fb->fix.visual = FB_VISUAL_PSEUDOCOLOR;

	fbi->fb->fix.line_length     = (var->width*var->bits_per_pixel)/8;

	/* activate this new configuration */

	as3310fb_activate_var(fbi, var);
	return 0;
}

static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}
/* from pxafb.c */
//static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
//{
//	chan &= 0xffff;
//	chan >>= 16 - bf->length;
//	return chan << bf->offset;
//}

static int as3310fb_setcolreg(unsigned regno,
                              unsigned red, unsigned green, unsigned blue,
                              unsigned transp, struct fb_info *info)
{
      struct as3310fb_info *fbi = info->par;
      unsigned int val;

      switch (fbi->fb->fix.visual) {
      case FB_VISUAL_TRUECOLOR:
       /* true-colour, use pseuo-palette */
         if (regno < 16) {
             u32 *pal = fbi->fb->pseudo_palette;

             val  = chan_to_field(red,   &fbi->fb->var.red);
             val |= chan_to_field(green, &fbi->fb->var.green);
             val |= chan_to_field(blue,  &fbi->fb->var.blue);

             pal[regno] = val;
         }
         break;
      default:
      printk(KERN_ERR " unknown type - %d\n",fbi->fb->fix.visual);
      return 1;   /* unknown type */
     }

      dprintk("setcol: regno=%d, rgb=%d,%d,%d\n", regno, red, green, blue); //
      return 0;
}


/**
 *      as3310fb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
static int as3310fb_blank(int blank_mode, struct fb_info *info)
{ 
    // need fix here to enable low power-- hoffer 
	dprintk("blank(mode=%d, _rgb_mode=%d)\n", blank_mode, _rgb_mode);
    if (!blank_mode) { return 0; }

    if(_rgb_mode==LCD_MODE_RGB){
        memset(info->screen_base, 0x1f, info->screen_size);
    }
    else {
        dprintk("info->screen_base=%p, (info->screen_size>>1)=%d)\n", info->screen_base,(info->screen_size>>1));
        memset(info->screen_base, 0x10, (info->screen_size>>1)); // Y
        memset(info->screen_base + (info->screen_size>>1), 0x80, (info->screen_size>>2)); // UV
    }
	return 0;
}

static int as3310fb_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", lcd_debug ? "on" : "off");
}
static int as3310fb_debug_store(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t len)
{
    if (mach_info == NULL)
        return -EINVAL;

    if (len < 1)
        return -EINVAL;

    if (strnicmp(buf, "on", 2) == 0 ||
        strnicmp(buf, "1", 1) == 0) {
        lcd_debug = 1;
        printk(KERN_DEBUG "as3310fb: Debug On");
    } else if (strnicmp(buf, "off", 3) == 0 ||
               strnicmp(buf, "0", 1) == 0) {
        lcd_debug = 0;
        printk(KERN_DEBUG "as3310fb: Debug Off");
    } else {
        return -EINVAL;
    }

    return len;
}

void * fb_cpu;
int fb_size;
EXPORT_SYMBOL_GPL(fb_cpu);
EXPORT_SYMBOL_GPL(fb_size);

int as3310fb_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg){
    int ret = 0;
    unsigned long exp;
    struct as3310fb_info *fbi = info->par;
    switch (cmd) {
    case FBIOBLANK:
            return as3310fb_blank(1, info);
		break;
    case FBIO_SYNC:
		//refresh_lcd();
		Display_cam(_rgb_mode);
		break;
    case FBIO_RGB://set RGB mode
//        lcd_set_mode(0);
		_rgb_mode=LCD_MODE_RGB;
		break;
        break;
    case FBIO_YUV://set YUV mode
//        lcd_set_mode(1);
		_rgb_mode=LCD_MODE_YUV;
		break;
    case FBIO_OFF://stop refresh
        is_lcd_on = 0;
		break;
    case FBIO_ON:
        is_lcd_on = 1;//start refresh
        break;
    case FBIO_GET_DISP_MODE:      
        return put_user(_rgb_mode, (int *)arg);        
    default:
        dprintk("as3310fb:IOCTRL=0x%x is not support",cmd);
        ret = -EINVAL;//invaild cmd
        break;
    }
    return ret;
}



static DEVICE_ATTR(debug, 0666,
		   as3310fb_debug_show,
		   as3310fb_debug_store);

static struct fb_ops as3310fb_ops = {
    .owner      = THIS_MODULE,
    .fb_check_var   = as3310fb_check_var,
    .fb_set_par = as3310fb_set_par,
    .fb_blank   = as3310fb_blank,
    .fb_setcolreg   = as3310fb_setcolreg,
    .fb_fillrect    = cfb_fillrect,
    .fb_copyarea    = cfb_copyarea,
    .fb_imageblit   = cfb_imageblit,
    .fb_ioctl   = as3310fb_fb_ioctl,
};

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


/*
 * as3310fb_map_video_memory():
 *	Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *	allow palette and pixel writes to occur without flushing the
 *	cache.  Once this area is remapped, all virtual memory
 *	access to the video memory should occur at the new region.
 */
static int __init as3310fb_map_video_memory(struct as3310fb_info *fbi)
{
	dprintk("map_video_memory(fbi=%p)\n", fbi);
        int i,j;
	u8 Y,U,V;

#ifdef CONFIG_FB_VIRTUAL_VGA

	fbi->map_size1 = PAGE_ALIGN(SCREEN_SIZE_COL * SCREEN_SIZE_ROW *2 + PAGE_SIZE);
        fbi->map_cpu1  = dma_alloc_writecombine(fbi->dev, fbi->map_size1,
					       &fbi->map_dma1, GFP_KERNEL);

	if (fbi->map_cpu1) {
		dprintk("map_video_phy_memory 1: dma1=%08x cpu1 =%p size1=%08x\n",
			fbi->map_dma1, fbi->map_cpu1, fbi->map_size1);
	}

	fbi->map_size = PAGE_ALIGN(fbi->fb->fix.smem_len + PAGE_SIZE);
        fbi->map_cpu  = vmalloc(fbi->map_size);

#else

	fbi->map_size = PAGE_ALIGN(fbi->fb->fix.smem_len + PAGE_SIZE);
        fbi->map_cpu  = dma_alloc_writecombine(fbi->dev, fbi->map_size,
					       &fbi->map_dma, GFP_KERNEL);
#endif


	fbi->map_size = fbi->fb->fix.smem_len;

	if (fbi->map_cpu) {
		/* prevent initial garbage on screen */
		printk("map_video_memory: clear %p:%08x\n",
			fbi->map_cpu, fbi->map_size);
		fbi->screen_dma		= fbi->map_dma;//HW_LCDIF_YUV_TIMING
		fbi->fb->screen_base	= fbi->map_cpu;
		fbi->fb->screen_size	= fbi->map_size;
		fbi->fb->fix.smem_start  = fbi->screen_dma;

		fb_cpu = fbi->map_cpu;
       	fb_size = fbi->map_size;
        _rgb_mode=LCD_MODE_YUV;/*****可省略******/
		
#if 1
     if(_rgb_mode==LCD_MODE_RGB){
        for(i=0;i<320*240;i++)
        {  
			if(i<320*240/8)
				*((u16 *)fbi->map_cpu+i) = RGB565(255,0,0);
				else if(i<320*240*2/8)
						*((u16 *)fbi->map_cpu+i) = RGB565(255,97,0);
				else if(i<320*240*3/8)
						*((u16 *)fbi->map_cpu+i) = RGB565(255,255,0);
				else if(i<320*240*4/8)
						*((u16 *)fbi->map_cpu+i) = RGB565(0,255,0);
				else if(i<320*240*5/8)
						*((u16 *)fbi->map_cpu+i) = RGB565(0,255,255);
				else if(i<320*240*6/8)
						*((u16 *)fbi->map_cpu+i) = RGB565(0,0,255);
				else if(i<320*240*7/8)
						*((u16 *)fbi->map_cpu+i) = RGB565(160,32,240);
				else
						*((u16 *)fbi->map_cpu+i) = RGB565(255,255,255);
        }
      
     }
    else if(_rgb_mode==LCD_MODE_YUV){    
        for(i=0;i<_rgb_max_y;i++)
			for(j=0;j<_rgb_max_x;j++)
			{
				if(j<80)
					{
					rgb2yuv(255,0,0,Y,U,V);
					}
				else if(j<160)
					{
					rgb2yuv(0,255,0,Y,U,V);
					}
				else
					{
					rgb2yuv(0,0,255,Y,U,V);
					}

				*((u8 *)fbi->map_cpu+i*_rgb_max_x+j)=Y;
				*((u8 *)fbi->map_cpu+_rgb_max_x*_rgb_max_y+i/2*_rgb_max_x/2+j/2)=U;
				*((u8 *)fbi->map_cpu+_rgb_max_x*_rgb_max_y*5/4+i/2*_rgb_max_x/2+j/2)=V;
			}
        }
	else{}
#endif
		printk("map_video_phy_memory: dma=%08x cpu =%p size=%08x\n",
			fbi->map_dma, fbi->map_cpu, fbi->fb->fix.smem_len);

	}

        fbi->dmachain = request_as3310_dma_chain(fbi->dev,LCD_PKG_NUM,LCD_CH_NUM_Y);
        fbi->dmachain_y = request_as3310_dma_chain(fbi->dev,LCD_PKG_NUM_Y,LCD_CH_NUM_Y);
        fbi->dmachain_u = request_as3310_dma_chain(fbi->dev,LCD_PKG_NUM_U,LCD_CH_NUM_U);
        fbi->dmachain_v = request_as3310_dma_chain(fbi->dev,LCD_PKG_NUM_V,LCD_CH_NUM_V);

        if (fbi->dmachain) {
		dprintk("lcd_dma_pkg_phy_memory: dma=%08x cpu =%p\n",
			fbi->dmachain->chain_phy_addr, fbi->dmachain->chain_head);
                chain_head_p = fbi->dmachain->chain_head;
	}

        

        irq_lcd_pkg_addr = fbi->dmachain->chain_phy_addr;
        irq_lcd_pkg_addr_y = fbi->dmachain_y->chain_phy_addr;
        irq_lcd_pkg_addr_u = fbi->dmachain_u->chain_phy_addr;
        irq_lcd_pkg_addr_v = fbi->dmachain_v->chain_phy_addr;

	return fbi->map_cpu ? 0 : -ENOMEM;
}

static inline void as3310fb_unmap_video_memory(struct as3310fb_info *fbi)
{
	dma_free_writecombine(fbi->dev,fbi->map_size,fbi->map_cpu, fbi->map_dma);
}

static inline void modify_gpio(void __iomem *reg,
			       unsigned long set, unsigned long mask)
{
	unsigned long tmp;

	tmp = readl(reg) & ~mask;
	writel(tmp | set, reg);
}


/*
 * as3310fb_init_registers - Initialise all LCD-related registers
 */

int as3310fb_init_registers(struct as3310fb_info *fbi)
{
    int j,pix_clk;
#ifdef CONFIG_FB_EVK_480_272
    printk("Configure for LCDIF (320X240) TFT LCD\n");
#endif

#ifdef CONFIG_FB_FTG500C06Z_800_480
    printk("Configure for FTG500C06Z (800X480) TFT LCD\n");
#endif


#ifdef CONFIG_FB_VIRTUAL_VGA
    printk("Configure for Vitual VGA Framebuffer Screen\n");
#endif

    /*     Pin Assign      */    
    for (j = 0; j <= 7; j++)
	{
		set_pin_mux(2, j, PIN_FUNCTION_3);
	}
    for (j = 0; j <= 7; j++)
	{
		set_pin_mux(3, j, PIN_FUNCTION_3);
	}
		set_pin_mux(1, 3,PIN_FUNCTION_3);
		set_pin_mux(1, 6, PIN_FUNCTION_3);
		set_pin_mux(1, 5, PIN_FUNCTION_3);
		set_pin_mux(1, 7, PIN_FUNCTION_3);
		set_pin_mux(1, 4,PIN_FUNCTION_3);


    /*     lcdif_interface_init      */
    as3310_writel(0x4108,HW_AHBCLKCTRL1+4);
    as3310_writel(0x1<<3,HW_PRESETCTRL1+8);
    as3310_writel(0x1<<3,HW_PRESETCTRL1+4);
   
    as3310_writel_lcd( 0x40000000, HW_LCDIF_YUV_CTRL+8); // CLEAR bit 31,30 CLKGATE
    as3310_writel_lcd( 0x80000000, HW_LCDIF_YUV_CTRL+8); // CLEAR bit 31,30 SFTRST   
    as3310_writel_lcd( 0x02050507, HW_LCDIF_YUV_TIMING); // Timing
    as3310_writel_lcd( 0x00100000, HW_LCDIF_YUV_CTRL+4); // SET RESET = 1    
    as3310_writel_lcd( 0 ,HW_LCDIF_YUV_CTRL+4);
   
    as3310_writel_lcd( 0x10000 ,HW_LCDIF_YUV_CTRL+8); // STOP LCD
    as3310_writel_lcd( 0x40000, HW_LCDIF_YUV_CTRL+8); // CLEAR DATA MODE->CMD MODE
    as3310_writel_lcd( 0xffff, HW_LCDIF_YUV_CTRL+4); // SET Count 
    as3310_writel_lcd( 0x10000, HW_LCDIF_YUV_CTRL+4); // RUN LCD
   
    pix_clk = LCD_DEFAULT_PIXCLK_DIVIDER;

    printk("LCD Pixel Clock: %d\n",pix_clk);

    dma_apbh_lcd_init();

    PowerOnLcdinit_ili9320();
    LCDDEV_SetWindow(0, 0, fbi->mach_info->width -1, fbi->mach_info->height -1);
    change_to_data_mode();
    as3310_writel_lcd(0x20000000,HW_LCDIF_YUV_CTRL1+4);//set dma en
    
    as3310_writel_lcd(((_rgb_max_y-1)<<16)|(_rgb_max_x-1), HW_LCDIF_YUV_TIMING1+4);//set the ppl
   
    refresh_lcd(); 
 
    return 0;
}

static void as3310fb_write_palette(struct as3310fb_info *fbi)
{
    unsigned int i;
    unsigned long ent;

    fbi->palette_ready = 0;

    for (i = 0; i < 256; i++) {
        if ((ent = fbi->palette_buffer[i]) == PALETTE_BUFF_CLEAR)
            continue;

        writel(ent, AS3310_TFTPAL(i));

        /* it seems the only way to know exactly
         * if the palette wrote ok, is to check
         * to see if the value verifies ok
         */

        if (readw(AS3310_TFTPAL(i)) == ent)
            fbi->palette_buffer[i] = PALETTE_BUFF_CLEAR;
        else
            fbi->palette_ready = 1;   /* retry */
    }
}


static char driver_name[]="as3310fb";



int init_as3310_lcd_dma_pkg(struct as3310fb_info *bf_i){

    struct as3310_dma_chain * pdmachain;
    struct as3310_dma_pkg_s * dmapkg;

    struct as3310_dma_pkg_s * dmapkg_y;
    struct as3310_dma_pkg_s * dmapkg_u;
    struct as3310_dma_pkg_s * dmapkg_v;
    u8 * lcd_yptr,*lcd_uptr,*lcd_vptr;
    ulong dma_phy_addr;
    
    int i;
    as3310_writel_lcd(0x40003800,HW_APBH_LCD_CTRL0_CLR);//clear the clk gate sft rst
    as3310_writel_lcd(0x80000000,HW_APBH_LCD_CTRL0_CLR);//clear the clk gate sft rst   

#ifdef CONFIG_FB_VIRTUAL_VGA
    dma_phy_addr = bf_i->map_dma1;  // display actual size
//    len = SCREEN_SIZE_COL*SCREEN_SIZE_ROW*2;//16bpp
#else
    dma_phy_addr = bf_i->map_dma;
//    len = AS3310_LCD_XRES*AS3310_LCD_YRES*2;//16bpp
#endif
/*  ================= for RGB Packages ==================*/


    /*     Prepare DMA PKG */
    pdmachain = bf_i->dmachain;
    dmapkg = pdmachain->chain_head;

    for (i=0; i<LCD_PKG_NUM; i++)
	{
		dmapkg[i].CTRL =
				0x00001086
				+ (((_rgb_max_x*_rgb_max_y*2)/LCD_PKG_NUM)<<16 );
		dmapkg[i].BUFFER =
				(dma_phy_addr
				+ i*((_rgb_max_x*_rgb_max_y*2)/LCD_PKG_NUM));
		dmapkg[i].CMD0 =
				0x30000000+(((_rgb_max_x*_rgb_max_y*2)/LCD_PKG_NUM)>>2 );
	}
        
    dmapkg[0].CMD0 =
				0x38000000+(((_rgb_max_x*_rgb_max_y*2)/LCD_PKG_NUM)>>2 );	
    dmapkg[LCD_PKG_NUM-1].CTRL =
				0x000010c2
		        + (((_rgb_max_x*_rgb_max_y*2)/LCD_PKG_NUM)<<16 );  

/*  ================= for YUV Packages ==================*/
    lcd_yptr = dma_phy_addr;
    lcd_uptr = lcd_yptr + _rgb_max_x*_rgb_max_y;
    lcd_vptr = lcd_uptr + (_rgb_max_x*_rgb_max_y>>2);

    dmapkg_y = bf_i->dmachain_y->chain_head;
    dmapkg_u = bf_i->dmachain_u->chain_head;
    dmapkg_v = bf_i->dmachain_v->chain_head;

         /*     Prepare Y DMA PKG */

    for (i=0; i<LCD_PKG_NUM_Y; i++)
      {
            dmapkg_y[i].CTRL = 0x00001086
                  + (((_rgb_max_x*_rgb_max_y)/(LCD_PKG_NUM_Y))<<16 );
            dmapkg_y[i].BUFFER
                  =(char *) (lcd_yptr + i*((_rgb_max_x*_rgb_max_y)/(LCD_PKG_NUM_Y)));
            dmapkg_y[i].CMD0 = 0x34000000
                  + (((_rgb_max_x*_rgb_max_y)/(LCD_PKG_NUM_Y))>>2 );
      }
	

    dmapkg_y[0].CMD0 = 0x3c000000
        + (((_rgb_max_x*_rgb_max_y)/(LCD_PKG_NUM_Y))>>2 );//start_f, dmaen dmaenrev , yuv mode
    dmapkg_y[LCD_PKG_NUM_Y-1].CTRL = 0x000010c2
        + (((_rgb_max_x*_rgb_max_y)/(LCD_PKG_NUM_Y))<<16 );
	/*     Prepare U DMA PKG */

    for (i=0; i<LCD_PKG_NUM_U; i++)
         {
               dmapkg_u[i].CTRL = 0x00001086
                     + ((((_rgb_max_x*_rgb_max_y)>>2)/(LCD_PKG_NUM_U))<<16);
               dmapkg_u[i].BUFFER
                     = (lcd_uptr + i*(((_rgb_max_x*_rgb_max_y)>>2)/(LCD_PKG_NUM_U)));
               dmapkg_u[i].CMD0 = 0x00000000
                     + ((((_rgb_max_x*_rgb_max_y)>>2)/(LCD_PKG_NUM_U))>>2);
         }

    dmapkg_u[LCD_PKG_NUM_U-1].CTRL = 0x000010c2
            + ((((_rgb_max_x*_rgb_max_y)>>2)/(LCD_PKG_NUM_U))<<16 );
    /*     Prepare V DMA PKG */


    for (i=0; i<LCD_PKG_NUM_V; i++)
	{
               dmapkg_v[i].CTRL = 0x00001086
                     + ((((_rgb_max_x*_rgb_max_y)>>2)/(LCD_PKG_NUM_V))<<16 );
               dmapkg_v[i].BUFFER
                     = (lcd_vptr + i*(((_rgb_max_x*_rgb_max_y)>>2)/(LCD_PKG_NUM_V)));
               dmapkg_v[i].CMD0 = 0x00000000
                     + ((((_rgb_max_x*_rgb_max_y)>>2)/(LCD_PKG_NUM_V))>>2 );
	}
	
	dmapkg_v[LCD_PKG_NUM_V-1].CTRL = 0x000010c2
		+ ((((_rgb_max_x*_rgb_max_y) >> 2) / (LCD_PKG_NUM_V)) << 16);
return 0;

}



/*
 * IRQ handler for the lcd
 */

int __init as3310fb_probe(struct device *dev)
{
	struct as3310fb_info *info;
	struct fb_info	   *fbinfo;
  	struct as3310fb_hw *mregs;
	int ret;
	int i;

        dprintk("LCD as3310fb_probe()!\n");
 
	mach_info = (struct as3310fb_mach_info *)dev->platform_data;
	if (mach_info == NULL) {
		dev_err(dev,"no platform data for lcd, cannot attach\n");
		return -EINVAL;
	}

	mregs = &mach_info->regs;

	fbinfo = framebuffer_alloc(sizeof(struct as3310fb_info), dev);
	if (!fbinfo) {
		return -ENOMEM;
	}

	info = fbinfo->par;
	info->fb = fbinfo;
	dev_set_drvdata(dev, fbinfo);

	strcpy(fbinfo->fix.id, driver_name);

	memcpy(&info->regs, &mach_info->regs, sizeof(info->regs));

	info->mach_info		    = dev->platform_data;

	fbinfo->fix.type	    = FB_TYPE_PACKED_PIXELS;
	fbinfo->fix.type_aux    = 0;
	fbinfo->fix.xpanstep	= 0;
	fbinfo->fix.ypanstep	= 0;
	fbinfo->fix.ywrapstep	= 0;
	fbinfo->fix.accel	    = FB_ACCEL_NONE;
	fbinfo->fix.visual      = FB_VISUAL_TRUECOLOR; // add by hoffer

	fbinfo->var.nonstd	    = 0;
	fbinfo->var.activate	= FB_ACTIVATE_NOW;
	fbinfo->var.height	    = mach_info->height;
	fbinfo->var.width	    = mach_info->width;

        dprintk("fbinfo->var.height = %d, fbinfo->var.width = %d\n",mach_info->height,mach_info->width);

	fbinfo->var.accel_flags     = 0;
	fbinfo->var.vmode	    = FB_VMODE_NONINTERLACED;

	fbinfo->fbops		    = &as3310fb_ops;
	fbinfo->flags		    = FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette      = &info->pseudo_pal;

        //dprintk("FB FLAG = %08x\n",fbinfo->flags);

        fbinfo->var.xres	    = mach_info->xres.defval;
	fbinfo->var.xres_virtual    = mach_info->xres.defval;
	fbinfo->var.yres	    = mach_info->yres.defval;
	fbinfo->var.yres_virtual    = mach_info->yres.defval;
	fbinfo->var.bits_per_pixel  = mach_info->bpp.defval;

	fbinfo->var.upper_margin    = 1;//AS3310_LCDCON2_GET_VBPD(mregs->lcdcon2) +1;
	fbinfo->var.lower_margin    = 1;//AS3310_LCDCON2_GET_VFPD(mregs->lcdcon2) +1;
	fbinfo->var.vsync_len	    = mach_info->xres.defval * mach_info->yres.defval +1 ;//AS3310_LCDCON2_GET_VSPW(mregs->lcdcon2) + 1;

	fbinfo->var.left_margin	    = 1;//AS3310_LCDCON3_GET_HFPD(mregs->lcdcon3) + 1;
	fbinfo->var.right_margin    = 1;//AS3310_LCDCON3_GET_HBPD(mregs->lcdcon3) + 1;
	fbinfo->var.hsync_len	    = mach_info->xres.defval + 1;//AS3310_LCDCON4_GET_HSPW(mregs->lcdcon4) + 1;

	fbinfo->var.red.offset      = AS3310_LCD_R_OFFSET;//565 mode R at bit 11
	fbinfo->var.green.offset    = AS3310_LCD_G_OFFSET;//565 mode  G at bit 5
	fbinfo->var.blue.offset     = AS3310_LCD_B_OFFSET;//565 mode B at bit 0
	fbinfo->var.transp.offset   = 0;
	fbinfo->var.red.length      = AS3310_LCD_R_LEN;//565 mode
	fbinfo->var.green.length    = AS3310_LCD_G_LEN;
	fbinfo->var.blue.length     = AS3310_LCD_B_LEN; 
	fbinfo->var.transp.length   = 0;
	fbinfo->fix.smem_len        =	mach_info->xres.max *
					mach_info->yres.max *
					mach_info->bpp.max / 8;//bpp.max=32bits
        fbinfo->fix.line_length     =(fbinfo->var.width*fbinfo->var.bits_per_pixel)/8;//bits_per_pixel=16bits

	for (i = 0; i < AS3310_LCD_PALETTE_NUM; i++)
		info->palette_buffer[i] = PALETTE_BUFF_CLEAR;

	if (!request_mem_region((unsigned long)0x80800000, 0x9000, "as3310-lcd")) {
		ret = -EBUSY;
		goto dealloc_fb;
	}


        info->framerate = AS3310_DEFAULT_FRAME_RATE;

	msleep(1);
	/* Initialize video memory */
	ret = as3310fb_map_video_memory(info);
	if (ret) {
		printk( KERN_ERR "Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto release_clock;
	}
        init_as3310_lcd_dma_pkg(info);

	dprintk("got video memory\n");

	ret = as3310fb_init_registers(info);

	dprintk("as3310fb_init_registers\n");

	ret = as3310fb_check_var(&fbinfo->var, fbinfo);

        dprintk("as3310fb_check_var\n");

	ret = register_framebuffer(fbinfo);
	if (ret < 0) {
		printk(KERN_ERR "Failed to register framebuffer device: %d\n", ret);
		goto free_video_memory;
	}
	/* create device files */
	device_create_file(dev, &dev_attr_debug);

	printk(KERN_INFO "fb%d: %s frame buffer device\n",
		fbinfo->node, fbinfo->fix.id);

	return 0;

free_video_memory:
	as3310fb_unmap_video_memory(info);
        del_timer(&lcd_temer);
release_clock:

dealloc_fb:
	framebuffer_release(fbinfo);
	return ret;
}


/* as3310fb_stop_lcd
 *
 * shutdown the lcd controller
*/

static void as3310fb_stop_lcd(void)
{
	unsigned long flags;

	local_irq_save(flags);
	local_irq_restore(flags);
}

/*
 *  Cleanup
 */
static int as3310fb_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fb_info	   *fbinfo = dev_get_drvdata(dev);
	struct as3310fb_info *info = fbinfo->par;
	int irq;

	as3310fb_stop_lcd();
	msleep(1);

	as3310fb_unmap_video_memory(info);

 	if (info->clk) {
 
 		info->clk = NULL;
	}

	irq = platform_get_irq(pdev, 0);
	free_irq(irq,info);
	unregister_framebuffer(fbinfo);

	return 0;
}

#ifdef CONFIG_PM

/* suspend and resume support for the lcd controller */

static int as3310fb_suspend(struct device *dev, pm_message_t state, u32 level)
{
	return 0;
}

static int as3310fb_resume(struct device *dev, u32 level)
{
	return 0;
}

#else
#define as3310fb_suspend NULL
#define as3310fb_resume  NULL
#endif


static struct as3310fb_mach_info fb_platform_data = {
	.width      =   AS3310_LCD_XRES,
	.height     =   AS3310_LCD_YRES,
         .xres               ={
        .defval     =   AS3310_LCD_XRES,
        .min        =   (AS3310_LCD_XRES>>3),
        .max        =   AS3310_LCD_XRES,
         },
         .yres               ={
        .defval     =   AS3310_LCD_YRES,
        .min        =   (AS3310_LCD_YRES>>3),
        .max        =   AS3310_LCD_YRES,
         },
         .bpp               ={
        .defval     =   16,
        .min        =   8,
        .max        =   16,
         },
         .regs               ={
        .lcdcon1     =   LCDIF_CTRL_VALUE,
        .lcdcon2     =   LCDIF_TIMING_VALUE,
         },

};

static struct platform_device fb_device = {
	.name			= "as3310-lcd",
	.id			= 0,
	.dev			= {
		.platform_data	= &fb_platform_data,
	},
};


static struct device_driver as3310fb_driver = {
	.name		= "as3310-lcd",
	.bus		= &platform_bus_type,
	.probe		= as3310fb_probe,
	.suspend	= as3310fb_suspend,
	.resume		= as3310fb_resume,
	.remove		= as3310fb_remove
};

int __devinit as3310fb_init(void)
{
        dprintk("as3310fb_init()!\n");
	platform_device_register(&fb_device);
        if (IS_ERR(&fb_device)) {
		PTR_ERR(&fb_device);
        dprintk("as3310-lcd platform_device_register ERROR!\n");
	}
	return driver_register(&as3310fb_driver);
}

static void __exit as3310fb_cleanup(void)
{
        dprintk("as3310fb_EXIT()!\n");
	driver_unregister(&as3310fb_driver);
}


module_init_lcd(as3310fb_init);
module_exit(as3310fb_cleanup);

MODULE_AUTHOR("AlpScale Inc");
MODULE_DESCRIPTION("Framebuffer driver for the as3310");
MODULE_LICENSE("GPL");
