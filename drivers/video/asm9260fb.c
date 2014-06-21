/*
 * linux/drivers/video/asm9260fb.c
 *	Copyright (c) Shanjs
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    ASM9260 LCD Controller Frame Buffer Driver
 *	    based on skeletonfb.c, sa1100fb.c and others
 *
 * ChangeLog
 * 2012-12-05: Shanjs <shanjs@alpscale.cn>
 *        Create File
 *
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
#include <mach/dma.h>
#include <asm/hardware/iomd.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/uart_reg.h>
#include <mach/fb.h>
#include <mach/hardware.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include "asm9260fb.h"

static struct asm9260fb_mach_info *mach_info;
static struct timer_list lcd_temer;
static volatile struct asm9260_lcd_table * _lcd_table;
static dma_addr_t irq_lcd_pkg_addr;
static dma_addr_t irq_lcd_pkg_addr_y;
static dma_addr_t irq_lcd_pkg_addr_u;
static dma_addr_t irq_lcd_pkg_addr_v;
struct asm9260_dma_pkg_s * chain_head_p;
static int is_rgb_mode;
static int is_lcd_on;
extern void dbg_putc(unsigned char c);
extern void set_pin_mux(int port,int pin,int mux_type);

/* Debugging stuff */
#ifdef CONFIG_FB_ASM9260_DEBUG
static int lcd_debug	   = LCD_DBG_ON;//on
#else
static int lcd_debug	   = LCD_DBG_OFF;//off
#endif

#ifdef CONFIG_FB_ASM9260_DEBUG
#define DBG(x...) printk("ASM9260_LCD_DBG: " x)
#else
#define DBG(x...)
#endif


/******************************
*input:ms delay time(msecond)
*return :null
*/
void delayms(int ms){
    mdelay(ms);
}

/*
 *	asm9260fb_check_var():
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 */
static int asm9260fb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct asm9260fb_info *fbi = info->par;


	if (var->yres > fbi->mach_info->yres.max)
		var->yres = fbi->mach_info->yres.max;
	else if (var->yres < fbi->mach_info->yres.min)
		var->yres = fbi->mach_info->yres.min;

	if (var->xres > fbi->mach_info->xres.max)
		var->xres = fbi->mach_info->xres.max;
	else if (var->xres < fbi->mach_info->xres.min)
		var->xres = fbi->mach_info->xres.min;


	if (var->bits_per_pixel > fbi->mach_info->bpp.max)
		var->bits_per_pixel = fbi->mach_info->bpp.max;
	else if (var->bits_per_pixel < fbi->mach_info->bpp.min)
		var->bits_per_pixel = fbi->mach_info->bpp.min;


    if (var->bits_per_pixel == 16) {
        var->red.offset     = ASM9260_565_R_OFFSET;
        var->green.offset   = ASM9260_565_G_OFFSET;
        var->blue.offset    = ASM9260_565_B_OFFSET;
        var->red.length     = ASM9260_565_R_LEN;
        var->green.length   = ASM9260_565_G_LEN;
        var->blue.length    = ASM9260_565_B_LEN;
        var->transp.length  = 0;
    } else if (var->bits_per_pixel == 32) {
        var->red.offset     = ASM9260_888_R_OFFSET;
        var->green.offset   = ASM9260_888_G_OFFSET;
        var->blue.offset    = ASM9260_888_B_OFFSET;
        var->red.length     = ASM9260_888_R_LEN;
        var->green.length   = ASM9260_888_G_LEN;
        var->blue.length    = ASM9260_888_B_LEN;
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

/* asm9260fb_activate_var
 *
 * Activate (set) the controller from the given framebuffer
 * information.
*/

static void asm9260fb_activate_var(struct asm9260fb_info *fbi,
				   struct fb_var_screeninfo *var)
{
    DBG("Entered asm9260fb_activate_var()\n");
}


/*
 *      asm9260fb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 */
static int asm9260fb_set_par(struct fb_info *info)
{
	struct asm9260fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;

        DBG("asm9260fb_set_par()\n");
	if (var->bits_per_pixel == 16)
		fbi->fb->fix.visual = FB_VISUAL_TRUECOLOR;
	else
		fbi->fb->fix.visual = FB_VISUAL_PSEUDOCOLOR;

	fbi->fb->fix.line_length     = (var->width*var->bits_per_pixel)/8;


	asm9260fb_activate_var(fbi, var);
	return 0;
}


/* from pxafb.c */
static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int asm9260fb_setcolreg(unsigned regno,
                              unsigned red, unsigned green, unsigned blue,
                              unsigned transp, struct fb_info *info)
{
    struct asm9260fb_info *fbi = info->par;
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

    return 0;    
}


/**
 *      asm9260fb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. 
 *	Returns negative errno on error, or zero on success.
 *
 */
static int asm9260fb_blank(int blank_mode, struct fb_info *info)
{ 
   DBG("blank(mode=%d, is_rgb_mode=%d)\n", blank_mode, is_rgb_mode);
    if (!blank_mode) { return 0; }

    if(is_rgb_mode==LCD_MODE_RGB){
        memset(info->screen_base, RGB_BLANK, info->screen_size);
    }
    else{
        DBG("info->screen_base=%p, (info->screen_size>>1)=%d)\n", info->screen_base,(info->screen_size>>1));
        memset(info->screen_base, Y_BLANK, (info->screen_size>>1)); // Y
        memset(info->screen_base + (info->screen_size>>1), UV_BLANK, (info->screen_size>>2)); // UV
    }
	return 0;
}

static int asm9260fb_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", lcd_debug ? "on" : "off");
}
static int asm9260fb_debug_store(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t len)
{
    if (mach_info == NULL)
        return -EINVAL;

    if (len < 1)
        return -EINVAL;

    if (strnicmp(buf, "on", 2) == 0 ||
        strnicmp(buf, "1", 1) == 0) {
        lcd_debug = LCD_DBG_ON;
        printk(KERN_DEBUG "asm9260fb: Debug On");
    } else if (strnicmp(buf, "off", 3) == 0 ||
               strnicmp(buf, "0", 1) == 0) {
        lcd_debug = LCD_DBG_OFF;
        printk(KERN_DEBUG "asm9260fb: Debug Off");
    } else {
        return -EINVAL;
    }

    return len;
}

 /* Start the DMA, dedicated channel for U DMA. */ 
 void u_dma_start(ulong pkg_addr,int pkg_num){
    as3310_writel_lcd(pkg_addr ,HW_APBH_LCD_CH2_NXTCMDAR);
    as3310_writel_lcd(LCD_CH2_SEMA ,HW_APBH_LCD_CH2_SEMA);
}

 /* Start the DMA, dedicated channel for V DMA. */ 
 void v_dma_start(ulong pkg_addr,int pkg_num){
    as3310_writel_lcd(pkg_addr ,HW_APBH_LCD_CH1_NXTCMDAR);
    as3310_writel_lcd(LCD_CH1_SEMA ,HW_APBH_LCD_CH1_SEMA);
}

 /* Start the DMA, dedicated channel for LCD DMA. */ 
 void lcd_dma_start(ulong pkg_phy_addr,int pkg_num){
     as3310_writel_lcd(pkg_phy_addr ,HW_APBH_LCD_CH0_NXTCMDAR);
     as3310_writel_lcd(LCD_CH0_SEMA ,HW_APBH_LCD_CH0_SEMA);
}

/*
 *	Start the LCD.
 */ 
int lcd_start_lcd(int mode)
{
	if(mode==LCD_MODE_RGB)
		{
			lcd_dma_start(irq_lcd_pkg_addr ,LCD_PKG_NUM-1);
			as3310_writel_lcd(CYCLE_MODE_RGB,HW_APBH_LCD_CTRL0_SET);
			as3310_writel_lcd(LCD_BURST_CLEAR,HW_APBH_LCD_CTRL0_CLR);
			as3310_writel_lcd(LCD_BURST_MODE,HW_APBH_LCD_CTRL0_SET);
		}
	else
  		{
  			lcd_dma_start(irq_lcd_pkg_addr_y ,LCD_PKG_NUM_Y);
  			u_dma_start(irq_lcd_pkg_addr_u ,LCD_PKG_NUM_U);
  			v_dma_start(irq_lcd_pkg_addr_v ,LCD_PKG_NUM_V);
			as3310_writel_lcd(CYCLE_MODE_YUV,HW_APBH_LCD_CTRL0_SET);
			as3310_writel_lcd(LCD_BURST_CLEAR,HW_APBH_LCD_CTRL0_CLR);
			as3310_writel_lcd(LCD_BURST_MODE,HW_APBH_LCD_CTRL0_SET);
  		}
	return 0;
}
EXPORT_SYMBOL_GPL(lcd_start_lcd);

/*
 *	Set the colour mode of LCD.
 */
static int  lcd_set_mode(int mode)
{
    if(mode==LCD_MODE_RGB)
        {
    		lcd_dma_start(irq_lcd_pkg_addr ,LCD_PKG_NUM-1);
    		as3310_writel_lcd(CYCLE_MODE_RGB,HW_APBH_LCD_CTRL0_SET);
		as3310_writel_lcd(LCD_BURST_CLEAR,HW_APBH_LCD_CTRL0_CLR);
		as3310_writel_lcd(LCD_BURST_MODE,HW_APBH_LCD_CTRL0_SET);
		#ifdef CONFIG_FB_EVK_480_272
			as3310_writel_lcd(LCD_TIMING0_RGB + ((SCREEN_SIZE_COL - ASM9260_LCD_XRES)<<16)
	          + (ASM9260_LCD_XRES - 1),HW_LCD_CTRL_TIMING0);//ASM9260_LCD_XRES-1  pixels/line 
	          as3310_writel_lcd(LCD_TIMING1_RGB + ((SCREEN_SIZE_ROW - ASM9260_LCD_YRES)<<16)
	          + (ASM9260_LCD_YRES - 1),HW_LCD_CTRL_TIMING1);//ASM9260_LCD_YRES-1  lines 
	          as3310_writel_lcd(LCD_TIMING2_RGB,HW_LCD_CTRL_TIMING2);//pix_clk;
		#endif
		#ifdef CONFIG_FB_FTG500C06Z_800_480
			as3310_writel_lcd(LCD_TIMING0_RGB + ((SCREEN_SIZE_COL - ASM9260_LCD_XRES)<<16)
		     + (ASM9260_LCD_XRES - 1),HW_LCD_CTRL_TIMING0);//ASM9260_LCD_XRES-1  pixels/line 
		     as3310_writel_lcd(LCD_TIMING1_RGB + ((SCREEN_SIZE_ROW - ASM9260_LCD_YRES)<<16)
		     + (ASM9260_LCD_YRES - 1),HW_LCD_CTRL_TIMING1);//ASM9260_LCD_YRES-1  lines 
		     as3310_writel_lcd(LCD_TIMING2_RGB |(1<<LCD_HSYNC_ACTIVE_LOW)
		     |(1<<LCD_VSYNC_ACTIVE_LOW),HW_LCD_CTRL_TIMING2);
		#endif
    	}
    else
	{
		lcd_dma_start(irq_lcd_pkg_addr_y ,LCD_PKG_NUM_Y);
		u_dma_start(irq_lcd_pkg_addr_u ,LCD_PKG_NUM_U);
		v_dma_start(irq_lcd_pkg_addr_v ,LCD_PKG_NUM_V);
    		as3310_writel_lcd(CYCLE_MODE_YUV,HW_APBH_LCD_CTRL0_SET);
		as3310_writel_lcd(LCD_BURST_CLEAR,HW_APBH_LCD_CTRL0_CLR);
		as3310_writel_lcd(LCD_BURST_MODE,HW_APBH_LCD_CTRL0_SET);
		as3310_writel_lcd(LCD_TIMING0_YUV + ((SCREEN_SIZE_COL - ASM9260_LCD_XRES)<<16)
     		+ (ASM9260_LCD_XRES - 1),HW_LCD_CTRL_TIMING0);//ASM9260_LCD_XRES-1  pixels/line 
    		as3310_writel_lcd(LCD_TIMING1_YUV + ((SCREEN_SIZE_ROW - ASM9260_LCD_YRES)<<16)
    		 + (ASM9260_LCD_YRES - 1),HW_LCD_CTRL_TIMING1);//ASM9260_LCD_YRES-1  lines 
    		as3310_writel_lcd(LCD_TIMING2_YUV,HW_LCD_CTRL_TIMING2);
	}
    return 0;
}

/*
 *	/dev/fb0 handling.
 */
int asm9260fb_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg){
    int ret = 0;
    struct fb_var_screeninfo var;
    struct fb_fix_screeninfo fix;
    void __user *argp = (void __user*)arg;
    switch (cmd) {
    case FBIO_RGB://set RGB mode
         is_rgb_mode = LCD_MODE_RGB;
	    break;
    case FBIO_YUV://set YUV mode
         is_rgb_mode = LCD_MODE_YUV;
	    break;
    case FBIO_OFF://stop refresh
        is_lcd_on = LCD_ON;
	    break;
    case FBIO_ON:
        is_lcd_on = LCD_OFF;//start refresh
         break;
    case FBIO_SYNC:
	   lcd_start_lcd(is_rgb_mode);
         break;
    case FBIO_GET_DISP_MODE:      
        return put_user(is_rgb_mode, (int *)arg);
    case FBIOGET_VSCREENINFO:
        return copy_to_user(argp, &info->var, sizeof(var)) ? -EFAULT: 0;
    case FBIOGET_FSCREENINFO: 
        return copy_to_user(argp, &info->fix, sizeof(fix)) ? -EFAULT: 0;
    default:
        DBG("asm9260fb:IOCTRL=0x%x is not support",cmd);
        ret = -EINVAL;//invaild cmd
        break;
    }
    return ret;
}

/* Kernel interface. */
static DEVICE_ATTR(debug, 0666,
		   asm9260fb_debug_show,
		   asm9260fb_debug_store);

static struct fb_ops asm9260fb_ops = {
    .owner      = THIS_MODULE,
    .fb_check_var   = asm9260fb_check_var,
    .fb_set_par = asm9260fb_set_par,
    .fb_blank   = asm9260fb_blank,
    .fb_setcolreg   = asm9260fb_setcolreg,
    .fb_fillrect    = cfb_fillrect,
    .fb_copyarea    = cfb_copyarea,
    .fb_imageblit   = cfb_imageblit,
    .fb_ioctl   = asm9260fb_fb_ioctl,
};


/*
 * asm9260fb_map_video_memory():
 *	Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *	allow palette and pixel writes to occur without flushing the
 *	cache.  Once this area is remapped, all virtual memory
 *	access to the video memory should occur at the new region.
 */
static int __init asm9260fb_map_video_memory(struct asm9260fb_info *fbi)
{
	DBG("map_video_memory(fbi=%p)\n", fbi);

        ulong temp;
        int i,j;
	u8 Y,U,V;
#ifdef CONFIG_FB_VIRTUAL_VGA

	fbi->map_size1 = PAGE_ALIGN(SCREEN_SIZE_COL * SCREEN_SIZE_ROW *2 + PAGE_SIZE);
        fbi->map_cpu1  = dma_alloc_writecombine(fbi->dev, fbi->map_size1,
					       &fbi->map_dma1, GFP_KERNEL);


	if (fbi->map_cpu1) {
		DBG("map_video_phy_memory 1: dma1=%08x cpu1 =%p size1=%08x\n",
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

     if (fbi->map_cpu) 
     {
     	/* prevent initial garbage on screen */
     DBG("map_video_memory: clear %p:%08x\n",
     		fbi->map_cpu, fbi->map_size);
     
     fbi->screen_dma	= fbi->map_dma;
     fbi->fb->screen_base	= fbi->map_cpu;
     fbi->fb->screen_size	= fbi->map_size;
     fbi->fb->fix.smem_start  = fbi->screen_dma;
     is_rgb_mode = 0;
	  
#ifdef CONFIG_FB_EVK_480_272
      if(is_rgb_mode==LCD_MODE_RGB){
          for (i=0;i<_rgb_max_y;i++)
              for (j=0;j<_rgb_max_x;j++) {
			#ifdef CONFIG_FB_LCD_BPP_16	
	                  if (j<160) {
	                      *((u16 *)fbi->map_cpu+i*_rgb_max_x+j) = RGB565(0,0,255);
	                  } else if (j>320) {
	                      *((u16 *)fbi->map_cpu+i*_rgb_max_x+j) = RGB565(0,255,0);
	                  } else {
	                      *((u16 *)fbi->map_cpu+i*_rgb_max_x+j) = RGB565(255,0,0);
	                  }
			#else
				   if (i<90) {
	                      *((u32 *)fbi->map_cpu+i*_rgb_max_x+j) = RGB888(255,0,0);
	                  } else if (i>180) {
	                      *((u32 *)fbi->map_cpu+i*_rgb_max_x+j) = RGB888(0,255,0);
	                  } else {
	                      *((u32 *)fbi->map_cpu+i*_rgb_max_x+j) = RGB888(0,0,255);
	                  }
			#endif	  
              }
     	}
     else
     	{
      	for(i=0;i<_rgb_max_y;i++)
      		for(j=0;j<_rgb_max_x;j++)
      		{
      			if(i<90)
      				{
      				rgb2yuv(255,0,0,Y,U,V);
      				}
      			else if(i>180)
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
#endif 
     
#ifdef CONFIG_FB_FTG500C06Z_800_480
         for (i=0;i<SCREEN_SIZE_ROW;i++)
             for (j=0;j<SCREEN_SIZE_COL;j++) {
                 if (j<266) {
                     *((u16 *)fbi->map_cpu+i*SCREEN_SIZE_COL+j) = RGB565(255,0,0);
                 } else if (j>534) {
                     *((u16 *)fbi->map_cpu+i*SCREEN_SIZE_COL+j) = RGB565(0,255,0);
                 } else {
                     *((u16 *)fbi->map_cpu+i*SCREEN_SIZE_COL+j) = RGB565(0,0,255);
                 }               
             }
#endif  
     DBG("map_video_phy_memory: dma=%08x cpu =%p size=%08x\n",
     		fbi->map_dma, fbi->map_cpu, fbi->fb->fix.smem_len);
     
     }
     
     fbi->dmachain = request_asm9260_dma_chain(fbi->dev,LCD_PKG_NUM,ASM9260_DMA_LCD_CH);
     fbi->dmachain_y = request_asm9260_dma_chain(fbi->dev,LCD_PKG_NUM_Y,ASM9260_DMA_LCD_Y_CH);
     fbi->dmachain_u = request_asm9260_dma_chain(fbi->dev,LCD_PKG_NUM_U,ASM9260_DMA_LCD_U_CH);
     fbi->dmachain_v = request_asm9260_dma_chain(fbi->dev,LCD_PKG_NUM_V,ASM9260_DMA_LCD_V_CH);
     
     if (fbi->dmachain) {
     	DBG("lcd_dma_pkg_phy_memory: dma=%08x cpu =%p\n",
     		fbi->dmachain->chain_phy_addr, fbi->dmachain->chain_head);
         chain_head_p = fbi->dmachain->chain_head;
     }
     
     _lcd_table = dma_alloc_writecombine(fbi->dev, sizeof(struct asm9260_lcd_table),
     				       (dma_addr_t *)&temp, GFP_KERNEL);
     
     
     if (_lcd_table)
     {
         _lcd_table->dma_addr = temp;
     DBG("lcd_color_table_phy_memory: dma=%08x cpu =%p size=%08x\n",
     		_lcd_table->dma_addr, _lcd_table, sizeof(struct asm9260_lcd_table));
     }
     
	return fbi->map_cpu ? 0 : -ENOMEM;
}

static inline void asm9260fb_unmap_video_memory(struct asm9260fb_info *fbi)
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
 * asm9260fb_init_registers - Initialise all LCD-related registers.
 */

int asm9260fb_init_registers(struct asm9260fb_info *fbi)
{
    int dma_time_out;
    int j,pix_clk;
#ifdef CONFIG_FB_EVK_480_272
    printk("Configure for EVK (480X272) TFT LCD\n");
#endif

#ifdef CONFIG_FB_FTG500C06Z_800_480
    printk("Configure for FTG500C06Z (800X480) TFT LCD\n");
#endif


#ifdef CONFIG_FB_VIRTUAL_VGA
    printk("Configure for Vitual VGA Framebuffer Screen\n");
#endif
    /*     Pin Assign      */
     for(j=4;j<=7;j++)
	{
	     set_pin_mux(1, j, 6);//LCD LP FP CP AC
	}
      
    for(j=0;j<=7;j++)
	{
	     set_pin_mux(2, j, 6);//LCD PIXEL
	}

    for(j=0;j<=7;j++)
	{
	     set_pin_mux(3, j, 6);//LCD PIXEL
	}
    for(j=0;j<=7;j++)
	{
	     set_pin_mux(4, j, 6);//LCD PIXEL
	}
	set_pin_mux(16, 0, 2);//UART4 TX
	set_pin_mux(16, 1, 2);//UART4 RX
    pix_clk = LCD_DEFAULT_PIXCLK_DIVIDER;

    DBG("LCD Pixel Clock: %d\n",pix_clk);
     //HW_CLKCTRL_PIXELCLKCTRL
    as3310_writel(1<<14,HW_AHBCLKCTRL1+4);
    as3310_writel(1<<14,HW_PRESETCTRL1+8);
    as3310_writel(1<<14,HW_PRESETCTRL1+4);
    as3310_writel(LCDCLKDIV,HW_LCDCLKDIV);

    /*     LCD Init  registers      */
    as3310_writel_lcd(LCD_CTRL0_GATE,HW_LCD_CTRL_CTRL0+8);//clear the gate used to sync rst
    as3310_writel_lcd(LCD_CTRL0_RESET,HW_LCD_CTRL_CTRL0+4);//set the rst        
    as3310_writel_lcd(LCD_CTRL0_GATE,HW_LCD_CTRL_CTRL0+8);//clear the gate used to sync rst
    as3310_writel_lcd(LCD_CTRL0_RESET,HW_LCD_CTRL_CTRL0+8);//clear the rst           


#ifdef CONFIG_FB_EVK_480_272
    as3310_writel_lcd(LCD_TIMING0_RGB + ((SCREEN_SIZE_COL - ASM9260_LCD_XRES)<<16)
    + (ASM9260_LCD_XRES - 1),HW_LCD_CTRL_TIMING0);//ASM9260_LCD_XRES-1  pixels/line 
    as3310_writel_lcd(LCD_TIMING1_RGB + ((SCREEN_SIZE_ROW - ASM9260_LCD_YRES)<<16)
    + (ASM9260_LCD_YRES - 1),HW_LCD_CTRL_TIMING1);//ASM9260_LCD_YRES-1  lines 
    as3310_writel_lcd(LCD_TIMING2_RGB,HW_LCD_CTRL_TIMING2);//pix_clk
#endif

#ifdef CONFIG_FB_FTG500C06Z_800_480  
    as3310_writel_lcd(LCD_TIMING0_RGB + ((SCREEN_SIZE_COL - ASM9260_LCD_XRES)<<16)
    + (ASM9260_LCD_XRES - 1),HW_LCD_CTRL_TIMING0);//ASM9260_LCD_XRES-1  pixels/line 
    as3310_writel_lcd(LCD_TIMING1_RGB + ((SCREEN_SIZE_ROW - ASM9260_LCD_YRES)<<16)
    + (ASM9260_LCD_YRES - 1),HW_LCD_CTRL_TIMING1);//ASM9260_LCD_YRES-1  lines 
    as3310_writel_lcd(LCD_TIMING2_RGB,HW_LCD_CTRL_TIMING2);//pix_clk
#endif


    as3310_writel_lcd((ASM9260_LCD_XRES - 1),HW_LCD_CTRL_TIMING3);//PPL
  
    as3310_writel_lcd(APBH_CTRL0_GATE,HW_APBH_LCD_CTRL0_CLR);//clear the clk gate
    as3310_writel_lcd(APBH_CTRL0_RESET,HW_APBH_LCD_CTRL0_SET);//set sftrst                                                  
    as3310_writel_lcd(APBH_CTRL0_GATE,HW_APBH_LCD_CTRL0_CLR);//clear the clk gate
    as3310_writel_lcd(APBH_CTRL0_RESET,HW_APBH_LCD_CTRL0_CLR);//clear sftrst  
    as3310_writel_lcd(APBH_CHAN_GATE,HW_APBH_LCD_CTRL0_CLR);  
    as3310_writel_lcd(LCD_BURST_CLEAR,HW_APBH_LCD_CTRL0_CLR);
    as3310_writel_lcd(LCD_BURST_MODE,HW_APBH_LCD_CTRL0_SET);//set burst mode
  
    as3310_writel_lcd(LOWTHESHOLD,HW_LCD_LOWTHESHOLD);//lcd_ctrl->LOWTHESHOLD[0] = LOWTHESHOLD;
    as3310_writel_lcd(UPTHESHOLD,HW_LCD_UPTHESHOLD);//lcd_ctrl->UPTHESHOLD[0] = UPTHESHOLD;     fbi->dmachain->chain_phy_addr                                         
    lcd_dma_start(fbi->dmachain->chain_phy_addr,1); // sent color table
    DBG("started lcd table chain at 0x%08x\n",fbi->dmachain->chain_phy_addr); 
    as3310_writel_lcd(LCD_CTRL1|(1<<LCD_CTRL_LCDEN)|(1<<LCD_CTRL_LCDTFT)
    |(LCD_PALETTE_LOAD<<20),HW_LCD_CTRL_CTRL1+4);//load p and lcd_en=1  tff
    as3310_writel_lcd(LCD_CTRL0,HW_LCD_CTRL_CTRL0);//en_receive and startp and receive 16*16bits  
    dma_time_out = LCD_DMA_TIMEOUT;
    while(((as3310_readl_lcd(HW_LCD_CTRL_STAT))&LCD_CTRL_STAT)==0){
        if (dma_time_out-- < 0 ) 
           {printk("Table DMA Time Out\n"); 
             break;
           }
    }

    as3310_writel_lcd(LCD_CTRL1|(1<<LCD_CTRL_LCDTFT)
    |(LCD_DATA_LOAD<<20),HW_LCD_CTRL_CTRL1);//load data and lcd_en=0 tff
    as3310_writel_lcd(LCD_CTRL1|(1<<LCD_CTRL_LCDEN)|(1<<LCD_CTRL_LCDTFT)
    |(LCD_DATA_LOAD<<20),HW_LCD_CTRL_CTRL1);//load data and lcd_en=1 tff 
    irq_lcd_pkg_addr = fbi->dmachain->chain_head[0].NEXT_PKG;
    irq_lcd_pkg_addr_y = fbi->dmachain_y->chain_phy_addr;
    irq_lcd_pkg_addr_u = fbi->dmachain_u->chain_phy_addr;
    irq_lcd_pkg_addr_v = fbi->dmachain_v->chain_phy_addr;

    as3310_writel_lcd(LCD_DMA_CYCLE_EN,HW_APBH_LCD_CTRL0_SET);
    as3310_writel_lcd(DMA_CH_CYCLE,HW_APBH_LCD_CTRL0_CLR);
    is_lcd_on = LCD_ON;

    lcd_set_mode(is_rgb_mode);

    return 0;
}

static void asm9260fb_write_palette(struct asm9260fb_info *fbi)
{
    unsigned int i;
    unsigned long ent;

    fbi->palette_ready = 0;

    for (i = 0; i < 256; i++) {
        if ((ent = fbi->palette_buffer[i]) == PALETTE_BUFF_CLEAR)
            continue;

        as3310_writel(ent, ASM9260_TFTPAL(i));

        /* it seems the only way to know exactly
         * if the palette wrote ok, is to check
         * to see if the value verifies ok
         */

        if (as3310_readw(ASM9260_TFTPAL(i)) == ent)
            fbi->palette_buffer[i] = PALETTE_BUFF_CLEAR;
        else
            fbi->palette_ready = 1;   /* retry */
    }
}


static char driver_name[]="asm9260fb";


/*
 *  Prepare DMA packets for the lcd.
 */
int init_asm9260_lcd_dma_pkg(struct asm9260fb_info *bf_i){

    struct asm9260_dma_chain * pdmachain;
    struct asm9260_dma_pkg_s * dmapkg;

    struct asm9260_dma_pkg_s * dmapkg_y;
    struct asm9260_dma_pkg_s * dmapkg_u;
    struct asm9260_dma_pkg_s * dmapkg_v;

    ulong dma_phy_addr;
    ulong len,len_div;
    
    ulong cmd;
    int i;


#ifdef CONFIG_FB_VIRTUAL_VGA
    dma_phy_addr = bf_i->map_dma1;  // display actual size
    len = SCREEN_SIZE_COL*SCREEN_SIZE_ROW*LCD_BPP;//16bpp
#else
    dma_phy_addr = bf_i->map_dma;
    len = ASM9260_LCD_XRES*ASM9260_LCD_YRES*LCD_BPP;//16bpp
#endif
/*  ================= for RGB Packages ==================*/

    len_div=len/(LCD_PKG_NUM-1);

    _lcd_table->table[0] = LCD_TABLE;

    /*     Prepare DMA PKG */
    pdmachain = bf_i->dmachain;
    dmapkg = pdmachain->chain_head;


           // load color table
    dmapkg[0].CTRL = APBH_CH0_CMD_TABLE; //DMA channel0 command register
    dmapkg[0].BUFFER = _lcd_table->dma_addr; //DMA channel0 buffer address register
    dmapkg[0].CMD0 = LCD_CH0_CTRL0_TABLE; //LCD ctrl0 register
    DBG("\nFrameBuffer Physical Address: 0x%08x\n",dma_phy_addr);
  

	     // load data
    cmd = (LCD_CH0_CTRL0  + (len_div>>1)) | (1<<LCD_FIFO_PRIV_HIGH) | (1<<LCD_FIFO_PRIV_LOW);
    for(i=1;i<LCD_PKG_NUM;i++)
	{
	    dmapkg[i].CTRL = APBH_CH0_CMD + (len_div<<16);    //0x9600  bytes and one chain and to lcdctrl one pio write and no dec semphone
	    dmapkg[i].BUFFER = (char *)(dma_phy_addr+(i-1)*len_div);
	    dmapkg[i].CMD0 = cmd;//enrev and startf and en irq and 0x6*16bits
	}
    dmapkg[LCD_PKG_NUM-1].CTRL = APBH_CH0_CMD_LAST + (len_div<<16);
    dmapkg[1].CMD0 = cmd | LCD_CH_STARTF;    


/*  ================= for YUV Packages ==================*/

    len = ASM9260_LCD_XRES*ASM9260_LCD_YRES;
    len_div=(len/(LCD_PKG_NUM-1));

    dmapkg_y = bf_i->dmachain_y->chain_head;
    dmapkg_u = bf_i->dmachain_u->chain_head;
    dmapkg_v = bf_i->dmachain_v->chain_head;

    /*     Prepare DMA PKG */
    cmd = (LCD_CH0_CTRL0_Y + (len_div>>2)) | (1<<LCD_FIFO_PRIV_HIGH) | (1<<LCD_FIFO_PRIV_LOW);
		
    for(i=0;i<LCD_PKG_NUM-1;i++)
	{
	    dmapkg_y[i].CTRL = APBH_CH0_CMD + (len_div<<16);    
	    dmapkg_y[i].BUFFER = (char *)(dma_phy_addr+i*len_div);
	    dmapkg_y[i].CMD0 = cmd;
	}

    dmapkg_y[LCD_PKG_NUM-2].CTRL = APBH_CH0_CMD_LAST + (len_div<<16);
    dmapkg_y[0].CMD0 = cmd | LCD_CH_STARTF;

    cmd = ( (len_div>>3)) | (1<<LCD_FIFO_PRIV_HIGH) | (1<<LCD_FIFO_PRIV_LOW);
	
    for(i=0;i<(LCD_PKG_NUM-1)/2;i++)
	{
	    dmapkg_u[i].CTRL = APBH_CH2_CMD + (len_div<<15);    
	    dmapkg_u[i].BUFFER = (char *)(dma_phy_addr+len+i*len_div/2);
	    dmapkg_u[i].CMD0 = cmd;
	}
    dmapkg_u[(LCD_PKG_NUM-1)/2-1].CTRL = APBH_CH2_CMD_LAST + (len_div<<15);

    cmd = ( (len_div>>3)) | (1<<LCD_FIFO_PRIV_HIGH) | (1<<LCD_FIFO_PRIV_LOW);
    for(i=0;i<(LCD_PKG_NUM-1)/2;i++)
	{
	    dmapkg_v[i].CTRL = APBH_CH1_CMD + (len_div<<15);    
	    dmapkg_v[i].BUFFER = (char *)(dma_phy_addr+len*5/4+i*len_div/2);
	    dmapkg_v[i].CMD0 = cmd;
	}
	dmapkg_v[(LCD_PKG_NUM-1)/2-1].CTRL = APBH_CH1_CMD_LAST + (len_div<<15);
    return 0;

}



/*
 *  Probe for the lcd.
 */
int __init asm9260fb_probe(struct device *dev)
{
	struct asm9260fb_info *info;
	struct fb_info	   *fbinfo;
  	struct asm9260fb_hw *mregs;
	int ret;
	int i;

	mach_info = (struct asm9260fb_mach_info *)dev->platform_data;
	if (mach_info == NULL) {
		dev_err(dev,"no platform data for lcd, cannot attach\n");
		return -EINVAL;
	}

	mregs = &mach_info->regs;

	fbinfo = framebuffer_alloc(sizeof(struct asm9260fb_info), dev);
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
	fbinfo->fix.visual      = FB_VISUAL_TRUECOLOR; 
	
	fbinfo->var.nonstd	    = 0;
	fbinfo->var.activate	= FB_ACTIVATE_NOW;
	fbinfo->var.height	    = mach_info->height;
	fbinfo->var.width	    = mach_info->width;

     DBG("fbinfo->var.height = %d, fbinfo->var.width = %d\n",mach_info->height,mach_info->width);

	fbinfo->var.accel_flags     = 0;
	fbinfo->var.vmode	    = FB_VMODE_NONINTERLACED;


	fbinfo->fbops		    = &asm9260fb_ops;
	fbinfo->flags		    = FBINFO_FLAG_DEFAULT;
	fbinfo->pseudo_palette      = &info->pseudo_pal;

	
     fbinfo->var.xres	    = mach_info->xres.defval;
	fbinfo->var.xres_virtual    = mach_info->xres.defval;
	fbinfo->var.yres	    = mach_info->yres.defval;
	fbinfo->var.yres_virtual    = mach_info->yres.defval;
	fbinfo->var.bits_per_pixel  = mach_info->bpp.defval;
	
	fbinfo->var.upper_margin    = 1;
	fbinfo->var.lower_margin    = 1;
	fbinfo->var.vsync_len	    = mach_info->xres.defval * mach_info->yres.defval +1 ;
	
	fbinfo->var.left_margin	    = 1;
	fbinfo->var.right_margin    = 1;
	fbinfo->var.hsync_len	    = mach_info->xres.defval + 1;
	
	fbinfo->var.red.offset      = ASM9260_888_R_OFFSET;//888 mode R at bit 16
	fbinfo->var.green.offset    = ASM9260_888_G_OFFSET;//888 mode  G at bit 8
	fbinfo->var.blue.offset     = ASM9260_888_B_OFFSET;//888 mode B at bit 0
	fbinfo->var.transp.offset   = 0;
	fbinfo->var.red.length      = ASM9260_888_R_LEN;//888 mode
	fbinfo->var.green.length    = ASM9260_888_G_LEN;
	fbinfo->var.blue.length     = ASM9260_888_B_LEN; 
	fbinfo->var.transp.length   = 0;
	fbinfo->fix.smem_len        =	mach_info->xres.max *
					mach_info->yres.max *
					mach_info->bpp.max / 8;//bpp.max=32bits
     fbinfo->fix.line_length     =(fbinfo->var.width*fbinfo->var.bits_per_pixel)/8;//bits_per_pixel=16bits

	for (i = 0; i < ASM9260_LCD_PALETTE_NUM; i++)
		info->palette_buffer[i] = PALETTE_BUFF_CLEAR;
	
	if (!request_mem_region((unsigned long)LCD_REQ_MEM, SZ_256K, "asm9260-lcd")) {
		ret = -EBUSY;
		goto dealloc_fb;
	}


        info->framerate = ASM9260_DEFAULT_FRAME_RATE;

	msleep(1);

	ret = asm9260fb_map_video_memory(info);
	if (ret) {
		printk( KERN_ERR "Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto release_clock;
	}
       init_asm9260_lcd_dma_pkg(info);

       DBG("got video memory\n");

       ret = asm9260fb_init_registers(info);

       DBG("asm9260fb_init_registers\n");

       ret = asm9260fb_check_var(&fbinfo->var, fbinfo);

       DBG("asm9260fb_check_var\n");

       ret = register_framebuffer(fbinfo);
       if (ret < 0) {
		printk(KERN_ERR "Failed to register framebuffer device: %d\n", ret);
		goto free_video_memory;
       }
	/* create device files */
       device_create_file(dev, &dev_attr_debug);

       printk(KERN_INFO "fb%d: %s frame buffer device\n",
                  fbinfo->node, fbinfo->fix.id);
	   
	  printk("%s success!\n",__func__);
	return 0;

free_video_memory:
	asm9260fb_unmap_video_memory(info);
        del_timer(&lcd_temer);
release_clock:

dealloc_fb:
	framebuffer_release(fbinfo);
	return ret;
}


/* asm9260fb_stop_lcd
 *
 * Shutdown the lcd controller.
*/

static void asm9260fb_stop_lcd(void)
{
	unsigned long flags;

	local_irq_save(flags);
	local_irq_restore(flags);
}

/*
 *  Cleanup.
 */
static int asm9260fb_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fb_info	   *fbinfo = dev_get_drvdata(dev);
	struct asm9260fb_info *info = fbinfo->par;
	int irq;

	asm9260fb_stop_lcd();
	msleep(1);
	asm9260fb_unmap_video_memory(info);
 	if (info->clk) {
 
 		info->clk = NULL;
	}

	irq = platform_get_irq(pdev, 0);
	free_irq(irq,info);
	unregister_framebuffer(fbinfo);

	return 0;
}

#ifdef CONFIG_PM

/* Suspend and resume support for the lcd controller. */

static int asm9260fb_suspend(struct device *dev, pm_message_t state, u32 level)
{
	return 0;
}

static int asm9260fb_resume(struct device *dev, u32 level)
{
	return 0;
}

#else
#define asm9260fb_suspend NULL
#define asm9260fb_resume  NULL
#endif


static struct asm9260fb_mach_info fb_platform_data = {
	.width      =   ASM9260_LCD_XRES,
	.height     =   ASM9260_LCD_YRES,
    .xres               ={
        .defval     =   ASM9260_LCD_XRES,
        .min        =   (ASM9260_LCD_XRES>>3),
        .max        =   ASM9260_LCD_XRES,
    },
    .yres               ={
        .defval     =   ASM9260_LCD_YRES,
        .min        =   (ASM9260_LCD_YRES>>3),
        .max        =   ASM9260_LCD_YRES,
    },
#ifdef CONFIG_FB_LCD_BPP_16
    .bpp               ={
        .defval     =   16,
        .min        =   8,
        .max        =  16,
    },
#else
    .bpp               ={
        .defval     =   32,
        .min        =   8,
        .max        =  32,
    },
#endif
    .regs               ={
        .lcdcon1     =   LCDIF_CTRL_VALUE,
        .lcdcon2     =   LCDIF_TIMING_VALUE,
    },

};

static struct platform_device fb_device = {
	.name			= "asm9260-lcd",
	.id			= 0,
	.dev			= {
		.platform_data	= &fb_platform_data,
	},
};


static struct device_driver asm9260fb_driver = {
	.name		= "asm9260-lcd",
	.bus		= &platform_bus_type,
	.probe		= asm9260fb_probe,
	.suspend	= asm9260fb_suspend,
	.resume		= asm9260fb_resume,
	.remove		= asm9260fb_remove
};

int __devinit asm9260fb_init(void)
{
    DBG("asm9260fb_init()!\n");
    platform_device_register(&fb_device);
    if (IS_ERR(&fb_device)) {
	   PTR_ERR(&fb_device);
        DBG("asm9260-lcd platform_device_register ERROR!\n");
    }
	return driver_register(&asm9260fb_driver);
}

static void __exit asm9260fb_exit(void)
{
     DBG("asm9260fb_EXIT()!\n");
	driver_unregister(&asm9260fb_driver);
}

module_init(asm9260fb_init);
module_exit(asm9260fb_exit);

MODULE_AUTHOR("AlpScale Inc");
MODULE_DESCRIPTION("Framebuffer driver for the asm9260");
MODULE_LICENSE("GPL");

