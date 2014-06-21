/*
 * A driver for the CMOS camera controller in the Asm9260
 * multifunction chip .  Currently works with the Omnivision OV7670
 * sensor.
 *
 * 2012-12-15: Shanjs <shanjs@alpscale.cn>
 * 	   Create File
 *	    based on cafe-ccic.c and others.
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-chip-ident.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <mach/pincontrol.h>
#include <mach/dma.h>

#include "asm9260camif.h"

static int loop = 0;
static int oldloop = 1;
static int preloop = 0;
static int irq;
static int colour_mode;

DmaChain *cam_dma;
static u_char *map_cpu = NULL;
extern void set_pin_mux(int port,int pin,int mux_type);

#ifdef CONFIG_SOC_CAMERA_ASM9260_DEBUG
#define DBG(x...) printk("ASM9260_CAMERA_DBG: " x)
#else
#define DBG(x...)
#endif

/*
 * Internal DMA buffer management.  Since the controller cannot do S/G I/O,
 * we must have physically contiguous buffers to bring frames into.
 * These parameters control how many buffers we use, whether we
 * allocate them at load time (better chance of success, but nails down
 * memory) or when somebody tries to use the camera (riskier), and,
 * for load-time allocation, how big they should be.
 *
 * The controller can cycle through three buffers.  We could use
 * more by flipping pointers around, but it probably makes little
 * sense.
 */

static int alloc_bufs_at_read;
module_param(alloc_bufs_at_read, bool, 0444);
MODULE_PARM_DESC(alloc_bufs_at_read,
		"Non-zero value causes DMA buffers to be allocated when the "
		"video capture device is read, rather than at module load "
		"time.  This saves memory, but decreases the chances of "
		"successfully getting those buffers.");

#ifdef CONFIG_CAM_QVGA
static int n_dma_bufs = 3;
#endif

#ifdef CONFIG_CAM_VGA
static int n_dma_bufs = 2;
#endif

module_param(n_dma_bufs, uint, 0644);
MODULE_PARM_DESC(n_dma_bufs,
		"The number of DMA buffers to allocate.  Can be either two "
		"(saves memory, makes timing tighter) or three.");

#ifdef CONFIG_CAM_QVGA
static int dma_buf_size = QVGA_WIDTH * QVGA_HEIGHT * 2;  /* Worst case */
#endif

#ifdef CONFIG_CAM_VGA
static int dma_buf_size = VGA_WIDTH * VGA_HEIGHT * 2;  /* Worst case */
#endif

module_param(dma_buf_size, uint, 0444);
MODULE_PARM_DESC(dma_buf_size,
		"The size of the allocated DMA buffers.  If actual operating "
		"parameters require larger buffers, an attempt to reallocate "
		"will be made.");

static int min_buffers = 1;
module_param(min_buffers, uint, 0644);
MODULE_PARM_DESC(min_buffers,
		"The minimum number of streaming I/O buffers we are willing "
		"to work with.");

static int max_buffers = 10;
module_param(max_buffers, uint, 0644);
MODULE_PARM_DESC(max_buffers,
		"The maximum number of streaming I/O buffers an application "
		"will be allowed to allocate.  These buffers are big and live "
		"in vmalloc space.");

static int flip;
module_param(flip, bool, 0444);
MODULE_PARM_DESC(flip,
		"If set, the sensor will be instructed to flip the image "
		"vertically.");


/* Initialise the clock of related modules. */ 
static void clock_init(void)
{
    as3310_writel(0x1<<2, HW_AHBCLKCTRL1+0x4);  // open camera module apb_clk
    as3310_writel(0x1<<2, HW_PRESETCTRL1+0x8);
    mdelay(100);
    as3310_writel(0x1<<2, HW_PRESETCTRL1+0x4);  // softrst camera moule  RESET
    mdelay(200);

    as3310_writel(20, HW_CAMMCLKDIV);  // set camera mclk to 1/4 apb_clk
    as3310_writel(0x1<<2, HW_AHBCLKCTRL0+0x4);  // open ahb_ram module clk
    as3310_writel(0x1<<25, HW_AHBCLKCTRL0+0x4);  // open ioconfig module apb_clk
    as3310_writel(0x1<<4, HW_AHBCLKCTRL0+0x4);  // open GPIO module clk
    as3310_writel(0x1<<10, HW_AHBCLKCTRL0+4);  //dma1
}

/* Set the pins of camera controller. */ 
static void cam_pin_init(void)
{
    int i; 
    set_pin_mux(11, 7, 3); //CAM_MCLK 
    for(i=1;i<=7;i++){
        set_pin_mux(8, i, 3); //CAM_PCLK CAM_VSYN CAM_HREF CAM_DATA0~3
    }
	
    for(i=0;i<=5;i++){
        set_pin_mux(9, i, 3); //CAM_DATA4~9
    }
}

/*
 *  Create DMA packets for the camera.
 */
static int camera_dma_pkg_init(struct alp_camera *cam)
{
	int i=0,j=0;
	for (j=0; j<n_dma_bufs; j++){
      	for (i=0; i<CAMERA_PKG_NUM; i++){
      			cam_dma->chain_head[i+j*CAMERA_PKG_NUM].SAR = (u32)HW_DCMI_DR_ADDR;
      			cam_dma->chain_head[i+j*CAMERA_PKG_NUM].DAR = (u32)(cam->dma_handles[j]
      					+ i*((_rgb_max_x*_rgb_max_y*CAM_BPP)/CAMERA_PKG_NUM));
      			cam_dma->chain_head[i+j*CAMERA_PKG_NUM].CTRL_L = 
      					0x00000000 + (1<<0) 			// INT_EN
      							   + (2<<1) 			// DST_TR_WIDTH(8,16,32,64...) --[3:1]
      							   + (2<<4) 			// SRC_TR_WIDTH(8,16,32,64...) --[6:4]
      							   + (0<<7) 			// DINC   1x --> nochange
      							   + (2<<9) 			// SINC   00 --> increase
      							   + (2<<11)			// DST_BURST_SIZE 1
      							   + (1<<14)			// SRC_BURST_SIZE 1
      							   + (2<<20)			// TT_FC  00-->m2m 01-->m2p 10-->p2m 11-->p2p
      							   + (0<<23)			// DMS destination master interface
      							   + (0<<25)			// SMS source master interface
      							   + (1<<27)			// LLP_DST_EN
      							   + (0<<28);			// LLP_SRT_EN
      			cam_dma->chain_head[i+j*CAMERA_PKG_NUM].CTRL_H = 
      					0x00000000 + (((_rgb_max_x*_rgb_max_y/2)/CAMERA_PKG_NUM)<< 0); 	 //words
      	}
	}
	for (i=0; i<n_dma_bufs*CAMERA_PKG_NUM-1; i++)
		{
			cam_dma->chain_head[i].LLP= cam_dma->chain_phy_addr + ((i+1)*sizeof(DmaPkg));
		}
	
	cam_dma->chain_head[n_dma_bufs*CAMERA_PKG_NUM-1].LLP= cam_dma->chain_phy_addr;
	return 0;
}

/* Initialise the DMA related registers for camera. */ 
static int dma_apbh_camera_init(void)
{
	as3310_writel(0x1<<10, HW_AHBCLKCTRL0+4);  
	as3310_writel(HW_DCMI_DR_ADDR,HW_DMA1_SAR0);
	as3310_writel(0x0+(1<<0)+(1<<27)+(0<<28),HW_DMA1_CTL0); 
	as3310_writel(0x0+(1<<10)+(0<<30)+(0<<31),HW_DMA1_CFG0);  		 
	as3310_writel(0x0+(1<<1)+(0<<7)+(0<<11),HW_DMA1_CFG0+4);		
	as3310_writel(0x101,HW_DMA1_MaskTFR);
	as3310_writel(0x1,HW_DMA1_DMACFGREG);					 
	return 0;
}

static int dma_start_camera_apbh(ulong pkg_addr, int pkg_num)
{
	as3310_writel((u8*)pkg_addr,HW_DMA1_LLP0);
	as3310_writel(0x101,HW_DMA1_CHENREG);
	return 0;
}

/*Start the DMA for camera. */ 
static void cameraif_dma_start(void)
{
	dma_start_camera_apbh((ulong)(cam_dma->chain_phy_addr), 
	CAMERA_PKG_NUM);  
}


/*
 * Start over with DMA buffers - dev_lock needed.
 */
static void alp_reset_buffers(struct alp_camera *cam)
{
	int i;
	cam->next_buf = -1;
	for (i = 0; i < cam->nbufs; i++)
		clear_bit(i, &cam->flags);
	cam->specframes = 0;
}

static inline int alp_needs_config(struct alp_camera *cam)
{
	return test_bit(CF_CONFIG_NEEDED, &cam->flags);
}

static void alp_set_config_needed(struct alp_camera *cam, int needed)
{
	if (needed)
		set_bit(CF_CONFIG_NEEDED, &cam->flags);
	else
		clear_bit(CF_CONFIG_NEEDED, &cam->flags);
}


/* ---------------------------------------------------------------------*/
/*
 * We keep a simple list of known devices to search at open time.
 */
static LIST_HEAD(alp_dev_list);
static DEFINE_MUTEX(alp_dev_list_lock);

static void alp_add_dev(struct alp_camera *cam)
{
	mutex_lock(&alp_dev_list_lock);
	list_add_tail(&cam->dev_list, &alp_dev_list);
	mutex_unlock(&alp_dev_list_lock);
}

static void alp_remove_dev(struct alp_camera *cam)
{
	mutex_lock(&alp_dev_list_lock);
	list_del(&cam->dev_list);
	mutex_unlock(&alp_dev_list_lock);
}

static struct alp_camera *alp_find_dev(int minor)
{
	struct alp_camera *cam;

	mutex_lock(&alp_dev_list_lock);
	list_for_each_entry(cam, &alp_dev_list, dev_list) {
		if (cam->v4ldev.minor == minor)
			goto done;
	}
	cam = NULL;
  done:
	mutex_unlock(&alp_dev_list_lock);
	return cam;
}


static struct alp_camera *alp_find_by_pdev(struct pci_dev *pdev)
{
	struct alp_camera *cam;

	mutex_lock(&alp_dev_list_lock);
	list_for_each_entry(cam, &alp_dev_list, dev_list) {
		if (cam->pdev == pdev)
			goto done;
	}
	cam = NULL;
  done:
	mutex_unlock(&alp_dev_list_lock);
	return cam;
}


static int alp_cam_init(struct alp_camera *cam);
static void alp_ctlr_stop_dma(struct alp_camera *cam);

/* Attach the sensor on the bus. */
static int alp_smbus_attach(struct i2c_client *client)
{
	struct alp_camera *cam;
	int cam_num = 0;

	list_for_each_entry(cam, &alp_dev_list, dev_list) {
	    cam_num++;
	}

	if( unlikely( (cam_num > 1)||(cam_num == 0) ) ) {

	   if(cam_num > 1){
	    		DBG(KERN_ERR "%s multiple camera confusion!\n", __func__);
	   }else{
			DBG(KERN_ERR "%s no camera present!\n", __func__);
	   }	
	    return -1;
	}

	cam = list_entry( (&alp_dev_list)->next, struct alp_camera, dev_list); 
	
	if (client->driver->id == I2C_DRIVERID_OV7670) {
		cam->sensor = client;
		return alp_cam_init(cam);
	}
	
	printk("%s fail!\n", __func__);
	return -EINVAL;
}

/* Detach the sensor on the bus. */
static int alp_smbus_detach(struct i2c_client *client)
{
	struct alp_camera *cam;
	int cam_num = 0;

	list_for_each_entry(cam, &alp_dev_list, dev_list) {
	    cam_num++;
	}

	if( unlikely( (cam_num > 1)||(cam_num == 0) ) ) {
	    printk(KERN_ERR "%s multiple camera confusion!\n", __func__);
	    return -1;
	}

	cam = list_entry( (&alp_dev_list)->next, struct alp_camera, dev_list); 

	if (cam->sensor == client) {
		alp_ctlr_stop_dma(cam);
		cam_err(cam, "lost the sensor!\n");
		cam->sensor = NULL;  /* Bummer, no camera */
		cam->state = S_NOTREADY;
	}
	return 0;
}

/* Setup the bus for i2c_adapter. */
static int alp_smbus_setup(struct alp_camera *cam)
{
    struct i2c_adapter *adap;

    adap = i2c_get_adapter(I2C_CAM_INTERFACE_INDEX);

    if(adap == NULL) {
        DBG(KERN_ERR "%s: No required index i2c adapter.\n", __func__);
        return -1;
    }

    if( !(adap->class & I2C_CLASS_CAM_DIGITAL) ) {
        DBG(KERN_ERR "%s: Required i2c adapter CAM func not supported.\n", __func__);
        return -1;
    }

    adap->id = I2C_HW_SMBUS_ALP;
    adap->client_register = alp_smbus_attach;
    adap->client_unregister = alp_smbus_detach;
    
    return 0;
}

static void alp_smbus_shutdown(struct alp_camera *cam)
{
	i2c_del_adapter(&cam->i2c_adapter);
}

/* -------------------------------------------------------------------- */
/*
 * DMA buffer management.  These functions need s_mutex held.
 */

/* FIXME: this is inefficient as hell, since dma_alloc_coherent just
 * does a get_free_pages() call, and we waste a good chunk of an orderN
 * allocation.  Should try to allocate the whole set in one chunk.
 */
static int alp_alloc_dma_bufs(struct alp_camera *cam, int loadtime)
{
	int i;

	alp_set_config_needed(cam, 1);
	if (loadtime)
		cam->dma_buf_size = dma_buf_size;
	else
		cam->dma_buf_size = cam->pix_format.sizeimage;

	cam->nbufs = 0;
	for (i = 0; i < n_dma_bufs; i++) {
		cam->dma_bufs[i] = dma_alloc_writecombine(&cam->pdev->dev,
				cam->dma_buf_size, cam->dma_handles + i,
				GFP_KERNEL);
		if (cam->dma_bufs[i] == NULL) {
			cam_warn(cam, "Failed to allocate DMA buffer\n");
			break;
		}
		/* For debug, remove eventually */
		memset(cam->dma_bufs[i], 0x00, cam->dma_buf_size);
		map_cpu = (u_char *)cam->dma_bufs[0];
	}
	cam->nbufs = n_dma_bufs;
	return 0;
}


/*
 * Free the DMA buffer.
 */
static void alp_free_dma_bufs(struct alp_camera *cam)
{
	int i;
	for (i = 0; i < cam->nbufs; i++) {
		dma_free_coherent(&cam->pdev->dev, cam->dma_buf_size,
				cam->dma_bufs[i], cam->dma_handles[i]);
		cam->dma_bufs[i] = NULL;
	}
	cam->nbufs = 0;
}


/*
 *  Prepare DMA for the camera.
 */
static void alp_ctlr_dma(struct alp_camera *cam)
{
	int ret;
	int channelready = 1;
	
	ret=DmaInit(DMA_MODULE_1, DMA_CHANNEL_0);

	cam_dma = DmaRequestChain(NULL, DMA_MODULE_1, DMA_CHANNEL_0, CAMERA_PKG_NUM*n_dma_bufs, &channelready);
	if( (cam_dma == NULL)||(channelready == 0) )
		{
			if(channelready == 0)
				{
						printk("CAM: channel busy!\n");
				}
			printk("CAM: request dma chain failed!\n");
		}

	/*
	 * If so requested, try to get our DMA buffers now.!alloc_bufs_at_read
	 */
	
	camera_dma_pkg_init(cam);
		
	dma_apbh_camera_init();

	cameraif_dma_start();
}



/*
 * Configure the controller for operation; caller holds the
 * device mutex.
 */
static int alp_ctlr_configure(struct alp_camera *cam)
{
	unsigned long flags;

	spin_lock_irqsave(&cam->dev_lock, flags);
	alp_set_config_needed(cam, 0);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	return 0;
}

static void alp_ctlr_irq_enable(struct alp_camera *cam)
{
	/*
	 * Clear any pending interrupts, since we do not
	 * expect to have I/O active prior to enabling.
	 */
	as3310_writel(DCMI_IER, HW_DCMI_IER_ADDR);  // enable frame complete intr 
}

static void alp_ctlr_irq_disable(struct alp_camera *cam)
{
	as3310_writel(DCMI_IER, HW_DCMI_IER_ADDR+0x8);  // disable frame complete intr 
}

/*
 * Make the controller start grabbing images.  Everything must
 * be set up before doing this.
 */
static void alp_ctlr_start(struct alp_camera *cam)
{
	/*dcmi enable,8bit,hardware sync,capture enable */
	u32 reg;
	reg = as3310_readl(HW_DCMI_CR_ADDR);
	as3310_writel(reg | DCMI_CR_START, HW_DCMI_CR_ADDR);  
}

static void alp_ctlr_stop(struct alp_camera *cam)
{
	as3310_writel(DCMI_CR_STOP, HW_DCMI_CR_ADDR+0x8);  // dcmi disable
}



/*
 * Stop the controller, and don't return until we're really sure that no
 * further DMA is going on.
 */
static void alp_ctlr_stop_dma(struct alp_camera *cam)
{
	unsigned long flags;

	/*
	 * Theory: stop the camera controller (whether it is operating
	 * or not).  Delay briefly just in case we race with the SOF
	 * interrupt, then wait until no DMA is active.
	 */
	spin_lock_irqsave(&cam->dev_lock, flags);
	alp_ctlr_stop(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	mdelay(1);
	spin_lock_irqsave(&cam->dev_lock, flags);
	cam->state = S_IDLE;
	alp_ctlr_irq_disable(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
}


/* -------------------------------------------------------------------- */
/*
 * Communications with the sensor.
 */

static int __alp_cam_cmd(struct alp_camera *cam, int cmd, void *arg)
{
	struct i2c_client *sc = cam->sensor;
	int ret;

	if (sc == NULL || sc->driver == NULL || sc->driver->command == NULL)
		return -EINVAL;
	ret = sc->driver->command(sc, cmd, arg);
	if (ret == -EPERM) /* Unsupported command */
		return 0;
	return ret;
}


/* Reset the sensor. */
static int __alp_cam_reset(struct alp_camera *cam)
{
	int zero = 0;
	return __alp_cam_cmd(cam, VIDIOC_INT_RESET, &zero);
}

/*
 * We have found the sensor on the i2c.  Let's try to have a
 * conversation.
 */
static int alp_cam_init(struct alp_camera *cam)
{
	struct v4l2_chip_ident chip = { V4L2_CHIP_MATCH_I2C_ADDR, 0, 0, 0 };
	int ret;

	mutex_lock(&cam->s_mutex);
	if (cam->state != S_NOTREADY)
		cam_warn(cam, "Cam init with device in funky state %d",
				cam->state);
	
	ret = __alp_cam_reset(cam);
	if (ret)
		goto out;
	ret =__alp_cam_cmd(cam, VIDIOC_INT_INIT, NULL);
	if (ret)
		goto out;
	chip.match_chip = cam->sensor->addr;
	ret = __alp_cam_cmd(cam, VIDIOC_G_CHIP_IDENT, &chip);
	if (ret)
		goto out;
	cam->sensor_type = chip.ident;
	if (cam->sensor_type != V4L2_IDENT_OV7670) {
		cam_err(cam, "Unsupported sensor type %d", cam->sensor->addr);
		ret = -EINVAL;
		goto out;
	}
	ret = 0;
	cam->state = S_IDLE;
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Configure the sensor to match the parameters we have.  Caller should
 * hold s_mutex
 */
static int alp_cam_set_flip(struct alp_camera *cam)
{
	struct v4l2_control ctrl;

	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = V4L2_CID_VFLIP;
	ctrl.value = flip;
	return __alp_cam_cmd(cam, VIDIOC_S_CTRL, &ctrl);
}


static int alp_cam_configure(struct alp_camera *cam)
{
	int ret = 0;

	if (cam->state != S_IDLE)
		return -EINVAL;

	ret = alp_cam_set_flip(cam);
	return ret;
}


/*
 * Get everything ready, and start grabbing frames.
 */
static int alp_read_setup(struct alp_camera *cam, enum alp_state state)
{
	int ret;
	unsigned long flags;

	
	/*
	 * Configuration.  If we still don't have DMA buffers,
	 * make one last, desperate attempt.
	 */

	if (alp_needs_config(cam)) {
		alp_cam_configure(cam);
		ret = alp_ctlr_configure(cam);
		if (ret)
			return ret;
	}
	
	/*
	 * Turn it loose.
	 */
	spin_lock_irqsave(&cam->dev_lock, flags);
	alp_reset_buffers(cam);
	alp_ctlr_irq_enable(cam);
	cam->state = state;
	alp_ctlr_start(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	return 0;
}



/*
 * Streaming I/O support.
 */
static int alp_vidioc_streamon(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct alp_camera *cam = filp->private_data;
	int ret = -EINVAL;
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_IDLE || cam->n_sbufs == 0)   
		goto out_unlock;

	cam->sequence = 0;
	
	ret = alp_read_setup(cam, S_STREAMING);
	
	wait_event_interruptible(cam->iowait,preloop);
	
	DBG("%s ret = %d.\n",__func__,ret); 
  out_unlock:
	mutex_unlock(&cam->s_mutex);
  out:
	return ret;
}


static int alp_vidioc_streamoff(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct alp_camera *cam = filp->private_data;
	int ret = -EINVAL;

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_STREAMING)
		goto out_unlock;

	alp_ctlr_stop_dma(cam);
	ret = 0;

  out_unlock:
	mutex_unlock(&cam->s_mutex);
  out:
	return ret;
}



static int alp_setup_siobuf(struct alp_camera *cam, int index)
{
	struct alp_sio_buffer *buf = cam->sb_bufs + index;

	INIT_LIST_HEAD(&buf->list);
	buf->v4lbuf.length = PAGE_ALIGN(cam->pix_format.sizeimage);
	buf->buffer = vmalloc_user(buf->v4lbuf.length);
	if (buf->buffer == NULL)
		return -ENOMEM;
	buf->mapcount = 0;
	buf->cam = cam;

	buf->v4lbuf.index = index;
	buf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->v4lbuf.field = V4L2_FIELD_NONE;
	buf->v4lbuf.memory = V4L2_MEMORY_MMAP;
	/*
	 * Offset: must be 32-bit even on a 64-bit system.  videobuf-dma-sg
	 * just uses the length times the index, but the spec warns
	 * against doing just that - vma merging problems.  So we
	 * leave a gap between each pair of buffers.
	 */
	buf->v4lbuf.m.offset = 2*index*buf->v4lbuf.length;
	return 0;
}

static int alp_free_sio_buffers(struct alp_camera *cam)
{
	int i;

	/*
	 * If any buffers are mapped, we cannot free them at all.
	 */
	for (i = 0; i < cam->n_sbufs; i++)
		if (cam->sb_bufs[i].mapcount > 0)
			return -EBUSY;
	
	for (i = 0; i < cam->n_sbufs; i++)
		vfree(cam->sb_bufs[i].buffer);
	cam->n_sbufs = 0;
	kfree(cam->sb_bufs);
	cam->sb_bufs = NULL;
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	return 0;
}


/*
 * Initiate Memory Mapping or User Pointer I/O.
 */
static int alp_vidioc_reqbufs(struct file *filp, void *priv,
		struct v4l2_requestbuffers *req)
{
	struct alp_camera *cam = filp->private_data;
	int ret = 0;  

	/*
	 * Make sure it's something we can do.  User pointers could be
	 * implemented without great pain, but that's not been done yet.
	 */
	if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (req->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;
	/*
	 * If they ask for zero buffers, they really want us to stop streaming
	 * (if it's happening) and free everything. 
	 */
	mutex_lock(&cam->s_mutex);
	if (req->count == 0) {
		if (cam->state == S_STREAMING)
			alp_ctlr_stop_dma(cam);
		ret = alp_free_sio_buffers (cam);
		goto out;
	}
	/*
	 * Device needs to be idle and working.  We *could* try to do the
	 * right thing in S_SPECREAD by shutting things down, but it
	 * probably doesn't matter.
	 */
	if (cam->state != S_IDLE || (cam->owner && cam->owner != filp)) {
		ret = -EBUSY;
		goto out;
	}
	cam->owner = filp;
	if (req->count < min_buffers)
		req->count = min_buffers;
	else if (req->count > max_buffers)
		req->count = max_buffers;
	if (cam->n_sbufs > 0) {
		ret = alp_free_sio_buffers(cam);
		if (ret)
			goto out;
	}

	cam->sb_bufs = kzalloc(req->count*sizeof(struct alp_sio_buffer),
			GFP_KERNEL);
	if (cam->sb_bufs == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	for (cam->n_sbufs = 0; cam->n_sbufs < req->count; (cam->n_sbufs++)) {
		ret = alp_setup_siobuf(cam, cam->n_sbufs);
		if (ret)
			break;
	}

	if (cam->n_sbufs == 0)  /* no luck at all - ret already set */
		kfree(cam->sb_bufs);
	req->count = cam->n_sbufs;  /* In case of partial success */
	
	DBG("%s right.\n",__func__);
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;

}

/*
 * Query the status of a buffer.
 */
static int alp_vidioc_querybuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct alp_camera *cam = filp->private_data;
	int ret = -EINVAL;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	if (buf->index < 0 || buf->index >= cam->n_sbufs)
		goto out;
	*buf = cam->sb_bufs[buf->index].v4lbuf;
	ret = 0;
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Enqueue an empty buffer in the driver's incoming queue.
 */
static int alp_vidioc_qbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct alp_camera *cam = filp->private_data;
	struct alp_sio_buffer *sbuf;
	int ret = -EINVAL;
	unsigned long flags;
	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	if (buf->index < 0 || buf->index >= cam->n_sbufs)
		goto out;
	sbuf = cam->sb_bufs + buf->index;
	if (sbuf->v4lbuf.flags & V4L2_BUF_FLAG_QUEUED) {
		ret = 0; /* Already queued */
		goto out;
	}
	if (sbuf->v4lbuf.flags & V4L2_BUF_FLAG_DONE) {
		ret = -EBUSY;
		goto out;
	}
	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
	spin_lock_irqsave(&cam->dev_lock, flags);
	list_add(&sbuf->list, &cam->sb_avail);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	ret = 0;
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Dequeue a output buffer from the driver's outgoing queue.
 */
static int alp_vidioc_dqbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct alp_camera *cam = filp->private_data;
	struct alp_sio_buffer *sbuf;
	int ret = -EINVAL;
	unsigned long flags;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out_unlock;
	if (cam->state != S_STREAMING)
		goto out_unlock;
	if (list_empty(&cam->sb_full) && filp->f_flags & O_NONBLOCK) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	while (list_empty(&cam->sb_full) && cam->state == S_STREAMING) {
		mutex_unlock(&cam->s_mutex);
		if (wait_event_interruptible(cam->iowait,
						!list_empty(&cam->sb_full))) {
			ret = -ERESTARTSYS;
			goto out;
		}
		mutex_lock(&cam->s_mutex);
	}
	
	wait_event_timeout(cam->iowait, preloop, HZ);
	
	if (cam->state != S_STREAMING)
		ret = -EINTR;
	else {
      		spin_lock_irqsave(&cam->dev_lock, flags);
      		/* Should probably recheck !list_empty() here */
      		sbuf = list_entry(cam->sb_full.next,
      				struct alp_sio_buffer, list);
      		list_del_init(&sbuf->list);
      		spin_unlock_irqrestore(&cam->dev_lock, flags);
      		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
      		*buf = sbuf->v4lbuf;
      		preloop = 0;
      		ret = 0;
      		DBG("%s right.\n",__func__);
	}
  out_unlock:
	mutex_unlock(&cam->s_mutex);
  out:
	return ret;
}



static void alp_v4l_vm_open(struct vm_area_struct *vma)
{
	struct alp_sio_buffer *sbuf = vma->vm_private_data;
	/*
	 * Locking: done under mmap_sem, so we don't need to
	 * go back to the camera lock here.
	 */
	sbuf->mapcount++;
}


static void alp_v4l_vm_close(struct vm_area_struct *vma)
{
	struct alp_sio_buffer *sbuf = vma->vm_private_data;

	mutex_lock(&sbuf->cam->s_mutex);
	sbuf->mapcount--;
	if (sbuf->mapcount == 0)
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_MAPPED;
	mutex_unlock(&sbuf->cam->s_mutex);
}

static struct vm_operations_struct alp_v4l_vm_ops = {
	.open = alp_v4l_vm_open,
	.close = alp_v4l_vm_close
};


/*
 * Map buffers from kernel space to user space.
 */
static int alp_v4l_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct alp_camera *cam = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret = -EINVAL;
	int i;
	struct alp_sio_buffer *sbuf = NULL;
	if (! (vma->vm_flags & VM_WRITE) || ! (vma->vm_flags & VM_SHARED))
		return -EINVAL;
	/*
	 * Find the buffer they are looking for.
	 */
	mutex_lock(&cam->s_mutex);
	for (i = 0; i < cam->n_sbufs; i++)
		if (cam->sb_bufs[i].v4lbuf.m.offset == offset) {
			sbuf = cam->sb_bufs + i;
			break;
		}
	if (sbuf == NULL)
		goto out;
	
	ret = remap_vmalloc_range(vma, sbuf->buffer, 0);
	if (ret)
		goto out;
	vma->vm_flags |= VM_DONTEXPAND;
	vma->vm_private_data = sbuf;
	vma->vm_ops = &alp_v4l_vm_ops;
	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_MAPPED;
	alp_v4l_vm_open(vma);
	DBG("%s right.\n",__func__);
	ret = 0;
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}


/*
*  Open the video device.
*/
static int alp_v4l_open(struct inode *inode, struct file *filp)
{
	struct alp_camera *cam;
	cam = alp_find_dev(iminor(inode));
	if (cam == NULL)
		return -ENODEV;
	filp->private_data = cam;
	mutex_lock(&cam->s_mutex);
	if (cam->users == 0) {
		alp_set_config_needed(cam, 1);
	/* FIXME make sure this is complete */
	}
	(cam->users)++;
	mutex_unlock(&cam->s_mutex);
        return 0;
}

/*
*  Release the resource of video device.
*/
static int alp_v4l_release(struct inode *inode, struct file *filp)
{
	struct alp_camera *cam = filp->private_data;

	mutex_lock(&cam->s_mutex);
	(cam->users)--;
	if (filp == cam->owner) {
		alp_ctlr_stop_dma(cam);
		alp_free_sio_buffers(cam);
		cam->owner = NULL;
	}
	if (cam->users == 0) {
		if (alloc_bufs_at_read)
			alp_free_dma_bufs(cam);
	}
	mutex_unlock(&cam->s_mutex);
	return 0;
}


/*
*  Suspend the process.
*/
static unsigned int alp_v4l_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct alp_camera *cam = filp->private_data;

	poll_wait(filp, &cam->iowait, pt);
	if (cam->next_buf >= 0)
		return POLLIN | POLLRDNORM;

	return 0;
}


/*
 * Query the attributes of a control applications.
 */
static int alp_vidioc_queryctrl(struct file *filp, void *priv,
		struct v4l2_queryctrl *qc)
{
	struct alp_camera *cam = filp->private_data;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __alp_cam_cmd(cam, VIDIOC_QUERYCTRL, qc);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Get the attributes of a control applications.
 */
static int alp_vidioc_g_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct alp_camera *cam = filp->private_data;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __alp_cam_cmd(cam, VIDIOC_G_CTRL, ctrl);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Set the attributes of a control applications.
 */
static int alp_vidioc_s_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct alp_camera *cam = filp->private_data;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __alp_cam_cmd(cam, VIDIOC_S_CTRL, ctrl);
	mutex_unlock(&cam->s_mutex);
	return ret;
}



/*
 * Query device capabilities.
 */
static int alp_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
	strcpy(cap->driver, DRIVER_NAME);
	strcpy(cap->card, CARD_NAME);
	cap->version = ALP_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
	return 0;
}


/*
 * The default format we use until somebody says otherwise.
 */
#ifdef CONFIG_CAM_QVGA
	static struct v4l2_pix_format alp_def_pix_format = {
		.width		= QVGA_WIDTH,
		.height 		= QVGA_HEIGHT,
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.field		= V4L2_FIELD_NONE,
		.bytesperline	= QVGA_WIDTH*CAM_BPP,
		.sizeimage	= QVGA_WIDTH*QVGA_HEIGHT*CAM_BPP,
	};
#endif


#ifdef CONFIG_CAM_VGA
static struct v4l2_pix_format alp_def_pix_format = {
	.width		= VGA_WIDTH,
	.height		= VGA_HEIGHT,
	.pixelformat	= V4L2_PIX_FMT_YUYV,
	.field		= V4L2_FIELD_NONE,
	.bytesperline	= VGA_WIDTH*CAM_BPP,
	.sizeimage	= VGA_WIDTH*VGA_HEIGHT*CAM_BPP,
};
#endif
/*
 * Enumerate image formats.
 */
static int alp_vidioc_enum_fmt_vid_cap(struct file *filp,
		void *priv, struct v4l2_fmtdesc *fmt)
{
	struct alp_camera *cam = priv;
	int ret;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	mutex_lock(&cam->s_mutex);
	ret = __alp_cam_cmd(cam, VIDIOC_ENUM_FMT, fmt);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Try a image format.
 */
static int alp_vidioc_try_fmt_vid_cap(struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct alp_camera *cam = priv;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __alp_cam_cmd(cam, VIDIOC_TRY_FMT, fmt);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Set the data format.
 */
static int alp_vidioc_s_fmt_vid_cap(struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct alp_camera *cam = priv;
	int ret;
	/*
	 * Can't do anything if the device is not idle
	 * Also can't if there are streaming buffers in place.
	 */
	if (cam->state != S_IDLE || cam->n_sbufs > 0)
		return -EBUSY;
	/*
	 * See if the formatting works in principle.
	 */

	ret = __alp_cam_cmd(cam, VIDIOC_S_FMT, fmt);
	if (ret)
		return ret;
	/*
	 * Now we start to change things for real, so let's do it
	 * under lock.
	 */
	mutex_lock(&cam->s_mutex);
	cam->pix_format = fmt->fmt.pix;
	if(fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565)
	{
		colour_mode=0;
	}
	else
	{
		colour_mode=1;			
	}
	mutex_unlock(&cam->s_mutex);
	return 0;
}

/*
 * Return our stored notion of how the camera is/should be configured.
 * The V4l2 spec wants us to be smarter, and actually get this from
 * the camera (and not mess with it at open time). 
 */
/*
 * Get the data format.
 */
static int alp_vidioc_g_fmt_vid_cap(struct file *filp, void *priv,
		struct v4l2_format *f)
{
	struct alp_camera *cam = priv;

	f->fmt.pix = cam->pix_format;
	
	return 0;
}



/*
 * G/S_PARM.  Most of this is done by the sensor, but we are
 * the level which controls the number of read buffers.
 */
static int alp_vidioc_g_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct alp_camera *cam = priv;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __alp_cam_cmd(cam, VIDIOC_G_PARM, parms);
	mutex_unlock(&cam->s_mutex);
	parms->parm.capture.readbuffers = n_dma_bufs;
	return ret;
}

static int alp_vidioc_s_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct alp_camera *cam = priv;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __alp_cam_cmd(cam, VIDIOC_S_PARM, parms);
	mutex_unlock(&cam->s_mutex);
	parms->parm.capture.readbuffers = n_dma_bufs;
	return ret;
}

/*
 * Set the mode of ASM9260 DCMI.
 */
static int alp_vidioc_default(struct file * file,void * priv,int cmd,void* arg)
{
	struct alp_camera *cam = priv;
	unsigned int *i = arg;
	
	mutex_lock(&cam->s_mutex);
	switch(*i){
	case CONTINUES:
		printk("CONTINUES!\n");
		as3310_writel(DCMI_ESCR, HW_DCMI_ESCR_ADDR);
     		as3310_writel(DCMI_ESUR, HW_DCMI_ESUR_ADDR);
		as3310_writel(0xB0, HW_DCMI_CR_ADDR);
		break;
	case SNAP:
		printk("SNAP!\n");
		as3310_writel(DCMI_ESCR, HW_DCMI_ESCR_ADDR);
     		as3310_writel(DCMI_ESUR, HW_DCMI_ESUR_ADDR);
		as3310_writel(0xB2, HW_DCMI_CR_ADDR);
		break;
	case JPEG:
		printk("JPEG!\n");
		as3310_writel(0x28, HW_DCMI_CR_ADDR);
		break;
	case CROP:
		printk("CROP!\n");
		as3310_writel(DCMI_ESCR, HW_DCMI_ESCR_ADDR);
     		as3310_writel(DCMI_ESUR, HW_DCMI_ESUR_ADDR);
		as3310_writel(0xA4, HW_DCMI_CR_ADDR);
		break;
	default:
		printk("asm9260cam: This cmd is not support\n");
		break;
	}
		
	mutex_unlock(&cam->s_mutex);
	return 0;
}


static void alp_v4l_dev_release(struct video_device *vd)
{
	struct alp_camera *cam = container_of(vd, struct alp_camera, v4ldev);

	kfree(cam);
}


/*
 * This template device holds all of those v4l2 methods; we
 * clone it for specific real devices.
 */

static const struct file_operations alp_v4l_fops = {
	.owner = THIS_MODULE,
	.open = alp_v4l_open,
	.release = alp_v4l_release,
	.poll = alp_v4l_poll,
	.mmap = alp_v4l_mmap,
	.ioctl = video_ioctl2,
	.llseek = no_llseek,
};

static const struct v4l2_ioctl_ops alp_v4l_ioctl_ops = {
	.vidioc_querycap 	= alp_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = alp_vidioc_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= alp_vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= alp_vidioc_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= alp_vidioc_g_fmt_vid_cap,
	.vidioc_reqbufs		= alp_vidioc_reqbufs,
	.vidioc_querybuf	= alp_vidioc_querybuf,
	.vidioc_qbuf		= alp_vidioc_qbuf,
	.vidioc_dqbuf		= alp_vidioc_dqbuf,
	.vidioc_streamon	= alp_vidioc_streamon,
	.vidioc_streamoff	= alp_vidioc_streamoff,
	.vidioc_queryctrl	= alp_vidioc_queryctrl,
	.vidioc_g_ctrl		= alp_vidioc_g_ctrl,
	.vidioc_s_ctrl		= alp_vidioc_s_ctrl,
	.vidioc_g_parm		= alp_vidioc_g_parm,
	.vidioc_s_parm		= alp_vidioc_s_parm,
	.vidioc_default          = alp_vidioc_default,          
};

static struct video_device alp_v4l_template = {
	.name = DRIVER_NAME,
	.minor = -1, /* Get one dynamically */
	.tvnorms = V4L2_STD_NTSC_M,
	.current_norm = V4L2_STD_NTSC_M,  /* make mplayer happy */

	.fops = &alp_v4l_fops,
	.ioctl_ops = &alp_v4l_ioctl_ops,
	.release = alp_v4l_dev_release,
};

			

/* ---------------------------------------------------------------------- */
/*
 * Interrupt handler stuff
 */
static int alp_frame_tasklet(struct alp_camera *cam,int bufno)
{
	int i;
	unsigned long flags;
	struct alp_sio_buffer *sbuf;
	spin_lock_irqsave(&cam->dev_lock, flags);
	
	if (bufno < 0) {  /* "will never happen" */
		cam_err(cam, "No valid bufs in tasklet!\n");
		return -EINVAL;
	}
	if (! test_bit(bufno, &cam->flags))
		return -EINVAL;
	
	if (list_empty(&cam->sb_avail)){
		return -EINVAL;  /* Leave it valid, hope for better later */
	}
	clear_bit(bufno, &cam->flags);
	sbuf = list_entry(cam->sb_avail.next,
			struct alp_sio_buffer, list);
	/*
	 * Drop the lock during the big copy.  
	 */
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	memcpy(sbuf->buffer, cam->dma_bufs[bufno],
			cam->pix_format.sizeimage);
	sbuf->v4lbuf.bytesused = cam->pix_format.sizeimage;
	sbuf->v4lbuf.sequence = cam->buf_seq[bufno];
	sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;
	spin_lock_irqsave(&cam->dev_lock, flags);
	list_move_tail(&sbuf->list, &cam->sb_full);
	
	if (! list_empty(&cam->sb_full))
		wake_up_interruptible(&cam->iowait);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	DBG("%s right.\n",__func__);
	return 0;
}



static int alp_frame_complete(struct alp_camera *cam,int frame)
{
	/*
	 * Basic frame housekeeping.
	 */
		
	set_bit(frame, &cam->flags);
	cam->next_buf = frame;
	cam->buf_seq[frame] = ++(cam->sequence);

	switch (cam->state) {
	/*
	 * If in single read mode, try going speculative.
	 */
	    case S_SINGLEREAD:
		cam->state = S_SPECREAD;
		cam->specframes = 0;
		wake_up(&cam->iowait);
		break;

	/*
	 * If we are already doing speculative reads, and nobody is
	 * reading them, just stop.
	 */
	    case S_SPECREAD:
		if (++(cam->specframes) >= cam->nbufs) {
			alp_ctlr_stop(cam);
			alp_ctlr_irq_disable(cam);
			cam->state = S_IDLE;
		}
		wake_up(&cam->iowait);
		break;
	/*
	 * For the streaming case, we defer the real work to the
	 * camera tasklet.
	 *
	 * FIXME: if the application is not consuming the buffers,
	 * we should eventually put things on hold and restart in
	 * vidioc_dqbuf().
	 */
	    case S_STREAMING:
			alp_frame_tasklet(cam,frame);
		break;

	    default:
		cam_err(cam, "Frame interrupt in non-operational state\n");
		break;
	}
	return 0;
}


static int alp_frame_irq(struct alp_camera *cam)
{
	if(loop != oldloop){
		alp_frame_complete(cam,(loop>0?(loop-1):n_dma_bufs-1));//(loop>0?(loop-1):n_dma_bufs-1)
		oldloop = loop;
	}
	
	if(loop == n_dma_bufs-1){
  		loop = 0;
  	}else{
   		loop++;
  	}
	
	preloop = 1;
	return 0;
}

/*
 * Camera Interrupt handle
 */
static int camif_irq_handler(int irq, void *dev_id)
{
	struct alp_camera *cam = (struct alp_camera *)dev_id;
	spin_lock(&cam->dev_lock);
	if(as3310_readl(HW_DCMI_RIS_ADDR)&LINE_IRQ_STAUS)
     {  // line intr
         as3310_writel(LINE_IRQ_STAUS, HW_DCMI_ICR_ADDR);  // clear line intr 
     }
    
	if(as3310_readl(HW_DCMI_RIS_ADDR)&VSYNC_IRQ_STAUS)
     {  // vsync intr
         as3310_writel(VSYNC_IRQ_STAUS, HW_DCMI_ICR_ADDR);  // clear vsync intr 
     }

	if(as3310_readl(HW_DCMI_RIS_ADDR)&FRAME_END_IRQ_STAUS)
     {  // Frame end intr
         as3310_writel(FRAME_END_IRQ_STAUS, HW_DCMI_ICR_ADDR);  // clear frame complete intr
	    alp_frame_irq(cam);
	}

	if(as3310_readl(HW_DCMI_RIS_ADDR)&OVER_IRQ_STAUS)
     {  // Overrun intr
         as3310_writel(OVER_IRQ_STAUS, HW_DCMI_ICR_ADDR);  // clear Overrun intr   
     }
    
     if(as3310_readl(HW_DCMI_RIS_ADDR)&ERROR_IRQ_STAUS)
     {  // error intr
         as3310_writel(ERROR_IRQ_STAUS, HW_DCMI_ICR_ADDR);  // clear error intr  
     }
	spin_unlock(&cam->dev_lock);
	
	return IRQ_HANDLED;
}



/* ------------------------------------------------------------------------*/
/*
 *alp_probe.
 */

static int alp_probe(struct platform_device *pdev)
{
	int ret;
	static struct device dev;
	struct alp_camera *cam;
	/*
	 * Start putting together one of our big camera structures.
	 */
	ret = -ENOMEM;
	cam = kzalloc(sizeof(struct alp_camera), GFP_KERNEL);
	if (cam == NULL)
		goto out;
	
	mutex_init(&cam->s_mutex);
	mutex_lock(&cam->s_mutex);
	spin_lock_init(&cam->dev_lock);
	cam->state = S_NOTREADY;
	alp_set_config_needed(cam, 1);
	init_waitqueue_head(&cam->smbus_wait);
	init_waitqueue_head(&cam->iowait);
	cam->pix_format = alp_def_pix_format;
	INIT_LIST_HEAD(&cam->dev_list);
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	
	cam->pdev = pdev;
	dev = (*pdev).dev;

	/*	   Get irq resource.	*/
	irq = platform_get_irq(pdev, 0);
	ret = request_irq(irq, camif_irq_handler, 0, CARD_NAME, cam);
	if (ret)
		goto out_free;
	
	/*
	 * Set up I2C/SMBUS communications.  We have to drop the mutex here
	 * because the sensor could attach in this call chain, leading to
	 * unsightly deadlocks.
	 */
	mutex_unlock(&cam->s_mutex);  /* attach can deadlock */

	ret = alp_smbus_setup(cam);
	if (ret)
		goto out_freeirq;
	
	mdelay(0x10);
	
	/*     Configure  Clock.    */
	clock_init();
	
	/*     Assign Pins.     */
	cam_pin_init();
	mdelay(0x10);
	
	/*	   Get DMA Buffer.	   */
	if (alp_alloc_dma_bufs(cam, 1))
		cam_warn(cam, "Unable to alloc DMA buffers at load\n");
	
     /*       Set the DMA controller.*/
	alp_ctlr_dma(cam);
    
	/*      Get the v4l2 setup done. */
	mutex_lock(&cam->s_mutex);
	cam->v4ldev = alp_v4l_template;
	cam->v4ldev.debug = 0;
	cam->v4ldev.parent = &pdev->dev;
	ret = video_register_device(&cam->v4ldev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto out_smbus;

	mutex_unlock(&cam->s_mutex);
	alp_add_dev(cam);
	
	printk("Asm9260 camera probe!\n");
	return 0;
	
out_smbus:
	alp_smbus_shutdown(cam);
	
out_freeirq:
	free_irq(irq, cam);

out_free:
	kfree(cam);
out:
	return ret;
}


/*
 * Shut down an initialized device
 */
static void alp_shutdown(struct alp_camera *cam)
{
/* FIXME: Make sure we take care of everything here */
	if (cam->n_sbufs > 0)
	alp_free_sio_buffers(cam);
	alp_remove_dev(cam);
	alp_ctlr_stop_dma(cam);
	alp_smbus_shutdown(cam);
	alp_free_dma_bufs(cam);
	free_irq(irq, cam);
	video_unregister_device(&cam->v4ldev);
	/* kfree(cam); done in v4l_release () */
}


static void alp_remove(struct platform_device *pdev)
{
	struct alp_camera *cam = alp_find_by_pdev(pdev);

	if (cam == NULL) {
		DBG(KERN_WARNING "pci_remove on unknown pdev %p\n", pdev);
		return;
	}
	mutex_lock(&cam->s_mutex);
	if (cam->users > 0)
		cam_warn(cam, "Removing a device with users!\n");
	alp_shutdown(cam);
/* No unlock - it no longer exists */
}


static struct platform_driver alp_driver = {
	.probe = alp_probe,
	.remove = alp_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
};


static int __init alp_init(void)
{
	int ret;
	
	printk(KERN_NOTICE "ASM9260 'ALP' Camera Controller version %d\n",
			ALP_VERSION);
	ret = platform_driver_register(&alp_driver);
	if (ret) {
		printk(KERN_ERR "Unable to register alp_ccic driver\n");
		goto out;
	}
	request_module("ov7670");  /* FIXME want something more general */
	ret = 0;

  out:
	return ret;
}


static void __exit alp_exit(void)
{
	platform_driver_unregister(&alp_driver);
}


#ifdef CONFIG_SOC_CAMERA_ASM9260
	 device_initcall_sync(alp_init);
#endif

#ifdef CONFIG_SOC_CAMERA_ASM9260_MODULE
	 module_init(alp_init);
#endif

module_exit(alp_exit);


MODULE_AUTHOR("AlpScale Inc");
MODULE_DESCRIPTION("ASM9260 CMOS Camera Controller driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("Video");
