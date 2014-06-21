/*
 * linux/drivers/media/video/asm9260camif.h
 */
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <media/videobuf-core.h>

/*
 * Useful stuff that probably belongs somewhere global.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT 240

#define LCD_MODE_RGB  		0
#define LCD_MODE_YUV  		1
#define CAM_BPP 2

#ifdef CONFIG_CAM_QVGA
#define _rgb_max_x	240
#define _rgb_max_y	320
#define CAMERA_PKG_NUM	40//(_rgb_max_y/2)
#endif

#ifdef CONFIG_CAM_VGA
#define _rgb_max_x	480
#define _rgb_max_y	640
#define CAMERA_PKG_NUM	160//(_rgb_max_y/2)
#endif

#define ALP_VERSION 0x000002
#define CARD_NAME		"asm9260-camif"
#define DRIVER_NAME		"asm9260-camif"
#define I2C_CAM_INTERFACE_INDEX 1

#define MAX_DMA_BUFS 3
#define ALP_SMBUS_TIMEOUT (HZ)  /* generous */

#define CF_BUF0_VALID	 0	/* Buffers valid - first three */
#define CF_BUF1_VALID	 1
#define CF_BUF2_VALID	 2
#define CF_DMA_ACTIVE	 3	/* A frame is incoming */
#define CF_CONFIG_NEEDED 4	/* Must configure hardware */

#define	RGB565(r, g, b)			((u16)(	\
									(((r) & 0xF8) << 8) \
									| (((g) & 0xFC) << 3) \
									| (((b) & 0xF8) >> 3) \
									))
/*
* Debugging and related.
*/
#define cam_err(cam, fmt, arg...) \
											dev_err(&(cam)->pdev->dev, fmt, ##arg);
#define cam_warn(cam, fmt, arg...) \
											dev_warn(&(cam)->pdev->dev, fmt, ##arg);
#define cam_dbg(cam, fmt, arg...) \
											dev_dbg(&(cam)->pdev->dev, fmt, ##arg);
	
#define alp_dfs_setup()
#define alp_dfs_shutdown()
#define alp_dfs_cam_setup(cam)
#define alp_dfs_cam_shutdown(cam)

#define	ASM9260_S_PRIVATE	_IOW('V', BASE_VIDIOC_PRIVATE + 0, int)
#define	ASM9260_G_PRIVATE	_IOR('V', BASE_VIDIOC_PRIVATE + 1, int)
#define	CONTINUES  1
#define	SNAP         2
#define	JPEG         3
#define	CROP        4
#define	GN_OK				(0x0)
#define	GN_ERR				(0x101)

#define DCMI_IER 0x00000003
#define DCMI_CR_START 0x4001
#define DCMI_CR_STOP 0x00000001
#define DCMI_ESCR 0xFFDAC7FF
#define DCMI_ESUR 0xFFFFFFFF
#define LINE_IRQ_STAUS 0x00000010
#define VSYNC_IRQ_STAUS 0x00000008
#define FRAME_END_IRQ_STAUS 0x00000001
#define OVER_IRQ_STAUS 0x00000002
#define ERROR_IRQ_STAUS 0x00000004

enum alp_state {
	S_NOTREADY,	/* Not yet initialized */
	S_IDLE,		/* Just hanging around */
	S_FLAKED,	/* Some sort of problem */
	S_SINGLEREAD,	/* In read() */
	S_SPECREAD,   	/* Speculative read (for future read()) */
	S_STREAMING	/* Streaming data */
};

/*
 * Tracking of streaming I/O buffers.
 */
struct alp_sio_buffer {
	struct list_head list;
	struct v4l2_buffer v4lbuf;
	u_char *buffer;   /* Where it lives in kernel space */
	int mapcount;
	struct alp_camera *cam;
};

/*
 * A description of one of our devices.
 * Locking: controlled by s_mutex.  Certain fields, however, require
 * 	    the dev_lock spinlock; they are marked as such by comments.
 *	    dev_lock is also required for access to device registers.
 */
struct alp_camera
{
	enum alp_state state;
	unsigned long flags;   		/* Buffer status, mainly (dev_lock) */
	int users;			/* How many open FDs */
	struct file *owner;		/* Who has data access (v4l2) */
	u_char * map_cpu;
	dma_addr_t map_dma;
	struct videobuf_queue      vb_vidq;
	/*
	 * Subsystem structures.
	 */
	struct platform_device *pdev;
	struct video_device v4ldev;
	struct i2c_adapter i2c_adapter;
	struct i2c_client *sensor;
	unsigned char __iomem *regs;
	struct list_head dev_list;	/* link to other devices */

	/* DMA buffers */
	unsigned int nbufs;		/* How many are alloc'd */
	int next_buf;			/* Next to consume (dev_lock) */
	unsigned int dma_buf_size;  	/* allocated size */
	void *dma_bufs[MAX_DMA_BUFS];	/* Internal buffer addresses */
	dma_addr_t dma_handles[MAX_DMA_BUFS]; /* Buffer bus addresses */
	unsigned int specframes;	/* Unconsumed spec frames (dev_lock) */
	unsigned int sequence;		/* Frame sequence number */
	unsigned int buf_seq[MAX_DMA_BUFS]; /* Sequence for individual buffers */

	/* Streaming buffers */
	unsigned int n_sbufs;		/* How many we have */
	struct alp_sio_buffer *sb_bufs; /* The array of housekeeping structs */
	struct list_head sb_avail;	/* Available for data (we own) (dev_lock) */
	struct list_head sb_full;	/* With data (user space owns) (dev_lock) */
	struct tasklet_struct s_tasklet;

	/* Current operating parameters */
	u32 sensor_type;		/* Currently ov7670 only */
	struct v4l2_pix_format pix_format;

	/* Locks */
	struct mutex s_mutex; /* Access to this structure */
	spinlock_t dev_lock;  /* Access to device */

	/* Misc */
	wait_queue_head_t smbus_wait;	/* Waiting on i2c events */
	wait_queue_head_t iowait;	/* Waiting on frame data */
#ifdef CONFIG_VIDEO_ADV_DEBUG
	struct dentry *dfs_regs;
	struct dentry *dfs_cam_regs;
#endif
};

									

