#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/i2c.h>

#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/semaphore.h>
#include <linux/dma-mapping.h>
#include <asm/dma-mapping.h>
#include <mach/dma.h>
#include <mach/pincontrol.h>

extern void set_pin_mux(int port,int pin,int mux_type);
#define PFX "asm9260-wm8731-superlp: "

#define ASM9260_IIS_WM8731_DBG	0
#if ASM9260_IIS_WM8731_DBG
#define DBG(x...) printk(PFX x)
#else
#define DBG(x...) do { } while(0)
#endif

#define AUDIO_NAME "2M8731"
#define AUDIO_NAME_VERBOSE "\nwm8731 audio driver"

#define AUDIO_FMT_MASK (AFMT_S16_LE)
#define AUDIO_FMT_DEFAULT (AFMT_S16_LE)

#define AUDIO_CHANNELS_DEFAULT 2
#define AUDIO_RATE_DEFAULT 44100

#define AUDIO_NBFRAGS_DEFAULT 8
#define AUDIO_FRAGSIZE_DEFAULT 0x8000
#define AUDIO_SCLK_DEFAULT 12000000

struct audio_clk_info {
            int clk_source; /* 0:12M  1:sys pll480M  2:mclk_input */
		    int mclk; /*i2s mclk input data,if clk_source!=2 then pass 0 */
		    int sclk;	/* if clk_source=0 then pass 0*/
		    int audio_rate;	/* rate */
		};

#define SNDCTL_DSP_SETCLK		_SIOR ('p', 80, struct audio_clk_info)
/*
 * The wm8731 sits on i2c with ID 0x34>>2
 */
#define WM8731_I2C_ADDR 0x1a
#define I2C_WM8731_INTERFACE_INDEX 1
struct i2c_client wm8731_i2c_client;
static int wm8731_write_reg(unsigned char reg, unsigned int data);

struct regval_pair {
	unsigned long reg_num;
	unsigned long value;
};



static struct regval_pair wm8731_default_regs[] = {
	{0x0f, 0}, {0x00, 0x012},{0x01, 0x012},	
	{0x02, (0x060|(1<<7))},{0x03, (0x060|(1<<7))}, {0x04, 0x12},  
	{0x05, 0x005}, {0x06, 0x002}, {0x07, 0x002}, 
	{0x08, 0x023},{0x09, 0x001}, 
	{ 0xff, 0xff },	/* END MARKER */
};


typedef struct {
	int size; /* buffer size */
	char *start; /* point to actual buffer */
	dma_addr_t dma_addr; /* physical buffer address */
	struct semaphore sem; /* down before touching the buffer */
	int master; /* owner for buffer allocation, contain size when true */
} audio_buf_t;

typedef struct {
	audio_buf_t *buffers; /* pointer to audio buffer structures */
	audio_buf_t *buf; /* current buffer used by read/write */
	u_int buf_idx; /* index for the pointer above */
	u_int fragsize; /* fragment i.e. buffer size */
	u_int nbfrags; /* nbr of fragments */
	u_int dma_ch; /* DMA channel (channel2 for audio) */
	u_int dma_ok;
} audio_stream_t;

static audio_stream_t output_stream;
static audio_stream_t input_stream; /* input */

#define MAX_NBFRAGS	8

struct i2s_dma_private_info
{
	uint32_t b_addr;
	volatile int dma_used;
	int dma_size;
	dma_addr_t i2s_dma_addr;
};

struct i2s_dma_buf_runtime_info
{
	int i2s_dma_now_ptr;
	struct i2s_dma_private_info audio_buf_info[MAX_NBFRAGS];
};

#define NEXT_DMA_NUM(_s_,_n_) { \
(_n_)++; \
(_n_) %= (_s_)->nbfrags; }

#define NEXT_BUF(_s_,_b_) { \
(_s_)->_b_##_idx++; \
(_s_)->_b_##_idx %= (_s_)->nbfrags; \
(_s_)->_b_ = (_s_)->buffers + (_s_)->_b_##_idx; }

struct i2s_dma_buf_runtime_info *i2s_dma_info_tx = NULL;
struct i2s_dma_buf_runtime_info *i2s_dma_info_rx = NULL;
static u_int audio_rate;
static int audio_channels;
static int audio_fmt;
static u_int audio_fragsize;
static u_int audio_nbfrags;
static uint32_t audio_dma_rx_size = 0;
static uint32_t audio_dma_tx_size = 0;
static int audio_rd_refcount;
static int audio_wr_refcount;
#define audio_active (audio_rd_refcount | audio_wr_refcount)
static int audio_dev_dsp;
static int audio_dev_mixer;
static int audio_mix_modcnt;
static int flag1 = 0, flag2 = 0;
static int audio_clk_source = 0;
static int audio_sclk;
static int audio_mclk;
static int first_start = 0;

static int wm8731_volume;
static int mixer_igain=0x4; /* -6db*/


void i2s_dma_tx_pkg_config(uint32_t audio_frag_size);
void i2s_dma_rx_pkg_config(uint32_t audio_frag_size);



struct dma_llp
{
	u32 sar;
	u32 dar;
	u32 llp;
	u32 ctrl_l;
	u32 ctrl_h;
};

struct dma_llp *dma_llp_tx_vir;
struct dma_llp *dma_llp_rx_vir;
dma_addr_t dma_llp_tx_phy;
dma_addr_t dma_llp_rx_phy;


/*
 *   free audio buf when module_exit
 */
static void audio_clear_buf(audio_stream_t * s)
{

	int timeout = 0,i;
	
	DBG("audio_clear_buf\n");
	if (s == &output_stream)
	{
		for (i=0; i<audio_nbfrags; i++)
		{
			while (i2s_dma_info_tx->audio_buf_info[i].dma_used != 0)
			{
				if (timeout++ > 0x2000000)
				{
					printk("dma tx timeout\n");
					break;
				}
			}
		}
	}
	else
	{
		for (i=0; i<audio_nbfrags; i++)
		{
			while ((i2s_dma_info_rx->audio_buf_info[i].dma_used) != 0)
			{

				if (timeout++ > 0x2000000)
				{
					printk("dma rx timeout\n");
					break;
				}
			}
		}
	}

	if (s->buffers) {
		int frag;

		for (frag = 0; frag < s->nbfrags; frag++) {
			if (!s->buffers[frag].master)
				continue;
			dma_free_coherent(NULL,
					  s->buffers[frag].master,
					  s->buffers[frag].start,
					  s->buffers[frag].dma_addr);
		}
		kfree(s->buffers);
		s->buffers = NULL;
	}

	s->buf_idx = 0;
	s->buf = NULL;

}

/*
 *   clean audio buf when audio closes
 */
static void audio_stop_run(audio_stream_t * s)
{
	int timeout = 0,i;
	int frag;

	DBG("audio_clear_buf\n");
	if (s == &output_stream)
	{
		for (i=0; i<audio_nbfrags; i++)
		{
			while (i2s_dma_info_tx->audio_buf_info[i].dma_used != 0)
			{
				if (timeout++ > 0x2000000)
				{
					printk("dma tx timeout\n");
					break;
				}
			}
		}
	}
	else
	{
		for (i=0; i<audio_nbfrags; i++)
		{
			while ((i2s_dma_info_rx->audio_buf_info[i].dma_used) != 0)
			{

				if (timeout++ > 0x2000000)
				{
					printk("dma rx timeout\n");
					break;
				}
			}
		}
	}


	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_buf_t *b = &s->buffers[frag];
		sema_init(&b->sem, 1);
		b->size= 0;
	}

	s->buf_idx = 0;
	s->buf = &s->buffers[0];
	
}

/*
 *   set up audio buf when audio init
 */
static int audio_setup_buf(audio_stream_t * s)
{
	int frag;
	int dmasize = 0;
	char *dmabuf = 0;
	dma_addr_t dmaphys = 0;

	if (s->buffers)
		return -EBUSY;

	s->nbfrags = audio_nbfrags;
	s->fragsize = audio_fragsize;

	s->buffers = (audio_buf_t *)
			kmalloc(sizeof(audio_buf_t) * s->nbfrags, GFP_KERNEL);
	if (!s->buffers)
		goto err;
	memset(s->buffers, 0, sizeof(audio_buf_t) * s->nbfrags);

	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_buf_t *b = &s->buffers[frag];

		if (!dmasize) {
			dmasize = (s->nbfrags - frag) * s->fragsize;
			do {
				dmabuf = dma_alloc_coherent(NULL, dmasize, &dmaphys, GFP_KERNEL|GFP_DMA);
				if (!dmabuf)
					dmasize -= s->fragsize;
			} while (!dmabuf && dmasize);
			if (!dmabuf)
				goto err;
			b->master = dmasize;
		}

		b->start = dmabuf;
		b->dma_addr = dmaphys;
		sema_init(&b->sem, 1);
		DBG("buf %d: start 0x%x dma 0x%x\n", frag, (unsigned int)(b->start), b->dma_addr);

		dmabuf += s->fragsize;
		dmaphys += s->fragsize;
		dmasize -= s->fragsize;
	}

	s->buf_idx = 0;
	s->buf = &s->buffers[0];
	return 0;

err:
       printk(AUDIO_NAME ": unable to allocate audio memory\n ");
       audio_clear_buf(s);
return -ENOMEM;
}

/*
 *   DMA mem to per init
 */
void dma_tx_init(dma_addr_t dma_src_addr, uint32_t size)
{
    as3310_writel(dma_src_addr, HW_DMA1_SAR0);
    as3310_writel(0x8104912, HW_DMA1_CTL0);
    as3310_writel(audio_dma_tx_size, HW_DMA1_CTL0+4);
    as3310_writel(dma_llp_tx_phy, HW_DMA1_LLP0);
    as3310_writel(0x1, HW_DMA1_DMACFGREG);
    as3310_writel(0x101, HW_DMA1_CHENREG);
}

/*
 *   DMA per to mem init
 */
void dma_rx_init(dma_addr_t dma_dar_addr, uint32_t size)
{
    as3310_writel(HW_I2S1_RXDMA,HW_DMA1_SAR1);  //DMA1 channel1 receive
    as3310_writel(dma_dar_addr,HW_DMA1_DAR1);
    as3310_writel(0x00000000
        +(0<<0)	   //INT_EN
        +(1<<1)	   //DST_TR_WIDTH 16bit
		+(1<<4)	   //SRC_TR_WIDTH 16bit
        +(0<<7)	   //DINC 00 = Increment
        +(2<<9)	   //SINC 1x = No change  
        +(1<<11)   //DEST_MSIZE
        +(1<<14)   //SRC_MSIZE
        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
        +(0<<23)   //DMS Destination Master interface.
        +(0<<25)   //SMS source Master interface
        +(0<<27)   //LLP_DST_DISABLE
        +(1<<28)   //LLP_SRC_DISABLE
		,HW_DMA1_CTL1);

    as3310_writel(audio_dma_rx_size, HW_DMA1_CTL1+4);
    as3310_writel(dma_llp_rx_phy, HW_DMA1_LLP1);
    as3310_writel(0x1,HW_DMA1_DMACFGREG);
    as3310_writel(0x202,HW_DMA1_CHENREG);

}

/*
 * start dma to transfer
 *   when write buf full or read buf empty
 */
static void asm9260_audio_dma_start(u_int dma_ch, u_int buf_idx, dma_addr_t dma_addr, int size)
{
    u_int buf_ptr = buf_idx;
    unsigned long flags;
	

    if (dma_ch == 0)
    {
    	i2s_dma_info_tx->audio_buf_info[buf_ptr].dma_used = 1;
        if (as3310_readl(HW_DMA1_CHENREG) & (1<<0))
        {
        	local_irq_save(flags);
			i2s_dma_info_tx->audio_buf_info[buf_ptr].dma_size = size;
			i2s_dma_info_tx->audio_buf_info[buf_ptr].i2s_dma_addr = dma_addr;
			local_irq_restore(flags);
        }
        else
        {
        	local_irq_save(flags);
			dma_tx_init(dma_addr, size);
			i2s_dma_info_tx->i2s_dma_now_ptr = buf_ptr;
			if (flag1 == 0)
			{
				flag1 = 1;
				as3310_writel(0x0,HW_I2S1_IMR0);
				as3310_writel(0x1,HW_I2S1_ITER);
				as3310_writel(0x1,HW_I2S1_CER);
                                as3310_writel(0x1,HW_I2S1_IER);	//enable I2S0
			}
			local_irq_restore(flags);
        }
    }
    else if (dma_ch == 1)
    {
    	i2s_dma_info_rx->audio_buf_info[buf_ptr].dma_used = 1;
	i2s_dma_info_rx->audio_buf_info[buf_ptr].dma_size = size;
        if (as3310_readl(HW_DMA1_CHENREG) & (1<<1))
        {
        	local_irq_save(flags);
		i2s_dma_info_rx->audio_buf_info[buf_ptr].i2s_dma_addr = dma_addr;
		local_irq_restore(flags);
        }
        else
        {

		local_irq_save(flags);
		dma_rx_init(dma_addr, size);
		i2s_dma_info_rx->i2s_dma_now_ptr = buf_ptr;
		if (flag2 == 0)
		{
			flag2 = 1;
			as3310_writel(0x0,HW_I2S1_IMR0);
			as3310_writel(0x1,HW_I2S1_IRER);
			as3310_writel(0x1,HW_I2S1_CER);
                        as3310_writel(0x1,HW_I2S1_IER);	//enable I2S0
		}
		local_irq_restore(flags);
        }
    }
}

/* 
* dma men to per  interrupt handler: when a dma operation finishs it cause this interrupt.
*and another dma operation of audio out starts in this function.
*i2s_dma_tx_info->audio_buf_info[i2s_dma_ptr].dma_used=0 means this buffer is empty
*i2s_dma_tx_info->audio_buf_info[i2s_dma_ptr].dma_used=1 means this buffer is full
*/
static irqreturn_t asm9260_iis_write_irq(int irq, void *dev_id)
{
	struct i2s_dma_buf_runtime_info *i2s_dma_tx_info = (struct i2s_dma_buf_runtime_info *)dev_id;
	int i2s_dma_ptr;
	audio_stream_t *s = &output_stream;
	audio_buf_t *b;
	
	as3310_writel(1<<0, HW_DMA1_ClearTFR);//clear
	i2s_dma_ptr = i2s_dma_tx_info->i2s_dma_now_ptr;
	i2s_dma_tx_info->audio_buf_info[i2s_dma_ptr].dma_used = 0;
	b = &s->buffers[i2s_dma_ptr];
	up(&b->sem);
	
	NEXT_DMA_NUM(s, i2s_dma_ptr);
	if (i2s_dma_tx_info->audio_buf_info[i2s_dma_ptr].dma_used == 1)
	{
		dma_tx_init(i2s_dma_tx_info->audio_buf_info[i2s_dma_ptr].i2s_dma_addr, i2s_dma_tx_info->audio_buf_info[i2s_dma_ptr].dma_size);
		i2s_dma_tx_info->i2s_dma_now_ptr = i2s_dma_ptr;
	}
	
	return IRQ_HANDLED;
}

/* 
* dma per to mem  interrupt handler: when a dma operation finishs it cause this interrupt.
*and another dma operation of audio in  starts in this function.
*i2s_dma_tx_info->audio_buf_info[i2s_dma_ptr].dma_used=0 means this buffer is empty
*i2s_dma_tx_info->audio_buf_info[i2s_dma_ptr].dma_used=1 means this buffer is full
*/
static irqreturn_t asm9260_iis_read_irq(int irq, void *dev_id)
{
	struct i2s_dma_buf_runtime_info *i2s_dma_rx_info = (struct i2s_dma_buf_runtime_info *)dev_id;
	int i2s_dma_ptr;
	audio_stream_t *s = &input_stream;
	audio_buf_t *b;

	as3310_writel(1<<1, HW_DMA1_ClearTFR);//clear
	i2s_dma_ptr = i2s_dma_rx_info->i2s_dma_now_ptr;
	i2s_dma_rx_info->audio_buf_info[i2s_dma_ptr].dma_used = 0;
	b = &s->buffers[i2s_dma_ptr];
	b->size = i2s_dma_rx_info->audio_buf_info[i2s_dma_ptr].dma_size;
	up(&b->sem);

	NEXT_DMA_NUM(s, i2s_dma_ptr);
	if (i2s_dma_rx_info->audio_buf_info[i2s_dma_ptr].dma_used == 1)
	{
		dma_rx_init(i2s_dma_rx_info->audio_buf_info[i2s_dma_ptr].i2s_dma_addr, i2s_dma_rx_info->audio_buf_info[i2s_dma_ptr].dma_size);
		i2s_dma_rx_info->i2s_dma_now_ptr = i2s_dma_ptr;
	}

	return IRQ_HANDLED;
}

/* using when write */
static int audio_sync(struct file *file)
{
	audio_stream_t *s = &output_stream;
	audio_buf_t *b = s->buf;

	DBG("audio_sync\n");

        if (!s)
		return 0;


	if (b->size != 0) {
		down(&b->sem);
		memset(b->start+b->size,0,(s->fragsize-b->size));	
		asm9260_audio_dma_start(s->dma_ch, s->buf_idx, b->dma_addr, s->fragsize);
		b->size = 0;
		NEXT_BUF(s, buf);
	}

	b = s->buffers + ((s->nbfrags + s->buf_idx - 1) % s->nbfrags);
	if (down_interruptible(&b->sem))
		return -EINTR;
	up(&b->sem);

	return 0;
}

/* using for mono */
static inline int copy_from_user_mono_stereo(char *to, const char *from, int count)
{
	u_int *dst = (u_int *)to;
	const char *end = from + count;

	if (access_ok(VERIFY_READ, from, count))
		return -EFAULT;

	if ((int)from & 0x2) {
		u_int v;
		__get_user(v, (const u_short *)from); from += 2;
		*dst++ = v | (v << 16);
	}

	while (from < end-2) {
		u_int v, x, y;
		__get_user(v, (const u_int *)from); from += 4;
		x = v << 16;
		x |= x >> 16;
		y = v >> 16;
		y |= y << 16;
		*dst++ = x;
		*dst++ = y;
	}

	if (from < end) {
		u_int v;
		__get_user(v, (const u_short *)from);
		*dst = v | (v << 16);
	}

	return 0;
}


/*	write interface:
 *	One of the most importent interfaces,main functions are
 *	playing aduio data and copying data of APP buffer to DMA buffer.
*/
static ssize_t asm9260_audio_write(struct file *file, const char *buffer,
				    size_t count, loff_t * ppos)
{
	const char *buffer0 = buffer;
	audio_stream_t *s = &output_stream;
	int chunksize, ret = 0;

	DBG("audio_write : start count=%d\n", count);
	
	switch (file->f_flags & O_ACCMODE) {
		case O_WRONLY:
		case O_RDWR:
			break;
		default:
			return -EPERM;
	}

	if (!s->buffers && audio_setup_buf(s))
		return -ENOMEM;
	count &= ~0x03;
	if(audio_fmt == 24)
		count -= count % 3;

	while (count > 0) {
		audio_buf_t *b = s->buf;

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (down_trylock(&b->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (down_interruptible(&b->sem))
				break;
		}

		if (audio_channels == 2) {
			chunksize = s->fragsize - b->size;
			if(audio_fmt == 24)
				chunksize = chunksize/4*3;
			if (chunksize > count)
				chunksize = count;
			DBG("write %d to %d\n", chunksize, s->buf_idx);
			if (copy_from_user(b->start + b->size, buffer, chunksize)) {
				up(&b->sem);
				return -EFAULT;
			}
			if(audio_fmt == 24)
			{
			    char *end ;
				int k = 0;
			    char *start = b->start + b->size;
			    char *vend = b->start + b->size + chunksize;

				
				b->size += chunksize/3*4;
				end = b->start + b->size;;
			
				while((--end)!=start)
				{
				    if(!((k++)%4))
						*end = 0;
					else
						*end = *(--vend);
				}
			}
			else
			    b->size += chunksize;
		} else {
			chunksize = (s->fragsize - b->size) >> 1;

			if (chunksize > count)
				chunksize = count;
			DBG("write %d to %d\n", chunksize*2, s->buf_idx);
			if (copy_from_user_mono_stereo(b->start + b->size,
			    buffer, chunksize)) {
				    up(&b->sem);
				    return -EFAULT;
			    }

			    b->size += chunksize*2;
		}

		buffer += chunksize;
		count -= chunksize;
		if (b->size < s->fragsize) {
			up(&b->sem);
			break;
		}

        asm9260_audio_dma_start(s->dma_ch, s->buf_idx, b->dma_addr, b->size);
		
		b->size = 0;
		NEXT_BUF(s, buf);
	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;

	//DBG("audio_write : end count=%d\n\n", ret);

	return ret;
}

/*	read interface:
 *	Another importent interface,main functions are
 *	recording and copy data of DMA buffer to APP buffer.
*/
static ssize_t asm9260_audio_read(struct file *file, char *buffer,
				   size_t count, loff_t * ppos)
{
	const char *buffer0 = buffer;
	audio_stream_t *s = &input_stream;
	int chunksize, ret = 0;
	int i = 0;

	DBG("audio_read: count=%d\n", count);

	if (!first_start) {
		int i;
                first_start++;

		for (i = 0; i < s->nbfrags; i++) {
			audio_buf_t *b = s->buf;
			down(&b->sem);
			asm9260_audio_dma_start(s->dma_ch, s->buf_idx, b->dma_addr, s->fragsize);

			NEXT_BUF(s, buf);
		}
	}

	if(audio_fmt == 24)
        count =count - (count % 3);
	
	while (count > 0) {
		audio_buf_t *b = s->buf;

		/* Wait for a buffer to become full */
		if (file->f_flags & O_NONBLOCK) {
		    ret = -EAGAIN;
		    if (down_trylock(&b->sem))
		        break;
		} else {
		    ret = -ERESTARTSYS;
		    if (down_interruptible(&b->sem))
		        break;
		}

		if(audio_fmt == 24)
                     chunksize =( b->size)/4*3;
		else
                    chunksize = b->size;
		
		if (chunksize > count)
			chunksize = count;
		DBG("read %d from %d\n", chunksize, s->buf_idx);
		if(audio_fmt == 24)
		{
                    char *start = b->start + s->fragsize - b->size;
                    char *end = b->start + s->fragsize - b->size;
		    for(i=0;i<chunksize;i++)
		    {
		        if(i&&(i%3==0))
			    *start = *(++end);
		        else
			        *start = *end;
			start++;
			end++;
		    }
		}
                if (copy_to_user(buffer, b->start + s->fragsize - b->size,
	            chunksize)) {
                    up(&b->sem);
                    return -EFAULT;
                }
                if(audio_fmt == 24)
                    b->size -= chunksize/3*4;
                else
	            b->size -= chunksize;
                buffer += chunksize;
                count -= chunksize;
                if (b->size > 0) {
	            up(&b->sem);
                    break;
                }
                asm9260_audio_dma_start(s->dma_ch, s->buf_idx, b->dma_addr, s->fragsize);
                NEXT_BUF(s, buf);
	}

	if ((buffer - buffer0))
                ret = buffer - buffer0;

	// DPRINTK("audio_read: return=%d\n", ret);

	return ret;
}


static unsigned int asm9260_audio_poll(struct file *file,
					struct poll_table_struct *wait)
{

    return 0;
}


static loff_t asm9260_audio_llseek(struct file *file, loff_t offset,
				    int origin)
{
	return -ESPIPE;
}

/*
* mixer interface 
*  used by mixer or audio ioctl
*/
static int asm9260_mixer_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int ret;
	long val = 0;
	int  timeout;

	switch (cmd) {
		case SOUND_MIXER_INFO:
		{
			mixer_info info;
			strncpy(info.id, "WM8731", sizeof(info.id));
			strncpy(info.name,"WOLFSON WM8731", sizeof(info.name));
			info.modify_counter = audio_mix_modcnt;
			return copy_to_user((void *)arg, &info, sizeof(info));
		}

		case SOUND_OLD_MIXER_INFO:
		{
			_old_mixer_info info;
			strncpy(info.id, "WM8731", sizeof(info.id));
			strncpy(info.name,"WOLFSON WM8731", sizeof(info.name));
			return copy_to_user((void *)arg, &info, sizeof(info));
		}

		case SOUND_MIXER_READ_STEREODEVS:
			return put_user(0, (long *) arg);

		case SOUND_MIXER_READ_CAPS:
			val = SOUND_CAP_EXCL_INPUT;
			return put_user(val, (long *) arg);

		case SOUND_MIXER_WRITE_VOLUME:
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;

			if(val>100)				
				val = 100;			
			else if(val<0)				
				val = 0;	

			wm8731_write_reg(0x9,0);
			wm8731_volume = ((val + 1) * 31) / 100;	
			wm8731_write_reg(0x0, (1<<8)|wm8731_volume);					
			wm8731_write_reg(0x2, (1<<7)|(1<<8)|((val*79)/100 + 48));
			wm8731_write_reg(0x9,1);
			break;

		case SOUND_MIXER_READ_VOLUME:
			val = (wm8731_volume * 100) / 31;
			return put_user(val, (long *) arg);

		case SOUND_MIXER_READ_IGAIN:
			val = ((31- mixer_igain) * 100) / 31;
			return put_user(val, (int *) arg);

		case SOUND_MIXER_WRITE_IGAIN:
			ret = get_user(val, (int *) arg);
			if (ret)
				return ret;
			mixer_igain = 31 - (val * 31 / 100);
			/* use mixer gain channel 1*/
			break;

		case SOUND_MIXER_LINE:
			timeout = 0;
			while((as3310_readl(HW_DMA1_CHENREG)& (1<<0))||
					(as3310_readl(HW_DMA1_CHENREG) & (1<<1)))
			{
				if(timeout++>0x40000)
					 return -EBUSY;
			}
			as3310_writel(0x1,HW_I2S1_RRXDMA);
			as3310_writel(0x1,HW_I2S1_RTXDMA);
			wm8731_write_reg(0x9,0x00);
			wm8731_write_reg(0x4,0x12);
			wm8731_write_reg(0x6,0x02);
			wm8731_write_reg(0x9,0x01);
			
			break;

		case SOUND_MIXER_MIC:
			timeout = 0;
			while((as3310_readl(HW_DMA1_CHENREG)& (1<<0))|| \
					(as3310_readl(HW_DMA1_CHENREG) & (1<<1)))
			{
				if(timeout++>0x40000)
					 return -EBUSY;
			}
			as3310_writel(0x1,HW_I2S1_RRXDMA);
			as3310_writel(0x1,HW_I2S1_RTXDMA);
			wm8731_write_reg(0x9,0x00);
			wm8731_write_reg(0x4,0x14);
			wm8731_write_reg(0x6,0x01);
			wm8731_write_reg(0x9,0x01);
			
			break;

                case SOUND_MIXER_MUTE:
			ret = get_user(val, (long *) arg);
			if (ret)
				return ret;
                       	wm8731_write_reg(0x9,0x00);
                        if (val) {
                           wm8731_write_reg(0x5,0xd); 
                        }else
                           wm8731_write_reg(0x5,0x5); 		
			wm8731_write_reg(0x9,0x01);
			break;

		default:
			DBG("mixer ioctl %u unknown\n", cmd);
			return -ENOSYS;
	}

	audio_mix_modcnt++;
	return 0;
}



/*
* set audio sample rate
*/
static long audio_set_dsp_speed(int val)
{
       int sampling_rate, reg_ccr, lrc_hdiv, lrc_ldiv, sclk;

       sampling_rate = val;
       audio_rate = val;
       reg_ccr = as3310_readl(HW_I2S1_CCR);
       reg_ccr &= 0x1F;
       sclk = audio_sclk;

	if(val == 44100)                                             //44.1kHz  sclk--12M
		as3310_writel(0x80878700,HW_I2S1_CCR);
	else if(val == 48000)                                     //48kHz  sclk--12M
		as3310_writel(0x807c7c00,HW_I2S1_CCR);
	else if(val == 32000)                                      //32kHz  sclk--12M
		as3310_writel(0x80babb00,HW_I2S1_CCR); 
	else if(val == 96000)                                      //96kHz  sclk--12M
		as3310_writel(0x803e3d00,HW_I2S1_CCR);
	else if(val == 88200)                                      //88.2kHz  sclk--12M
		as3310_writel(0x80434300,HW_I2S1_CCR);
        else{
               lrc_hdiv = lrc_ldiv = (((sclk / sampling_rate) >> 1) - 1);
               reg_ccr |= 0x80000000 | (lrc_hdiv << 16) | (lrc_ldiv << 8);
               DBG("REG_CCR : 0x%x\n",reg_ccr);
               as3310_writel(reg_ccr,HW_I2S1_CCR);	//sclk-12M 
        }
    /*modify the wm8731*/
	wm8731_write_reg(0x09, 0x000);

	if(val == 44100)
		wm8731_write_reg(0x08,0x23);
	else if(val == 48000)
		wm8731_write_reg(0x08,0x01);
	else if(val == 32000)
		wm8731_write_reg(0x08,0x19);
	else if(val == 96000)
		wm8731_write_reg(0x08,0x1D);
	else if(val == 88200)
		wm8731_write_reg(0x08,0x3F);

	wm8731_write_reg(0x09, 0x001);

    return audio_rate;
}

/*
* set audio fmt
*/
static int asm9260_set_fmt(long val)
{
    uint32_t tmp;
    int timeout = 0;
	
    while(as3310_readl(HW_DMA1_CHENREG)& (1<<0)|| \
		as3310_readl(HW_DMA1_CHENREG) & (1<<1))
    {
        if(timeout++>0x40000)
           return -EBUSY;
    }
    flag1=0;
    flag2=0;
    as3310_writel(0x0,HW_I2S1_ITER);
    as3310_writel(0x0,HW_I2S1_IRER);
    as3310_writel(0x0,HW_I2S1_CER);
    as3310_writel(0x1,HW_I2S1_RRXDMA);
    as3310_writel(0x1,HW_I2S1_RTXDMA);
    audio_fmt = val;

    tmp = as3310_readl(HW_DMA1_CTL0);
    if(val==16)
        tmp = (tmp&~126)|(1<<1)|(1<<4);
    else
        tmp = (tmp&~126)|(2<<1)|(2<<4);
    as3310_writel(tmp,HW_DMA1_CTL0);        
    
    tmp = as3310_readl(HW_DMA1_CTL1);
    if(val==16)
        tmp = (tmp&~126)|(1<<1)|(1<<4);
    else
        tmp = (tmp&~126)|(2<<1)|(2<<4);
    as3310_writel(tmp,HW_DMA1_CTL1);
	
    i2s_dma_tx_pkg_config(audio_fragsize);
    i2s_dma_rx_pkg_config(audio_fragsize);
    wm8731_write_reg(0x9,0x00);
    if(val == 16){
        as3310_writel(0x2,HW_I2S1_RCR0);
        as3310_writel(0x2,HW_I2S1_TCR0);
        wm8731_write_reg(0x7,0x02);
    }else if(val == 32){
        as3310_writel(0x5,HW_I2S1_RCR0);
        as3310_writel(0x5,HW_I2S1_TCR0);
        wm8731_write_reg(0x7,0x0e);
    }else if(val == 24){
        as3310_writel(0x4,HW_I2S1_RCR0);
        as3310_writel(0x4,HW_I2S1_TCR0);
        wm8731_write_reg(0x7,0x0a);
    }
    wm8731_write_reg(0x9,0x01);
    return 0;
}

/*
* set audio clock source
*/
static void asm9260_set_clk(struct audio_clk_info *clk_info)
{
    int cs,mc,sc;
    cs = clk_info->clk_source;
    mc = clk_info->mclk;
    sc = clk_info->sclk;

    audio_clk_source = cs;
    audio_sclk = sc;
    audio_mclk = mc;
    as3310_writel(cs,HW_I2S1CLKSEL);  	
    as3310_writel(0x0,HW_I2S1CLKUEN);
    as3310_writel(0x1,HW_I2S1CLKUEN);
    if(cs==0)
    {
        audio_sclk = 12000000;
        as3310_writel(1,HW_I2S1_SCLKDIV);
        as3310_writel(1,HW_I2S1_MCLKDIV);
    }else if(cs==1)
    {
        int div;

        if(sc==0)
        {
	    as3310_writel(0,HW_I2S1_SCLKDIV);
	    as3310_writel(0,HW_I2S1_MCLKDIV);
        }
	else
	{
	    div = as3310_readl(0x80040100)&0xfff;    //sys pll
	    div = div/(sc/1000000);
            as3310_writel(div,HW_I2S1_SCLKDIV);	
            as3310_writel(div,HW_I2S1_MCLKDIV);
	}
    }else if(cs==2)
    {
        if(mc==0)
        {
	    as3310_writel(0,HW_I2S1_MCLKDIV);
	    as3310_writel(0,HW_I2S1_SCLKDIV);
        }else
        {
	    if(sc==0)
	    {
	        as3310_writel(0,HW_I2S1_SCLKDIV);
	        as3310_writel(0,HW_I2S1_MCLKDIV);
	     }
	    else
	    {
	         as3310_writel((mc/sc),HW_I2S1_SCLKDIV);
		 as3310_writel((mc/sc),HW_I2S1_MCLKDIV);
	    }
        }
    }
    audio_set_dsp_speed(clk_info->audio_rate);
}

/*
* audio interface
*/
static int asm9260_audio_ioctl(struct inode *inode, struct file *file,
				uint cmd, ulong arg)
{
	long val;
	
	switch (cmd) {
		case SNDCTL_DSP_SETFMT:
			get_user(val, (long *) arg);	
			if ((!(val & AUDIO_FMT_MASK))&&(val != 32)&&(val != 24) ){
				return -EINVAL;
			}else{
			    if(asm9260_set_fmt(val))
					return -EFAULT;
				return 0;
			}	

		case SNDCTL_DSP_CHANNELS:
		case SNDCTL_DSP_STEREO:
			get_user(val, (long *) arg);
			if (cmd == SNDCTL_DSP_STEREO)
				val = val ? 2 : 1;
			if (val != 1 && val != 2)
				return -EINVAL;
			audio_channels = val;
			break;

		case SOUND_PCM_READ_CHANNELS:
			put_user(audio_channels, (long *) arg);
			break;

        case SNDCTL_DSP_GETFMTS:
		case SOUND_PCM_READ_BITS:
			put_user(audio_fmt,(long *) arg);
			break;

		case SNDCTL_DSP_SPEED:
			get_user(val, (long *) arg);
			val = audio_set_dsp_speed(val);
			if (val < 0)
				return -EINVAL;
			put_user(val, (long *) arg);
			break;

		case SOUND_PCM_READ_RATE:
			put_user(audio_rate, (long *) arg);
			break;

		case SNDCTL_DSP_GETBLKSIZE:
			if(file->f_mode & FMODE_WRITE)
				return put_user(audio_fragsize, (long *) arg);
			else
				return put_user(audio_fragsize, (int *) arg);

		case SNDCTL_DSP_SETFRAGMENT:
			if (file->f_mode & FMODE_WRITE) {
				if (output_stream.buffers)
					return -EBUSY;
				get_user(val, (long *) arg);
				audio_fragsize = 1 << (val & 0xFFFF);
				if (((audio_fragsize >> 10) << 10) != audio_fragsize)
					return -EBUSY;
				if (audio_fragsize < 0x400)
					audio_fragsize = 0x400;
				if (audio_fragsize > 0x8000)
					audio_fragsize = 0x8000;
				audio_nbfrags = (val >> 16) & 0x7FFF;
				if (audio_nbfrags < 2)
					audio_nbfrags = 2;
				if (audio_nbfrags * audio_fragsize > 256 * 1024)
					audio_nbfrags = 256 * 1024 / audio_fragsize;
				if (audio_setup_buf(&output_stream))
					return -ENOMEM;
				i2s_dma_tx_pkg_config(audio_fragsize);
			}
			if (file->f_mode & FMODE_READ) {
				if (input_stream.buffers)
					return -EBUSY;
				get_user(val, (int *) arg);
				audio_fragsize = 1 << (val & 0xFFFF);
				if (((audio_fragsize >> 10) << 10) != audio_fragsize)
					return -EBUSY;
				if (audio_fragsize < 0x400)
					audio_fragsize = 0x400;
				if (audio_fragsize > 0x8000)
					audio_fragsize = 0x8000;
				audio_nbfrags = (val >> 16) & 0x7FFF;
				if (audio_nbfrags < 2)
					audio_nbfrags = 2;
				if (audio_nbfrags * audio_fragsize > 256 * 1024)
					audio_nbfrags = 256 * 1024 / audio_fragsize;
				if (audio_setup_buf(&input_stream))
					return -ENOMEM;
				i2s_dma_rx_pkg_config(audio_fragsize);
			}
			break;

		case SNDCTL_DSP_SYNC:
			return audio_sync(file);

		case SNDCTL_DSP_GETOSPACE:
		{
			spinlock_t getospace;
			audio_stream_t *s = &output_stream;
			audio_buf_info *inf = (audio_buf_info *) arg;
			int err = access_ok(VERIFY_WRITE, inf, sizeof(*inf));
			int i;
			unsigned int count;
			int frags = 0, bytes = 0;
			
			spin_lock_init(&getospace);

			if (!err)
				return -EFAULT;
			for (i = 0; i < s->nbfrags; i++) {
				spin_lock(&getospace);
				count = s->buffers[i].sem.count;
				spin_unlock(&getospace);
				if (count > 0) {
					if (s->buffers[i].size == 0) frags++;
					bytes += s->fragsize - s->buffers[i].size;
				}
			}
			put_user(frags, &inf->fragments);
			put_user(s->nbfrags, &inf->fragstotal);
			put_user(s->fragsize, &inf->fragsize);
			put_user(bytes, &inf->bytes);
			break;
		}

		case SNDCTL_DSP_GETISPACE:
		{
			spinlock_t getispace;
			audio_stream_t *s = &input_stream;
			audio_buf_info *inf = (audio_buf_info *) arg;
			int err = access_ok(VERIFY_WRITE, inf, sizeof(*inf));
			int i;
			unsigned int count;
			int frags = 0, bytes = 0;

			spin_lock_init(&getispace);

			if (!(file->f_mode & FMODE_READ))
				return -EINVAL;

			if (!err)
				return -EFAULT;
			for(i = 0; i < s->nbfrags; i++){
				spin_lock(&getispace);
				count = s->buffers[i].sem.count;
				spin_unlock(&getispace);
				if (count > 0)
				{
					if (s->buffers[i].size == s->fragsize)
						frags++;
					bytes += s->buffers[i].size;
				}
			}
			put_user(frags, &inf->fragments);
			put_user(s->nbfrags, &inf->fragstotal);
			put_user(s->fragsize, &inf->fragsize);
			put_user(bytes, &inf->bytes);
			break;
		}
		case SNDCTL_DSP_RESET:
			if (file->f_mode & FMODE_READ) {
				audio_stop_run(&input_stream);
			}
			if (file->f_mode & FMODE_WRITE) {
				audio_stop_run(&output_stream);
			}
			return 0;
		case SNDCTL_DSP_NONBLOCK:
			file->f_flags |= O_NONBLOCK;
			 
			return 0;
		case SNDCTL_DSP_SETCLK:
			{
			struct audio_clk_info clk_info;

			if(copy_from_user(&clk_info, (struct audio_clk_info *)arg,sizeof(struct audio_clk_info)))
				return -EFAULT;
			if(clk_info.clk_source<0||clk_info.clk_source>2)
		        return -EFAULT;
			asm9260_set_clk(&clk_info);
			}
		    return 0;
		case SNDCTL_DSP_SETDUPLEX:
			break;
		case SNDCTL_DSP_POST:
		case SNDCTL_DSP_SUBDIVIDE:
		case SNDCTL_DSP_GETCAPS:
		case SNDCTL_DSP_GETTRIGGER:
		case SNDCTL_DSP_SETTRIGGER:
		case SNDCTL_DSP_GETIPTR:
		case SNDCTL_DSP_GETOPTR:
		case SNDCTL_DSP_MAPINBUF:
		case SNDCTL_DSP_MAPOUTBUF:
		case SNDCTL_DSP_SETSYNCRO:
			return -ENOSYS;
		default:
			return asm9260_mixer_ioctl(inode, file, cmd, arg);
	}

	return 0;
}

#define I2S_OUTPUT		0
#define I2S_INPUT		1
static int asm9260_i2s_dma_info_alloc(audio_stream_t *s)
{
	if (s == &output_stream)
	{
		i2s_dma_info_tx = (struct i2s_dma_buf_runtime_info *)kmalloc(sizeof(struct i2s_dma_buf_runtime_info), GFP_KERNEL);
		if (!i2s_dma_info_tx)
		{
			printk("unable to allocate i2s_dma_info_tx memory");
			return -ENOMEM;
		}
		memset(i2s_dma_info_tx, 0, sizeof(struct i2s_dma_buf_runtime_info));
	}

	if (s == &input_stream)
	{
		i2s_dma_info_rx = (struct i2s_dma_buf_runtime_info *)kmalloc(sizeof(struct i2s_dma_buf_runtime_info), GFP_KERNEL);
		if (!i2s_dma_info_rx)
		{
			printk("unable to allocate i2s_dma_info_rx memory");
			return -ENOMEM;
		}
		memset(i2s_dma_info_rx, 0, sizeof(struct i2s_dma_buf_runtime_info));
	}

	return 0;
}

/*
* audio interface
*/
static int asm9260_audio_open(struct inode *inode, struct file *file)
{
 	int cold = !audio_active;

   	DBG("audio_open\n");

   	if ((file->f_flags & O_ACCMODE) == O_RDONLY) 
        {
		if (audio_rd_refcount || audio_wr_refcount)
			return -EBUSY;
		audio_rd_refcount++;
        }
        else if ((file->f_flags & O_ACCMODE) == O_WRONLY) 
        {
		if (audio_wr_refcount)
			return -EBUSY;
		audio_wr_refcount++;
        }
        else if ((file->f_flags & O_ACCMODE) == O_RDWR) 
        {
		if (audio_rd_refcount || audio_wr_refcount)
			return -EBUSY;
		audio_rd_refcount++;
		audio_wr_refcount++;
        }
        else
        {
		return -EINVAL;
        }
 
        if (cold) 
        {
		if(audio_fmt != AUDIO_FMT_DEFAULT)
		{
		    asm9260_set_fmt(AUDIO_FMT_DEFAULT);
		}
		if(audio_clk_source!=0)
		{
		    audio_clk_source=0;
		    audio_sclk = 12000000;
		    as3310_writel(0,HW_I2S1CLKSEL);  	
	            as3310_writel(0x0,HW_I2S1CLKUEN);
		    as3310_writel(0x1,HW_I2S1CLKUEN);
	            as3310_writel(1,HW_I2S1_SCLKDIV);
		    as3310_writel(1,HW_I2S1_MCLKDIV);

		}
                if(audio_rate != AUDIO_RATE_DEFAULT)
                   audio_set_dsp_speed(AUDIO_RATE_DEFAULT);
                     
                            
                audio_channels = AUDIO_CHANNELS_DEFAULT;        
               if( (audio_fragsize != AUDIO_FRAGSIZE_DEFAULT)||(audio_nbfrags != AUDIO_NBFRAGS_DEFAULT)) 
               {
                  audio_nbfrags = AUDIO_NBFRAGS_DEFAULT;	
                  audio_fragsize = AUDIO_FRAGSIZE_DEFAULT;
                  audio_clear_buf(&output_stream);
                  audio_clear_buf(&input_stream);
                  audio_setup_buf(&output_stream);
                  audio_setup_buf(&input_stream);
               }
		
    		wm8731_write_reg(0x9,0x01);
    }

    return 0;
}


static int asm9260_mixer_open(struct inode *inode, struct file *file)
{
	return 0;
}

/*
* audio interface 
*    when audio closes 
*/
static int asm9260_audio_release(struct inode *inode, struct file *file)
{
	DBG("audio_release\n");

	if (file->f_mode & FMODE_READ) {
		if (audio_rd_refcount == 1)
                        audio_stop_run(&input_stream);
		audio_rd_refcount = 0;
	}

	if(file->f_mode & FMODE_WRITE) {
		if (audio_wr_refcount == 1) {
			audio_sync(file);
			audio_stop_run(&output_stream);
			audio_wr_refcount = 0;
		}
	}
	flag1=0;
	flag2=0;
        first_start = 0;
	as3310_writel(0x0,HW_I2S1_ITER);
	as3310_writel(0x0,HW_I2S1_IRER);
	as3310_writel(0x0,HW_I2S1_CER);
	as3310_writel(0x0,HW_I2S1_IER);	//enable I2S0
        wm8731_write_reg(0x9,0x00);

	return 0;
}


static int asm9260_mixer_release(struct inode *inode, struct file *file)
{
	return 0;
}


static struct file_operations asm9260_audio_fops = {
	llseek: asm9260_audio_llseek,
	write: asm9260_audio_write,
	read: asm9260_audio_read,
	poll: asm9260_audio_poll,
	ioctl: asm9260_audio_ioctl,
	open: asm9260_audio_open,
	release: asm9260_audio_release
};

static struct file_operations asm9260_mixer_fops = {
	ioctl: asm9260_mixer_ioctl,
	open: asm9260_mixer_open,
	release: asm9260_mixer_release
};





static int wm8731_i2c_write(struct i2c_client *c, unsigned char reg,
		unsigned char value)
{
    int ret;


	ret = i2c_smbus_write_byte_data(c, reg, value);

	return ret;
}

/*
* write wm8731 registers interface
*/
static int wm8731_write_reg(unsigned char reg, unsigned int data)
{
    if(wm8731_i2c_client.adapter == NULL) {
        return -EINVAL;
    }

    return wm8731_i2c_write( &wm8731_i2c_client, (u8)((reg<<1)|(data>>8)), (u8)(data&0xff) );
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int wm8731_write_array(struct i2c_client *c, struct regval_pair *vals)
{
	while (vals->reg_num != 0xff || vals->value != 0xff) {
		int ret = wm8731_i2c_write(c, (u8)((vals->reg_num<<1)|(vals->value>>8)), (u8)((vals->value)&0xff));
		msleep(5);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}


static int wm8731_regs_init(struct i2c_client *client)
{
    if(client ==NULL ) {
        return -EINVAL;
    }
	return wm8731_write_array(client, wm8731_default_regs);
}


static int init_wm8731(void)
{
    struct i2c_adapter *adap;
	int ret;

    adap = i2c_get_adapter(I2C_WM8731_INTERFACE_INDEX);

    if(adap == NULL) {
        printk(KERN_ERR "%s: No required index i2c adapter.\n", __func__);
        return -1;
    }

    if( !(adap->class & I2C_CLASS_SOUND) ) {
        printk(KERN_ERR "%s: Required i2c adapter Sound func not supported.\n", __func__);
        return -1;
    }

	wm8731_i2c_client.addr = WM8731_I2C_ADDR;
	strcpy(wm8731_i2c_client.name, "wm8731_i2c");

    wm8731_i2c_client.adapter = adap;
	
    ret = wm8731_regs_init(&wm8731_i2c_client);
	if (ret < 0)
		return ret;
    else
        return 0;



}

/*
* i2s hardware init
*/
static void init_asm9260_iis_bus(void)
{
	as3310_writel(as3310_readl(HW_MACPHY_SEL) | (1<<5), HW_MACPHY_SEL);
	as3310_writel(as3310_readl(HW_DMA_CTRL) | (1<<22) | (1<<23) ,HW_DMA_CTRL);

	as3310_writel(0x0,HW_I2S1_IER);	//enable I2S0
        as3310_writel(0x1,HW_I2S1_TER0);    //enable channel 0 TX
        as3310_writel(0x1,HW_I2S1_RER0);	//enable channel 0 RX
        as3310_writel(0x0,HW_I2S1_TER1);	//disable channel 1 TX
        as3310_writel(0x0,HW_I2S1_RER1);	//disable channel 1 RX
        as3310_writel(0x0,HW_I2S1_TER2);	//disable channel 2 TX
        as3310_writel(0x0,HW_I2S1_RER2);	//disable channel 2 RX
        as3310_writel(0x2,HW_I2S1_RCR0);	//Rx bits=16
        as3310_writel(0x2,HW_I2S1_TCR0);	//Tx bits=16
        as3310_writel(0x4,HW_I2S1_TFCR0);   //trigger level=5
        as3310_writel(0x4,HW_I2S1_RFCR0);   //trigger level=5
   

	as3310_writel(0x80878700,HW_I2S1_CCR);	//44.1kHz  sclk--12M

}

/*
* dma mem to per transfer pkg config
*/
void i2s_dma_tx_pkg_config(uint32_t audio_frag_size)
{
	int i, pkg_num;
        if(audio_fmt == 16)
	    pkg_num = (audio_frag_size / audio_dma_tx_size) >>1;
	else
		pkg_num = (audio_frag_size / audio_dma_tx_size) >>2;
	for (i=0; i<pkg_num - 1; i++)
	{
		dma_llp_tx_vir[i].dar    = HW_I2S1_TXDMA;
		dma_llp_tx_vir[i].llp    = dma_llp_tx_phy + sizeof(struct dma_llp) * (i + 1);
		if(audio_fmt == 16)
			dma_llp_tx_vir[i].ctrl_l = 0x00000000
							        +(0<<0)	   //INT_EN
							        +(1<<1)	   //DST_TR_WIDTH 
									+(1<<4)	   //SRC_TR_WIDTH
							        +(2<<7)	   //DINC 1x = No change
							        +(0<<9)	   //SINC 00 = Increment
							        +(1<<11)   //DEST_MSIZE
							        +(1<<14)   //SRC_MSIZE
							        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
							        +(0<<23)   //DMS Destination Master interface.
							        +(0<<25)   //SMS source Master interface
							        +(1<<27)   //LLP_DST_DISABLE
							        +(0<<28);   //LLP_SRC_DISABLE
		else
		    dma_llp_tx_vir[i].ctrl_l = 0x00000000
							        +(0<<0)	   //INT_EN
							        +(2<<1)	   //DST_TR_WIDTH 
									+(2<<4)	   //SRC_TR_WIDTH
							        +(2<<7)	   //DINC 1x = No change
							        +(0<<9)	   //SINC 00 = Increment
							        +(1<<11)   //DEST_MSIZE
							        +(1<<14)   //SRC_MSIZE
							        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
							        +(0<<23)   //DMS Destination Master interface.
							        +(0<<25)   //SMS source Master interface
							        +(1<<27)   //LLP_DST_DISABLE
							        +(0<<28);   //LLP_SRC_DISABLE
		dma_llp_tx_vir[i].ctrl_h = audio_dma_tx_size;
	}
	dma_llp_tx_vir[i].dar    = HW_I2S1_TXDMA;
	dma_llp_tx_vir[i].llp    = 0;
	if(audio_fmt == 16)
		dma_llp_tx_vir[i].ctrl_l = 0x00000000
						        +(1<<0)	   //INT_EN
						        +(1<<1)	   //DST_TR_WIDTH 
								+(1<<4)	   //SRC_TR_WIDTH
						        +(2<<7)	   //DINC 1x = No change
						        +(0<<9)	   //SINC 00 = Increment
						        +(1<<11)   //DEST_MSIZE
						        +(1<<14)   //SRC_MSIZE
						        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
						        +(0<<23)   //DMS Destination Master interface.
						        +(0<<25)   //SMS source Master interface
						        +(0<<27)   //LLP_DST_DISABLE
						        +(0<<28);   //LLP_SRC_DISABLE
	else
		dma_llp_tx_vir[i].ctrl_l = 0x00000000
						        +(1<<0)	   //INT_EN
						        +(2<<1)	   //DST_TR_WIDTH 
								+(2<<4)	   //SRC_TR_WIDTH
						        +(2<<7)	   //DINC 1x = No change
						        +(0<<9)	   //SINC 00 = Increment
						        +(1<<11)   //DEST_MSIZE
						        +(1<<14)   //SRC_MSIZE
						        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
						        +(0<<23)   //DMS Destination Master interface.
						        +(0<<25)   //SMS source Master interface
						        +(0<<27)   //LLP_DST_DISABLE
						        +(0<<28);   //LLP_SRC_DISABLE	
	dma_llp_tx_vir[i].ctrl_h = audio_dma_tx_size;
	
}

/*
* dma per to mem transfer pkg config
*/
void i2s_dma_rx_pkg_config(uint32_t audio_frag_size)
{
	int i, pkg_num;
	if(audio_fmt == 16)
	    pkg_num = (audio_frag_size / audio_dma_rx_size) >>1;
	else
		pkg_num = (audio_frag_size / audio_dma_rx_size) >>2;
	for (i=0; i<pkg_num - 1; i++)
	{
		dma_llp_rx_vir[i].sar    = HW_I2S1_RXDMA;
		dma_llp_rx_vir[i].llp    = dma_llp_rx_phy + sizeof(struct dma_llp) * (i + 1);
		if(audio_fmt == 16)
			dma_llp_rx_vir[i].ctrl_l = 0x00000000
							        +(0<<0)	   //INT_EN
							        +(1<<1)	   //DST_TR_WIDTH 
								+(1<<4)	   //SRC_TR_WIDTH
							        +(0<<7)	   //DINC 00 = Increment
							        +(2<<9)	   //SINC 1x = No change  
							        +(1<<11)   //DEST_MSIZE
							        +(1<<14)   //SRC_MSIZE
							        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
							        +(0<<23)   //DMS Destination Master interface.
							        +(0<<25)   //SMS source Master interface
							        +(0<<27)   //LLP_DST_DISABLE
							        +(1<<28);   //LLP_SRC_DISABLE
		else
			dma_llp_rx_vir[i].ctrl_l = 0x00000000
							        +(0<<0)	   //INT_EN
							        +(2<<1)	   //DST_TR_WIDTH 
								+(2<<4)	   //SRC_TR_WIDTH
							        +(0<<7)	   //DINC 00 = Increment
							        +(2<<9)	   //SINC 1x = No change  
							        +(1<<11)   //DEST_MSIZE
							        +(1<<14)   //SRC_MSIZE
							        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
							        +(0<<23)   //DMS Destination Master interface.
							        +(0<<25)   //SMS source Master interface
							        +(0<<27)   //LLP_DST_DISABLE
							        +(1<<28);   //LLP_SRC_DISABLE			
		dma_llp_rx_vir[i].ctrl_h = audio_dma_rx_size;
	}
	dma_llp_rx_vir[i].sar    = HW_I2S1_RXDMA;
	dma_llp_rx_vir[i].llp    = 0;
	if(audio_fmt == 16)
		dma_llp_rx_vir[i].ctrl_l = 0x00000000
						        +(1<<0)	   //INT_EN
						        +(1<<1)	   //DST_TR_WIDTH 
							+(1<<4)	   //SRC_TR_WIDTH
						        +(0<<7)	   //DINC 1x = No change
						        +(7<<9)	   //SINC 00 = Increment
						        +(1<<11)   //DEST_MSIZE
						        +(1<<14)   //SRC_MSIZE
						        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
						        +(0<<23)   //DMS Destination Master interface.
						        +(0<<25)   //SMS source Master interface
						        +(0<<27)   //LLP_DST_DISABLE
						        +(0<<28);   //LLP_SRC_DISABLE
	else
		dma_llp_rx_vir[i].ctrl_l = 0x00000000
						        +(1<<0)	   //INT_EN
						        +(2<<1)	   //DST_TR_WIDTH 
							+(2<<4)	   //SRC_TR_WIDTH
						        +(0<<7)	   //DINC 1x = No change
						        +(7<<9)	   //SINC 00 = Increment
						        +(1<<11)   //DEST_MSIZE
						        +(1<<14)   //SRC_MSIZE
						        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
						        +(0<<23)   //DMS Destination Master interface.
						        +(0<<25)   //SMS source Master interface
						        +(0<<27)   //LLP_DST_DISABLE
						        +(0<<28);   //LLP_SRC_DISABLE		
	dma_llp_rx_vir[i].ctrl_h = audio_dma_rx_size;
}

/*
* dma hardware and transfer pkg init
*/
#define ASM9260_AUDIO_DMA_PACKAGE   1
#define ASM9260_DMA1_CH0            0
#define ASM9260_DMA1_CH1            1
static int audio_init_dma(audio_stream_t * s)
{
        int ret = 0;
	
	as3310_writel((1<<9) | (1<<10), HW_AHBCLKCTRL0+4);  //DMA  clk gate

	if (s->dma_ch == ASM9260_DMA1_CH0)
	{
		dma_llp_tx_vir = (struct dma_llp *)dma_alloc_coherent(NULL, sizeof(struct dma_llp) * 40, &dma_llp_tx_phy, GFP_KERNEL|GFP_DMA);
		if (!dma_llp_tx_vir)
		{
			printk("unable to alloc dma_llp_rx_vir\n");
			return -ENOMEM;
		}
		as3310_writel(HW_I2S1_TXDMA,HW_DMA1_DAR0); //DMA1 channel0 Transfer
		as3310_writel(0x00000000
			        +(0<<0)	   //INT_EN
			        +(1<<1)	   //DST_TR_WIDTH 
					+(1<<4)	   //SRC_TR_WIDTH
			        +(2<<7)	   //DINC 1x = No change
			        +(0<<9)	   //SINC 00 = Increment
			        +(1<<11)   //DEST_MSIZE
			        +(1<<14)   //SRC_MSIZE
			        +(1<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
			        +(0<<23)   //DMS Destination Master interface.
			        +(0<<25)   //SMS source Master interface
			        +(1<<27)   //LLP_DST_DISABLE
			        +(0<<28)   //LLP_SRC_DISABLE
					,HW_DMA1_CTL0);
		as3310_writel(audio_dma_tx_size, HW_DMA1_CTL0+4);
		as3310_writel(dma_llp_tx_phy, HW_DMA1_LLP0);
		as3310_writel(((as3310_readl(HW_DMA1_CFG0) & (~(1<<10))) & (~(1<<30))) & (~(1<<31)),HW_DMA1_CFG0); //hardware handshaking
		as3310_writel(0x00000000+(6<<7)    //SRC_PRE req2 to channel0                   
						+(6<<11),HW_DMA1_CFG0+4); //DST_PRE req2 to channel0

		i2s_dma_tx_pkg_config(AUDIO_FRAGSIZE_DEFAULT);
	
		as3310_writel(0x101,HW_DMA1_MaskTFR); //	channel0 and channel1 transfer finish interrupt open

		ret = asm9260_i2s_dma_info_alloc(&output_stream);
		if (ret)
		{
			return ret;
		}
		
		ret = request_irq(INT_DMA1_CH0, asm9260_iis_write_irq, IRQF_DISABLED, "asm9260_dma1_ch0", (void *)(i2s_dma_info_tx));
		if (ret) 
		{
			printk(KERN_ERR PFX "request irq %d failed, ret = %x\n", INT_DMA1_CH0,ret);
			kfree(i2s_dma_info_tx);
			dma_free_coherent(NULL, sizeof(struct dma_llp) * 40, dma_llp_tx_vir, dma_llp_tx_phy);
			return ret;
		}

	}


	if (s->dma_ch == ASM9260_DMA1_CH1)
	{
		dma_llp_rx_vir = (struct dma_llp *)dma_alloc_coherent(NULL, sizeof(struct dma_llp) * 80, &dma_llp_rx_phy, GFP_KERNEL|GFP_DMA);
		if (!dma_llp_rx_vir)
		{
			printk("unable to alloc dma_llp_rx_vir\n");
			return -ENOMEM;
		}
		as3310_writel(HW_I2S1_RXDMA,HW_DMA1_SAR1);  //DMA1 channel1 receive
		as3310_writel(0x00000000
			        +(0<<0)	   //INT_EN
			        +(1<<1)	   //DST_TR_WIDTH 
					+(1<<4)	   //SRC_TR_WIDTH
			        +(0<<7)	   //DINC 00 = Increment
			        +(2<<9)	   //SINC 1x = No change  
			        +(1<<11)   //DEST_MSIZE
			        +(1<<14)   //SRC_MSIZE
			        +(2<<20)   //TT_FC 00=m2m,01=m2p,02=p2m,11=p2p
			        +(0<<23)   //DMS Destination Master interface.
			        +(0<<25)   //SMS source Master interface
			        +(0<<27)   //LLP_DST_DISABLE
			        +(1<<28)   //LLP_SRC_DISABLE
					,HW_DMA1_CTL1);

		as3310_writel(audio_dma_rx_size,HW_DMA1_CTL1+4);
		as3310_writel(dma_llp_rx_phy, HW_DMA1_LLP1);
		as3310_writel(((as3310_readl(HW_DMA1_CFG1) & (~(1<<11))) & (~(1<<31))) & (~(1<<31)),HW_DMA1_CFG1); //hardware handshaking
		as3310_writel(0x00000000+(7<<7)    //SRC_PRE req2 to channel0                   
						+(7<<11),HW_DMA1_CFG1+4); //DST_PRE req2 to channel0

		i2s_dma_rx_pkg_config(AUDIO_FRAGSIZE_DEFAULT);

		as3310_writel(0x202,HW_DMA1_MaskTFR); //	channel0 and channel1 transfer finish interrupt open

		ret = asm9260_i2s_dma_info_alloc(&input_stream);
		if (ret)
		{
			return ret;
		}

		ret = request_irq(INT_DMA1_CH1, asm9260_iis_read_irq, IRQF_DISABLED, "asm9260_dma1_ch1", (void *)(i2s_dma_info_rx));
		if (ret) 
		{
			printk(KERN_ERR PFX "request irq %d failed, ret = %x\n", INT_DMA1_CH1,ret);
			kfree(i2s_dma_info_rx);
			dma_free_coherent(NULL, sizeof(struct dma_llp) * 80, dma_llp_rx_vir, dma_llp_rx_phy);
			return ret;
		}
	}

	return ret;
}

/*
* i2s clock init
*/
static void asm9260_iis_clk_init(void)
{
        as3310_writel(0x1<<15,HW_AHBCLKCTRL1 + SET_OFFSET);  //open i2s1 clk gate
	as3310_writel(0x0,HW_I2S1CLKSEL);  		//Select Syspll as I2S1 clk source
	as3310_writel(0x0,HW_I2S1CLKUEN);
	as3310_writel(0x1,HW_I2S1CLKUEN);
	as3310_writel(0x1,HW_I2S1_MCLKDIV);  		//set I2S1 MCLK DIV=0x1
	as3310_writel(0x1,HW_I2S1_SCLKDIV);		//I2S1 SCLK 
}


static int asm9260_iis_probe(struct platform_device *dev) 
{
	/* Configure the I2S pins in correct mode */
	set_pin_mux(5, 0, 3);           //I2S1 MCLK
	set_pin_mux(5, 1, 3);           //I2S1 BCLK
	set_pin_mux(5, 2, 3);           //I2S1 LRC
	set_pin_mux(5, 3, 3);           //I2S1 RX0
	set_pin_mux(5, 4, 3);           //I2S1 TX0
	set_pin_mux(5, 5, 3);           //I2S1 TX1
	set_pin_mux(5, 6, 3);           //I2S1 TX2

	/*enable iis clock*/
	asm9260_iis_clk_init( );
	init_asm9260_iis_bus( );

	if(init_wm8731()<0)
	{
	    printk("init_wm8731 failed\n");
		return -EBUSY;
	}
			

	output_stream.dma_ch = ASM9260_DMA1_CH0;
	if (output_stream.dma_ch == ASM9260_DMA1_CH0)
	{
		audio_dma_tx_size = 0x400;
	}
	if (audio_init_dma(&output_stream)) {
		printk( KERN_WARNING AUDIO_NAME_VERBOSE
				": unable to get irq number or alloc dma_llp_tx_vir\n" );
		return -EBUSY;
	}
    
	input_stream.dma_ch = ASM9260_DMA1_CH1;
	if (input_stream.dma_ch == ASM9260_DMA1_CH1)
	{
		audio_dma_rx_size = 0x200;
	}
	if (audio_init_dma(&input_stream)) {
		printk( KERN_WARNING AUDIO_NAME_VERBOSE
				": unable to get irq number or alloc dma_llp_rx_vir\n" );
		return -EBUSY;
	}


	audio_dev_dsp = register_sound_dsp(&asm9260_audio_fops, -1);
	audio_dev_mixer = register_sound_mixer(&asm9260_mixer_fops, -1);
	audio_sclk = AUDIO_SCLK_DEFAULT;

        audio_rate = AUDIO_RATE_DEFAULT;        
	audio_channels = AUDIO_CHANNELS_DEFAULT;        
	audio_fragsize = AUDIO_FRAGSIZE_DEFAULT;        
	audio_nbfrags = AUDIO_NBFRAGS_DEFAULT;	
        audio_setup_buf(&output_stream);
	audio_setup_buf(&input_stream);
	printk(AUDIO_NAME_VERBOSE " initialized\n"); 

	return 0;
}

static int asm9260_iis_remove(struct platform_device *dev) {
    unregister_sound_dsp(audio_dev_dsp);
    unregister_sound_mixer(audio_dev_mixer);
    free_irq(INT_DMA1_CH0, (void *)(i2s_dma_info_tx));
    free_irq(INT_DMA1_CH1, (void *)(i2s_dma_info_rx));

    if(i2s_dma_info_tx)
        kfree(i2s_dma_info_tx);
    if(dma_llp_tx_vir)
	dma_free_coherent(NULL, sizeof(struct dma_llp) * 40, dma_llp_tx_vir, dma_llp_tx_phy);
    if(i2s_dma_info_rx)
	kfree(i2s_dma_info_rx);
    if(dma_llp_rx_vir)
	dma_free_coherent(NULL, sizeof(struct dma_llp) * 80, dma_llp_rx_vir, dma_llp_rx_phy);

    audio_clear_buf(&input_stream);
    audio_clear_buf(&output_stream);

    printk(AUDIO_NAME_VERBOSE " unloaded\n");
    return 0;
}

static struct platform_driver asm9260_iis_driver = {
    .probe = asm9260_iis_probe,
    .remove = asm9260_iis_remove,
    .driver = {
        .name = "asm9260-iis",
        .owner = THIS_MODULE,
    }
};

static int __init asm9260_wm8731_init(void) 
{
    memzero(&input_stream, sizeof(audio_stream_t));
    memzero(&output_stream, sizeof(audio_stream_t));
    return platform_driver_register(&asm9260_iis_driver);
}

static void __exit asm9260_wm8731_exit(void) 
{
    platform_driver_unregister(&asm9260_iis_driver);
}


module_init(asm9260_wm8731_init);
module_exit(asm9260_wm8731_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lu <lujy@alpscale.cn>");
MODULE_DESCRIPTION("ASM9260 WM8731 sound driver");

