#if 0
/*
 * sound/oss/as3310_snd.c
 *
 *
 *  Copyright (C) 2011   alpscale
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/linkage.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/delay.h>

#include <mach/dma.h>
#include <mach/pincontrol.h>
#include <mach/power.h>
#include <mach/hardware.h>
#include <mach/as3310x_monitor.h>
#include <mach/audio.h>
#include "as3310snd.h"
/********************************************************/
static struct as3310snd_info     *snd;
/*
* for dma debuging,open in menuconfig
*/
#ifdef CONFIG_DMA_DEBUG
int dma_status;
#endif

/*
timer handle function
*/
static void detect_handler(unsigned long t)
{
    audio_para_change(HP_SP_DETCT,no_arg);
    snd->detect.expires = jiffies + DETECT_TIME;
    add_timer(&snd->detect);
}

static void demute_handler(unsigned long t)
{
    audio_para_change(OUT_DEMUTE,no_arg);
}

/*
make sure DMA has stopped
*/
static void adc_audio_sync(void)
{
    unsigned int waittimes;

    waittimes = 0x08000000;
    while(snd->audio_in_dma_status != AUDIO_IN_DMA_STOPPED){
        if (waittimes-- <= 0)
        {
            printk("error: adc_audio_sync time out!!!\n");
            break;
        }
    }
}

/*
must make sure all chain_full are 0 and sema is 0.
*/
static void dac_audio_sync(void)
{
    int i;
    unsigned int y=0x50000000;
    volatile int temp = 0;

    while (1) {

        temp = 0;
        for(i=0;i<snd->buffer_num;i++)
           temp += snd->pkg_buf->chain_full[i];

        if (!temp) {
            if (is_apbx_complete(AS3310_DMA_SND_CH)) {
                break;
            }
        }

        if(y--<=0) {
            printk("error: dac_audio_sync time out!!!\n");
            break;
        }

        #ifdef CONFIG_INFO_DEBUG
        printk("syncing...\n");
        #endif
    }

    #ifdef CONFIG_INFO_DEBUG
    for(i=0;i<snd->buffer_num;i++)
        printk("snd->pkg_buf->chain_full[%d] is %d\n",i,snd->pkg_buf->chain_full[i]);
    printk("HW_APBX_CH1_SEMA is 0x%x\n",as3310_readl(HW_APBX_CH1_SEMA));
    #endif

}

/*
Init member_info of struct as3310snd_info
*/
static void dac_audio_reset(void)
{
    unsigned char i;

    #ifdef CONFIG_INFO_DEBUG
    printk("dac_audio_reset()\n");
    #endif

    snd->audio_out_dma_status = DMA_WAIT_TIMES;
    snd->audio_in_dma_status = AUDIO_IN_DMA_STOPPED;//DMA_WAIT_TIMES;

    snd->buffer_size = BUFFER_SIZE;
    snd->buffer_num = BUFS;

    snd->pkg_buf->dma_ptr = 0;
    snd->pkg_buf->usr_ptr = 0;
    snd->pkg_buf_in->dma_ptr = 0;
    snd->pkg_buf_in->usr_ptr = 0;

    for(i=0;i<snd->buffer_num;i++){
        snd->pkg_buf->chain_full[i] = 0;
        snd->pkg_buf->buf_chain_start_dma[i] = (dma_addr_t)(snd->map_dma + i*snd->buffer_size);
        snd->pkg_buf->buf_chain_start_cpu[i] = snd->map_cpu + i*snd->buffer_size;
        snd->pkg_buf->send_len[i] = 0;
        snd->pkg_buf->recv_len[i] = snd->buffer_size;
    }

    for(i=0;i<snd->buffer_num;i++){
        snd->pkg_buf_in->chain_full[i] = 0;
        snd->pkg_buf_in->buf_chain_start_dma[i] = (dma_addr_t)(snd->map_dma_in + i*snd->buffer_size);
        snd->pkg_buf_in->buf_chain_start_cpu[i] = snd->map_cpu_in + i*snd->buffer_size;
        snd->pkg_buf_in->send_len[i] = 0;
        snd->pkg_buf_in->recv_len[i] = snd->buffer_size;
    }

    snd->audio_status = AUDIO_OUT;
    snd->audio_dma_type = 0;

    snd->scale = 1;// scale factor for low bit rate data source, 16bit 2ch output
              // eg. 8bit 1ch source -> 16bit 2ch output, scale = 4
    snd->channels = 2;  //1:single 2:stereo
    snd->bits = 16;
    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("snd->temp_buffer = %p\n",snd->temp_buffer);
    #endif
    memset(snd->temp_buffer,0,BUFFER_SIZE);
}

/*
 * as3310fb_map_snd_memory():
 *	Allocates the snd structure and buffer memory for the sound.
 *	buffer memory is non-cached, non-buffered for the dma operation.
 */
static int __init as3310snd_map_memory(struct as3310snd_info *sndi)
{
    /*  snd buffer malloc    */
    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
	printk("map_sound_memory(sndi=%p)\n", sndi);
    #endif

    sndi->buf_size = SND_BUFFER_SIZE;
    sndi->map_size = PAGE_ALIGN(sndi->buf_size + PAGE_SIZE);
    printk("as3310x audio buffer total length:0x%x\n",sndi->buf_size);

    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("sndi->map_size:0x%x,sndi->map_dma addr:%p\n",sndi->map_size,&(sndi->map_dma));
    #endif

    sndi->map_cpu  = dma_alloc_coherent(NULL,sndi->map_size,&(sndi->map_dma),GFP_KERNEL);

    /*  snd zero buffer malloc    */
    sndi->buf_size_first = PAGE_ALIGN(ZERO_BUFFER_SIZE + PAGE_SIZE);
    sndi->map_cpu_first  = dma_alloc_coherent(NULL,sndi->buf_size_first,&(sndi->map_dma_first),GFP_KERNEL);

    if(!(sndi->temp_buffer = kmalloc(BUFFER_SIZE,GFP_KERNEL))) {
       printk("snd->buf memory alloc error\n");
       return -ENOMEM;
    }
    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("sndi->temp_buffer is %p\n",sndi->temp_buffer);
    #endif

    if (sndi->map_cpu)
    {
		printk("map_snd_phy_memory: dma=%08x cpu =%p size=%08x\n",
			sndi->map_dma, sndi->map_cpu, sndi->map_size);
	}



    sndi->buf_size_in = SND_BUFFER_SIZE;
    sndi->map_size_in = PAGE_ALIGN(sndi->buf_size_in + PAGE_SIZE);
    printk("as3310x audio in buffer total length:0x%x\n",sndi->buf_size_in);

    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("sndi->map_size_in:0x%x,sndi->map_dma_in addr:%p\n",sndi->map_size_in,&(sndi->map_dma_in));
    #endif

    sndi->map_cpu_in  = dma_alloc_coherent(NULL,sndi->map_size_in,&(sndi->map_dma_in),GFP_KERNEL);

    if (sndi->map_cpu_in)
    {
        printk("map_snd_phy_memory: dma_in=%08x cpu_in =%p size_in=%08x\n",
            sndi->map_dma_in, sndi->map_cpu_in, sndi->map_size_in);
    }



    /*  snd out dma alloc    */
    sndi->dmachain = request_as3310_dma_chain(NULL,SND_PKG_NUM,AS3310_DACOUT_CH);
    if (sndi->dmachain)
    {
		printk("sound_dma_pkg_phy_memory: dma=%08x cpu =%p\n",
			sndi->dmachain->chain_phy_addr, sndi->dmachain->chain_head);;
	}

    /*  first zero snd out dma alloc    */
    sndi->dmachain_first = request_as3310_dma_chain(NULL,1,AS3310_DACOUT_CH);
    memset(sndi->map_cpu_first,0,sndi->buf_size_first);

    sndi->dmachain_first->chain_head->NEXT_PKG  = sndi->dmachain->chain_phy_addr;
    sndi->dmachain_first->chain_head->CTRL      = 0x00001046 + (sndi->buf_size_first<<16);
    sndi->dmachain_first->chain_head->BUFFER    = (ulong)(sndi->map_dma_first);
    sndi->dmachain_first->chain_head->CMD0      = 0x00000041;


    /*  snd in dma alloc    */
    sndi->dmachain_in = request_as3310_dma_chain(NULL,SND_PKG_NUM,AS3310_ADCIN_CH);

    if (sndi->dmachain_in)
    {
        printk("sound_in_dma_pkg_phy_memory: dma=%08x cpu =%p\n",
             sndi->dmachain_in->chain_phy_addr, sndi->dmachain_in->chain_head);;
    }
	return sndi->map_cpu ? 0 : -ENOMEM;
}

/*
release DMA buffer
*/
static inline void as3310snd_unmap_memory(struct as3310snd_info *sndi)
{
    kfree(sndi->temp_buffer);
    dma_free_coherent(NULL,sndi->buf_size_first,sndi->map_cpu_first,sndi->map_dma_first);
	dma_free_coherent(NULL,sndi->map_size,sndi->map_cpu, sndi->map_dma);
    dma_free_coherent(NULL,sndi->map_size_in,sndi->map_cpu_in,sndi->map_dma_in);
    free_as3310_dma_chain(NULL,sndi->dmachain);
    free_as3310_dma_chain(NULL,sndi->dmachain_in);
    free_as3310_dma_chain(NULL,sndi->dmachain_first);
}

/*
set member_info of struct as3310_dma_pkg_s
*/
static int snd_pkg_set(unsigned char which,int sent,struct as3310snd_info *sndi)
{
    struct as3310_dma_chain * pdmachain;
    struct as3310_dma_pkg_s * dmapkg;
    unsigned long dma_phy_addr;
    int i;

    pdmachain = sndi->dmachain;
    dmapkg = pdmachain->chain_head;
    dma_phy_addr = (ulong)sndi->pkg_buf->buf_chain_start_dma[which];

    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("dma buffer which:%d\n",which);
    printk("dma_phy_addr:0x%x\n",(unsigned int)dma_phy_addr );
    printk("dma sent set:%d\n",sent);
    #endif

    if (SND_PKG_NUM == 1)
    {
        dmapkg[0].NEXT_PKG = 0;
        dmapkg[0].CTRL = 0x0000104a + (sent<<16);
        dmapkg[0].BUFFER = (ulong)(dma_phy_addr);
        dmapkg[0].CMD0 = 0x00000041;
    }
    else
    {
        for(i = 0; i < SND_PKG_NUM; i++)
        {
            dmapkg[i].CTRL = 0x00001046 + (sent << 16);
            dmapkg[i].BUFFER = (ulong)(dma_phy_addr + i * snd->buffer_size);
            dmapkg[i].CMD0 = 0x00000041;
        }
        dmapkg[SND_PKG_NUM-1].NEXT_PKG = 0;
        dmapkg[SND_PKG_NUM-1].CTRL = 0x0000104A + (sent << 16);
        dmapkg[SND_PKG_NUM-1].BUFFER = (ulong)(dma_phy_addr
                                               + (SND_PKG_NUM - 1)
                                                 * snd->buffer_size);
    }
    return 0;
}

/*
set member_info of struct as3310_dma_pkg_s
*/
static int audio_in_pkg_set(unsigned char which,int sent,struct as3310snd_info *sndi)
{
    struct as3310_dma_chain * pdmachain;
    struct as3310_dma_pkg_s * dmapkg;
    unsigned long dma_phy_addr;
    unsigned char i;

    pdmachain = sndi->dmachain_in;
    dmapkg = pdmachain->chain_head;
    dma_phy_addr = (ulong)sndi->pkg_buf_in->buf_chain_start_dma[which];

    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("dma in buffer which:%d\n",which);
    printk("dma_phy_addr in :0x%x\n",(unsigned int)dma_phy_addr );
    printk("dma in sent set:%d\n",sent);
    #endif

    for(i=0;i<SND_PKG_NUM;i++)
    {
        dmapkg[i].CTRL = 0x00001045 + (sent<<16);
        dmapkg[i].BUFFER = (ulong)(dma_phy_addr + i*snd->buffer_size);
        dmapkg[i].CMD0 = 0x000000a1;
    }
    dmapkg[SND_PKG_NUM-1].CTRL = 0x00001049 + (sent<<16);
    dmapkg[SND_PKG_NUM-1].BUFFER = (ulong)(dma_phy_addr + (SND_PKG_NUM-1)*snd->buffer_size);
    dmapkg[i].CMD0 = 0x000000a1;

    return 0;
}

#ifdef CONFIG_DMA_DEBUG
static void check_dma_status(int dma)
{
    if (dma_status != DMA_OK)
    {
        if (dma == DMA_FAIL_INT)
        {
            printk("dma semophone fail in interupt\n");
        }
        else if (dma == DMA_FAIL_NON_INT)
        {
            printk("dma semophone fail out interupt\n");
        }
        else
        {
            printk("dma semophone fail, status is %d\n",dma_status);
        }
        dma_status = DMA_OK;
    }
}
#endif

/*
 *start_audioout_dma:
 *	If all dma buffer is empty, first time DMA started,zero buffer transfered,
 *	or play audio data.
*/
static void start_audioout_dma(void)
{
    int wait_ch_ready = 0;
	//printk("start dac dma\n");
    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("snd->dmachain->chain_phy_addr:0x%x\n",snd->dmachain->chain_phy_addr);
    #endif

    if (snd->audio_out_dma_status == FIRST_DMA_START)
    {
        //printk("First time DMA started,zero buffer transfered\n");
        while(dma_start_apbx(snd->dmachain_first->chain_phy_addr,
                             SND_PKG_NUM + 1,
                             AS3310_DACOUT_CH) == -AS3310_DMA_CHANNEL_BUSY)
        {
            if(wait_ch_ready++ > MAX_DATA_WAIT)
            {
                printk("DMA ch(%d) busy,can't started\n",1);
                break;
            }
        }
        snd->demute.expires = jiffies + DEMUTE_TIME;
        add_timer(&snd->demute);
    }
    else
    {
        while(dma_start_apbx(snd->dmachain->chain_phy_addr,
                             SND_PKG_NUM,
                             AS3310_DACOUT_CH) == -AS3310_DMA_CHANNEL_BUSY)
        {
            if(wait_ch_ready++ > MAX_DATA_WAIT)
            {
                printk("DMA ch(%d) busy,can't started\n",1);

                #ifdef CONFIG_DMA_DEBUG
                dma_status = DMA_FAIL;
                #endif
                break;
            }
        }
    }
}

/*
start audioin dma
*/
static void start_audioin_dma(void)
{
    int wait_ch_ready;
    wait_ch_ready = 0;

    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("snd->dmachain_in->chain_phy_addr:0x%x\n",snd->dmachain_in->chain_phy_addr);
    #endif

    while(dma_start_apbx((ulong)snd->dmachain_in->chain_phy_addr,
                         SND_PKG_NUM,
                         AS3310_ADCIN_CH) == -AS3310_DMA_CHANNEL_BUSY)
    {
        if(wait_ch_ready++ > MAX_DATA_WAIT)
        {
            printk("DMA ch(%d) busy,can't started\n", AS3310_ADCIN_CH);
            break;
        }
    }
}

/*dac interrupt handler: when a dma operation finishs it cause this interrupt.
and another dma operation of audio out starts in this function.if there is no
buffer left,this handler will reset the first_call to start dma in next
write of driver.
chain_full[snd->pkg_buf->dma_ptr]=0 means this buffer is empty
chain_full[snd->pkg_buf->dma_ptr]=1 means this buffer is full
*/
static irqreturn_t as3310_dac_interrupt(int irq, void *dev_id)
{
    snd->pkg_buf->chain_full[snd->pkg_buf->dma_ptr] = 0;
    //printk("transfer complete,snd->pkg_buf->dma_ptr:%d\n",snd->pkg_buf->dma_ptr);
    snd->pkg_buf->send_len[snd->pkg_buf->dma_ptr] = 0;

    snd->pkg_buf->dma_ptr = (snd->pkg_buf->dma_ptr+1)%snd->buffer_num;

    if(snd->pkg_buf->chain_full[snd->pkg_buf->dma_ptr] == 1)
    {
        snd_pkg_set(snd->pkg_buf->dma_ptr,snd->pkg_buf->send_len[snd->pkg_buf->dma_ptr],snd);
        //snd_pkg_set(snd->pkg_buf->dma_ptr,snd->buffer_size,snd);
        start_audioout_dma();

        #ifdef CONFIG_DMA_DEBUG
        check_dma_status(DMA_FAIL_INT);
        #endif
    }
    else
    {
        snd->audio_out_dma_status = FIRST_DMA_START;

        /*we should do audio_mute() here, because there's no data any more,
        call demute() when start playing again -- hoffer*/
        audio_para_change(OUT_MUTE, no_arg);

        #ifdef CONFIG_INFO_DEBUG
        printk("all buffer is clear,audio out first_call is %d\n",snd->audio_out_dma_status);
        #endif
    }
    complete(snd->snd_dma_complete);
    return IRQ_HANDLED;
}

/*
adc interrupt handler: when a dma operation finishs it cause this interrupt.
and another dma operation of audio in starts in this function.if there is no
buffer left,this handler will reset the first_call to start dma in next
read of driver.
*/
static irqreturn_t as3310_adc_interrupt(int irq, void *dev_id)
{
    snd->pkg_buf_in->chain_full[snd->pkg_buf_in->dma_ptr] = 1;
    //printk("transfer complete,snd->pkg_buf->dma_ptr:%d\n",snd->pkg_buf->dma_ptr);

    snd->pkg_buf_in->dma_ptr = (snd->pkg_buf_in->dma_ptr+1)%snd->buffer_num;

    if(snd->pkg_buf_in->chain_full[snd->pkg_buf_in->dma_ptr] == 0)
    {
        audio_in_pkg_set(snd->pkg_buf_in->dma_ptr,snd->buffer_size,snd);
        start_audioin_dma();
        printk("<");
    }
    else
    {
        #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
        printk("all audio in buffer is full. stop recording.\n");
        #endif
        snd->audio_in_dma_status = AUDIO_IN_DMA_STOPPED;
    }
    complete(snd->snd_in_dma_complete);
    /*
	printk( "HW_AUDIOIN_CTRL: %x\n",as3310_readl(HW_AUDIOIN_CTRL));
	printk( "HW_AUDIOIN_STAT: %x\n",as3310_readl(HW_AUDIOIN_STAT));
	printk( "HW_AUDIOIN_ADCSRR: %x\n",as3310_readl(HW_AUDIOIN_ADCSRR));
	printk( "HW_AUDIOIN_ADCVOLUME: %x\n",as3310_readl(HW_AUDIOIN_ADCVOLUME));
	printk( "HW_AUDIOIN_ADCDEBUG: %x\n",as3310_readl(HW_AUDIOIN_ADCDEBUG));
	printk( "HW_AUDIOIN_ADCVOL: %x\n",as3310_readl(HW_AUDIOIN_ADCVOL));
	printk( "HW_AUDIOIN_MICLINE: %x\n",as3310_readl(HW_AUDIOIN_MICLINE));
	printk( "HW_AUDIOIN_ANACLKCTRL: %x\n",as3310_readl(HW_AUDIOIN_ANACLKCTRL));
	printk( "HW_AUDIOIN_DATA: %x\n",as3310_readl(0x8004C080));
    */
    return IRQ_HANDLED;
}

/*
* audio interface
*/
static int dac_audio_ioctl(struct inode *inode, struct file *file,
			   unsigned int cmd, unsigned long arg)
{
	int val,i,ret;
    int buf_num;
    audio_buf_info info;
    int __user * p = (int __user *)arg;

    #ifdef CONFIG_IOCTL_DEBUG
    get_user(val,p);
    printk("cmd:0x%x,arg:0x%x %d\n",cmd,val,val);
    #endif


	switch (cmd)
	{
    case OSS_GETVERSION:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("OSS_GETVERSION is:0x%x\n",OSS_GETVERSION);
        #endif
        return put_user(SOUND_VERSION, (int *)arg);

    case SNDCTL_DSP_SYNC:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP_SYNC is:0x%x\n",SNDCTL_DSP_SYNC);
        #endif
		dac_audio_sync();
		return 0;

    case SNDCTL_DSP_RESET:
        #ifdef CONFIG_IOCTL_DEBUG
		printk("SNDCTL_DSP_RESET is:0x%x\n",SNDCTL_DSP_RESET);
        #endif
        dac_audio_reset();
		return 0;

    case SNDCTL_DSP_GETFMTS:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP_GETFMTS is:0x%x\n",SNDCTL_DSP_GETFMTS);
		#endif
        return put_user(AFMT_S16_LE|AFMT_U8|AFMT_S8, (int *)arg);

    case SNDCTL_DSP_SETFMT:
        get_user(val,p);
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP + SET_OFFSETFMT is:0x%x\n",SNDCTL_DSP + SET_OFFSETFMT);
        printk("arg is 0x%x or %d\n",val,val);
        #endif

        switch (val)
        {
        case AFMT_S16_LE:
            snd->bits = 16; break;
        case AFMT_U8:
        case AFMT_S8:
            snd->bits = 8; break;
        default:
            printk("User Request format 0x%x is NOT supported.\n",val);
            return -EINVAL;
        }

        snd->scale = 16*2 / (snd->channels*snd->bits);
        //snd->buffer_size = BUFFER_SIZE/snd->scale;
        #ifdef CONFIG_INFO_DEBUG
        printk("bits = %d, channels = %d, scale = %d\n",snd->bits,snd->channels,snd->scale);
        #endif

		return 0;//put_user(AFMT_S16_LE, (int *)arg);

    case SNDCTL_DSP_NONBLOCK:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP_NONBLOCK is:0x%x\n",SNDCTL_DSP_NONBLOCK);
        #endif
		file->f_flags |= O_NONBLOCK;
		return 0;

    case SNDCTL_DSP_GETCAPS:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP_GETCAPS is:0x%x\n",SNDCTL_DSP_GETCAPS);
        #endif
		return 0;

    case SOUND_PCM_WRITE_RATE:
        get_user(val,p);
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SOUND_PCM_WRITE_RATE is:0x%x\n",SOUND_PCM_WRITE_RATE);
        printk("rate is:0x%x %d\n",*(int *)arg,*(int *)arg);
        #endif
		if (val > 0)
		{
            ret = audio_para_change(OUT_SAMPLE_RATE,val);
            return put_user(ret, (int *)arg);
        }
		return -EINVAL;

    case SNDCTL_DSP_GETOSPACE:
        #ifdef CONFIG_IOCTL_DEBUG
        //printk("SNDCTL_DSP_GETOSPACE is:0x%x\n",SNDCTL_DSP_GETOSPACE);
        #endif

        if(snd->pkg_buf->usr_ptr == snd->pkg_buf->dma_ptr)
        {
            if(snd->pkg_buf->chain_full[snd->pkg_buf->usr_ptr]==1)
            {
                info.fragments = 0;
            }
            else
            {
                info.fragments = 1;//BUFS;
            }
        }
        else
        {
            info.fragments = 1;
            /* if(snd->pkg_buf->usr_ptr > snd->pkg_buf->dma_ptr)
                info.fragments = snd->pkg_buf->dma_ptr + snd->buffer_num - snd->pkg_buf->usr_ptr;
            else info.fragments = snd->pkg_buf->dma_ptr - snd->pkg_buf->usr_ptr;
            */
        }
        info.fragstotal = snd->buffer_num*snd->buffer_size/snd->scale;
        info.fragsize = snd->buffer_size/snd->scale;
        info.bytes = snd->buffer_size*info.fragments/snd->scale;

        #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
        printk("fragments:%d, SNDCTL_DSP_GETOSPACE is:0x%x\n",info.fragments,SNDCTL_DSP_GETOSPACE);
        printk("bits = %d, channels = %d, scale = %d\n",snd->bits,snd->channels,snd->scale);
        #endif

        if (copy_to_user((void __user *)arg, &info, sizeof(info)))
			return -EFAULT;
		return 0;

    case SNDCTL_DSP_STEREO:

		if (get_user(val,p))
			return -EFAULT;
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP_STEREO is:0x%x\n",SNDCTL_DSP_STEREO);
        printk("arg is 0x%x or %d\n",val,val);
        #endif

        snd->channels = val ? 2 : 1;
        snd->scale = 16*2 / (snd->channels*snd->bits);

        #ifdef CONFIG_INFO_DEBUG
        printk("bits = %d, channels = %d, scale = %d\n",snd->bits,snd->channels,snd->scale);
        #endif

        return 0;

    case SOUND_PCM_WRITE_CHANNELS:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SOUND_PCM_WRITE_CHANNELS is:0x%x\n",SOUND_PCM_WRITE_CHANNELS);
        #endif
		return put_user(1, (int *)arg);

    case SNDCTL_DSP_SETDUPLEX:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP + SET_OFFSETDUPLEX is:0x%x\n",SNDCTL_DSP + SET_OFFSETDUPLEX);
        #endif
        ret = audio_status_change(AUDIO_IN_OUT);

        snd->audio_status = AUDIO_IN_OUT;
		return 0;

    case SNDCTL_DSP_PROFILE:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP_PROFILE is:0x%x\n",SNDCTL_DSP_PROFILE);
        #endif
		return -EINVAL;

    case SNDCTL_DSP_GETBLKSIZE:
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP_GETBLKSIZE is:0x%x\n",SNDCTL_DSP_GETBLKSIZE);
        #endif
		return put_user(snd->buffer_size/snd->scale, (int *)arg);

    case SNDCTL_DSP_SETFRAGMENT:
		if (get_user(val, p))
			return -EFAULT;

        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP + SET_OFFSETFRAGMENT is:0x%x\n",SNDCTL_DSP + SET_OFFSETFRAGMENT);
        #endif

        snd->buffer_size = (1 << (val & 0xFFFF)) * snd->scale;
        buf_num = (val >> 16);
        if (buf_num * snd->buffer_size > SND_BUFFER_SIZE)
        {
            buf_num = SND_BUFFER_SIZE / snd->buffer_size;
            if (buf_num >MOST_BUFS)
            {
                buf_num = MOST_BUFS;
            }
            val &= 0xFFFF;
            val |= (buf_num << 16);
            printk("don't support so many buffers,only support %d buffers\n",buf_num);
        }
        snd->buffer_num = buf_num;

        #ifdef CONFIG_IOCTL_DEBUG
        printk("snd->buffer_size is 0x%x, snd->buffer_num is %d\n",snd->buffer_size,snd->buffer_num);
        #endif

        for(i=0;i<snd->buffer_num;i++)
        {
             snd->pkg_buf->chain_full[i] = 0;
             snd->pkg_buf->buf_chain_start_dma[i] = (dma_addr_t)(snd->map_dma + i*snd->buffer_size);
             snd->pkg_buf->buf_chain_start_cpu[i] = snd->map_cpu + i*snd->buffer_size;
             snd->pkg_buf->send_len[i] = 0;
             snd->pkg_buf->recv_len[i] = snd->buffer_size;

            #ifdef CONFIG_IOCTL_DEBUG
             printk("snd->pkg_buf->buf_chain_start_dma[%d] is 0x%x, snd->pkg_buf->buf_chain_start_cpu[%d] 0x%p\n",
                    i,snd->pkg_buf->buf_chain_start_dma[i],i,snd->pkg_buf->buf_chain_start_cpu[i]);
            #endif
        }

        for(i=0;i<snd->buffer_num;i++)
        {
             snd->pkg_buf_in->chain_full[i] = 0;
             snd->pkg_buf_in->buf_chain_start_dma[i] = (dma_addr_t)(snd->map_dma_in + i*snd->buffer_size);
             snd->pkg_buf_in->buf_chain_start_cpu[i] = snd->map_cpu_in + i*snd->buffer_size;
             snd->pkg_buf_in->send_len[i] = 0;
             snd->pkg_buf_in->recv_len[i] = snd->buffer_size;

            #ifdef CONFIG_IOCTL_DEBUG
             printk("snd->pkg_buf_in->buf_chain_start_dma[%d] is 0x%x, snd->pkg_buf_in->buf_chain_start_cpu[%d] 0x%p\n",
                    i,snd->pkg_buf_in->buf_chain_start_dma[i],i,snd->pkg_buf_in->buf_chain_start_cpu[i]);
            #endif
        }

		return put_user(val,(int *)arg);

    case SNDCTL_DSP_RECORD:
        if (get_user(val, p))
        {
              return -EFAULT;
        }
        #ifdef CONFIG_IOCTL_DEBUG
        printk("SNDCTL_DSP_RECORD is:0x%x\n",SNDCTL_DSP_RECORD);
        #endif
        snd->audio_status = AUDIO_IN;
        del_timer(&snd->detect);
        ret = audio_status_change(AUDIO_IN);
        ret = audio_para_change(IN_CHANNEL, val);
        return 0;

    case SNDCTL_PCM_RECORD_RATE:
         if (get_user(val, p))
         {
               return -EFAULT;
         }
         #ifdef CONFIG_IOCTL_DEBUG
         printk("SNDCTL_PCM_RECORD_RATE is:0x%x\n",SNDCTL_PCM_RECORD_RATE);
         #endif
         ret = audio_para_change(IN_SAMPLE_RATE,val);

         return 0;

    case SOUND_MIXER_READ_VOLUME:

         #ifdef CONFIG_IOCTL_DEBUG
         printk("SOUND_MIXER_READ_VOLUME is:0x%x\n",SOUND_MIXER_READ_VOLUME);
         #endif

         val = snd->playback_vol;
         return put_user(val,p);

    case SOUND_MIXER_WRITE_VOLUME:
         if (get_user(val, p))
         {
               return -EFAULT;
         }
         #ifdef CONFIG_IOCTL_DEBUG
         printk("SOUND_MIXER_WRITE_VOLUME is:0x%x\n",SOUND_MIXER_WRITE_VOLUME);
         #endif

         ret = audio_para_change(OUT_HP_VOL,val);
         if(ret < 0)
             return -EINVAL;
         ret = audio_para_change(OUT_SP_VOL,val);
         if(ret < 0)
             return -EINVAL;
         if(ret != val) {
             printk("change out_vol error\n");
         }
         snd->playback_vol = ret;
         return put_user(val,p);

    case SNDCTL_MIXER_WRITE_INVOL:
         if (get_user(val, p))
         {
               return -EFAULT;
         }
         #ifdef CONFIG_IOCTL_DEBUG
         printk("SOUND_MIXER_WRITE_VOLUME is:0x%x\n",SOUND_MIXER_WRITE_VOLUME);
         #endif
         ret = audio_para_change(IN_VOL,val);
         if(ret < 0)
         {
             return -EINVAL;
         }
         if(ret != val) {
             printk("change in_vol error\n");
         }
         snd->capture_vol = ret;
         return put_user(val,p);

    case SNDCTL_MIXER_READ_INVOL:

         #ifdef CONFIG_IOCTL_DEBUG
         printk("SOUND_MIXER_WRITE_VOLUME is:0x%x\n",SOUND_MIXER_WRITE_VOLUME);
         #endif

         val = snd->capture_vol;

         return put_user(val,p);
    default:
        #ifdef CONFIG_IOCTL_DEBUG
		printk(KERN_ERR "sh_dac_audio: unimplemented ioctl=0x%x\n",cmd);
        #endif
		return -EINVAL;
	}
	return -EINVAL;
}

/*	write interface:
 *	One of the most importent interfaces,main functions are
 *	playing aduio data and copying data of APP buffer to DMA buffer.
*/
static ssize_t dac_audio_write(struct file *file, const char __user *buf, size_t count,
			       loff_t * ppos)
{
    int left;
    char *data_buffer;
    unsigned long ret = 0;
    //printk("snd->pkg_buf->buf_chain_start_cpu[%d] is 0x%x\n",snd->pkg_buf->usr_ptr,snd->pkg_buf->buf_chain_start_cpu[snd->pkg_buf->usr_ptr]);
    data_buffer = (char*)buf;
    if(count > snd->buffer_size / snd->scale)
    {
        count = snd->buffer_size / snd->scale;
    }

    left = count;
    //snd->pkg_buf->send_len[snd->pkg_buf->usr_ptr] = count;

    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("in audio_write buf addr:0x%p,count:%d,\n", buf, count);
    printk("data to be write :%d\n", left);
    #endif

    if (snd->pkg_buf->chain_full[snd->pkg_buf->usr_ptr])
    {
        #ifdef CONFIG_INFO_DEBUG
        printk("all buffer is full, now wait for buffer %d\n", snd->pkg_buf->usr_ptr);
        #endif
        init_completion(snd->snd_dma_complete);
        wait_for_completion(snd->snd_dma_complete);
    }

    if (snd->scale == 1)
    { // direct 16bit-2ch
        ret = copy_from_user((void*)(snd->pkg_buf->buf_chain_start_cpu[snd->pkg_buf->usr_ptr]),data_buffer,left);
        snd->pkg_buf->send_len[snd->pkg_buf->usr_ptr] = left;
    }
    else if (snd->scale == 4)
    { // 8bit-1ch
        int i;
        short *tmp = (short *)snd->temp_buffer;
        char *tmpb = snd->temp_buffer;
        if ((left << 2) > snd->buffer_size)
        {
            printk("Too big data size for convert 0x%x\n",left);
            left = (snd->buffer_size >> 2);
        }
        ret = copy_from_user((void*)tmp,data_buffer,left);

        for (i = left-1; i >= 0; i--)
        {
            tmpb[(i<<2)+1] = tmpb[(i<<2)+3] = (tmpb[i]^0x80);
        }
        memcpy((void*)(snd->pkg_buf->buf_chain_start_cpu[snd->pkg_buf->usr_ptr]),
		        tmp,
		        (left<<2));
        snd->pkg_buf->send_len[snd->pkg_buf->usr_ptr] = (left << 2);
    }
    else
    {
        printk("Unknow data type for convert scale = %d\n",snd->scale);
    }

    if(ret)
    {
        printk("copy from user error, ret=%d\n",(int)ret);
    }

    snd->pkg_buf->chain_full[snd->pkg_buf->usr_ptr] = 1;
    snd->pkg_buf->usr_ptr = (snd->pkg_buf->usr_ptr + 1) % snd->buffer_num;

    if(snd->audio_out_dma_status > FIRST_DMA_START)
    {
        snd->audio_out_dma_status--;
        #ifdef CONFIG_INFO_DEBUG
        printk("audio out first_call is %d\n", snd->audio_out_dma_status);
        #endif
    }
    else if(snd->audio_out_dma_status == FIRST_DMA_START)
    {
        snd_pkg_set(snd->pkg_buf->dma_ptr,
        			snd->pkg_buf->send_len[snd->pkg_buf->dma_ptr],
        			snd);
        //snd_pkg_set(snd->pkg_buf->dma_ptr,left,snd);
        start_audioout_dma();
        #ifdef CONFIG_DMA_DEBUG
        check_dma_status(DMA_FAIL_NON_INT);
        #endif
        //audio_para_change(OUT_DEMUTE, 0);

        snd->audio_out_dma_status--;
        snd->audio_dma_type |= AUDIO_DMA_OUT;

        #ifdef CONFIG_INFO_DEBUG
        printk("first start dma transfer,audio out first_call is %d\n",snd->audio_out_dma_status);
        printk("HW_APBX_CH1_SEMA is 0x%x\n", as3310_readl(HW_APBX_CH1_SEMA));
        #endif
    }

    return left;
}

/*	read interface:
 *	Another importent interface,main functions are
 *	recording and copy data of DMA buffer to APP buffer.
*/
static ssize_t dac_audio_read(struct file *file, char __user *buf, size_t count,
			       loff_t * ppos)
{
    int num;
    char *data_buffer;
    unsigned long ret;

    data_buffer = (char *)buf;
    num = count;
    if (snd->pkg_buf_in->recv_len[snd->pkg_buf_in->usr_ptr] == snd->buffer_size) {
        snd->pkg_buf_in->recv_len[snd->pkg_buf_in->usr_ptr] = 0;
    }

    if(num > snd->buffer_size - snd->pkg_buf_in->recv_len[snd->pkg_buf_in->usr_ptr])
        num = snd->buffer_size - snd->pkg_buf_in->recv_len[snd->pkg_buf_in->usr_ptr];


    if(snd->audio_in_dma_status == AUDIO_IN_DMA_STOPPED)
    {
        //printk("need dma free HW_APBX_CH0_SEMA is 0x%x\n",as3310_readl(HW_APBX_CH0_SEMA));
        if (snd->pkg_buf_in->chain_full[snd->pkg_buf_in->dma_ptr] == 0) {

            while (as3310_readl(HW_APBX_CH0_SEMA)) {
                printk("dma is still working\n");
            }
            //printk("first start audio in dma\n");
            audio_in_pkg_set(snd->pkg_buf_in->dma_ptr,snd->buffer_size,snd);
            init_completion(snd->snd_in_dma_complete);
            start_audioin_dma();
            snd->audio_in_dma_status = AUDIO_IN_DMA_STARTED;
            snd->audio_dma_type |= AUDIO_DMA_IN;

            #ifdef CONFIG_INFO_DEBUG
            printk("first start dma transfer,first_call is %d\n",snd->audio_in_dma_status);
            #endif
        }
    }

    if (snd->pkg_buf_in->chain_full[snd->pkg_buf_in->usr_ptr] == 0) {
        //printk("all buffer is clear, now wait for buffer %d\n",snd->pkg_buf->usr_ptr);
        wait_for_completion(snd->snd_in_dma_complete);
        init_completion(snd->snd_in_dma_complete);
    }
    ret = copy_to_user(data_buffer,(void*)(snd->pkg_buf_in->buf_chain_start_cpu[snd->pkg_buf_in->usr_ptr] + snd->pkg_buf_in->recv_len[snd->pkg_buf_in->usr_ptr]),num);
    if(ret){
        printk("copy from user error, ret=%d\n",(int)ret);
    }
    snd->pkg_buf_in->recv_len[snd->pkg_buf_in->usr_ptr] += num;

    if (snd->pkg_buf_in->recv_len[snd->pkg_buf_in->usr_ptr] == snd->buffer_size) {
        snd->pkg_buf_in->chain_full[snd->pkg_buf_in->usr_ptr] = 0;
        snd->pkg_buf_in->usr_ptr = (snd->pkg_buf_in->usr_ptr + 1)%snd->buffer_num;
    }
    //printk("count=%d\n",count);
    return num;
}


static int dac_audio_open(struct inode *inode, struct file *file)
{
    printk("================audio opened==============\n");
    module_power(DEVICE_ADUIO, POWER_ON);
    if (snd->in_use)
    {
        printk("audio device already in use\n");
        return -EBUSY;
    }
	snd->in_use = 1;
    dac_audio_reset();
    audio_status_change(AUDIO_OUT);
    change_out_status(1);	//headphone

    #if 0
    printk( "HW_CLK_XTALCLKCTRL: %x\n",as3310_readl(HW_CLK_XTALCLKCTRL));
    printk( "HW_AUDIOIN_CTRL: %x\n",as3310_readl(HW_AUDIOIN_CTRL));
    printk( "HW_AUDIOIN_STAT: %x\n",as3310_readl(HW_AUDIOIN_STAT));
    printk( "HW_AUDIOIN_ADCSRR: %x\n",as3310_readl(HW_AUDIOIN_ADCSRR));
    printk( "HW_AUDIOIN_ADCVOLUME: %x\n",as3310_readl(HW_AUDIOIN_ADCVOLUME));
    printk( "HW_AUDIOIN_ADCDEBUG: %x\n",as3310_readl(HW_AUDIOIN_ADCDEBUG));
    printk( "HW_AUDIOIN_ADCVOL: %x\n",as3310_readl(HW_AUDIOIN_ADCVOL));
    printk( "HW_AUDIOIN_MICLINE: %x\n",as3310_readl(HW_AUDIOIN_MICLINE));
    printk( "HW_AUDIOIN_ANACLKCTRL: %x\n",as3310_readl(HW_AUDIOIN_ANACLKCTRL));
    printk( "HW_AUDIOIN_DATA: %x\n",as3310_readl(0x8004C080));

    printk("HW_AUDIOOUT_DACDEBUG is 0x%X\n",as3310_readl(HW_AUDIOOUT_DACDEBUG));
    printk("HW_AUDIOOUT_DACVOLUME is 0x%X\n",as3310_readl(HW_AUDIOOUT_DACVOLUME));
    printk("HW_AUDIOOUT_STAT is 0x%X\n",as3310_readl(HW_AUDIOOUT_STAT));
    printk("HW_AUDIOOUT_ANACTRL is 0x%X\n",as3310_readl(HW_AUDIOOUT_ANACTRL));
    printk("HW_AUDIOOUT_DACSRR is 0x%X\n",as3310_readl(HW_AUDIOOUT_DACSRR));
    printk("HW_AUDIOOUT_PWRDN is 0x%X\n",as3310_readl(HW_AUDIOOUT_PWRDN));
    printk("HW_AUDIOOUT_CTRL is 0x%X\n",as3310_readl(HW_AUDIOOUT_CTRL));
    printk("HW_AUDIOOUT_HPVOL is 0x%X\n",as3310_readl(HW_AUDIOOUT_HPVOL));
    printk("HW_AUDIOOUT_SPKRVOL is 0x%X\n",as3310_readl(HW_AUDIOOUT_SPKRVOL));
    printk("HW_AUDIOOUT_REFCTRL is 0x%X\n",as3310_readl(HW_AUDIOOUT_REFCTRL));
    printk("HW_AUDIOOUT_BISTCTRL is 0x%X\n",as3310_readl(0x800480b0));
    #endif
    snd->detect.expires = jiffies + DETECT_TIME;
    add_timer(&snd->detect);
    return 0;
}

static int dac_audio_release(struct inode *inode, struct file *file)
{
    if (snd->audio_dma_type & AUDIO_DMA_OUT) {
         dac_audio_sync();
    }

    if (snd->audio_dma_type & AUDIO_DMA_IN) {
         adc_audio_sync();
    }

    if(snd->audio_status==AUDIO_IN_OUT) {
        dac_audio_sync();
        adc_audio_sync();
    }

    #if 0
    printk( "HW_AUDIOIN_CTRL: %x\n",as3310_readl(HW_AUDIOIN_CTRL));
    printk( "HW_AUDIOIN_STAT: %x\n",as3310_readl(HW_AUDIOIN_STAT));
    printk( "HW_AUDIOIN_ADCSRR: %x\n",as3310_readl(HW_AUDIOIN_ADCSRR));
    printk( "HW_AUDIOIN_ADCVOLUME: %x\n",as3310_readl(HW_AUDIOIN_ADCVOLUME));
    printk( "HW_AUDIOIN_ADCDEBUG: %x\n",as3310_readl(HW_AUDIOIN_ADCDEBUG));
    printk( "HW_AUDIOIN_ADCVOL: %x\n",as3310_readl(HW_AUDIOIN_ADCVOL));
    printk( "HW_AUDIOIN_MICLINE: %x\n",as3310_readl(HW_AUDIOIN_MICLINE));
    printk( "HW_AUDIOIN_ANACLKCTRL: %x\n",as3310_readl(HW_AUDIOIN_ANACLKCTRL));
    printk( "HW_AUDIOIN_DATA: %x\n",as3310_readl(0x8004C080));

    printk("HW_AUDIOOUT_DACDEBUG is 0x%X\n",as3310_readl(HW_AUDIOOUT_DACDEBUG));
    printk("HW_AUDIOOUT_DACVOLUME is 0x%X\n",as3310_readl(HW_AUDIOOUT_DACVOLUME));
    printk("HW_AUDIOOUT_STAT is 0x%X\n",as3310_readl(HW_AUDIOOUT_STAT));
    printk("HW_AUDIOOUT_ANACTRL is 0x%X\n",as3310_readl(HW_AUDIOOUT_ANACTRL));
    printk("HW_AUDIOOUT_DACSRR is 0x%X\n",as3310_readl(HW_AUDIOOUT_DACSRR));
    printk("HW_AUDIOOUT_PWRDN is 0x%X\n",as3310_readl(HW_AUDIOOUT_PWRDN));
    printk("HW_AUDIOOUT_CTRL is 0x%X\n",as3310_readl(HW_AUDIOOUT_CTRL));
    printk("HW_AUDIOOUT_HPVOL is 0x%X\n",as3310_readl(HW_AUDIOOUT_HPVOL));
    printk("HW_AUDIOOUT_SPKRVOL is 0x%X\n",as3310_readl(HW_AUDIOOUT_SPKRVOL));
    printk("HW_AUDIOOUT_REFCTRL is 0x%X\n",as3310_readl(HW_AUDIOOUT_REFCTRL));
    printk("HW_AUDIOOUT_BISTCTRL is 0x%X\n",as3310_readl(0x800480b0));
    #endif

    snd->audio_dma_type = 0;
    del_timer(&snd->detect);
    //del_timer(&snd->demute);
    audio_status_change(AUDIO_IDLE);
    snd->in_use = 0;
    module_power(DEVICE_ADUIO, POWER_OFF);

    printk("===============audio release================\n");
	return 0;
}

struct file_operations dac_audio_fops = {
      .read =		dac_audio_read,
      .write =	    dac_audio_write,
      .ioctl =	    dac_audio_ioctl,
      .open =		dac_audio_open,
      .release =	dac_audio_release,
};

static int __init as3310_dac_audio_init(void)
{
	int ret;

    if(!(snd = kmalloc(sizeof(struct as3310snd_info),GFP_KERNEL))){
        printk("snd memory alloc error\n");
        return (int)NULL;
    }
    if(!(snd->snd_dma_complete = kmalloc(sizeof(struct completion),GFP_KERNEL))){
        printk("snd completion alloc error\n");
        return (int)NULL;
    }
    if(!(snd->snd_in_dma_complete = kmalloc(sizeof(struct completion),GFP_KERNEL))){
        printk("snd completion alloc error\n");
        return (int)NULL;
    }

    if(!(snd->pkg_buf = kmalloc(sizeof(struct as3310_snd_buf_info),GFP_KERNEL))){
        printk("snd->buf memory alloc error\n");
        return -ENOMEM;
    }

    if(!(snd->pkg_buf_in = kmalloc(sizeof(struct as3310_snd_buf_info),GFP_KERNEL))){
        printk("snd->buf_in memory alloc error\n");
        return -ENOMEM;
    }

    if ((snd->device_minor = register_sound_dsp(&dac_audio_fops, -1)) < 0) {
        printk(KERN_ERR "Cannot register dsp device");
        return snd->device_minor;
    }

    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("dac audio device minor:%d\n", snd->device_minor);
    #endif

    request_as3310_gpio(GPIO_SPK_PORT, GPIO_SPK_PIN, PIN_FUNCTION_3);
    snd->in_use = 0;
    init_timer(&snd->detect);
    snd->detect.function = &detect_handler;
    init_timer(&snd->demute);
    snd->demute.function = &demute_handler;

    ret = request_irq(INT_ASAP1820_DAC_DMA, as3310_dac_interrupt, IRQF_DISABLED,"as3310-dac", NULL);
    if (ret)
    {
    	printk("cannot get irq %d - err %d\n", INT_ASAP1820_DAC_DMA, ret);
    	return -EBUSY;
    }
    ret = request_irq(INT_ASAP1820_ADC_DMA, as3310_adc_interrupt, IRQF_DISABLED,"as3310-adc", NULL);
    if (ret) {
        printk("cannot get irq %d - err %d\n", INT_ASAP1820_DAC_DMA, ret);
        return -EBUSY;
    }
    ret = as3310snd_map_memory(snd);
    if (ret) {
        printk( KERN_ERR "Failed to allocate snd RAM: %d\n", ret);
        ret = -ENOMEM;
        goto release_snd_memory;
    }

    audio_status_change(AUDIO_INIT);
    snd->playback_vol = 10;
	snd->capture_vol = 8;

    #ifdef CONFIG_SOUND_AS3310_SND_DEBUG
    printk("snd->buf struct inited\n");
    #endif

    init_completion(snd->snd_dma_complete);
#ifdef CONFIG_DMA_DEBUG
    dma_status = DMA_OK;
#endif
    return 0;

release_snd_memory:
    as3310snd_unmap_memory(snd);
	return ret;
}

static void __exit as3310_dac_audio_exit(void)
{
	//free_irq(TIMER1_IRQ, 0);
    free_irq(INT_ASAP1820_DAC_DMA,NULL);
    free_irq(INT_ASAP1820_ADC_DMA,NULL);
    as3310snd_unmap_memory(snd);
	unregister_sound_dsp(snd->device_minor);
    kfree(snd->pkg_buf);
    kfree(snd->pkg_buf_in);
    kfree(snd->snd_dma_complete);
    kfree(snd->snd_in_dma_complete);
    kfree((void *)snd);
}

module_init(as3310_dac_audio_init);
module_exit(as3310_dac_audio_exit);

MODULE_AUTHOR("zorro zb");
MODULE_DESCRIPTION("as3310x sound driver");
MODULE_LICENSE("GPL");
#endif
