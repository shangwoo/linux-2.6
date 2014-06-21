#ifndef AS3310SND_H
#define AS3310SND_H


#define PKG_SIZE_IN_BUFFER          0x5a00                 
#define BUFFER_SIZE                 0x5a00
#define SND_PKG_NUM                 BUFFER_SIZE/PKG_SIZE_IN_BUFFER
#define BUFS                        5
#define MOST_BUFS                   0x20
#define SND_BUFFER_SIZE             BUFFER_SIZE*BUFS                 //24kX4   buffer 
#define ZERO_BUFFER_SIZE            0xCF00                          //about 300 ms data


#define AS3310_DMA_SND_CH           1
#define AS3310_DMA_SND_IN_CH        0
#define FIRST_DMA_START             0
#define DMA_WAIT_TIMES              1
#define AUDIO_IN_DMA_STOPPED        0
#define AUDIO_IN_DMA_STARTED        1
#define no_arg                      0

/***************************** ioctl ********************************/
#define SNDCTL_DSP_RECORD           0xFA  
#define SNDCTL_DSP_LINE_IN          0xFB  
#define SNDCTL_DSP_MIC              0xFC  
#define SNDCTL_PCM_RECORD_RATE      0xFD  
#define SNDCTL_MIXER_READ_INVOL     0xFE    
#define SNDCTL_MIXER_WRITE_INVOL    0xFF
#define MAX_DATA_WAIT               0x100
#define MAX_RBB_WRITE_WAIT          0xffffff


#ifdef CONFIG_DMA_DEBUG
#define DMA_FAIL                    -1
#define DMA_OK                      0
#define DMA_FAIL_INT                1
#define DMA_FAIL_NON_INT            2
#endif   

/*************************************************************/
#define as3310_audio_read(reg)	as3310_readl(reg)
#define as3310_audio_write(val ,reg)	as3310_writel(val , reg)

struct as3310_snd_buf_info {
    dma_addr_t          buf_chain_start_dma[MOST_BUFS];
    unsigned char *     buf_chain_start_cpu[MOST_BUFS];
    unsigned char       dma_ptr;
    unsigned char       usr_ptr;
    //u_int               one_buffer_len;
    unsigned char       chain_full[MOST_BUFS];
    unsigned int        send_len[MOST_BUFS];
    unsigned int        recv_len[MOST_BUFS];
};

struct as3310snd_info {
    struct as3310_snd_buf_info      * pkg_buf; 
    struct as3310_snd_buf_info      * pkg_buf_in;
	struct device		*dev;

	//struct as3310snd_mach_info *mach_info;

	/* raw memory addresses */
	dma_addr_t		map_dma;	/* physical */
    dma_addr_t      map_dma_in;
	unsigned char *	map_cpu;	/* virtual */
    unsigned char * map_cpu_in;
	u_int			map_size;
    u_int           map_size_in;
    u_int           buf_size;
    u_int           buf_size_in;
	int 		    rate;  /* 60-75 Hz */

    /* buffer with all zero value  */
    dma_addr_t		map_dma_first;	/* physical */
    unsigned char *	map_cpu_first;	/* virtual */
    u_int           buf_size_first;

    struct as3310_dma_chain * dmachain_first;
    struct as3310_dma_chain * dmachain;
    struct as3310_dma_chain * dmachain_in;

    int buffer_size;
    int buffer_num;

    /* for dma irq completion */
    struct completion * snd_dma_complete;
    struct completion * snd_in_dma_complete;
    
    int capture_vol;
    int playback_vol;
    int in_use, device_minor;
    volatile int audio_in_dma_status;
    volatile int audio_out_dma_status;
    #define AUDIO_DMA_OUT   (1<<0)
    #define AUDIO_DMA_IN   (1<<1)
    int audio_dma_type;
    int audio_status;
    #define GAIN_SHIFT 8
    int channels;  //1:single 2:stereo
    int bits;  //8,16,32
    int scale;// scale factor for low bit rate data source, 16bit 2ch output
              // eg. 8bit 1ch source -> 16bit 2ch output, scale = 4
    char * temp_buffer; // buffer for temprary format transform
    //#ifdef FAKE_MONO
    //int single;  //1:single 0:stereo
    //#endif
    struct timer_list detect;
    struct timer_list demute;

};

#endif
