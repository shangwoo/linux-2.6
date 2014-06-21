/*
 *  linux/drivers/mmc/as9260mci.c - ALPSCALE AS9260 SD Driver
 *
 *  Copyright (C) 2013 ALPSCALE
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*   The three entry points are as9260_sdi_request, as9260_sdi_set_ios
   and as9260_sdi_get_ro.

   SET IOS
     This configures the device to put it into the correct mode and clock speed
     required.

   MCI REQUEST
     MCI request processes the commands sent in the mmc_request structure. This
     can consist of a processing command and a stop command in the case of
     multiple block transfers.

     There are three main types of request, commands, reads and writes.

     Commands are straight forward. The command is submitted to the controller and
     the request function returns. When the controller generates an interrupt to indicate
     the command is finished, the response to the command are read and the mmc_request_done
     function called to end the request.

     Reads and writes work in a similar manner to normal commands but involve the DMA
     controller to manage the transfers.

     A read is done from the controller directly to the scatterlist passed in from the request.
     Due to a bug in the AS9260 controller, when a read is completed, all the words are byte
     swapped in the scatterlist buffers. 

     The sequence of read interrupts is: ENDRX, RXBUFF, CMDRDY

     A write is slightly different in that the bytes to write are read from the scatterlist
     into a dma memory buffer (this is in case the source buffer should be read only). The
     entire write buffer is then done from this single dma memory buffer.

     The sequence of write interrupts is: ENDTX, TXBUFE, NOTBUSY, CMDRDY

   GET RO
     Gets the status of the write protect pin, if available.
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/mmc.h>

#include <mach/mci.h>
#include <mach/pincontrol.h>
#include <mach/dma.h>


//#define FPGA

#define DRIVER_NAME "as9260_sdi"
#define MMC_ERR_NONE 0
#define USE_DMA_IRQ 0


#define FL_SENT_COMMAND	(1 << 0)
#define FL_SENT_STOP	(1 << 1)

#define MAX_DATA_WAIT           0x100  
#define MAX_DMA_DATA_WAIT       0x200000
#define as9260_sdi_read(host, reg)	__raw_readl((host)->baseaddr + (reg))
#define as9260_sdi_write(host, reg, val)	__raw_writel((val), (host)->baseaddr + (reg))



#if 0 
#define sdi_dbg(x...) printk(x);
#else
#define sdi_dbg(x...) do{}while(0);
#endif

#define AS9260_MCI_ENDRX 1<<0
#define AS9260_MCI_TXBUFE 2<<0


DmaChain *sd_dma;

/*
 * Low level type for this driver
 */
struct as9260sdi_host
{
	struct mmc_host *mmc;
	struct mmc_command *cmd;
	struct mmc_request *request;
	struct device *  dev;
	struct as9260_mmc_data * board;
	void __iomem *baseaddr;
	int irq;
	/* DMA buffer used for transmitting */
	unsigned int* buffer;
	dma_addr_t physical_address;
	unsigned int total_length;
	int present;
	/*
	 * Flag indicating when the command has been sent. This is used to
	 * work out whether or not to send the stop
	 */
	unsigned int flags;
	/* flag for current bus settings */
	u32 bus_mode;
	u32 bus_width;
	u32 timing;
	spinlock_t		complete_lock;
    struct completion	complete_request;
	struct completion	complete_dma;
	enum as9260sdi_waitfor	complete_what;
};


static int as9260_reset_SD_module( void )
{
    as3310_writel( 1<<24, HW_AHBCLKCTRL0+4);
    as3310_writel( 1<<24, HW_PRESETCTRL0+8);
    as3310_writel( 1<<24, HW_AHBCLKCTRL0+4);
    as3310_writel( 1<<24, HW_PRESETCTRL0+4); 
    as3310_writel( 1<<24, HW_AHBCLKCTRL0+4);
	return 0;
}



/*
 * Do a DMA write
 */
static inline int as9260sdi_dma_write(struct as9260sdi_host *host)
{   
	unsigned int i,len,size,curlen;
	struct mmc_data *data;
	struct scatterlist *sg;	
    u32 temreg_source_aligned, temreg_source_not_aligned;
    int remaindbytes;
    int SDblockwait = MAX_DMA_DATA_WAIT;
    int err = 0;

	data=host->request->data;
	size = host->total_length;
	len =data->sg_len;

	/*
	 * Just loop through all entries. Size might not
	 * be the entire list though so make sure that
	 * we do not transfer too much.
	 */

    sg = data->sg;
    if (dma_map_sg(mmc_dev(host->mmc),data->sg,len, DMA_TO_DEVICE) == 0){
        printk("$as9260sdi_sg_to_dma$: dma_map_sg failed!\n");
        return -1;
    }




   temreg_source_aligned = 0x00000000+(0<<0)    //INT_EN, ch0 irq disable
                 	     +(2<<1)      // DST_TR_WIDTH, des transfer width, should set to HSIZE, here is 010, means 32bit
                 		 +(2<<4)      // SRC_TR_WIDTH, sor transfer width, should set to HSIZE, here is 010, means 32bit
                 	  	 +(2<<7)      // DINC, des addr increment, des is SD, so should set to 1x, means no change
                 	 	 +(0<<9)      // SINC, sor addr increment, src is sdram, so should set to 00, means to increase 
                 		 +(0<<11)     // DEST_MSIZE, des burst length, set to 000 means 1 DST_TR_WIDTH per burst transcation
                 	 	 +(0<<14)     // SRC_MSIZE, sor burst length, set to 000 means 1 SRC_TR_WIDTH per burst transcation
                 	 	 +(1<<20)     // TT_FC,transfer type and flow control,001 means memory to peripheral,dma is flow controller
                 		 +(0<<23)     // DMS, des master select, 0 means ahb master 0
                 		 +(0<<25)     // SMS, sor master select, 0 means ahb master 0
                 		 +(0<<27)     // LLP_DST_EN, des block chaining enable, set to 0 disable it
                		 +(0<<28) ;   // LLP_SOR_EN, sor block chaining enable, set to 0 disable it
   temreg_source_not_aligned = 0x00000000+(0<<0)    //INT_EN, ch0 irq disable
                 	         +(2<<1)      // DST_TR_WIDTH, des transfer width, should set to HSIZE, here is 010, means 32bit
                 		     +(0<<4)      // SRC_TR_WIDTH, sor transfer width, should set to HSIZE, here is 000, means 8bit
                 	  	     +(2<<7)      // DINC, des addr increment, des is SD, so should set to 1x, means no change
                 	 	     +(0<<9)      // SINC, sor addr increment, src is sdram, so should set to 00, means to increase 
                 		     +(0<<11)     // DEST_MSIZE, des burst length, set to 000 means 1 DST_TR_WIDTH per burst transcation
                 	 	     +(1<<14)     // SRC_MSIZE, sor burst length, set to 001 means 4 SRC_TR_WIDTH per burst transcation
                 	 	     +(1<<20)     // TT_FC,transfer type and flow control,001 means memory to peripheral,dma is flow controller
                 		     +(0<<23)     // DMS, des master select, 0 means ahb master 0
                 		     +(0<<25)     // SMS, sor master select, 0 means ahb master 0
                 		     +(0<<27)     // LLP_DST_EN, des block chaining enable, set to 0 disable it
                		     +(0<<28) ;   // LLP_SOR_EN, sor block chaining enable, set to 0 disable it

    as9260_sdi_write(host, HW_SSP_CMD0, host->request->cmd->opcode);
    as9260_sdi_write(host, HW_SSP_CMD1, host->request->cmd->arg);
    remaindbytes = data->blksz*data->blocks;


    curlen = 0;
    for(i=0; i<data->blocks; i++)
    {
   			sd_dma->chain_head[0].SAR = (u32)(sg->dma_address+data->blksz*curlen);
   			sd_dma->chain_head[0].DAR = (u32)(ALPAS9260_SSP_BASE+HW_SSP_DATA-HW_SSP_CTRL0);				    
			sd_dma->chain_head[0].LLP = (u32)0;

            if( (sd_dma->chain_head[0].SAR&0x03) == 0) //source addr 4bytes aligned.
            {
		        sd_dma->chain_head[0].CTRL_L = temreg_source_aligned;
		        sd_dma->chain_head[0].CTRL_H = 0x0+(data->blksz/4)+((data->blksz%4)!=0); 
            }
	        else
	        {
		        sd_dma->chain_head[0].CTRL_L = temreg_source_not_aligned;
		        sd_dma->chain_head[0].CTRL_H = 0x0+(data->blksz/1)+((data->blksz%1)!=0); 
	        }	
			
            curlen++; 
            if((curlen*data->blksz)==sg->length)
            {
                curlen=0;
                sg++;
            }

            as9260_sdi_write(host, HW_SSP_XFER, data->blksz);

            if(remaindbytes == data->blksz*data->blocks) 
            {
                 if (host->bus_width == 1)
                 {
                     as9260_sdi_write(host, HW_SSP_CTRL0, (u32)(0x05030000 | (0 << 25) |(1<<29)));
                 }
                 else
                 {
                     as9260_sdi_write(host, HW_SSP_CTRL0, (u32)(0x05430000 | (0 << 25) | (1<<29)));
                 }
            }
            else
            {
                 if (host->bus_width == 1)
                 {
                      as9260_sdi_write(host, HW_SSP_CTRL0, (u32)(0x05000000 | (0 << 25) |(1<<29))); 
                 }
                 else
                 {
                      as9260_sdi_write(host, HW_SSP_CTRL0, (u32)(0x05400000 | (0 << 25) | (1<<29)));
                 }
            }


            /*Before calling DmaStartRoutine, we must set Channel Configuration Register(CFG) properly.*/
            DmaChannelCFGset(sd_dma->dmamodule, sd_dma->dmachannel, 0, (0x00000000+(1<<7)+(1<<11)));

            if( DmaStartRoutine(sd_dma->dmamodule, sd_dma->dmachannel, sd_dma->chain_phy_addr, (void *)sd_dma->chain_head, 1, DMAtranstype1) )
            {
                printk("SD: DMA routine failed for write\n");
                err = -1;
            }

            if (err == -1) {
                break;
            }

            SDblockwait = MAX_DMA_DATA_WAIT;
            while( as9260_sdi_read(host, HW_SSP_STATUS)&0x01 ) //wait until SD card finishes the current BLOCK
            {
                if( (SDblockwait--) <= 0 )
                {
                    printk("SD: time out for current write block\n");
                    err = -1;
                    break;
                }
            }

            if (err == -1) {
                break;
            }

            remaindbytes -= data->blksz;       
    }

    if(err!=0)
    {
        DmaStop(sd_dma->dmamodule, sd_dma->dmachannel);  //warning: interrupt setting cleared!
        data->error=-ETIMEDOUT;
    }

    dma_unmap_sg(mmc_dev(host->mmc),data->sg, len, DMA_TO_DEVICE);				

    if(err!=0)
    {
        return -1;
    }

    return 0;
}

/*
 * Do a dma read
 */
static int as9260sdi_dma_read(struct as9260sdi_host *host)
{
    unsigned int i,len,curlen;
	struct scatterlist *sg;
	struct mmc_command *cmd;
	struct mmc_data *data;
    u32 temreg_source_aligned, temreg_source_not_aligned;
    int remaindbytes;
    int SDblockwait = MAX_DMA_DATA_WAIT;
    int err = 0;


  	cmd = host->cmd;
	if (!cmd) {
		return -1;
	}
    data=host->request->data;
	if (!data) {
		sdi_dbg("no data\n");
		return -1;
	}
    len =data->sg_len;


	sg = data->sg;
    if (dma_map_sg(mmc_dev(host->mmc),data->sg,len, DMA_FROM_DEVICE) == 0){
        printk("$as9260sdi_dma_read$: dma_map_sg failed!\n");
          return -1;
    }
 


   temreg_source_aligned = 0x00000000+(0<<0)      //INT_EN, ch0 irq disable
       			           +(2<<1)      // DST_TR_WIDTH, des transfer width, should set to HSIZE, here is 010, means 32bit
	                       +(2<<4)      // SRC_TR_WIDTH, sor transfer width, should set to HSIZE, here is 010, means 32bit
			               +(0<<7)      // DINC, des addr increment, des is sdram, so should set to 00, means to increase
	                       +(2<<9)      // SINC, sor addr increment, src is SD, so should set to 1x, means no change 
			               +(0<<11)     // DEST_MSIZE, des burst length, set to 000 means 1 DST_TR_WIDTH per burst transcation
			               +(0<<14)     // SRC_MSIZE, sor burst length, set to 000 means 1 SRC_TR_WIDTH per burst transcation
	                       +(2<<20)     // TT_FC,transfer type and flow control,010 means peripheral to memory,dma is flow controller
	                       +(0<<23)     // DMS, des master select, 0 means ahb master 0
	                       +(0<<25)     // SMS, sor master select, 0 means ahb master 0
	                       +(0<<27)     // LLP_DST_EN, des block chaining enable, set to 0 disable it
	                       +(0<<28);     // LLP_SOR_EN, sor block chaining enable, set to 0 disable it	

   temreg_source_not_aligned = 0x00000000+(0<<0)      //INT_EN, ch0 irq disable
       			           +(0<<1)      // DST_TR_WIDTH, des transfer width, should set to HSIZE, here is 000, means 8bit
	                       +(2<<4)      // SRC_TR_WIDTH, sor transfer width, should set to HSIZE, here is 010, means 32bit
			               +(0<<7)      // DINC, des addr increment, des is sdram, so should set to 00, means to increase
	                       +(2<<9)      // SINC, sor addr increment, src is SD, so should set to 1x, means no change 
			               +(1<<11)     // DEST_MSIZE, des burst length, set to 001 means 4 DST_TR_WIDTH per burst transcation
			               +(0<<14)     // SRC_MSIZE, sor burst length, set to 000 means 1 SRC_TR_WIDTH per burst transcation
	                       +(2<<20)     // TT_FC,transfer type and flow control,010 means peripheral to memory,dma is flow controller
	                       +(0<<23)     // DMS, des master select, 0 means ahb master 0
	                       +(0<<25)     // SMS, sor master select, 0 means ahb master 0
	                       +(0<<27)     // LLP_DST_EN, des block chaining enable, set to 0 disable it
	                       +(0<<28);     // LLP_SOR_EN, sor block chaining enable, set to 0 disable it	

    as9260_sdi_write(host, HW_SSP_CMD0, host->request->cmd->opcode);
    as9260_sdi_write(host, HW_SSP_CMD1, host->request->cmd->arg);
    remaindbytes = data->blksz*data->blocks;


    curlen = 0;
    for(i=0; i<data->blocks; i++)
    {
           	sd_dma->chain_head[0].SAR = (u32)(ALPAS9260_SSP_BASE+HW_SSP_DATA-HW_SSP_CTRL0);
   			sd_dma->chain_head[0].DAR = (u32)(sg->dma_address+data->blksz*curlen);				    
			sd_dma->chain_head[0].LLP = (u32)0;
	
            if( (sd_dma->chain_head[0].DAR &0x03) == 0)  //dst addr 4bytes aligned.
            {
                sd_dma->chain_head[0].CTRL_L = temreg_source_aligned;
                sd_dma->chain_head[0].CTRL_H = 0x0+(data->blksz/4)+((data->blksz%4)!=0); 
            }
            else
            {
                sd_dma->chain_head[0].CTRL_L = temreg_source_not_aligned;
                sd_dma->chain_head[0].CTRL_H = 0x0+(data->blksz/1)+((data->blksz%1)!=0); 
            }

	        curlen++;
            if((curlen*data->blksz)==sg->length)
            {
                curlen=0;
                sg++;
            }


            as9260_sdi_write(host, HW_SSP_XFER, data->blksz);

	        if(remaindbytes == data->blksz*data->blocks) 
            {
                 if (host->bus_width == 1)
                 {
                     as9260_sdi_write(host, HW_SSP_CTRL0, (u32)(0x05030000 | (1 << 25) |(1<<29)));
                 }
                 else
                 {
                     as9260_sdi_write(host, HW_SSP_CTRL0, (u32)(0x05430000 | (1 << 25) | (1<<29)));
                 }
            }
            else
            {
                 if (host->bus_width == 1)
                 {
                      as9260_sdi_write(host, HW_SSP_CTRL0, (u32)(0x05000000 | (1 << 25) |(1<<29))); 
                 }
                 else
                 {
                      as9260_sdi_write(host, HW_SSP_CTRL0, (u32)(0x05400000 | (1 << 25) | (1<<29)));
                 }
            }


            /*Before calling DmaStartRoutine, we must set Channel Configuration Register(CFG) properly.*/
            DmaChannelCFGset(sd_dma->dmamodule, sd_dma->dmachannel, 0, (0x00000000+(1<<7)+(1<<11)));
            if( DmaStartRoutine(sd_dma->dmamodule, sd_dma->dmachannel, sd_dma->chain_phy_addr, (void *)sd_dma->chain_head, 1, DMAtranstype1) )
            {
                printk("SD: DMA routine failed for read\n");
                err = -1;
            }

            if (err == -1) {
                break;
            }


            SDblockwait = MAX_DMA_DATA_WAIT;
            while( as9260_sdi_read(host, HW_SSP_STATUS)&0x01 ) //wait until SD card finishes the current BLOCK
            {
                if( (SDblockwait--) <= 0 )
                {
                    printk("SD: time out for current write block\n");
                    err = -1;
                    break;
                }
            }

            if (err == -1) {
                break;
            }

            remaindbytes -= data->blksz;  
    }

    if(err!=0)
    {
        DmaStop(sd_dma->dmamodule, sd_dma->dmachannel);  //warning: interrupt setting cleared!
        data->error=-ETIMEDOUT;
    }

    dma_unmap_sg(mmc_dev(host->mmc),data->sg, len, DMA_TO_DEVICE);				

    if(err!=0)
    {
        return -1;
    }

    return 0;     

}

/*
 * Enable the controller
 */
static void as9260_sdi_enable(struct as9260sdi_host *host)
{
    /*we config the pin muxsel and open the clkgate in the function 
    *before we use sdi we also need to config the register.
    */
	
}

/*
 * Disable the controller
 */
static void as9260_sdi_disable(struct as9260sdi_host *host)
{
    //we should close the clkgate and set 1 to the sftrest bit in the ctrl0 register.

    //we also might config the pin to gpio and selet input mux ,that we could ensure the less comsumption.But I am not very sure if it is necessary.
	
}

/*
 * Send a command
 * return the interrupts to enable
 */
static unsigned int as9260_sdi_data(struct as9260sdi_host *host, struct mmc_command *cmd)
{
    
	unsigned int cmdr;
	unsigned int block_length;
	struct mmc_data *data = cmd->data;
	unsigned int blocks;
	unsigned int ier = 0;
	host->cmd = cmd;    
	cmdr = cmd->opcode;
	block_length = data->blksz;
	blocks = data->blocks;
	data->bytes_xfered = 0;
	if (data->flags & MMC_DATA_READ) {
    			 // Handle a read
			host->total_length = 0;
			as9260sdi_dma_read(host);
			ier = AS9260_MCI_ENDRX;
	}
	else if(data->flags & MMC_DATA_WRITE){
		 	// Handle a write
			host->total_length = block_length * blocks;
			as9260sdi_dma_write(host);
			ier = AS9260_MCI_TXBUFE;
    	}

	return ier;
}


static void as9260_stop_trans(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct as9260sdi_host *host = mmc_priv(mmc);
    unsigned int sdi_ctrl0,sdi_carg,sdi_ctrl1;
    unsigned int sdi_cmd;

    sdi_ctrl0=0x26210000;
    sdi_ctrl1=0x00002473;     
    
    sdi_cmd=mrq->stop->opcode;
    sdi_carg=mrq->stop->arg;  
    if (mrq->stop->flags & MMC_RSP_PRESENT) 
    {
         host->complete_what = COMPLETION_RSPFIN;
         sdi_ctrl1 |=AS9260_SSP_CTRL1_RESP_TIMEOUTIRQ_EN;
         sdi_ctrl0 |=0x00020000;
	};

     if(mrq->stop->flags & MMC_RSP_136)
     {     
		 sdi_ctrl0|= AS9260_CTRL0_LONG_RESP;
	 };

     if (host->bus_width==4) {
          sdi_ctrl0 |= AS9260_CTRL0_BUS4;
     }

    host->request = mrq;
    as9260_sdi_write(host,HW_SSP_SDRESP0,0);
    as9260_sdi_write(host,HW_SSP_CTRL1,sdi_ctrl1);
    
    // as9260_sdi_write(host,HW_SSP_TIMING,sdi_timer);
    as9260_sdi_write(host,HW_SSP_CMD0,sdi_cmd);
    as9260_sdi_write(host,HW_SSP_CMD1,sdi_carg);
    as9260_sdi_write(host,HW_SSP_CTRL0,sdi_ctrl0);
       
    if(mrq->stop->flags & MMC_RSP_PRESENT)
    {  
        /*poll until busy*/
        while( (as9260_sdi_read(host,HW_SSP_STATUS)&0x01)==0 ) {
            ;
        }

         /*poll until idle*/
        while(as9260_sdi_read(host,HW_SSP_STATUS)&0x1)
        {
            // printk("w");
        };
    };

	mrq->stop->resp[0] = as9260_sdi_read(host,HW_SSP_SDRESP0);
	mrq->stop->resp[1] = as9260_sdi_read(host,HW_SSP_SDRESP1);
	mrq->stop->resp[2] = as9260_sdi_read(host,HW_SSP_SDRESP2);
	mrq->stop->resp[3] = as9260_sdi_read(host,HW_SSP_SDRESP3);

    //Cleanup controller
	as9260_sdi_write(host,HW_SSP_CMD0,0);
	as9260_sdi_write(host,HW_SSP_CMD1,0);
}
/*
 * Handle an MMC request
 */
static void as9260_sdi_request(struct mmc_host *mmc, struct mmc_request *mrq)
{       
	struct as9260sdi_host *host = mmc_priv(mmc);
	unsigned int sdi_ctrl0,sdi_carg,sdi_ctrl1;
	unsigned int sdi_cmd;

//printk("<<<as9260_sdi_request starts!\n");

	host->request = mrq;
	host->flags = 0;

	
    sdi_ctrl1=0x00002473;
	sdi_cmd=mrq->cmd->opcode;
 	sdi_carg=mrq->cmd->arg;    
 	host->complete_what=COMPLETION_NONE;
	switch(mrq->cmd->opcode)
	{
        case 2:
        case 9:
        case 10:sdi_ctrl0=0x2c0b0000;break;
        case 0:
        case 4:
        case 15:sdi_ctrl0=0x2c010000;break;
        default:sdi_ctrl0=0x2c030000;break;
	}

	if (mrq->cmd->flags & MMC_RSP_PRESENT) {
         host->complete_what = COMPLETION_RSPFIN;
         sdi_ctrl1 |=AS9260_SSP_CTRL1_RESP_TIMEOUTIRQ_EN;
	};

	if(mrq->cmd->flags & MMC_RSP_136) {
		sdi_ctrl0|= AS9260_CTRL0_LONG_RESP;
	};

	if (host->bus_width==4) {
        sdi_ctrl0 |= AS9260_CTRL0_BUS4;
	};  

	if (mrq->data) {
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;		
		sdi_ctrl1 |= AS9260_SSP_CTRL1_DATA_CRCIRQ_EN;
		sdi_ctrl1 |= AS9260_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN;
	}


//    printk("$CMD:%8x, ARG:%8x, ctrl0:%8x-->%8x, ctrl1:%8x-->%8x, status:%8x\n", sdi_cmd, sdi_carg, as9260_sdi_read(host, HW_SSP_CTRL0), sdi_ctrl0, as9260_sdi_read(host, HW_SSP_CTRL1), sdi_ctrl1, as9260_sdi_read(host, HW_SSP_STATUS));

	host->request = mrq;
	as9260_sdi_write(host,HW_SSP_SDRESP0,0);
    as9260_sdi_write(host,HW_SSP_CTRL1,sdi_ctrl1);


    if (!mrq->data)
    {        
 //       printk("$single CMD starts\n");

        as9260_sdi_write(host,HW_SSP_CMD0,sdi_cmd);
        as9260_sdi_write(host,HW_SSP_CMD1,sdi_carg);
        as9260_sdi_write(host,HW_SSP_CTRL0,sdi_ctrl0);

        /*poll until busy*/
        while( (as9260_sdi_read(host,HW_SSP_STATUS)&0x01)==0 ) {
            ;
        }

        /*poll until idle*/
        while( as9260_sdi_read(host,HW_SSP_STATUS)&0x01 )
        {  
             ;
        }            
    
    }

	if(mrq->data)
	{ 
//        printk("$CMD+DATA starts\n");
        as9260_sdi_data(host,mrq->cmd);      
//        printk("$CMD+DATA ends\n");         
 	}

	// Read response
	mrq->cmd->resp[0] = as9260_sdi_read(host,HW_SSP_SDRESP0);
	mrq->cmd->resp[1] = as9260_sdi_read(host,HW_SSP_SDRESP1);
	mrq->cmd->resp[2] = as9260_sdi_read(host,HW_SSP_SDRESP2);
	mrq->cmd->resp[3] = as9260_sdi_read(host,HW_SSP_SDRESP3);


//    printk("$CMD ends, ctrl0:%8x, ctrl1:%8x, status:%8x\n", as9260_sdi_read(host, HW_SSP_CTRL0), as9260_sdi_read(host, HW_SSP_CTRL1), as9260_sdi_read(host,HW_SSP_STATUS));   
//    printk("$resp0: %8x, resp1: %8x\n", mrq->cmd->resp[0], mrq->cmd->resp[1]);

    //Cleanup controller
	as9260_sdi_write(host,HW_SSP_CMD0,0);
	as9260_sdi_write(host,HW_SSP_CMD1,0);      

	// If we have no data transfer we are finished here
	if (!mrq->data) goto request_done;

	// Calulate the amout of bytes transfer, but only if there was no error	
	if(mrq->data->error == MMC_ERR_NONE)
    {
	     mrq->data->bytes_xfered = (mrq->data->blocks*mrq->data->blksz);
	}
    else
    {
         mrq->data->bytes_xfered = 0;
    }

    if (mrq->stop)
    {        
        as9260_stop_trans(mmc,mrq);
    }

    host->request = NULL;

	// If we had an error while transfering data we flush the
	// DMA channel to clear out any garbage
    if(mrq->data->error != MMC_ERR_NONE ) {
//         printk("$as9260_sdi_request$ flushing DMA.\n");
           //MAY reinitialize the DMA module here
     }

     request_done:
        //printk("as9260_sdi_request8\n");
           
      mmc_request_done(host->mmc,mrq); 

}

static unsigned long  get_pllvalue(void)
{
    u32 syspllreg;

    syspllreg=as3310_readl(HW_SYSPLLCTRL);   

    return  (syspllreg*1000000);

}

/*ok
 * Set the IOS
 */
static void as9260_sdi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	int clkdiv,clkrate;
	struct as9260sdi_host *host = mmc_priv(mmc);
	unsigned long  as9260_master_clock, timingsource;
    u32 mainclksel, sspclkdiv;
    unsigned int sdi_ctrl1=0x00002473;

// printk("timing setting starts\n");

     mainclksel = as3310_readl(HW_MAINCLKSEL);
     sspclkdiv = as3310_readl(HW_SSP0CLKDIV);

     if( (mainclksel&0x01)==0 ) {
         as9260_master_clock = 12000000;
     }else{
         as9260_master_clock = get_pllvalue();
         #ifdef FPGA
         as9260_master_clock = 50000000;
         #endif
     }

     
     if( (sspclkdiv&0xff)==0 ) {         
         printk("SD: sspclkdiv ZERO!\n");
         return;
     }else{
         timingsource = as9260_master_clock/(sspclkdiv&0xff);
     }

     if (ios->clock == 0){
        /* Disable the SD controller */
		clkdiv = 0;
        clkrate= 0;
     }else{
         if( (timingsource/ios->clock)>150 ) {
             clkdiv =4;                
         }else{
             clkdiv = 2;
         }
         clkrate=(timingsource/ios->clock)/clkdiv;
     }
     
	if (ios->bus_width == MMC_BUS_WIDTH_4 && host->board->wire4) {
        host->bus_width=4;          
	}
	else {
        host->bus_width=1;       
	}
     
    if( ios->clock == 25000000 ) {
        clkdiv*=10;
    }
    host->timing=0xffff0000|(clkdiv<<8)|clkrate;

    //printk("ios->clock: %d, master_clock: %d, sspdiv: %d, clkdiv:%d, clkrate:%d\n", ios->clock, (int)as9260_master_clock, (int)sspclkdiv, clkdiv, clkrate);

    if (host->board->vcc_pin) {
        switch (ios->power_mode) {
            case MMC_POWER_OFF:
                //as9260_sdi_write(host, HW_SSP_CTRL0+4, 0x80000000);
                break;
        case MMC_POWER_UP:
                case MMC_POWER_ON:
            default:
                //as9260_sdi_write(host, HW_SSP_CTRL0+8, 0xc0000000);
                break;
        }
    }  

    as9260_sdi_write(host,HW_SSP_CTRL1,sdi_ctrl1);
    as9260_sdi_write(host,HW_SSP_TIMING,host->timing);      
    as9260_sdi_write(host,HW_SSP_CTRL1+4,0x0);

    while(as9260_sdi_read(host,HW_SSP_STATUS)&0x1)
    {
         // wait if busy              
    }; 

//printk("timing setting ends\n");
}


/*
 * Handle an interrupt
 */
static irqreturn_t as9260_sdi_irq(int irq, void *devid)
{	

    unsigned int int_status,sdi_ctrl1=0x00002473;
    struct as9260sdi_host *host = devid;

//    printk("SD err irq, CMD:%8X, ARG:%8x\n", host->request->cmd->opcode, host->request->cmd->arg);

	if (!host) {
        printk("hot: NULL\n");
		return IRQ_HANDLED;
	}
	int_status = as9260_sdi_read(host, HW_SSP_CTRL1);
//	printk("1-SDI irq: ctrl1 = %08X, status =%8x\n", int_status, as9260_sdi_read(host,HW_SSP_STATUS));

	if ( (int_status & AS9260_SSP_CTRL1_IRQ_ERR) || (as9260_sdi_read(host,HW_SSP_STATUS) & 0x00002000) ) {
	if (int_status & AS9260_SSP_CTRL1_RESP_ERRIRQ)
        {
            host->request->cmd->error=-EIO;
            host->complete_what=COMPLETION_NONE;    
            as9260_sdi_write(host,HW_SSP_CTRL1+8,AS9260_SSP_CTRL1_RESP_ERRIRQ);
			printk("MMC: AS9260_RESP_ERR\n");
        }
	if (int_status & AS9260_SSP_CTRL1_RESP_TIMEOUTIRQ)
        {
            host->request->cmd->error=-ETIMEDOUT;
            host->complete_what=COMPLETION_NONE;
            as9260_sdi_write(host,HW_SSP_CTRL1+8,AS9260_SSP_CTRL1_RESP_TIMEOUTIRQ);
			printk("MMC: AS9260_RESP_TIMEOUT\n");
        }
	if (int_status & AS9260_SSP_CTRL1_DATA_TIMEOUT_IRQ)
	    {
            host->request->data->error=-ETIMEDOUT;
            host->complete_what=COMPLETION_NONE;     
            as9260_sdi_write(host,HW_SSP_CTRL1+8,AS9260_SSP_CTRL1_DATA_TIMEOUT_IRQ);
			printk("MMC: AS9260_DATA_TIMEOUT\n");
        }
        	
	if (int_status & AS9260_SSP_CTRL1_DATA_CRCIRQ || as9260_sdi_read(host,HW_SSP_STATUS) & 0x00002000)
        {
            host->request->data->error=-EILSEQ;
            host->complete_what=COMPLETION_NONE;      
            as9260_sdi_write(host,HW_SSP_CTRL1+8,AS9260_SSP_CTRL1_DATA_CRCIRQ);       
			printk("MMC: AS9260_DATA_CRC\n");
        }
            
	if (int_status & AS9260_SSP_CTRL1_RECV_TIMEOUTIRQ)
	    {
            host->request->cmd->error=-ETIMEDOUT;
            host->complete_what=COMPLETION_NONE;        
            as9260_sdi_write(host,HW_SSP_CTRL1+8,AS9260_SSP_CTRL1_RECV_TIMEOUTIRQ);
			printk("MMC: AS9260_RECV_TIMEOUT\n");
        }
        	
	if (int_status & AS9260_SSP_CTRL1_RECV_OVERFLWIRQ)
	    {
            host->request->cmd->error=-EILSEQ;
            host->complete_what=COMPLETION_NONE;    
            as9260_sdi_write(host,HW_SSP_CTRL1+8,AS9260_SSP_CTRL1_RECV_OVERFLWIRQ);
            printk("MMC: AS9260_RECV_OVERFLOW\n");	
        }      		
   
    as9260_reset_SD_module();
    }

    as9260_sdi_write(host,HW_SSP_CTRL1,sdi_ctrl1);
    as9260_sdi_write(host,HW_SSP_TIMING,host->timing);
    as9260_sdi_write(host,HW_SSP_CTRL1+4,0x0);

    while(as9260_sdi_read(host,HW_SSP_STATUS)&0x1)
    {
          // wait if busy              
    };   

//	printk("2-SDI irq: ctrl1 = %08X, status =%8x\n", as9260_sdi_read(host, HW_SSP_CTRL1), as9260_sdi_read(host,HW_SSP_STATUS));
    return IRQ_HANDLED;

}

#if 0
/*
 * get the value of detect pin 
 */
static int as9260_get_detect_value(void)
{
    return as3310_readl(PIN_CTRL_DIN_0)&(0x1<<25);
}

static irqreturn_t as9260_mci_dma(int irq,void *_host)
{   
    struct as9260sdi_host *host = _host;
    // printk("as9260_mci_dma\n");
    complete(&(host->complete_dma));    

    as3310_writel(0x2,HW_APBH_CTRL1_CLR);    

    return IRQ_HANDLED;   
}

/*
 * the handle function for the SD card insert and remove interrupt.
 */
static irqreturn_t as9260_mmc_det_irq(int irq, void *_host)
{
	struct as9260sdi_host *host = _host;
	int present =0;

	present= !as9260_get_detect_value();
	/*
	 * we expect this irq on both insert and remove,
	 * and use a short delay to debounce.
	 */
	if (present != host->present) {
		host->present = present;
		
        printk("%s: card %s\n", "sdi",
			present ? "insert" : "remove");
	   
    	if(present){
  	          as3310_writel(0x02000000,PIN_CTRL_IRQPOL0+4);
              // open the ssp clk
              // as3310_writel(0x80000000,HW_CLK_SSPCLKCTRL+8);
        }
        else{
            as3310_writel(0x02000000,PIN_CTRL_IRQPOL0+8);         
            // close the ssp clk
            // as3310_writel(0x80000000,HW_CLK_SSPCLKCTRL+4);
            // as9260_sdi_write(host, HW_SSP_CTRL0+4, 0x80000000);
            // as9260_sdi_write(host, HW_SSP_CTRL0+4, 0x40000000);
    	}

         if (!present) {
			sdi_dbg("****** Resetting SD-card bus width ******\n");
           	}
        	sdi_dbg("mmc_detect_change\n");
            mmc_detect_change(host->mmc, msecs_to_jiffies(100));
        	sdi_dbg("mmc_detect_change finished\n");
    	};
	return IRQ_HANDLED;
}
#endif 


/* 
 * Get the status of the write protect pin.
 */
int as9260_sdi_get_ro(struct mmc_host *mmc)
{
    int read_only = 0;
    /*
    *TODO:we don't have a write protect pin in the board.the sd card are writeable all the time.
    *if we have ,we should get the value from the gpio pin.
    */
	return read_only;
}

static const struct mmc_host_ops as9260_sdi_ops = {
	.request	= as9260_sdi_request,
	.set_ios	= as9260_sdi_set_ios,
	.get_ro		= as9260_sdi_get_ro,
};


static int __init initpins(void)
{
    //configure port[7,0~5] to function pins
    set_pin_mux(14, 0, 4);    
    set_pin_mux(14, 1, 4);   
    set_pin_mux(14, 2, 4);    
    set_pin_mux(14, 3, 4);    
    set_pin_mux(14, 4, 4);    
    set_pin_mux(14, 5, 4);    
    

#if 0
    if(read_GPIO(0,25))
    {
        //request irq level
        io_irq_enable_level(0,25,GPIO_IRQ_LEVEL_LOW);

    }else
    {
        //request irq level
        io_irq_enable_level(0,25,GPIO_IRQ_LEVEL_HIGH);
    }
#endif
    return 0;
}

static int sspdivset(void)
{
    int cpudiv, ahbdiv;

    cpudiv = as3310_readl(HW_CPUCLKDIV)&( (1<<8) -1 );
    ahbdiv = as3310_readl(HW_SYSAHBCLKDIV)&( (1<<8) -1);

    if( (cpudiv==0)||(ahbdiv==0) ) {
        return -1;
    }

    printk("cpudiv: %d, ahbdiv: %d\n", cpudiv, ahbdiv);
    as3310_writel( (u32)(cpudiv*ahbdiv), HW_SSP0CLKDIV);

    return 0;
}


/* 
 * Probe for the device
 */
static int __init as9260_sdi_probe(struct platform_device *pdev)
{  
	struct mmc_host *mmc;
	struct as9260sdi_host *host;
	struct resource *res;
	int ret;
    int channelready = 1;

    printk("as9260 sd probe\n");

    initpins();
    as9260_reset_SD_module();
    DmaInit(DMA_MODULE_0, DMA_CHANNEL_0); //DMA chain allocation MAY be put here;
    if( sspdivset()!=0 ) {
        return -1;
    }
    
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;
	if (!request_mem_region(res->start, res->end-res->start, DRIVER_NAME))
		return -EBUSY;
	mmc = mmc_alloc_host(sizeof(struct as9260sdi_host), &pdev->dev);
	if (!mmc) {
		sdi_dbg("Failed to allocate mmc host\n");
		release_mem_region(res->start, res->end-res->start);
		return -ENOMEM;
	}
	mmc->ops = &as9260_sdi_ops;
	mmc->f_min = 200000;
	mmc->f_max = 25000000; 
	mmc->ocr_avail = MMC_VDD_32_33|MMC_VDD_31_32|MMC_VDD_30_31;//|MMC_VDD_28_29|MMC_VDD_27_28|MMC_VDD_26_27;MMC_VDD_32_33|MMC_VDD_31_32|MMC_VDD_30_31|
	mmc->caps = MMC_CAP_MMC_HIGHSPEED|MMC_CAP_SD_HIGHSPEED;
    mmc->max_phys_segs = 64;
	mmc->max_hw_segs = 64;
	mmc->max_blk_size = 512;	/* BLEN is 11 bits (+1) */
	mmc->max_blk_count = 64;	/* NBLK is 11 bits (+1) */
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;


	host = mmc_priv(mmc);
    host->dev=&pdev->dev;
	host->mmc = mmc;    
	host->bus_mode = 0;
	host->board = pdev->dev.platform_data;
	if (host->board->wire4) {
#ifdef CONFIG_MMC_SUPPORT_4WIRE
		mmc->caps |= MMC_CAP_4_BIT_DATA;
        printk("ALPSCALE MMC: using 4 wire bus mode \n");
#else
		printk("ALPSCALE MMC: 4 wire bus mode not supported by this driver - using 1 wire\n");
#endif
	}

	host->baseaddr = (void *)IO_ADDRESS(res->start);
	as9260_sdi_disable(host);
	as9260_sdi_enable(host);
    host->irq = platform_get_irq(pdev, 0);

	ret = request_irq(host->irq, as9260_sdi_irq, 0, DRIVER_NAME, host);
	if (ret) {
		printk(KERN_ERR "AS9260 MMC: Failed to request SDI interrupt\n");
		mmc_free_host(mmc);
		return ret;
	}
 
	platform_set_drvdata(pdev, mmc);

#if 0
	if (host->board->det_pin)
		host->present = !(as3310_readl(PIN_CTRL_DIN_0)&0x02000000);
	else
		host->present = -1;
#else
        host->present = 1;
#endif 
        printk("adding mmc_host started\n");
	    mmc_add_host(mmc);

#if 0
    	if (host->board->det_pin) {
		ret = request_irq(INT_ASAP1820_PINCTRL0, as9260_mmc_det_irq,
				IRQF_SHARED, DRIVER_NAME, host);
        if (ret)
			printk(KERN_ERR "AS9260 MMC: Couldn't allocate MMC detect irq\n");
        }
#endif

        #if 0
        ret = request_irq(INT_ASAP1820_SPI_DMA,as9260_mci_dma,IRQF_DISABLED,DRIVER_NAME,host);
        if(ret)
            printk(KERN_ERR "AS9260 MMC: Couldn't allocate MMC DMA irq\n");
        as3310_writel(0x00020000,HW_APBH_CTRL1_SET);
        #endif

        
    sd_dma = DmaRequestChain(host->dev, DMA_MODULE_0, DMA_CHANNEL_0, 1, &channelready);
    if( (sd_dma == NULL)||(channelready == 0) )
    {
        if(channelready == 0)
        {
            printk("SD: channel busy!\n");
        }
        printk("SD: request dma chain failed!\n");
        return -1;
    }

                 
	printk("Added SD driver\n");    

//    printk("PRESETCTRL1:%8X, AHBCLKCTRL1:%8X\n", as3310_readl(HW_PRESETCTRL1), as3310_readl(HW_AHBCLKCTRL1));

	return 0;
}

/*
 * Remove a device
 */
static int __exit as9260_sdi_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct as9260sdi_host *host;

	if (!mmc)
		return -1;

	host = mmc_priv(mmc);

	if (host->present != -1) {
		//free_irq(host->board->det_pin, host);
		cancel_delayed_work(&host->mmc->detect);
	}

	as9260_sdi_disable(host);
	mmc_remove_host(mmc);
	free_irq(host->irq, host);

	mmc_free_host(mmc);
	platform_set_drvdata(pdev, NULL);

    
    DmaFreeChain(host->dev, sd_dma);
	sdi_dbg("SDI Removed\n");

	return 0;
}

#ifdef CONFIG_PM

static int as9260_sdi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	int ret = 0;

	if (mmc)
		ret = mmc_suspend_host(mmc, state);

	return ret;
}

static int as9260_sdi_resume(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	int ret = 0;

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define as9260_sdi_suspend	NULL
#define as9260_sdi_resume		NULL
#endif

static struct platform_driver as9260_sdi_driver = {
	.remove		= __exit_p(as9260_sdi_remove),
	.suspend		= as9260_sdi_suspend,
	.resume		= as9260_sdi_resume,
    	.probe      		= as9260_sdi_probe,
	.driver		= {
		.name		= "as9260_sdi",
		.owner	= THIS_MODULE,
	},
};

static int __init as9260_sdi_init(void)
{
    int n;

//    printk(">>>>as9260_sdi_init started!\n");

    sdi_dbg("as9260 sdi_init\n");
    n = platform_driver_register(&as9260_sdi_driver); 
    
//    printk("<<<<as9260_sdi_init exited!\n");
    
    return n; 
}


static void __exit as9260_sdi_exit(void)
{
	platform_driver_unregister(&as9260_sdi_driver);
}

module_init(as9260_sdi_init);
module_exit(as9260_sdi_exit);

MODULE_DESCRIPTION("Alpscale AS9260 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
