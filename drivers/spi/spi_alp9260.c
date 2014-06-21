/*
 * Support for ALPSCALE 9260 SPI controller
 *
 * Copyright (c) 2013 ALPSCALE Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License, version
 * 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 *Note: bits_per_word, currently expected to fix to 8, may try others later. 
 *      speed-switch, SPI_LSB_FIRST-switch not supported either for the moment.
 * 
 *      Due to our controller's internal bug, DMA read addr and bytes must be 4
 *      bytes aligned, or unreleated memory will be overwritten.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <mach/io.h>
#include <mach/dma.h>
#include <mach/spi.h>
#include <mach/pincontrol.h>

/*************************/
//#define FPGA
/*************************/


/**************************
SPI registerS DEFINITION 
***************************/ 
/**/
#define CTRL0  0x00
#define CTRL0_SET  0X04
#define CTRL0_CLR  0x08

#define CTRL0_LOCK_CS  (1<<27)   /*retain the CS*/
#define CTRL0_CLOCK_CS_SHIFT  27

/**/
#define CTRL1  0x10
#define CTRL1_SET  0x14
#define CTRL1_CLR  0x18

#define CTRL1_CPHA  (1<<10)
#define CTRL1_CPOL  (1<<9)

/**/
#define TIMING  0x20

/**/
#define DATA  0x30

//
#define STATUS  0x40

/**/
#define DEBUG_REG  0x50

/**/
#define XFER  0x60

/*************************************/
#define ALP_SPI_INVALID_DMA_ADDRESS	0xffffffff
#define MAX_SPI_BUSY_WAIT 0x1000000

struct alp9260_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;

	void __iomem		*regs;
	int			 irq;
	int			 len;
	int			 count;

	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;

	struct resource		*ioarea;
	struct spi_master	*master;
	struct device		*dev;	
    struct alp_spi_info  *pdata;

    u8 manual_mapped:1;
    u8 LOCK_CS:1; 
};


static inline struct alp9260_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}


/*
 * Not cosidering the SPI_CS_HIGH for the moment.
 */
static void alp_spi_chipsel(struct spi_device *spi, int value)
{
	struct alp9260_spi *hw = to_hw(spi);
	unsigned int ctrl1val;

	switch (value) {
	case BITBANG_CS_INACTIVE:
		hw->LOCK_CS = 0;
        as3310_writel(CTRL0_LOCK_CS, hw->regs + CTRL0_CLR);
		break;

	case BITBANG_CS_ACTIVE:
		ctrl1val = as3310_readl(hw->regs + CTRL1);

		if (spi->mode & SPI_CPHA)
			ctrl1val |= CTRL1_CPHA;
		else
			ctrl1val &= ~CTRL1_CPHA;

		if (spi->mode & SPI_CPOL)
			ctrl1val |= CTRL1_CPOL;
		else
			ctrl1val &= ~CTRL1_CPOL;

		/* write new configration when SLAVE switched*/
		as3310_writel(ctrl1val, hw->regs + CTRL1);
		hw->LOCK_CS = 1;  //just record it, we will use it when setup dat-transfer.

		break;
	}
}


/* alp_spi_setupxfer - override or restore speed and wordsize if some spi_transfer request
 * 
 * When t is null, restore speed and wordsize.
 */
static int alp_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct alp9260_spi *hw = to_hw(spi);
	unsigned int bpw;
	unsigned int hz;

	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if (bpw != 8) {
		dev_err(&spi->dev, "invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

    if( (t!=0)&&(t->speed_hz!=0) ) {
        //trying to switch the clock, supported only for XPT2046 for the momnet, but not switch back.
#ifdef CONFIG_TOUCHSCREEN_XPT2046
    if(strcmp(spi->modalias, "XPT2046") == 0) {
        /*switch the clock to 2 Mhz*/
        if ( (t->speed_hz == 2000000)&&(as3310_readl(HW_SYSPLLCTRL) == 480)&&(as3310_readl(HW_SPI0CLKDIV+spi->master->bus_num*0x04)==4) ) {
              as3310_writel(0xffff021d, hw->regs+TIMING);  
        }
        else{
            printk("XPT2046 clock failed!\n");
            return -EINVAL;
        }
    }
#endif
    }



	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}


/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA )

static int alp_spi_setup(struct spi_device *spi)
{
	int ret;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->mode & ~MODEBITS) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}

	ret = alp_spi_setupxfer(spi, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "setupxfer returned %d\n", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n",
		__func__, spi->mode, spi->bits_per_word,
		spi->max_speed_hz);

	return 0;
}



/*
 * For DMA, tx_buf/tx_dma have the same relationship as rx_buf/rx_dma:
 *  - The buffer is either valid for CPU access, else NULL
 *  - If the buffer is valid, so is its DMA addresss
 *
 * This driver manages the dma addresss unless message->is_dma_mapped.
 */
static int
alp_spi_dma_map_xfer(struct alp9260_spi *hw, struct spi_transfer *xfer)
{
	struct device	*dev = hw->dev;

	xfer->tx_dma = xfer->rx_dma = ALP_SPI_INVALID_DMA_ADDRESS;
	if (xfer->tx_buf) {
		xfer->tx_dma = dma_map_single(dev,
				(void *) xfer->tx_buf, xfer->len,
				DMA_TO_DEVICE);
		if (dma_mapping_error(dev, xfer->tx_dma))
			return -ENOMEM;
	}
	if (xfer->rx_buf) {
		xfer->rx_dma = dma_map_single(dev,
				xfer->rx_buf, xfer->len,
				DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, xfer->rx_dma)) {
			if (xfer->tx_buf)
				dma_unmap_single(dev,
						xfer->tx_dma, xfer->len,
						DMA_TO_DEVICE);
			return -ENOMEM;
		}
	}
	return 0;
}

static void alp_spi_dma_unmap_xfer(struct alp9260_spi *hw,
				     struct spi_transfer *xfer)
{
	struct device	*dev = hw->dev;

	if (xfer->tx_dma != ALP_SPI_INVALID_DMA_ADDRESS)
		dma_unmap_single(dev, xfer->tx_dma,
				 xfer->len, DMA_TO_DEVICE);
	if (xfer->rx_dma != ALP_SPI_INVALID_DMA_ADDRESS)
		dma_unmap_single(dev, xfer->rx_dma,
				 xfer->len, DMA_FROM_DEVICE);
}


int alp_spi_raw_dma_write(struct spi_device *spi, struct spi_transfer *t)
{
    u32 dmaCtrl0Temreg;
    DmaChain *spi_dma;
    struct alp9260_spi *hw = to_hw(spi);
    int dmaChannelCapacity;
    int channelready = 1;
    int xferedLength, curXferLength;
    int SpiBusyWait;
    int err = 0;

#if defined(DEBUG)
    printk("1TX regs phys:%8x dumps...\n", (u32)hw->regs);

    printk("HW_PRESETCTRL0: %8x, HW_AHBCLKCTRL0: %8x, HW_SPICLKDIV: %8x\n", as3310_readl(HW_PRESETCTRL0), as3310_readl(HW_AHBCLKCTRL0), (hw->pdata->dma_module==DMA_MODULE_0)?as3310_readl(HW_SPI0CLKDIV):as3310_readl(HW_SPI1CLKDIV) );

    printk("ctrl0: %8x, ctrl1: %8x, timing: %8x, status: %8x, xfer: %8x\n", as3310_readl(hw->regs+CTRL0), as3310_readl(hw->regs+CTRL1), as3310_readl(hw->regs+TIMING), as3310_readl(hw->regs+STATUS), as3310_readl(hw->regs+XFER) );
#endif

    spi_dma = DmaRequestChain(hw->dev, hw->pdata->dma_module, hw->pdata->dma_channel, 1, &channelready);
    if( (spi_dma == NULL)||(channelready == 0) )
    {
        if(channelready == 0)
        {
            printk("Std-spi%d W: dma channel busy!\n", hw->pdata->bus_num);
        }
        printk("Std-spi%d W: request dma chain failed!\n", hw->pdata->bus_num);
        return -1;
    }

	dmaCtrl0Temreg = 0x00000000+(0<<0)    //INT_EN, ch0 irq disable
		                       	+(0<<1)      // DST_TR_WIDTH, des transfer width, should set to HSIZE, here is 000, means 8bit
                               	+(0<<4)      // SRC_TR_WIDTH, sor transfer width, should set to HSIZE, here is 000, means 8bit
                               	+(2<<7)      // DINC, des addr increment, des is SPI, so should set to 1x, means no change
                               	+(0<<9)      // SINC, sor addr increment, src is sram, so should set to 00, means to increase 
                               	+(1<<11)     // DEST_MSIZE, des burst length, set to 001 means 4 DST_TR_WIDTH per burst transcation
                               	+(1<<14)     // SRC_MSIZE, sor burst length, set to 001 means 4 SOR_TR_WIDTH per burst transcation
                               	+(1<<20)     // TT_FC,transfer type and flow control,001 means memory to peripheral,dma is flow controller
                               	+(0<<23)     // DMS, des master select, 0 means ahb master 0
                               	+(0<<25)     // SMS, sor master select, 1 means ahb master 1
                               	+(0<<27)     // LLP_DST_EN, des block chaining enable, set to 0 disable it
                              	+(0<<28) ;   // LLP_SOR_EN, sor block chaining enable, set to 0 disable it

    dmaChannelCapacity = (1<<(DMAChannelBlockTsBits(hw->pdata->dma_module, hw->pdata->dma_channel)-1));

    for(xferedLength = 0; xferedLength < t->len;  xferedLength+=curXferLength) {        
        
        curXferLength = ((t->len - xferedLength) >= dmaChannelCapacity)?dmaChannelCapacity:(t->len - xferedLength);

        spi_dma->chain_head[0].SAR = (u32)(t->tx_dma+xferedLength);
        spi_dma->chain_head[0].DAR = (u32)(hw->regs+DATA);				    
        spi_dma->chain_head[0].LLP = (u32)0;
        spi_dma->chain_head[0].CTRL_L = dmaCtrl0Temreg;
		spi_dma->chain_head[0].CTRL_H = curXferLength; 


#if defined(DEBUG)
    printk("1.5TX regs phys:%8x dumps...\n", (u32)hw->regs);

    printk("SAR: %8X, DAR: %8X, CTRL_H: %d\n", (u32)(t->tx_dma+xferedLength), (u32)(hw->regs+DATA), curXferLength );

    printk("DMACFGREG: %8x, HW_DMA0_CHENREG: %8x\n", (hw->pdata->dma_module==DMA_MODULE_0)?as3310_readl(HW_DMA0_DMACFGREG):as3310_readl(HW_DMA1_DMACFGREG), (hw->pdata->dma_module==DMA_MODULE_0)?as3310_readl(HW_DMA0_CHENREG):as3310_readl(HW_DMA1_CHENREG));

    printk("ctrl0: %8x, ctrl1: %8x, timing: %8x, status: %8x, xfer: %8x\n", as3310_readl(hw->regs+CTRL0), as3310_readl(hw->regs+CTRL1), as3310_readl(hw->regs+TIMING), as3310_readl(hw->regs+STATUS), as3310_readl(hw->regs+XFER) );
#endif

        /*tirgger the controller*/
        as3310_writel(curXferLength, hw->regs+XFER);
        as3310_writel(0x30000000|(hw->LOCK_CS<<CTRL0_CLOCK_CS_SHIFT), hw->regs+CTRL0);

        /*Before calling DmaStartRoutine, we must set Channel Configuration Register(CFG) properly.*/
        DmaChannelCFGset(hw->pdata->dma_module, hw->pdata->dma_channel, 0, (0x00+(hw->pdata->tx_handshake_interface<<7)+(hw->pdata->tx_handshake_interface<<11)));

        if( DmaStartRoutine(hw->pdata->dma_module, hw->pdata->dma_channel, spi_dma->chain_phy_addr, (void *)spi_dma->chain_head, 1, DMAtranstype1) )
        {
            printk("Std-spi%d W: DMA routine failed for write\n", hw->pdata->bus_num);
            err = -1;
            break;
        }

        SpiBusyWait = MAX_SPI_BUSY_WAIT;
        while( (as3310_readl(hw->regs+STATUS)&0x01)||(as3310_readl(hw->regs+CTRL0)&(0x20000000)) ) //wait until SPI idle
        {
            if( (SpiBusyWait--) <= 0 )
            {
                printk("Std-spi%d W: time out for current write block\n", hw->pdata->bus_num);
                err = -1;
                break;
            }
        }
    }

    if(err!=0)
    {
        DmaStop(hw->pdata->dma_module, hw->pdata->dma_channel);  //warning: interrupt setting cleared!        
    }

    DmaFreeChain(hw->dev, spi_dma);


#if defined(DEBUG)
    printk("2TX regs phys:%8x dumps...\n", (u32)hw->regs);

    printk("HW_PRESETCTRL0: %8x, HW_AHBCLKCTRL0: %8x, HW_SPICLKDIV: %8x\n", as3310_readl(HW_PRESETCTRL0), as3310_readl(HW_AHBCLKCTRL0), (hw->pdata->dma_module==DMA_MODULE_0)?as3310_readl(HW_SPI0CLKDIV):as3310_readl(HW_SPI1CLKDIV) );
    
    printk("DMACFGREG: %8x, HW_DMA0_CHENREG: %8x\n", (hw->pdata->dma_module==DMA_MODULE_0)?as3310_readl(HW_DMA0_DMACFGREG):as3310_readl(HW_DMA1_DMACFGREG), (hw->pdata->dma_module==DMA_MODULE_0)?as3310_readl(HW_DMA0_CHENREG):as3310_readl(HW_DMA1_CHENREG));

    printk("ctrl0: %8x, ctrl1: %8x, timing: %8x, status: %8x, xfer: %8x\n", as3310_readl(hw->regs+CTRL0), as3310_readl(hw->regs+CTRL1), as3310_readl(hw->regs+TIMING), as3310_readl(hw->regs+STATUS), as3310_readl(hw->regs+XFER) );
    
    printk("dma_module: %8x, dma_channel: %8x, dmaChannelCapacity: %8x\n", hw->pdata->dma_module, hw->pdata->dma_channel, dmaChannelCapacity);
#endif

    return err;
}

int alp_spi_raw_dma_read(struct spi_device *spi, struct spi_transfer *t)
{

    u32 dmaCtrl0Temreg;
    DmaChain *spi_dma;
    struct alp9260_spi *hw = to_hw(spi);
    int dmaChannelCapacity;
    int channelready = 1;
    int xferedLength, curXferLength;
    int SpiBusyWait;
    int err = 0;


#if defined(DEBUG)
    printk("1RX regs phys:%8x dumps...\n", (u32)hw->regs);

    printk("HW_PRESETCTRL0: %8x, HW_AHBCLKCTRL0: %8x, HW_SPICLKDIV: %8x\n", as3310_readl(HW_PRESETCTRL0), as3310_readl(HW_AHBCLKCTRL0), (hw->pdata->dma_module==DMA_MODULE_0)?as3310_readl(HW_SPI0CLKDIV):as3310_readl(HW_SPI1CLKDIV) );

    printk("ctrl0: %8x, ctrl1: %8x, timing: %8x, status: %8x, xfer: %8x\n", as3310_readl(hw->regs+CTRL0), as3310_readl(hw->regs+CTRL1), as3310_readl(hw->regs+TIMING), as3310_readl(hw->regs+STATUS), as3310_readl(hw->regs+XFER) );
#endif

    if( (t->len&0x03)||(t->rx_dma&0x03) ) {
        printk("t->len: %d; t->rx_dma: %d\n", t->len, t->rx_dma);
        dev_err(&spi->dev, "%s not 4 bytes aligned.\n", __func__);
        dump_stack();
        return -EINVAL;
    }

    spi_dma = DmaRequestChain(hw->dev, hw->pdata->dma_module, hw->pdata->dma_channel, 1, &channelready);
    if( (spi_dma == NULL)||(channelready == 0) )
    {
        if(channelready == 0)
        {
            printk("Std-spi%d R: dma channel busy!\n", hw->pdata->bus_num);
        }
        printk("Std-spi%d R: request dma chain failed!\n", hw->pdata->bus_num);
        return -1;
    }

    dmaCtrl0Temreg = 0x00000000+(0<<0)       //INT_EN, ch0 irq disable
            				    +(2<<1)      // DST_TR_WIDTH, des transfer width, should set to HSIZE, here is 010, means 32bit
                                +(2<<4)      // SRC_TR_WIDTH, sor transfer width, should set to HSIZE, here is 010, means 32bit
            				    +(0<<7)      // DINC, des addr increment, des is sram, so should set to 00, means to increase
                                +(2<<9)      // SINC, sor addr increment, src is SPI, so should set to 1x, means no change 
            				    +(0<<11)     // DEST_MSIZE, des burst length, set to 000 means 1 DST_TR_WIDTH per burst transcation
            				    +(0<<14)     // SRC_MSIZE, sor burst length, set to 000 means 1 SRC_TR_WIDTH per burst transcation
                                +(2<<20)     // TT_FC,transfer type and flow control,010 means peripheral to memory,dma is flow controller
                                +(0<<23)     // DMS, des master select, 0 means ahb master 0
                                +(0<<25)     // SMS, sor master select, 1 means ahb master 1
                                +(0<<27)     // LLP_DST_EN, des block chaining enable, set to 0 disable it
                                +(0<<28);     // LLP_SOR_EN, sor block chaining enable, set to 0 disable it

    dmaChannelCapacity = (1<<(DMAChannelBlockTsBits(hw->pdata->dma_module, hw->pdata->dma_channel)-1));

    for(xferedLength = 0; xferedLength < t->len;  xferedLength+=curXferLength) {        
        
        curXferLength = ((t->len - xferedLength) >= dmaChannelCapacity)?dmaChannelCapacity:(t->len - xferedLength);

        spi_dma->chain_head[0].SAR = (u32)(hw->regs+DATA);
        spi_dma->chain_head[0].DAR = (u32)(t->rx_dma+xferedLength);			    
        spi_dma->chain_head[0].LLP = (u32)0;
        spi_dma->chain_head[0].CTRL_L = dmaCtrl0Temreg;
		spi_dma->chain_head[0].CTRL_H = curXferLength/4; 

        /*tirgger the controller*/
        as3310_writel(curXferLength, hw->regs+XFER);
        as3310_writel(0x34000000|(hw->LOCK_CS<<CTRL0_CLOCK_CS_SHIFT), hw->regs+CTRL0);

        /*Before calling DmaStartRoutine, we must set Channel Configuration Register(CFG) properly.*/
        DmaChannelCFGset(hw->pdata->dma_module, hw->pdata->dma_channel, 0, (0x00+(hw->pdata->rx_handshake_interface<<7)+(hw->pdata->rx_handshake_interface<<11)));

        if( DmaStartRoutine(hw->pdata->dma_module, hw->pdata->dma_channel, spi_dma->chain_phy_addr, (void *)spi_dma->chain_head, 1, DMAtranstype1) )
        {
            printk("Std-spi%d R: DMA routine failed for read\n", hw->pdata->bus_num);
            err = -1;
            break;
        }

        SpiBusyWait = MAX_SPI_BUSY_WAIT;
        while( as3310_readl(hw->regs+STATUS)&0x01 ) //wait until SPI idle
        {
            if( (SpiBusyWait--) <= 0 )
            {
                printk("Std-spi%d R: time out for current read block\n", hw->pdata->bus_num);
                err = -1;
                break;
            }
        }
    }

    if(err!=0)
    {
        DmaStop(hw->pdata->dma_module, hw->pdata->dma_channel);  //warning: interrupt setting cleared!        
    }

    DmaFreeChain(hw->dev, spi_dma);


#if defined(DEBUG)
    printk("2RX regs phys:%8x dumps...\n", (u32)hw->regs);

    printk("HW_PRESETCTRL0: %8x, HW_AHBCLKCTRL0: %8x, HW_SPICLKDIV: %8x\n", as3310_readl(HW_PRESETCTRL0), as3310_readl(HW_AHBCLKCTRL0), (hw->pdata->dma_module==DMA_MODULE_0)?as3310_readl(HW_SPI0CLKDIV):as3310_readl(HW_SPI1CLKDIV) );

    printk("ctrl0: %8x, ctrl1: %8x, timing: %8x, status: %8x, xfer: %8x\n", as3310_readl(hw->regs+CTRL0), as3310_readl(hw->regs+CTRL1), as3310_readl(hw->regs+TIMING), as3310_readl(hw->regs+STATUS), as3310_readl(hw->regs+XFER) );
#endif

    return err;
}


static int alp_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
    int ret = t->len;
	struct alp9260_spi *hw = to_hw(spi);
    void *buf_temp;
    unsigned len_temp;
    dma_addr_t tx_dma_temp, rx_dam_temp;


#if defined(DEBUG)
{
    printk("\n");
}
#endif
 
	dev_dbg(&spi->dev, "txrx: tx %p, rx %p, len %d\n",
		t->tx_buf, t->rx_buf, t->len);

    /*first check the spi_write_then_read, it suits for PIO polling, may be implemented later.*/
    if( ((t->rx_len!=0)||(t->tx_len!=0))&&(t->tx_buf==t->rx_buf) ) {

         if(t->tx_len) {
            buf_temp = t->rx_buf;
            t->rx_buf = 0;
            len_temp = t->len;
            t->len = t->tx_len;
            tx_dma_temp = t->tx_dma;
            rx_dam_temp = t->rx_dma;

            hw->manual_mapped = 0;
            if((t->tx_dma == 0)) {            
           		if (alp_spi_dma_map_xfer(hw, t) < 0)
        				return -ENOMEM;
                hw->manual_mapped = 1;
            }
            if(alp_spi_raw_dma_write(spi, t)!=0)
            {
                ret = 0;
            }

            if(hw->manual_mapped == 1) {
                alp_spi_dma_unmap_xfer(hw, t);
            }
            
            t->rx_buf = buf_temp;
            t->len = len_temp;
            t->tx_dma = tx_dma_temp;
            t->rx_dma = rx_dam_temp;

             if(ret==0) {
                return ret;
            }
         }

         if(t->rx_len==0) {
             return t->len;
         }else{             
             buf_temp = (void *)t->tx_buf;
             t->tx_buf = 0;
             len_temp = t->len;
             t->len = t->rx_len;
             tx_dma_temp = t->tx_dma;
             rx_dam_temp = t->rx_dma;           

             hw->manual_mapped = 0;
             if((t->rx_dma == 0)) {            
                 if (alp_spi_dma_map_xfer(hw, t) < 0)
                         return -ENOMEM;
                 hw->manual_mapped = 1;
             }
             if(alp_spi_raw_dma_read(spi, t)!=0)
             {
                 ret = 0;
             }

             if(hw->manual_mapped == 1) {
                 alp_spi_dma_unmap_xfer(hw, t);
             }

             t->tx_buf = buf_temp;
             t->len = len_temp;
             t->tx_dma = tx_dma_temp;
             t->rx_dma = rx_dam_temp;

             if(ret==0) {
                 return ret;
             }else{
                 return t->len;
             }    
         }

    }else{


         /*for the moment, we only support DMA HALF_DUPLEX, LOGIC TEAM recommended.*/
         if( (t->tx_buf!=0)&&(t->rx_buf!=0) ) {
             return -EOPNOTSUPP;
         }


#if defined(DEBUG)
if(t->tx_buf!=0) {
    dev_dbg(&spi->dev, "tx-buffer: %2x, %2x, %2x, %2x\n", ((char *)t->tx_buf)[0], ((char *)t->tx_buf)[1], ((char *)t->tx_buf)[2], ((char *)t->tx_buf)[3]);
}else{
    int i;
    for(i=0; i< t->len; i++) {
        ((char *)t->rx_buf)[i] = 0x55;
    }
    dev_dbg(&spi->dev, "rx-buffer before: %8x %8x %8x %8x %8x %8x %8x %8x\n", ((char *)t->rx_buf)[0], ((char *)t->rx_buf)[1], ((char *)t->rx_buf)[2], ((char *)t->rx_buf)[3], ((char *)t->rx_buf)[4], ((char *)t->rx_buf)[5], ((char *)t->rx_buf)[6], ((char *)t->rx_buf)[7]);            
}
#endif


        	/*
       	 * DMA map early, for performance (empties dcache ASAP) and
       	 * better fault reporting.  This is a DMA-only driver.
       	 *
       	 * NOTE that if dma_unmap_single() ever starts to do work on
       	 * platforms supported by this driver, we would need to clean
       	 * up mappings for previously-mapped transfers.
       	 */
         if( ( (t->tx_dma == 0)&&(t->rx_dma!=0) )||( (t->tx_dma != 0)&&(t->rx_dma==0) ) ) {
             /*unlikely, return error.*/
             return -EINVAL;
         }
         hw->manual_mapped = 0;
         if( (t->tx_dma == 0)&&(t->rx_dma == 0) ) {
        		if (alp_spi_dma_map_xfer(hw, t) < 0)
     				return -ENOMEM;
             hw->manual_mapped = 1;
         }

#if defined(DEBUG)
        if(t->tx_buf!=0){
             dev_dbg(&spi->dev, "tx_buf:%p, tx_dma:%p\n", t->tx_buf, (void *)t->tx_dma);
        }else{             
             dev_dbg(&spi->dev, "rx_buf:%p, rx_dma:%p\n", t->rx_buf, (void *)t->rx_dma);
        }
#endif

         /*RAW dma function, due to we have no definite idea about which dma channel can be selected, how many bytes within this spi_transfer, so we do awkward*/
         /*Maybe, we should use DMA chains later.*/
         if(t->tx_buf!=0) {
             if(alp_spi_raw_dma_write(spi, t)!=0)
                 ret = 0;
         }else{
             if(alp_spi_raw_dma_read(spi, t)!=0)
                 ret = 0;
         }

         if(hw->manual_mapped == 1) {
             alp_spi_dma_unmap_xfer(hw, t);
         }
   
 #if defined(DEBUG)
 if(t->rx_buf!=0) {
     dev_dbg(&spi->dev, "rx-buffer after: %8x %8x %8x %8x %8x %8x %8x %8x\n", ((char *)t->rx_buf)[0], ((char *)t->rx_buf)[1], ((char *)t->rx_buf)[2], ((char *)t->rx_buf)[3], ((char *)t->rx_buf)[4], ((char *)t->rx_buf)[5], ((char *)t->rx_buf)[6], ((char *)t->rx_buf)[7]);            
 }         
 #endif
               
         return ret;
    }
}

void alp_spi_pininit(struct alp9260_spi *hw)
{
    if(hw->master->bus_num == 0) {
        set_pin_mux(6, 0, 4);
        set_pin_mux(6, 1, 4);
        set_pin_mux(6, 2, 4);
        set_pin_mux(6, 3, 4);
    }else{
        set_pin_mux(13, 4, 4);
        set_pin_mux(13, 5, 4);
        set_pin_mux(13, 6, 4);
        set_pin_mux(13, 7, 4);
    }
}

static void alp_spi_initialsetup(struct alp9260_spi *hw)
{
    int sourceclock;
    int clock_rate;

    if(hw->master->bus_num == 0) {
       as3310_writel(1<<30, HW_PRESETCTRL0+4);
       as3310_writel(1<<30, HW_AHBCLKCTRL0+4);
    }else{        
       as3310_writel(1<<31, HW_PRESETCTRL0+4);
       as3310_writel(1<<31, HW_AHBCLKCTRL0+4);
    }

    DmaInit(hw->pdata->dma_module, hw->pdata->dma_channel);

    alp_spi_pininit(hw);

        /*timing setting*/
    if( as3310_readl(HW_MAINCLKSEL)&0x01 ) {  
           
        #ifdef FPGA
          sourceclock = 50;
        #else
          sourceclock = as3310_readl(HW_SYSPLLCTRL);
        #endif

        dev_info(hw->dev, "*******SPI timing from PLL: %dMhz\n", sourceclock);

    }else{

        dev_info(hw->dev, "*******SPI timing from OSC\n");

        sourceclock = 12;
    }

    as3310_writel(0x00002038, hw->regs+CTRL1);

    /*Timing setting for aroud 33Mhz*/
    if(  sourceclock==12 ) {
        as3310_writel(1, HW_SPI0CLKDIV+hw->master->bus_num*0x04);
        as3310_writel(0xffff0200, hw->regs+TIMING);
    }else{
        if(sourceclock<132) {
             as3310_writel(2, HW_SPI0CLKDIV+hw->master->bus_num*0x04);
             as3310_writel(0xffff0200, hw->regs+TIMING);
        }else{
             clock_rate = sourceclock/8/33;
             as3310_writel(4, HW_SPI0CLKDIV+hw->master->bus_num*0x04);
             as3310_writel(0xffff0200|clock_rate, hw->regs+TIMING);
        }
    }

    //printk("spi div: %8x, timing: %8x\n", as3310_readl(HW_SPI0CLKDIV+hw->master->bus_num*0x04), as3310_readl(hw->regs+TIMING));

     as3310_writel(0x00, hw->regs+CTRL1_SET);
}


static int __init alp_spi_probe(struct platform_device *pdev)
{
	struct alp_spi_info *pdata;
	struct alp9260_spi *hw;
	struct spi_master *master;
	struct resource *res;
	int err = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct alp9260_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct alp9260_spi));

	hw->master = spi_master_get(master);
	hw->pdata = pdata = pdev->dev.platform_data;
	hw->dev = &pdev->dev;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_no_pdata;
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	/* setup the master state. */

	master->num_chipselect = hw->pdata->num_cs;
	master->bus_num = pdata->bus_num;


	/* setup the state for the bitbang driver */

	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = alp_spi_setupxfer;
	hw->bitbang.chipselect     = alp_spi_chipsel;
	hw->bitbang.txrx_bufs      = alp_spi_txrx;
	hw->bitbang.master->setup  = alp_spi_setup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, (res->end - res->start)+1,
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	hw->regs = (void *)res->start;
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}


	alp_spi_initialsetup(hw);


	/* register our spi controller */
	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

 err_register:  
	iounmap(hw->regs);

 err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

 err_no_iores:
 err_no_pdata:
	spi_master_put(hw->master);;

 err_nomem:
	return err;
}


static int __exit alp_spi_remove(struct platform_device *dev)
{
	struct alp9260_spi *hw = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	iounmap(hw->regs);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}



#ifdef CONFIG_PM

static int alp_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int alp_spi_resume(struct platform_device *pdev)
{
	struct alp9260_spi *hw = platform_get_drvdata(pdev);

	alp_spi_initialsetup(hw);
	return 0;
}

#else
#define alp_spi_suspend NULL
#define alp_spi_resume  NULL
#endif

MODULE_ALIAS("platform:alp9260-spi");
static struct platform_driver alp_spi_driver = {
	.remove		= __exit_p(alp_spi_remove),
	.suspend	= alp_spi_suspend,
	.resume		= alp_spi_resume,
	.driver		= {
		.name	= "as9260_std_spi",
		.owner	= THIS_MODULE,
	},
};

static int __init alp_spi_init(void)
{
        return platform_driver_probe(&alp_spi_driver, alp_spi_probe);
}

static void __exit alp_spi_exit(void)
{
        platform_driver_unregister(&alp_spi_driver);
}

module_init(alp_spi_init);
module_exit(alp_spi_exit);

MODULE_DESCRIPTION("ALP9260 SPI Driver");
MODULE_LICENSE("GPL");
