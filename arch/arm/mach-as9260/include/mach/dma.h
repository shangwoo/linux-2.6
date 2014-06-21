/*
 *  linux/arch/arm/mach-as9260/include/mach/dma.h
 *
 *  Copyright (C) 2012 He Wei
 *  Copyright (C) 2005-2012 Alpscale  
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 *  Alpscale low level driver header file for SYNOPSYS DesignWare AHB Central Direct Memory Access (DMA) Controller.
 *  Changelog:
 *
 */


#ifndef __DMA_H__
#define __DMA_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>


#if 1 //We will delete the following forever when new one is finished for not interfering with other modules using 1826 DMA for the moment.

//#define DMA_CHANNEL_MIN 0
//#define DMA_CHANNEL_MAX 7
#define DMA_LCD_CHANNEL_MAX 5 // total 6 channels

#define ASM9260_DACOUT_CH    1
#define ASM9260_ADCIN_CH     0
#define ASM9260_LCDIF_CH     4
/* dma channel devices */
#define ASM9260_HWECC_CH     0  /* Channel 0 , Hw ECC */
#define ASM9260_DMA_LCD_CH   0  /* ASM9260C - 1, ASM9260D - 0 */
#define ASM9260_DMA_LCD_Y_CH 0
#define ASM9260_DMA_LCD_U_CH 2
#define ASM9260_DMA_LCD_V_CH 1
#define ASAP1820_DMA_LCDIF_Y_CH 3
#define ASAP1820_DMA_LCDIF_U_CH 4
#define ASAP1820_DMA_LCDIF_V_CH 5

/* define error codes */
#define ASM9260_DMA_CHANNEL_OK       0
#define ASM9260_DMA_CHANNEL_BUSY     1
#define ASM9260_DMA_CHANNEL_INVALID  2

/* define chain status codes */
#define ASM9260_DMA_STAT_READY 0
#define ASM9260_DMA_STAT_BUSY  -1

struct asm9260_dma_chain {
    int channel_num;
    int status;
    int pkg_num;
    char * own_dev; /* a char string of the device name */
    dma_addr_t chain_phy_addr; /* physical addr of 1st dma package */
    struct asm9260_dma_pkg_s * chain_head; /* point to the 1st dma package */
};


 /*
init_asm9260_dma_chain
inputs:
struct device * dev the device which use this dma chain
int pkg_num,        number of DMA package in this DMA chain
int channel_num,    channel number
*/
struct asm9260_dma_chain * request_asm9260_dma_chain(struct device * dev,int pkg_num,int channel_num);

 /*
free_asm9260_dma_chain
inputs:
struct device * dev the device which use this dma chain
dma_chain,          dma_chain struct which need free
*/
void free_asm9260_dma_chain(struct device * dev,struct asm9260_dma_chain * dma_chain);

 /*
Start APBX DMA Chain
inputs:
ulong pkg_addr, Physical address of DMA chain entry
int pkg_num,    number of DMA package in this DMA chain
int ch_num,     channel number
*/
int dma_start_apbx(ulong pkg_addr,int pkg_num,int ch_num);

 /*
Start APBH DMA Chain
inputs:
ulong pkg_addr, Physical address of DMA chain entry
int pkg_num,    number of DMA package in this DMA chain
int ch_num,     channel number
*/
int dma_start_apbh(ulong pkg_addr,int pkg_num,int ch_num);
int dma_start_apbh_lcd(ulong pkg_addr,int pkg_num,int ch_num);
int dma_apbh_lcd_rst_ch(int ch_num);
int dma_start_apbh_force(ulong pkg_addr,int pkg_num,int ch_num);
int dma_apbh_reset_ch(int ch_num);
int asm9260_dma_init(void);
int dma_apbh_reset_ch(int ch_num);
int dma_clear_interrupt(int ch_num);
int dma_enable_interrupt(int ch_num);



#define is_apbh_lcd_complete(ch) \
    ((as3310_readl(HW_APBH_LCD_CH0_SEMA + ((ch)*0x70))&0x00ff0000)==0)

#define is_apbh_complete(ch) \
    ((as3310_readl(HW_APBH_CH0_SEMA + ((ch)*0x70))&0x00ff0000)==0)

#define is_apbx_complete(ch)\
    ((as3310_readl(HW_APBX_CH0_SEMA + ((ch)*0x70))&0x00ff0000)==0)    

#define IS_DMA_LCD_COMPLETE() ((as3310_readl(HW_APBH_LCD_CH0_SEMA)&0x00ff0000)==0)
#define IS_DMA_APBH_CH0_COMPLETE() ((as3310_readl(HW_APBH_CH0_SEMA)&0x00ff0000)==0)
#define IS_DMA_APBH_CH1_COMPLETE() ((as3310_readl(HW_APBH_CH1_SEMA)&0x00ff0000)==0)
#define IS_DMA_APBH_CH2_COMPLETE() ((as3310_readl(HW_APBH_CH2_SEMA)&0x00ff0000)==0)
#define IS_DMA_APBH_CH3_COMPLETE() ((as3310_readl(HW_APBH_CH3_SEMA)&0x00ff0000)==0)
#define IS_DMA_APBH_CH4_COMPLETE() ((as3310_readl(HW_APBH_CH4_SEMA)&0x00ff0000)==0)

#endif

#if 1  //new LOW-LEVEL DMA routines for 9260

#define DMA_MODULE_MIN			(0)
#define DMA_MODULE_MAX			(1)

#define DMA_CHANNEL_MIN			(0)
#define DMA_CHANNEL_MAX			(7)

/*Bit domain Shift for CTRL_L*/
#define DMA_CTRL_INT_EN			(0) 
#define DMA_CTRL_DST_TR_WIDTH	(1) 		//see DMA Transfer Width
#define DMA_CTRL_SRC_TR_WIDTH 	(4) 		//see DMA Transfer Width
#define DMA_CTRL_DINC				(7)		//see DMA Address Changing
#define DMA_CTRL_SINC				(9)		//see DMA Address Changing
#define DMA_DEST_MSIZE				(11)		//see Burst Transaction Length
#define DMA_SRC_MSIZE				(14)		//see Burst Transaction Length
#define DMA_SRC_GATHER_EN 		(17)
#define DMA_DST_GATHER_EN 		(18)
#define DMA_TTC_FC					(20)		//see Transfer Type and Flow Control
#define DMA_DMS						(23)
#define DMA_SMS						(25)
#define DMA_LLP_DST_EN				(27)
#define DMA_LLP_SRC_EN				(28)

/*Bit domain Shift for CTRL_H*/
#define DMA_BLOCK_TS				(32)
#define DMA_DONE					(44)


/*DMA Transfer Width*/
#define DMA_WIDTH_8 				(0x00)
#define DMA_WIDTH_16 				(0x01)
#define DMA_WIDTH_32				(0x02)
#define DMA_WIDTH_64 				(0x03)
#define DMA_WIDTH_128 				(0x04)
#define DMA_WIDTH_256 				(0x05)

/*DMA Address Changing*/
#define DMA_ADDRESS_INCREMENT 	(0x00)
#define DMA_ADDRESS_DECREMENT 	(0x01)
#define DMA_ADDRESS_NO_CHANGE 	(0x02)

/*Burst Transaction Length*/
#define DMA_BURST_1_ITEM			(0x00)
#define DMA_BURST_4_ITEM			(0x01)
#define DMA_BURST_8_ITEM			(0x02)
#define DMA_BURST_16_ITEM			(0x03)
#define DMA_BURST_32_ITEM			(0x04)
#define DMA_BURST_64_ITEM			(0x05)
#define DMA_BURST_128_ITEM		(0x06)
#define DMA_BURST_256_ITEM		(0x07)

/*Transfer Type and Flow Control*/
#define DMA_MEM_TO_MEM_DMAC			(0x00)
#define DMA_MEM_TO_PER_DMAC			(0x01)
#define DMA_PER_TO_MEM_DMAC			(0x02)
#define DMA_PER_TO_PER_DMAC			(0x03)
#define DMA_PER_TO_MEM_PERI			(0x04)
#define DMA_PER_TO_PER_SOU_PER		(0x05)
#define DMA_MEM_TO_PER_PER			(0x06)
#define DMA_PER_TO_PER_DES_PER		(0x07)


/**
  *  _DmaPkg - For DMAmethod 1-5, it's used as a struct to retrieve the information to 
  *  start the DMA directly. For DMAmethod 6-10, it's used as node struct to set up link  
  *  list of DMA chains.   
  */
typedef struct _DmaPkg
{
	unsigned int SAR;
	unsigned int DAR;
	unsigned int LLP;
	unsigned int CTRL_L;
	unsigned int CTRL_H;
    unsigned int reserved[3];
}DmaPkg;

#define DMAModuleNum 2
typedef enum {DMA_MODULE_0 = 0, DMA_MODULE_1} DMAmodule;

#define DMAChannelNum 8
typedef enum {
		DMA_CHANNEL_0 = 0, 
		DMA_CHANNEL_1, 
		DMA_CHANNEL_2,
		DMA_CHANNEL_3,
		DMA_CHANNEL_4,
		DMA_CHANNEL_5,
		DMA_CHANNEL_6,
		DMA_CHANNEL_7	
}DMAchannel;

#define DMAMETHODNUM 10
typedef enum {
		DMAtranstype1 = 1,
		DMAtranstype2,
		DMAtranstype3,
		DMAtranstype4,
		DMAtranstype5,
		DMAtranstype6,
		DMAtranstype7,
		DMAtranstype8,
		DMAtranstype9,
		DMAtranstype10
}DMAmethod;


typedef enum {
		CHANNEL_IDLE =0,
		CHANNEL_BUSY		
}ChannelState;		

typedef enum {
		INT_TFR = 0,
		INT_BLOCK,
		INT_SRCTRAN,
		INT_DSTTRAN,
		INT_ERR
}DMAinterruptType;	
	

typedef struct _dma_chain
{
	DMAmodule dmamodule;
	DMAchannel dmachannel;
	int pkg_num;    
    char * own_dev; /* a char string of the device name */
	dma_addr_t chain_phy_addr;		/* physical addr of 1st dma package */
	DmaPkg* chain_head;			/* point to the 1st dma package, especially virtual address when operation system implemented */
}DmaChain;


int DMAChannelBlockTsBits(DMAmodule module, DMAchannel channel);
DmaChain* DmaRequestChain(struct device * dev, DMAmodule module, DMAchannel channel, int pkg_num, int* channel_ready);
void DmaFreeChain(struct device * dev, DmaChain* dma_chain);
ChannelState DmaIsBusy(DMAmodule module, DMAchannel channel);
int DmaInit(DMAmodule module, DMAchannel channel);
void DmaChannelInterrupt(DMAmodule module, DMAchannel channel, DMAinterruptType inttype, int funswitch );
int DmaChannelInterruptStatusBit(DMAmodule module, DMAchannel channel, DMAinterruptType inttype, int statustype );
void DmaChannelClearInterruptStatus(DMAmodule module, DMAchannel channel, DMAinterruptType inttype);
int DmaStop(DMAmodule module, DMAchannel channel);
void DmaChannelCFGset(DMAmodule module, DMAchannel channel, u32 lowregval, u32 highregval);
int DmaStartRoutine(DMAmodule module, DMAchannel channel, dma_addr_t pAddr, void* vAddr, int bWaitSemaAfter, DMAmethod dmamethod);
#endif

#endif //__DMA_H__
