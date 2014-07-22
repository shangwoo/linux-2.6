/*
 *  linux/arch/arm/mach-as9260/dma.c
 *
 *  Copyright (C) 2005-2014 Alpscale
 *
 *
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/serial_8250.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/dma.h>
#include <asm/hardware/iomd.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>


#if 1 //We will delete the following forever when new one is finished for not interfering with other modules using 1826 DMA for the moment.

spinlock_t asm9260_dma_apbh_lock[DMA_CHANNEL_MAX];
spinlock_t asm9260_dma_apbh_lcd_lock[DMA_LCD_CHANNEL_MAX];
spinlock_t asm9260_dma_apbx_lock[DMA_CHANNEL_MAX];


 /*
Start APBX DMA Chain
inputs:
ulong pkg_addr, Physical address of DMA chain entry
int pkg_num,    number of DMA package in this DMA chain
int ch_num,     channel number
*/
int dma_start_apbx(ulong pkg_addr,int pkg_num,int ch_num){
    int ret;

     if ((ch_num < DMA_CHANNEL_MIN)||(ch_num > DMA_CHANNEL_MAX))  {
         printk("APBX DMA Channel Error!\n");
         return -ASM9260_DMA_CHANNEL_INVALID;
     }
     else {
         spin_lock(&asm9260_dma_apbx_lock[ch_num]);
         if (is_apbx_complete(ch_num)) {
             ulong reg_next_addr = HW_APBX_CH0_NXTCMDAR + 
                 (ch_num)*(HW_APBX_CH1_NXTCMDAR - HW_APBX_CH0_NXTCMDAR);

             as3310_writel((ulong)pkg_addr ,reg_next_addr );
             as3310_writel(pkg_num ,reg_next_addr+(HW_APBX_CH0_SEMA-HW_APBX_CH0_NXTCMDAR));
             ret = ASM9260_DMA_CHANNEL_OK;
         }
         else{
        //     printk("APBX DMA Ch(%d) Busy!\n",ch_num);
             ret = -ASM9260_DMA_CHANNEL_BUSY;
         }
         spin_unlock(&asm9260_dma_apbx_lock[ch_num]);
         return ret;
     }
 }


 /*
Start APBH DMA Chain
inputs:
ulong pkg_addr, Physical address of DMA chain entry
int pkg_num,    number of DMA package in this DMA chain
int ch_num,     channel number
*/
int dma_start_apbh(ulong pkg_addr,int pkg_num,int ch_num){
    int ret;

     if ((ch_num < DMA_CHANNEL_MIN)||(ch_num > DMA_CHANNEL_MAX))  {
         printk("APBH DMA Channel Error!\n");
         return -ASM9260_DMA_CHANNEL_INVALID;
     }
     else {
         spin_lock(&asm9260_dma_apbh_lock[ch_num]);
         if (is_apbh_complete(ch_num)) {
          //   printk("APBH DMA Channel (%d) complete!\n",ch_num);
             ulong reg_next_addr = HW_APBH_CH0_NXTCMDAR + (ch_num*0x70);

             as3310_writel((ulong)pkg_addr ,reg_next_addr );
             as3310_writel(pkg_num ,reg_next_addr+ 0x30);
             ret = ASM9260_DMA_CHANNEL_OK;
         }
         else{
         //    printk("APBH DMA Ch(%d) Busy!\n",ch_num);
             ret = -ASM9260_DMA_CHANNEL_BUSY;
         }
         spin_unlock(&asm9260_dma_apbh_lock[ch_num]);
         return ret;
     }
 }

 /*
Start APBH LCD DMA Chain
inputs:
ulong pkg_addr, Physical address of DMA chain entry
int pkg_num,    number of DMA package in this DMA chain
int ch_num,     channel number
*/
int dma_start_apbh_lcd(ulong pkg_addr,int pkg_num,int ch_num){
    int ret;

     if ((ch_num < DMA_CHANNEL_MIN)||(ch_num > DMA_LCD_CHANNEL_MAX))  {
         printk("APBH LCD DMA Channel Error!\n");
         return -ASM9260_DMA_CHANNEL_INVALID;
     }
     else {
         spin_lock(&asm9260_dma_apbh_lcd_lock[ch_num]);
         if (is_apbh_lcd_complete(ch_num)) {
          //   printk("APBH DMA Channel (%d) complete!\n",ch_num);
             ulong reg_next_addr = HW_APBH_LCD_CH0_NXTCMDAR + (ch_num*0x70);

             as3310_writel((ulong)pkg_addr ,reg_next_addr );
             as3310_writel(pkg_num ,reg_next_addr+ 0x30);
             ret = ASM9260_DMA_CHANNEL_OK;
         }
         else{
         //    printk("APBH DMA Ch(%d) Busy!\n",ch_num);
             ret = -ASM9260_DMA_CHANNEL_BUSY;
         }
         spin_unlock(&asm9260_dma_apbh_lcd_lock[ch_num]);
         return ret;
     }
 }

 /*
Start APBH DMA Chain by force
inputs:
ulong pkg_addr, Physical address of DMA chain entry
int pkg_num,    number of DMA package in this DMA chain
int ch_num,     channel number
*/
int dma_start_apbh_force(ulong pkg_addr,int pkg_num,int ch_num){
    ulong reg_next_addr;

    spin_lock(&asm9260_dma_apbh_lock[ch_num]);
    reg_next_addr = HW_APBH_CH0_NXTCMDAR + (ch_num*0x70);

    as3310_writel((ulong)pkg_addr ,reg_next_addr );
    as3310_writel(pkg_num ,reg_next_addr+ 0x30);
    spin_unlock(&asm9260_dma_apbh_lock[ch_num]);
    return ASM9260_DMA_CHANNEL_OK;
 }

int dma_apbh_lcd_rst_ch (int ch_num)
{
    ulong reg_next_addr = HW_APBH_LCD_CH0_NXTCMDAR +
             (ch_num)*(HW_APBH_LCD_CH1_NXTCMDAR - HW_APBH_LCD_CH0_NXTCMDAR);
    as3310_writel(0x0,reg_next_addr+(HW_APBH_LCD_CH0_SEMA-HW_APBH_LCD_CH0_NXTCMDAR));
    as3310_writel((1<<(ch_num+16)),HW_APBH_LCD_CTRL0_SET);

   return ASM9260_DMA_CHANNEL_OK;

}


int dma_apbh_reset_ch(int ch_num)
{
     ulong reg_next_addr = HW_APBH_CH0_NXTCMDAR +
              (ch_num)*(HW_APBH_CH1_NXTCMDAR - HW_APBH_CH0_NXTCMDAR);
     as3310_writel(0x0,reg_next_addr+(HW_APBH_CH0_SEMA-HW_APBH_CH0_NXTCMDAR));
     as3310_writel((1<<(ch_num+16)),HW_APBH_CTRL0_SET);
  
    return ASM9260_DMA_CHANNEL_OK;
}

    
int asm9260_dma_init(void){
    int i;
    as3310_writel(0x40000000,HW_APBH_CTRL0_CLR);//clear the clk gate
    as3310_writel(0x80000000,HW_APBH_CTRL0_CLR);//clear the clk gate     
    as3310_writel(0x40000000,HW_APBH_LCD_CTRL0_CLR);//clear the clk gate
    as3310_writel(0x80000000,HW_APBH_LCD_CTRL0_CLR);//clear the clk gate     
    as3310_writel(0x40000000,HW_APBX_CTRL0_CLR);//clear the clk gate
    as3310_writel(0x80000000,HW_APBX_CTRL0_CLR);//clear the clk gate  
    as3310_writel(0x00000000,HW_APBH_CTRL1);  //dma ctrl1 irq=1;irq_en=1;
    // init spinlocks
    for (i = DMA_CHANNEL_MIN; i <=DMA_CHANNEL_MAX; i++) {
        spin_lock_init(&asm9260_dma_apbh_lock[i]);
        spin_lock_init(&asm9260_dma_apbx_lock[i]);
        /* already unlock?? */
//        spin_unlock(&asm9260_dma_apbh_lock[i]);
//        spin_unlock(&asm9260_dma_apbx_lock[i]);
    }

    for (i = DMA_CHANNEL_MIN; i <=DMA_LCD_CHANNEL_MAX; i++) {
        spin_lock_init(&asm9260_dma_apbh_lcd_lock[i]);
//        spin_unlock(&asm9260_dma_apbh_lcd_lock[i]);
    }

return ASM9260_DMA_CHANNEL_OK;
}


 /*
init_asm9260_dma_chain
inputs:
struct device * dev the device which use this dma chain
int pkg_num,        number of DMA package in this DMA chain
int channel_num,    channel number
*/
struct asm9260_dma_chain * request_asm9260_dma_chain(struct device * dev,int pkg_num,int channel_num){
    struct asm9260_dma_chain * dma_chain;
    struct asm9260_dma_pkg_s * chain_ptr;
    int i;

    if ((channel_num < DMA_CHANNEL_MIN)||(channel_num > DMA_CHANNEL_MAX))  {
        return NULL;
    }

	if (!(dma_chain = devm_kmalloc(dev, sizeof(struct asm9260_dma_chain), GFP_KERNEL))) {
		printk("DMA: Fail,No system memory\n");
        return NULL;
	}

    if (!(dma_chain->chain_head = (struct asm9260_dma_pkg_s *) dma_alloc_writecombine(dev,
       sizeof(struct asm9260_dma_pkg_s)*pkg_num, &dma_chain->chain_phy_addr, GFP_KERNEL))){
		printk("DMA: Fail,No dma memory\n");
        return NULL;
    }

    chain_ptr = dma_chain->chain_head;
    dma_chain->channel_num = channel_num;
    dma_chain->pkg_num = pkg_num;
    dma_chain->status = ASM9260_DMA_STAT_READY;

    /* init chain connections */
    for (i = 1; i<pkg_num ; i++) {
        chain_ptr->NEXT_PKG = dma_chain->chain_phy_addr + (i*sizeof(struct asm9260_dma_pkg_s));
        chain_ptr++;
    }

    return dma_chain;
}


 /*
free_asm9260_dma_chain
inputs:
struct device * dev the device which use this dma chain
dma_chain,          dma_chain struct which need free
*/
void free_asm9260_dma_chain(struct device * dev,struct asm9260_dma_chain * dma_chain){

    dma_free_writecombine(dev,sizeof(struct asm9260_dma_pkg_s)*dma_chain->pkg_num,
                  dma_chain->chain_head ,dma_chain->chain_phy_addr);
    devm_kfree(dev, dma_chain);
}


int dma_clear_interrupt(int ch_num) 
{ 
    as3310_writel((1<<(ch_num)),HW_APBH_CTRL1_CLR); 
    return 0; 
} 
 
int dma_enable_interrupt(int ch_num) 
{ 
  
     as3310_writel((1<<(ch_num+16)),HW_APBH_CTRL1_SET); 
     return 0; 
} 
 

     
EXPORT_SYMBOL(dma_start_apbh);
EXPORT_SYMBOL(dma_start_apbh_force);
EXPORT_SYMBOL(dma_start_apbx);
EXPORT_SYMBOL(asm9260_dma_init);
EXPORT_SYMBOL(dma_apbh_reset_ch); 
EXPORT_SYMBOL(dma_clear_interrupt); 
EXPORT_SYMBOL(dma_enable_interrupt); 
#endif



#if 1 //new LOW-LEVEL DMA routines for 9260

static unsigned long DmaChannelBitMap[DMAModuleNum][BITS_TO_LONGS(DMAChannelNum)];

#define MAX_RBB_WAIT				(0x1000000)
const int DmaChannelBlockTsBits[DMAModuleNum][DMAChannelNum] = {  {12, 10, 9, 9, 7, 7, 7, 7},
                                                                  {12, 10, 9, 9, 7, 7, 7, 7}};  //modified for 9260T, 01/31/2013.

/**
 *  DMAChannelBlockTsBits - get the BLOCK_TS bits for the target channel.  
 *  @dma_module: which dma module will be used.
 *  @dma_channel: which channel of the chosen module to be used.
 */
int DMAChannelBlockTsBits(DMAmodule module, DMAchannel channel)
{
    return DmaChannelBlockTsBits[module][channel];
}


/**
  * DmaRequestChain - alloc memory space for dma chain
  * @dma_module: which dma module will be used.
  * @dma_channel: which channel of the chosen module to be used.
  * @pkg_num: how many descriptors(AKA, Linked List Item) needed.
  * @channel_ready: the desired (dma_module, dma_channel) ready for use?  1,yes; 0, no.
  * 
  * We probably requesting a busy channel, then we should try another one according to channel_ready.
  */
DmaChain* DmaRequestChain(struct device * dev, DMAmodule module, DMAchannel channel, int pkg_num, int* channel_ready)
{
   DmaChain* dma_chain;
   DmaPkg* chain_ptr;
   int i;


   if((module<DMA_MODULE_0)||(module>DMA_MODULE_1)||(channel<DMA_CHANNEL_0)||(channel>DMA_CHANNEL_7)||(!pkg_num) )
   {
		return NULL;
   }

   if( (DmaIsBusy(module, channel)==CHANNEL_BUSY)|| (test_and_set_bit(channel,DmaChannelBitMap[module])!=0) )
   {
	   *channel_ready = 0;
	   return NULL;	   
   }

   *channel_ready = 1;  
    
   dma_chain = (DmaChain*)devm_kmalloc(dev, sizeof(DmaChain), GFP_KERNEL);   
   if (!dma_chain)
   {
	   return NULL;
   }

   memset(dma_chain,0,sizeof(DmaChain));

   dma_chain->chain_head = (DmaPkg*)dma_alloc_writecombine(dev,sizeof(DmaPkg)*pkg_num, &dma_chain->chain_phy_addr, GFP_KERNEL);  
   if ( (dma_chain->chain_head==NULL)||(dma_chain->chain_phy_addr== ~0) )
   {
	   devm_kfree(dev, dma_chain);
	   return NULL;
   }
   memset((void *)dma_chain->chain_head,0,(size_t)(sizeof(DmaPkg) * pkg_num));   
  
      
   dma_chain->dmamodule = module;
   dma_chain->dmachannel = channel;
   dma_chain->pkg_num = pkg_num;
 

   chain_ptr = dma_chain->chain_head;
   for(i=0; i<pkg_num; i++)
   {
		chain_ptr[i].LLP = dma_chain->chain_phy_addr + ((i+1)*sizeof(DmaPkg));
   }
   chain_ptr[pkg_num-1].LLP = (u32)NULL;
  

   return dma_chain;
}


/**
 *  DmaFreeChain - free up the memory space allocated for dma_chain.
 *  @dma_chain: the targeted one. 
 *
 */
void DmaFreeChain(struct device * dev, DmaChain* dma_chain)
{
    clear_bit( dma_chain->dmachannel, DmaChannelBitMap[dma_chain->dmamodule]);
	dma_free_writecombine(dev, sizeof(DmaChain)*dma_chain->pkg_num, dma_chain->chain_head, dma_chain->chain_phy_addr);
    devm_kfree(dev, dma_chain);
}


/**
  *  DmaIsBusy - check whether one channel is reday for use. 
  *  @dma_module: which dma module will be used.
  *  @dma_channel: which channel of the chosen module to be used.
  */
ChannelState DmaIsBusy(DMAmodule module, DMAchannel channel)
{
   if( as3310_readl(((module==DMA_MODULE_0) ? HW_DMA0_CHENREG : HW_DMA1_CHENREG))&(1<<channel) )
   		return CHANNEL_BUSY;
   else
   		return CHANNEL_IDLE;	
}


/**
  *  DmaInit - init the desired DMA channel.
  *  @dma_module: which dma module will be used.
  *  @dma_channel: which channel of the chosen module to be used.
  *
  *  If the channel is being used, we just return, and may retry for another one.
  */   
int DmaInit(DMAmodule module, DMAchannel channel)
{  

	if( DmaIsBusy(module, channel)==CHANNEL_BUSY ) 
		return -1;

	if( module==DMA_MODULE_0 )
	{
		 /*enable DMA0 clk*/
    	 as3310_writel(1<<9, HW_PRESETCTRL0+4);
    	 as3310_writel(1<<9, HW_AHBCLKCTRL0+4);

		 as3310_writel(0x01, HW_DMA0_DMACFGREG); //not interfere with other channels		 	 
	}
	else
	{
		 /*enable DMA1 clk*/
    	 as3310_writel(1<<10, HW_PRESETCTRL0+4);
    	 as3310_writel(1<<10, HW_AHBCLKCTRL0+4);

		 as3310_writel(0x01, HW_DMA1_DMACFGREG); //not interfere with other channels		 
	}	

	if ( DmaStop(module, channel) )
		return -1;
	else
		return 0;	
}


/**
   *  ChannelInterrupt - disable or enable a kind of interrupt for one channel.
   *  @dma_module: which dma module will be used.
   *  @dma_channel: which channel of the chosen module to be used.
   *  @inttype: the desired interrupt
   *  @switch: disable or enable. 0, disable; 1, enable.
   *
   */
void DmaChannelInterrupt(DMAmodule module, DMAchannel channel, DMAinterruptType inttype, int funswitch )
{
   if( module==DMA_MODULE_0 )
   {
		if( funswitch!=0 )
		{
			as3310_writel( ((1<<channel)<<8)+(1<<channel), (HW_DMA0_MaskTFR+8*inttype) );
		}
		else
		{
			as3310_writel( ((1<<channel)<<8), (HW_DMA0_MaskTFR+8*inttype) );
		}
   }
   else
   {
		if( funswitch!=0 )
		{
			as3310_writel( ((1<<channel)<<8)+(1<<channel), (HW_DMA1_MaskTFR+8*inttype) );
		}
		else
		{
			as3310_writel( ((1<<channel)<<8), (HW_DMA1_MaskTFR+8*inttype) );
		}
   }
}


/**
  *  DmaChannelInterruptStatusBit - read Interrupt Raw Status Registers or Interrupt Status Registers of
  *  one dma channel for the targeted interrupt status type.
  *  @dma_module: which dma module will be used.
  *  @dma_channel: which channel of the chosen module to be used.
  *  @inttype: the desired interrupt status type.
  *  @statustype: 0, raw; 1, masked. 
  *
  *  return value: 0, interrupt status bit not set; 1, interrupt status bit is set.
  */
int DmaChannelInterruptStatusBit(DMAmodule module, DMAchannel channel, DMAinterruptType inttype, int statustype )
{
	int retval;

	if( module==DMA_MODULE_0 )
	{
		if(statustype == 0) //raw interrupt status type
		{
			retval = as3310_readl( (HW_DMA0_RawTFR+8*inttype) )&(1<<channel);
		}
		else //masked 
		{
			retval = as3310_readl( (HW_DMA0_StatusTFR+8*inttype) )&(1<<channel);
		}

	}
	else
	{
		if(statustype == 0) //raw interrupt status type
		{
			retval = as3310_readl( (HW_DMA1_RawTFR+8*inttype) )&(1<<channel);
		}
		else //masked 
		{
			retval = as3310_readl( (HW_DMA1_StatusTFR+8*inttype) )&(1<<channel);
		}
	}

	return retval;
}


/**
  *  DmaChannelClearInterruptStatus - clear the corresponding bit in the chosen interrupt type's  
  *  interrupt Raw Status and interrupt Status registers of one dma channel.
  *  @dma_module: which dma module will be used.
  *  @dma_channel: which channel of the chosen module to be used.
  *  @inttype: the desired interrupt status type.
  *
  */
void DmaChannelClearInterruptStatus(DMAmodule module, DMAchannel channel, DMAinterruptType inttype)
{
   if( module==DMA_MODULE_0 )
   {
		as3310_writel( (1<<channel), (HW_DMA0_ClearTFR+8*inttype) );
   }
   else
   {
		as3310_writel( (1<<channel), (HW_DMA1_ClearTFR+8*inttype) );
   }
}


/**
  *  _WaitDmaSemaphore - wait for a dma channel to be idle. Like not ready before start   
  *  and timeout for finish.
  *  @dma_module: which dma module will be used.
  *  @dma_channel: which channel of the chosen module to be used.
  *  Note: For data service finish verification check, user may need to check the busy bit of the function controller, which requests DMA service, enve when _WaitDmaSemaphore finishes; 
  */
static int _WaitDmaSemaphore(DMAmodule module, DMAchannel channel)
{
   int wait_dma = MAX_RBB_WAIT;

   while( DmaIsBusy(module, channel)==CHANNEL_BUSY )
   {
	  if(!(wait_dma--))
	  {
		return -1;
	  }
   }
 
   return 0; 		
}


/**
  *  DmaStop - disable a dma channel, and clear the corresponding interrupt enable bits and interrupt status bits.
  *  @dma_module: which dma module will be used.
  *  @dma_channel: which channel of the chosen module to be used.
  *
  *  return value: 0, channel is disabled properly. -1, error of timeout.
  */
int DmaStop(DMAmodule module, DMAchannel channel)
{     
    int i;
	int wait_time = MAX_RBB_WAIT;

	if( module==DMA_MODULE_0 )
	{
		as3310_writel( ((1<<channel)<<8), HW_DMA0_CHENREG ); //disable the chosen one	
	}
	else
	{
		as3310_writel(((1<<channel)<<8), HW_DMA1_CHENREG); //disable the chosen one
	}

		/*disable all interrupts*/
	for(i=0; i<5; i++)
	{
	   DmaChannelInterrupt(module, channel, (DMAinterruptType)i, 0);
	}
	
	
	/*clear all interrupt status*/
	for(i=0; i<5; i++)
	{
	   DmaChannelClearInterruptStatus(module, channel, (DMAinterruptType)i);
	}
	

	/*according to the data book, we must poll the corresponding ChEnReg.CH_EN bit*/
	while( DmaIsBusy(module, channel)==CHANNEL_BUSY )
	{
	  if(!(wait_time--))
	  {
		return -1;
	  }
	}

	return 0;
}

/**
 *  DmaChannelCFGset - set DMA channel's Channel ConfigurationRegister(CFG)
 *  @dma_module: which dma module will be used.
 *  @dma_channel: which channel of the chosen module to be used.
 *  @lowregval: CFG's low 32-bit setting.
 *  @highregval: CFG's high 32-bit setting.
 * 
 *  Note: This must be called before DMA-start(DmaStartRoutine).
 */
void DmaChannelCFGset(DMAmodule module, DMAchannel channel, u32 lowregval, u32 highregval)
{
     as3310_writel(lowregval, HW_DMA0_CFG0+0x58*channel+module*0x100000);
     as3310_writel(highregval, 4+HW_DMA0_CFG0+0x58*channel+module*0x100000);
}


/**
  *  DmaStart - start DMA transmission.  
  *  @dma_module: which dma module will be used.
  *  @dma_channel: which channel of the chosen module to be used.
  *  @pAddr: the physical address of the the first LLI(dma descriptor).
  *  @vAddr: the virtual address of the the first LLI(dma descriptor).
  *  @dmamethod: Programming of Transfer Types and Channel Register Update Method.
  *
  *  Note: 
  *  1. For DMAmethod within ROW1-ROW5's range, we should have only one LLI(descriptor), and 
  *  use it in a special way. We strip the information from it to fill the hardware registers, then start DMA.
  *  2. For DMAmethod within ROW6-ROW10's range, !!**when CTRL's LLP_DST_EN or LLP_SRC_EN(or both)is set,
  *  the DMA just directly load the first LLI from LLP(register), no DMA data transmission before the loading for all cases.**!!
  *  
  */
static void DmaStart(DMAmodule module, DMAchannel channel, dma_addr_t pAddr, DmaPkg* vAddr, DMAmethod dmamethod)
{	

	if( (dmamethod<=DMAtranstype5)&&(dmamethod>=DMAtranstype1) )
	{
		as3310_writel( vAddr->SAR, HW_DMA0_SAR0+0x58*channel+module*0x100000);	
		as3310_writel( vAddr->DAR, HW_DMA0_DAR0+0x58*channel+module*0x100000);
		as3310_writel( vAddr->CTRL_L, HW_DMA0_CTL0+0x58*channel+module*0x100000);
		as3310_writel( vAddr->CTRL_H, HW_DMA0_CTL0+4+0x58*channel+module*0x100000);
		if(dmamethod!=DMAtranstype5)
		{
			as3310_writel( 0, HW_DMA0_LLP0+0x58*channel+module*0x100000);
		}
		else
		{
			as3310_writel( vAddr->LLP, HW_DMA0_LLP0+0x58*channel+module*0x100000);
		}
	}
	else
	{
		as3310_writel( (unsigned int)pAddr, HW_DMA0_LLP0+0x58*channel+module*0x100000); //first LLI address
	
		if( (dmamethod==DMAtranstype6)||(dmamethod==DMAtranstype7) )
		{
			as3310_writel( vAddr->SAR, HW_DMA0_SAR0+0x58*channel+module*0x100000);	
			as3310_writel( vAddr->CTRL_L, HW_DMA0_CTL0+0x58*channel+module*0x100000);
		}
		else if( (dmamethod==DMAtranstype8)||(dmamethod==DMAtranstype9 ) )
		{
			as3310_writel( vAddr->DAR, HW_DMA0_DAR0+0x58*channel+module*0x100000);
			as3310_writel( vAddr->CTRL_L, HW_DMA0_CTL0+0x58*channel+module*0x100000);
		}	
		else//DMAmethod==10
		{
			as3310_writel( vAddr->CTRL_L, HW_DMA0_CTL0+0x58*channel+module*0x100000);
		}
	}
 
	as3310_writel( 0x01, HW_DMA0_DMACFGREG+module*0x100000 );
	as3310_writel( ((1<<channel)<<8)+(1<<channel), HW_DMA0_CHENREG+module*0x100000 );

	return ;
}


/**
  *  DmaStartRoutine - do preparation for dma start, like cache operation. 
  *  @dma_module: which dma module will be used.
  *  @dma_channel: which channel of the chosen module to be used.
  *  @pAddr: the physical address of the first LLI(dma descriptor).
  *  @vAddr: the virtual address of the first LLI(dma descriptor).
  *  //@nPkgNum: how many LLIs. When just for src-dst-linked, it
  *  //          has no practical purpose, but when a channel
  *  //          configured as auto-reload, it can be used to
  *  //          track the current serviced block, "the
  *  //          end-of-block interrupt service routine that
  *  //          services the next-to-last block transfer should
  *  //          clear theCFGx.RELOAD_SRC and CFGx.RELOAD_DST
  *  //          reload bits".
  *  @bWaitSemaAfter: wait until this LLI finish? 0, no; !0, yes.
  *  @dmamethod: Programming of Transfer Types and Channel Register Update Method.
  *
  * Warning: 
  * 1. user must already set Channel Configuration Register(CFG) 
  * properly before calling this function. 
  *  
  * 2. user must ensure that the DmaPkg->SAR and DmaPkg->DAR are
  * aligned to the setting of CTRL->SRC_TR_WIDTH and 
  * CTRL->DST_TR_WIDTH, we just ignore it here for avoiding the 
  * unnecessary complexity. 
  *   For more, see DesignWare DW_ahb_dmac Databook.
  */
int DmaStartRoutine(DMAmodule module, DMAchannel channel, dma_addr_t pAddr, void* vAddr, int bWaitSemaAfter, DMAmethod dmamethod)
{
	int ret = 0;
	

    //spin_lock(&DmaChannelLock[module][channel]);//not nessary for UP MACHINE.
    if( DmaIsBusy(module, channel)!=CHANNEL_BUSY )
    {
        ret = CHANNEL_IDLE;
        DmaStart(module, channel, pAddr, (DmaPkg *)vAddr, dmamethod);        
    }
    else
    {
        ret = CHANNEL_BUSY;
    }
    //spin_unlock(&DmaChannelLock[module][channel]);//not nessary for UP MACHINE.


    if(ret!=CHANNEL_IDLE)
    {
        return -1;
    }
			

	if( !bWaitSemaAfter )
	{
		return 0;
	}
	else
	{	
		return _WaitDmaSemaphore(module, channel);
	}
}

#endif 


