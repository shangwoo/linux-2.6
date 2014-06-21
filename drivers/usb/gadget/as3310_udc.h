
/*
 * as3310_udc -- driver for as3310-series USB peripheral controller
 *
 * Copyright (C) 2007 by He Yong
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef AS3310_UDC_H
#define AS3310_UDC_H




#define HW_AUDIO_OUT_CTRL       0x80048000
#define HW_AUDIO_IN_CTRL        0x8004c000
#define HW_AUDIO_OUT_PWRD_CTRL  0x80048070
#define HW_AUDIO_OUT_REF_CTRL   0x80048080

#define as3310_auido_readl(reg)	__raw_readl(IO_ADDRESS(reg))
#define as3310_auido_writel(reg, val)	__raw_writel((val), IO_ADDRESS(reg))

#define as3310_phy_readl(reg)	__raw_readl(IO_ADDRESS(reg))
#define as3310_phy_writel(reg, val)	__raw_writel((val), IO_ADDRESS(reg))


#define as3310_phy_readw(reg)	__raw_readw(IO_ADDRESS(reg))
#define as3310_phy_writew(reg, val)	__raw_writew((val), IO_ADDRESS(reg))


#define as3310_phy_readb(reg)	__raw_readb(IO_ADDRESS(reg))
#define as3310_phy_writeb(reg, val)	__raw_writeb((val), IO_ADDRESS(reg))


#define as3310_usbc_readl(host, reg)	__raw_readl((host)->udp_baseaddr + (reg))
#define as3310_usbc_writel(host, reg, val)	__raw_writel((val), (host)->udp_baseaddr + (reg))

#define as3310_usbc_readw(host, reg)	__raw_readw((host)->udp_baseaddr + (reg))
#define as3310_usbc_writew(host, reg, val)	__raw_writew((val), (host)->udp_baseaddr + (reg))

#define as3310_usbc_readb(host, reg)	__raw_readb((host)->udp_baseaddr + (reg))
#define as3310_usbc_writeb(host, reg, val)	__raw_writeb((val), (host)->udp_baseaddr + (reg))


//define register address
// usb phy register
// 

#define USB_PWD        	0x8007C000
#define USB_TX       	0x8007C010
#define USB_RX          0x8007C020
#define USB_CTRL       	0x8007C030
#define USB_SYSCTRL     0x8007C090
#define USB_PHY_ANALOG  0x8007c0a0

// usb clk in clkctrl register
#define UTMI_CLK        0x80040070
#define USB_CLK         0x8001C000
#define CPU_CLK         0x80040020
#define CLK_CTRL        0x80040000

//irq register

#define USB_DMA_IRQ     11
#define USB_MC_IRQ      57
#define USB_NEGEDGE_IRQ 59
#define USB_POSEDGE_IRQ 60
#define PRIORITY_BASE   0x80000060
#define ICOLL_CLEAR1    0x800001E0



//usb mentor register

#define USB_BaseAddr 0x90000000	//USB base address

#define ARM

#define USB_BaseAddr 0x90000000	//USB base address



// //common registers
#define	USB_FAddr	   0x00 
#define	USB_Power	   0x01 
#define	USB_IntrTx	   0x02 
#define	USB_IntrRx	   0x04 
#define	USB_IntrTxE	   0x06 
#define	USB_IntrRxE	   0x08 
#define	USB_IntrUSB	   0x0A 
#define USB_IntrUSB_Suspend (1<<0)
#define USB_IntrUSB_Resume  (1<<1)
#define USB_IntrUSB_Reset   (1<<2)
#define USB_IntrUSB_SOF     (1<<3)
#define USB_IntrUSB_Conn    (1<<4)
#define USB_IntrUSB_Discon  (1<<5)
#define USB_IntrUSB_SessReq (1<<6)
#define USB_IntrUSB_VBusErr (1<<7)

#define	USB_IntrUSBE   0x0B 
#define	USB_Frame	   0x0C 
#define	USB_Index	   0x0E 
#define	USB_Testmode   0x0F 

#define USB_TXMAXP  0x0 
#define USB_TXCSR   0x2 
#define USB_RXMAXP  0x4 
#define USB_RXCSR   0x6 
#define USB_RXCOUNT 0x8   /* u8 */

#define USB_EP1_MAX_PACKET_SIZE 0x0008
#define USB_EP2_MAX_PACKET_SIZE 0x0040
#define USB_EP2_MAX_FIFO_SIZE 0x03  /* 64 Byte */

//peripheral mode
#define	USB_TxMaxP		   0x10 
#define	USB_CSR0           0x12 	//ep0
#define USB_CSR0_RxPktRdy  	1
#define USB_CSR0_TxPktRdy  	(1<<1)
#define USB_CSR0_SENTSTALL 	(1<<2)
#define USB_CSR0_DATAEND  	(1<<3)
#define USB_CSR0_SETUPEND  (1<<4)
#define USB_CSR0_SENDSTALL  (1<<5)
#define USB_CSR0_SERVICEDRXRDY  (1<<6)
#define USB_CSR0_SERVICEDSETUPEND (1<<7)
#define USB_CSR0_FLUSHFIFO 	(1<<8)


#define	USB_TxCSR          0x12 	//ep1-15
#define USB_TxCSR_TxPktRdy		(1<<0)
#define USB_TXCSR_FIFO_NotEmpty	(1<<1)
#define USB_TXCSR_UnderRun		(1<<2)
#define USB_TXCSR_FlushFIFO		(1<<3)
#define USB_TXCSR_SendStall		(1<<4)
#define USB_TXCSR_SentStall		(1<<5)
#define USB_TXCSR_ClrDataTog	(1<<6)
#define USB_TXCSR_IncompTx		(1<<7)
#define USB_TXCSR_DmaReqMode	(1<<10)
#define USB_TXCSR_FrcDataTog	(1<<11)
#define USB_TXCSR_DmaReqEnab	(1<<12)
#define USB_TXCSR_Mode			(1<<13)
#define USB_TXCSR_ISO			(1<<14)
#define USB_TXCSR_AutoSet		(1<<15)

#define	USB_RxMaxP         0x14 
#define	USB_RxCSR          0x16 
#define USB_RxCSR_RxPktRdy		(1<<0)
#define USB_RxCSR_FIFOFull		(1<<1)
#define USB_RxCSR_OverRun		(1<<2)
#define USB_RxCSR_DataErr		(1<<3)
#define USB_RxCSR_FlushFifo		(1<<4)
#define USB_RxCSR_SendStall		(1<<5)
#define USB_RxCSR_SentStall		(1<<6)
#define USB_RxCSR_ClrDataTog	(1<<7)
#define USB_RxCSR_IncompRx		(1<<8)
#define USB_RxCSR_DMAReqMode	(1<<11)
#define USB_RxCSR_PIDErr		(1<<12)
#define USB_RxCSR_DmaReqEnab	(1<<13)
#define USB_RxCSR_ISO			(1<<14)
#define USB_RxCSR_AutoClear		(1<<15)

#define	USB_Count0         0x18 
#define	USB_RxCount        0x18 
#define	USB_ConfigData     0x1F     /* u8 */
#define	USB_FIFOSize       0x1F     /* u8 */

//register map
#define	USB_FIFO0		    0x20 
#define	USB_FIFO1       	0x24 
#define	USB_FIFO2       	0x28 
#define	USB_FIFO3       	0x2C 
#define	USB_FIFO4       	0x30 
#define	USB_FIFO5       	0x34 
#define	USB_FIFO6       	0x38 
#define	USB_FIFO7       	0x3C 
#define	USB_FIFO8       	0x40 
#define	USB_FIFO9       	0x44 
#define	USB_FIFO10      	0x48 
#define	USB_FIFO11      	0x4C 
#define	USB_FIFO12      	0x50 
#define	USB_FIFO13      	0x54 
#define	USB_FIFO14      	0x58 
#define	USB_FIFO15      	0x5C 
        
//additional control & configuration
#define USB_DevCtl			   0x60      /*  short   */ 
#define USB_TxFIFOsz           0x62   	 /*  char*   */ 
#define	USB_RxFIFOsz           0x63   	 /*  char*   */ 
#define	USB_TxFIFOadd          0x64   	 /*  short   */ 
#define	USB_RxFIFOadd          0x66   	 /*  short   */ 
#define	USB_VControl_VStatus   0x68   	 /*  int*)   */ 
#define	USB_HWVers             0x6C   	 /*  short   */ 
#define	USB_EPInfo             0x78   	 /*  char*   */ 
#define	USB_RAMInfo            0x79   	 /*  char*   */ 
#define	USB_LinkInfo           0x7A   	 /*  char*   */ 
#define	USB_VPLen              0x7B   	 /*  char*   */ 
#define	USB_HS_EOF1            0x7C   	 /*  char*   */ 
#define	USB_FS_EOF1            0x7D   	 /*  char*   */ 
#define	USB_LS_EOF1            0x7E   	 /*  char*   */ 

#define USB_DMA_INTR           0x200
#define USB_DMA_CNTL           0x204
#define USB_DMA_ADDR           0x208
#define USB_DMA_COUNT          0x20C
////bit defination for TXCSR ep0
//#define USB_CSR0_RXPKTRDY   (1)
//#define USB_CSR0_TXPKTRDY   (1<<1)
//#define USB_CSR0_SENTSTALL  (1<<2)
//#define USB_CSR0_DATAEND    (1<<3)
//#define USB_CSR0_SETUPEND   (1<<4)
//#define USB_CSR0_SENDSTALL  (1<<5)
//#define USB_CSR0_SERVICEDRXPKTRDY   (1<<6)
//#define USB_CSR0_SERVICEDSETUPEND   (1<<7)
//#define USB_CSR0_FLUSHFIFO          (1<<8)
//

//bit defination for RXCSR
#define USB_RX_RXPKTRDY     (1)
#define USB_RX_FIFOFULL     (1<<1)
#define USB_RX_OVERRUN      (1<<2)
#define USB_RX_DATAERROR    (1<<3)
#define USB_RX_FLUSHFIFO    (1<<4)
#define USB_RX_SENDSTALL    (1<<5)
#define USB_RX_SENTSTALL    (1<<6)
#define USB_RX_CLRDATATOG   (1<<7)
#define USB_RX_INCOMRX      (1<<8)
#define USB_RX_DMAREQMODE   (1<<11)
#define USB_RX_PIDERR       (1<<12)
#define USB_RX_DMAREQEN     (1<<13)
#define USB_RX_ISO          (1<<14)
#define USB_RX_AUTOCLEAR    (1<<15)

//bit defination fro TXCSR  
#define USB_TX_TXPKTRDY     (1)
#define USB_TX_FIFONOTEPT   (1<<1)
#define USB_TX_UNDERRUN     (1<<2)
#define USB_TX_FLUSHFIFO    (1<<3)
#define USB_TX_SENDSTALL    (1<<4)
#define USB_TX_SNETSTALL    (1<<5)
#define USB_TX_CLRDATATOG   (1<<6)
#define USB_TX_INCOMTX      (1<<7)
#define USB_TX_DMAREQMODE   (1<<10)
#define USB_TX_FRCDATATOG   (1<<11)
#define USB_TX_DMAREQEN     (1<<12)
#define USB_TX_MODE         (1<<13)
#define USB_TX_ISO          (1<<14)
#define USB_TX_AUTOSET      (1<<15)


typedef volatile struct _mentor_reg_{
 u8         R_FAddr       ;        //(0x00 + USB_BaseAddr))
 u8         R_Power       ;        //(0x01 + USB_BaseAddr))
 u16        R_IntrTx      ;        //)(0x02 + USB_BaseAddr)
 u16        R_IntrRx      ;        //)(0x04 + USB_BaseAddr)
 u16        R_IntrTxE     ;        //)(0x06 + USB_BaseAddr)
 u16        R_IntrRxE     ;        //)(0x08 + USB_BaseAddr)
 u8         R_IntrUSB     ;        //(0x0A + USB_BaseAddr))
 u8         R_IntrUSBE    ;        //(0x0B + USB_BaseAddr))
 u16         R_Frame       ;        //(0x0C + USB_BaseAddr))
 u8         R_Index       ;        //(0x0E + USB_BaseAddr))
 u8         R_Testmode    ;        //(0x0F + USB_BaseAddr))
                                                                       
u16        R_TxMaxP      ;	         //  0x10 + USB_BaseAddr))      
u16        R_TxCSR       ;           //  0x12 + USB_BaseAddr)) 
u16        R_RxMaxP      ;         //  0x14 + USB_BaseAddr))  
u16        R_RxCSR       ;          //  0x16 + USB_BaseAddr))  
u16        R_RxCount     ;         //  0x18 + USB_BaseAddr))
u8         R_rsvd[5]     ;          //
u8         R_ConfigData  ;     //    0x1F + USB_BaseAddr))   
                                     
u32    	   R_FIFO0	   ;     //   (0x20 + USB_BaseAddr)                              
u32    	   R_FIFO1      ;     //   (0x24 + USB_BaseAddr)                          
u32    	   R_FIFO2      ;     //   (0x28 + USB_BaseAddr) 
u32    	   R_FIFO3      ;     //   (0x2C + USB_BaseAddr)                          
u32    	   R_FIFO4      ;     //   (0x30 + USB_BaseAddr)                          
u32    	   R_FIFO5      ;     //   (0x34 + USB_BaseAddr)                          
u32    	   R_FIFO6      ;     //   (0x38 + USB_BaseAddr)                          
u32    	   R_FIFO7      ;     //   (0x3C + USB_BaseAddr)                          
u32    	   R_FIFO8      ;     //   (0x40 + USB_BaseAddr)                          
u32    	   R_FIFO9      ;     //   (0x44 + USB_BaseAddr)                          
u32    	   R_FIFO10     ;     //   (0x48 + USB_BaseAddr)                          
u32    	   R_FIFO11     ;     //   (0x4C + USB_BaseAddr)                          
u32    	   R_FIFO12     ;     //   (0x50 + USB_BaseAddr)                          
u32    	   R_FIFO13     ;     //   (0x54 + USB_BaseAddr)                          
u32    	   R_FIFO14     ;     //   (0x58 + USB_BaseAddr)                          
u32    	   R_FIFO15     ;     //   (0x5C + USB_BaseAddr)                          

u16         R_DevCtl			    ;   //   0x60 + USB_BaseAddr)) 
u8          R_TxFIFOsz             ;   //   0x62 + USB_BaseAddr))  
u8          R_RxFIFOsz            ;   //   0x63 + USB_BaseAddr))  
u16        	R_TxFIFOadd           ;   //   0x64 + USB_BaseAddr)) 
u16        	R_RxFIFOadd           ;   //   0x66 + USB_BaseAddr)) 
u32         R_VControl_VStatus    ;   //   0x68 + USB_BaseAddr))   
u16        	R_HWVers              ;   //   0x6C + USB_BaseAddr)) 
u8        	rsvd2[10]             ;                          // 
u8          R_EPInfo              ;   //   0x78 + USB_BaseAddr))  
u8          R_RAMInfo             ;   //   0x79 + USB_BaseAddr))  
u8          R_LinkInfo            ;   //   0x7A + USB_BaseAddr))  
u8          R_VPLen               ;   //   0x7B + USB_BaseAddr))  
u8          R_HS_EOF1             ;   //   0x7C + USB_BaseAddr))  
u8          R_FS_EOF1             ;   //   0x7D + USB_BaseAddr))  
u8          R_LS_EOF1             ;   //   0x7E + USB_BaseAddr))  

}__attribute__((packed)) AS3310_USB_CTRL;


static inline AS3310_USB_CTRL * const AS3310_GetBase_USB_CTRL(void)
{
	return (AS3310_USB_CTRL * const)USB_BaseAddr;
}

/*
//global variable
extern CONTROL_XFER *CtrlData;
extern unsigned char * State;
extern unsigned char * config_status;
*/

//macro operation
// #define MSB(x) ( ((x)>>8) & 0xff )
// #define LSB(x) ( (x) & 0xff )
// #define SWAP(x) ( (((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF) )
// #define SWAP32(x)   ((((u32)x)>>24) | ((((u32)x)>>8)&0xff00) | ((((u32)x)<<8)&0xff0000) | (((u32)x)<<24))




/*
 * USB Device Port (UDP) registers.
 * Based on AS3310RM9200 datasheet revision E.
 */

#define AS3310_UDP_FRM_NUM	0x0c		/* Frame Number Register */
// #define     AS3310_UDP_NUM	(0x7ff <<  0)	/* Frame Number */
// #define     AS3310_UDP_FRM_ERR	(1     << 16)	/* Frame Error */
// #define     AS3310_UDP_FRM_OK	(1     << 17)	/* Frame OK */

#define AS3310_UDP_GLB_STAT	0x04		/* Global State Register */
#define     AS3310_UDP_FADDEN	(1 <<  0)	/* Function Address Enable */
#define     AS3310_UDP_CONFG	(1 <<  1)	/* Configured */
#define     AS3310_UDP_ESR	(1 <<  2)	/* Enable Send Resume */
#define     AS3310_UDP_RSMINPR	(1 <<  3)	/* Resume has been sent */
#define     AS3310_UDP_RMWUPE	(1 <<  4)	/* Remote Wake Up Enable */

#define AS3310_UDP_FADDR		0x00		/* Function Address Register */
//#define     AS3310_UDP_FADD	(0x7f << 0)	/* Function Address Value */
//#define     AS3310_UDP_FEN	(1    << 8)	/* Function Enable */

#define AS3310_UDP_IER		0x0b		/* Interrupt Enable Register */
//#define AS3310_UDP_IDR		0x14		/* Interrupt Disable Register */
//#define AS3310_UDP_IMR		0x18		/* Interrupt Mask Register */

#define AS3310_UDP_ISR		0x1c		/* Interrupt Status Register */
#define     AS3310_UDP_EP(n)	(1 << (n))	/* Endpoint Interrupt Status */
#define     AS3310_UDP_RXSUSP	(1 <<  8) 	/* USB Suspend Interrupt Status */
#define     AS3310_UDP_RXRSM	(1 <<  9)	/* USB Resume Interrupt Status */
#define     AS3310_UDP_EXTRSM	(1 << 10)	/* External Resume Interrupt Status [AS3310RM9200 only] */
#define     AS3310_UDP_SOFINT	(1 << 11)	/* Start of Frame Interrupt Status */
#define     AS3310_UDP_ENDBUSRES	(1 << 12)	/* End of Bus Reset Interrpt Status */
#define     AS3310_UDP_WAKEUP	(1 << 13)	/* USB Wakeup Interrupt Status [AS3310RM9200 only] */

#define AS3310_UDP_ICR		0x20		/* Interrupt Clear Register */
#define AS3310_UDP_RST_EP		0x28		/* Reset Endpoint Register */

#define AS3310_UDP_CSR(n)		(0x100+(((n+1)/2)*0x10))	/* Endpoint Control/Status Registers 0-7 */
#define     AS3310_UDP_RX_PktRdy	(1 <<  0)	/* Generates IN packet with data previously written in DPR */

#define     AS3310_UDP_TXCOMP       (1<<0)

#define     AS3310_UDP_RX_DATA_BK0 (1 <<  1)	/* Receive Data Bank 0 */
#define     AS3310_UDP_STALLSENT	(1 <<  2)	/* Stall Sent / Isochronous error (Isochronous endpoints) */
#define     AS3310_UDP_TXPKTRDY	(1 <<  1)	/* Transmit Packet Ready */
#define     AS3310_UDP_DATAEND  (1 <<  3)
#define     AS3310_UDP_FORCESTALL	(1 <<  5)	/* Force Stall */
#define     AS3310_UDP_RX_DATA_BK1 (1 <<  6)	/* Receive Data Bank 1 */
#define     AS3310_UDP_DIR	(1 <<  7)	/* Transfer Direction */
#define     AS3310_UDP_EPTYPE	(7 <<  8)	/* Endpoint Type */
#define		AS3310_UDP_EPTYPE_CTRL		(0 <<  8)
#define		AS3310_UDP_EPTYPE_ISO_OUT		(1 <<  8)
#define		AS3310_UDP_EPTYPE_BULK_OUT	(2 <<  8)
#define		AS3310_UDP_EPTYPE_INT_OUT		(3 <<  8)
#define		AS3310_UDP_EPTYPE_ISO_IN		(5 <<  8)
#define		AS3310_UDP_EPTYPE_BULK_IN		(6 <<  8)
#define		AS3310_UDP_EPTYPE_INT_IN		(7 <<  8)
#define     AS3310_UDP_DTGLE	(1 << 11)	/* Data Toggle */
#define     AS3310_UDP_EPEDS	(1 << 15)	/* Endpoint Enable/Disable */
#define     AS3310_UDP_RXBYTECNT	(0x7ff << 16)	/* Number of bytes in FIFO */

#define AS3310_UDP_FDR(n)		(0x50+((n)*4))	/* Endpoint FIFO Data Registers 0-7 */

#define AS3310_UDP_TXVC		0x74		/* Transceiver Control Register */
#define     AS3310_UDP_TXVC_TXVDIS (1 << 8)	/* Transceiver Disable */
#define     AS3310_UDP_TXVC_PUON   (1 << 9)	/* PullUp On [AS3310SAM9260 only] */

/*-------------------------------------------------------------------------*/

/*
 * controller driver data structures
 */

#define	NUM_ENDPOINTS	5

/*
 * hardware won't disable bus reset, or resume while the controller
 * is suspended ... watching suspend helps keep the logic symmetric.
 */
#define	MINIMUS_INTERRUPTUS \
	(USB_IntrUSB_Suspend | USB_IntrUSB_Resume | USB_IntrUSB_Reset)

struct as3310_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	struct as3310_udc			*udc;
	void __iomem			*creg;
    u8                  index;
	unsigned			maxpacket:16;
	u8				int_mask;
	unsigned			is_pingpong:1;

	unsigned			stopped:1;
	unsigned			is_in:1;
	unsigned			is_iso:1;
	unsigned			fifo_bank:1;

	const struct usb_endpoint_descriptor
					*desc;
};

struct ep_buf
{
    unsigned bytes;
    void * buf;
    dma_addr_t *dma;
};

/*
 * driver is non-SMP, and just blocks IRQs whenever it needs
 * access protection for chip registers or driver state
 */
struct as3310_udc {
	struct usb_gadget		gadget;
	struct as3310_ep			ep[NUM_ENDPOINTS];
	struct usb_gadget_driver	*driver;
    struct completion	complete_dma;
    unsigned            dma_in_use:1;
	unsigned			vbus:1;
	unsigned			enabled:1;
	unsigned			clocked:1;
	unsigned			suspended:1;
	unsigned			req_pending:1;
	unsigned			wait_for_addr_ack:1;
	unsigned			wait_for_config_ack:1;
	unsigned			selfpowered:1;
	u8				addr;
	struct clk			*iclk, *fclk;
	struct platform_device		*pdev;
	struct proc_dir_entry		*pde;
	void __iomem			*udp_baseaddr;
	int				udp_irq;
};

static inline struct as3310_udc *to_udc(struct usb_gadget *g)
{
	return container_of(g, struct as3310_udc, gadget);
}

#define REQUEST_FIRST 0
#define REQUEST_WAITFOR_DONE 1

struct as3310_request {
	struct usb_request		req;
	struct list_head		queue;
	unsigned	state;
};

/*-------------------------------------------------------------------------*/


#if 1
#define DBG(stuff...)		printk(stuff)
#else
#define DBG(stuff...)		do{}while(0)
#endif

#ifdef VERBOSE
#    define VDBG		DBG
#else
#    define VDBG(stuff...)	do{}while(0)
#endif

#ifdef PACKET_TRACE
#    define PACKET		VDBG
#else
#    define PACKET(stuff...)	do{}while(0)
#endif

#define ERR(stuff...)		printk(KERN_ERR "udc: " stuff)
#define WARN(stuff...)		printk(KERN_WARNING "udc: " stuff)
#define INFO(stuff...)		printk(KERN_INFO "udc: " stuff)

#endif

