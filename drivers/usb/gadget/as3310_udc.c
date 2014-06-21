
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

#undef	DEBUG
#undef	VERBOSE
#undef	PACKET_TRACE

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
//#include <linux/usb_gadget.h>
#include <linux/usb/gadget.h>//zhangyb20110425
#include <linux/dma-mapping.h>


#include <linux/semaphore.h>
//#include <linux/byteorder.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/irq.h>
#include <mach/system.h>
#include <asm/mach-types.h>
#if 0
#include <asm/semaphore.h>
#include <asm/byteorder.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#endif
//#include <asm/arch/gpio.h>
//#include <asm/arch/board.h>
//#include <asm/arch/cpu.h>


#include "as3310_udc.h"


/*
 * This controller is simple and PIO-only.  It's used in many AS3310-series
 * full speed USB controllers, including the * as3310x (arm926ejs, with MMU), 
 * and several no-mmu versions.
 *
 * This driver expects the board has been wired with two GPIOs suppporting
 * a VBUS sensing IRQ, and a D+ pullup.  (They may be omitted, but the
 * testing hasn't covered such cases.)
 *
 * The pullup is most important (so it's integrated on sam926x parts).  It
 * provides software control over whether the host enumerates the device.
 *
 * The VBUS sensing helps during enumeration, and allows both USB clocks
 * (and the transceiver) to stay gated off until they're necessary, saving
 * power.  During USB suspend, the 48 MHz clock is gated off in hardware;
 * it may also be gated off by software during some Linux sleep states.
 */

extern int dbg_printf(const char *fmt, ...);

#define	DRIVER_VERSION	"0.1 Aug 2007"

#define TX_NEW 0
#define TX_OLD 1
static const char driver_name [] = "as3310_udc";
static const char ep0name[] = "ep0";
int USB_disk_on=0;
#ifdef CONFIG_USB_AS3310_DEBUG
#define usb_dbg(x...) dbg_printf(x);
#else
#define usb_dbg(x...) do{}while(0);
#endif

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

#define as3310_usbc_readbl(dev, reg) \
	__raw_readl((dev)->udp_baseaddr + (reg))
#define as3310_usbc_writebl(dev, reg, val) \
	__raw_writel((val), (dev)->udp_baseaddr + (reg))
static void  as3310udc_dbg_info(struct as3310_udc *udc)
{
    
   //printk("register:%08x\n",as3310_usbc_readb(udc,USB_Power));
}

static void as3310_udc_tasklet(unsigned long);
struct tasklet_struct udc_tasklet;
DECLARE_TASKLET_DISABLED(udc_tasklet, as3310_udc_tasklet,0);
/*-------------------------------------------------------------------------*/



static void ep_init(struct as3310_udc	*udc);
/*-------------------------------------------------------------------------*/
//ok now
static void done(struct as3310_ep *ep, struct as3310_request *req, int status)
{
	unsigned	stopped = ep->stopped;
 	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;
	if (status && status != -ESHUTDOWN)
		usb_dbg("%s done %p, status %d\n", ep->ep.name, req, status);
	ep->stopped = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->stopped = stopped;
}

/*-------------------------------------------------------------------------*/

/* bits indicating OUT fifo has data ready */
#define	RX_DATA_READY	(AS3310_UDP_RX_DATA_BK0 | AS3310_UDP_RX_DATA_BK1)
#define	SET_FX	(USB_CSR0_TxPktRdy)
#define	CLR_FX	(USB_CSR0_RxPktRdy | USB_CSR0_SETUPEND \
		| USB_CSR0_SENTSTALL | USB_CSR0_SERVICEDSETUPEND)

/*
 * Endpoint FIFO CSR bits have a mix of bits, making it unsafe to just write
 * back most of the value you just read (because of side effects, including
 * bits that may change after reading and before writing).
 *
 * Except when changing a specific bit, always write values which:
 *  - clear SET_FX bits (setting them could change something)
 *  - set CLR_FX bits (clearing them could change something)
 *
 * There are also state bits like FORCESTALL, EPEDS, DIR, and EPTYPE
 * that shouldn't normally be changed.
 *
 * NOTE as3310sam9260 docs mention synch between UDPCK and MCK clock domains,
 * implying a need to wait for one write to complete (test relevant bits)
 * before starting the next write.  This shouldn't be an issue given how
 * infrequently we write, except maybe for write-then-read idioms.
 */

static int read_fifo0 (struct as3310_ep *ep,struct as3310_request *req)
{
     void __iomem		*creg = ep->creg+0x2;
	u8   __iomem	    *dreg = ep->udc->udp_baseaddr+ep->index*4+0x20;
	u16		csr;
	u8		*buf;
	unsigned int	    count, bufferspace, is_done;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;
   	/*
	 * there might be nothing to read if ep_queue() calls us,
	 * or if we already emptied both pingpong buffers
	 */
	csr = __raw_readw(creg);
    if ((csr & USB_CSR0_RxPktRdy) == 0)
    return 0;

	count = __raw_readw(ep->creg+0x8);
	if (count > ep->ep.maxpacket)
		count = ep->ep.maxpacket;
	if (count > bufferspace) {
		usb_dbg("%s buffer overflow\n", ep->ep.name);
		req->req.status = -EOVERFLOW;
		count = bufferspace;
	}
	__raw_readsb(dreg, buf, count);

  //  __raw_writew(csr, creg);

	req->req.actual += count;
	is_done = (count < ep->ep.maxpacket);
	if (count == bufferspace)
		is_done = 1;

	PACKET("%s %p out/%d%s\n", ep->ep.name, &req->req, count,
			is_done ? " (done)" : "");
	if (is_done)
    {   
        csr|=USB_CSR0_SERVICEDRXRDY;
        __raw_writew(csr,creg);
        csr = __raw_readw(creg);
        csr|=USB_CSR0_DATAEND;
        __raw_writew(csr,creg);
		done(ep, req, 0);
    }
    else
    {
        csr|=USB_CSR0_SERVICEDRXRDY;
        __raw_writew(csr,creg);
    }
	return is_done;
}
//#define USE_DMA
DECLARE_MUTEX(dma_udc_lock);
static void setup_udc_dma(struct as3310_ep *ep,u8 dir,u32  addr,u32 count)
{
    u32 ctrl;
    struct as3310_udc	*udc=ep->udc;
//    down_interruptible(&dma_udc_lock);
 //   init_completion(&(udc->complete_dma));
    usb_dbg("init_completion :%p\n",udc);
    ctrl=1|(dir<<1)|(0<<2)|(1<<3)|(ep->index<<4)|(2<<9);
    as3310_usbc_writel(udc, USB_DMA_ADDR, addr);
//    printk("addr\n");
    as3310_usbc_writel(udc, USB_DMA_COUNT, count);
 //   printk("count\n");
    as3310_usbc_writel(udc, USB_DMA_CNTL, ctrl);
 //   printk("xdir %s,addr 0x%08x,ep->index:%d,count:%d\n",dir?"in":"out",addr,ep->index,count);
 //   wait_for_completion(&(udc->complete_dma));
 //   usb_dbg("wait_for_complete :%p\n",udc);
 //   up(&dma_udc_lock);
}

#ifdef USE_DMA

static int read_fifo (struct as3310_ep *ep, struct as3310_request *req)
{
	void   __iomem		*creg = ep->creg+0x6;
	void   __iomem	    *dreg = ep->udc->udp_baseaddr+ep->index*4+0x20;
	u16		csr;
	u8		*buf;
	unsigned int	    j,count, bufferspace, is_done;
    unsigned char *p1;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;

	/*
	 * there might be nothing to read if ep_queue() calls us,
	 * or if we already emptied both pingpong buffers
	 */
    usb_dbg("index :%d ,txmaxp:%d,rxmaxp%d\n",as3310_usbc_readb(ep->udc,USB_Index),as3310_usbc_readw(ep->udc,USB_TxMaxP), as3310_usbc_readw(ep->udc,USB_RxMaxP));
        csr = __raw_readw(creg);
        csr = csr&(~USB_RX_DMAREQEN);
    __raw_writew(csr|USB_RX_AUTOCLEAR,creg);
    csr = __raw_readw(creg);
    
	if ((csr & USB_RxCSR_RxPktRdy) == 0)
		return 0;
    if ((csr & USB_RxCSR_SentStall)!=0) {
        csr = __raw_readw(creg);
        csr|=USB_RxCSR_SentStall;
        __raw_writew(csr,creg);
    }

	count = __raw_readw(ep->creg+0x8);
	if (count > ep->ep.maxpacket)
		count = ep->ep.maxpacket;
	if (count > bufferspace) {
		usb_dbg("%s buffer overflow\n", ep->ep.name);
		req->req.status = -EOVERFLOW;
		count = bufferspace;
	}
    usb_dbg("vread_fifo csr:0x%08x reqdma :0x%08x",csr,req->req.dma);
    
    setup_udc_dma(ep,0,(u32)req->req.dma+req->req.actual,count);

	req->req.actual += count;
	is_done = (count < ep->ep.maxpacket);
	if (count == bufferspace)
		is_done = 1;

	usb_dbg("%s %p out/%d%s\n", ep->ep.name, &req->req, count,
			is_done ? " (done)" : "");
	if (is_done)
    {   
        csr&=(~USB_RxCSR_RxPktRdy);
        __raw_writew(csr,creg);
		done(ep, req, 0);
    }
    else
    {
        csr&=(~USB_RxCSR_RxPktRdy);
        __raw_writew(csr,creg);
    }
	return is_done;
}

#else
/* pull OUT packet data from the endpoint's fifo */
static int read_fifo (struct as3310_ep *ep, struct as3310_request *req)
{
	void   __iomem		*creg = ep->creg+0x6;
	void   __iomem	    *dreg = ep->udc->udp_baseaddr+USB_FIFO1;//ep->index*4+0x20;
	u16		csr;
	u8		*buf;
	unsigned int    count, bufferspace, is_done;
    unsigned char   count8,count32,end32;
    volatile unsigned int  * reg32;
    volatile unsigned char * reg8;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;

	/*
	 * there might be nothing to read if ep_queue() calls us,
	 * or if we already emptied both pingpong buffers
	 */

	csr = __raw_readw(creg);
	if ((csr & USB_RxCSR_RxPktRdy) == 0)
		return 0;
    if ((csr & USB_RxCSR_SentStall)!=0) {
        csr = __raw_readw(creg);
        csr|=USB_RxCSR_SentStall;
        __raw_writew(csr,creg);
    }

	count = __raw_readw(ep->creg+0x8);
	if (count > ep->ep.maxpacket)
		count = ep->ep.maxpacket;
	if (count > bufferspace) {
		usb_dbg("%s buffer overflow\n", ep->ep.name);
		req->req.status = -EOVERFLOW;
		count = bufferspace;
	}

    end32=count/4;
    reg32=(volatile unsigned int  *)(req->req.buf + req->req.actual);
    
    for(count32=0;count32<end32;count32++)
    {
        *(reg32++)=*((volatile unsigned int  *)dreg);
    }
    count8=count32*4;
    reg8=(volatile unsigned char  *)(reg32);
    for(;count8<count;count8++)
    {
        *(reg8++)=*((volatile unsigned char  *)dreg);
    }
	req->req.actual += count;
	is_done = (count < ep->ep.maxpacket);
	if (count == bufferspace)
		is_done = 1;

	usb_dbg("%s %p out/%d%s\n", ep->ep.name, &req->req, count,
			is_done ? " (done)" : "");
	if (is_done)
    {   
        csr&=(~USB_RxCSR_RxPktRdy);
        __raw_writew(csr,creg);
		done(ep, req, 0);
    }
    else
    {
        csr&=(~USB_RxCSR_RxPktRdy);
        __raw_writew(csr,creg);
    }
	return is_done;
}
#endif

static int write_fifo0(struct as3310_ep *ep,struct as3310_request * req)
{
    void __iomem		*creg = ep->creg+0x2;
	u16		csr = __raw_readw(creg);
	void __iomem	*dreg = ep->udc->udp_baseaddr+USB_FIFO0;
	unsigned	total, count, is_last;
    unsigned *p1;
    total = req->req.length - req->req.actual;
	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = total;
		is_last = (count < ep->ep.maxpacket) || !req->req.zero;
	}
    p1=req->req.buf+req->req.actual;
    __raw_writesb(dreg, req->req.buf + req->req.actual, count);
    req->req.actual += count;
	if (is_last)
    {

        csr|=USB_CSR0_TxPktRdy;
        __raw_writew(csr,creg);
        csr=__raw_readw(creg);
        csr|=USB_CSR0_DATAEND;
        __raw_writew(csr,creg);
		done(ep, req, 0);
    }
    else
    {
        csr|=USB_CSR0_TxPktRdy;
        __raw_writew(csr,creg);
    }
	return is_last;
}

#define MAX_RETRY 200000000
#ifdef USE_DMA
static int write_fifo(struct as3310_ep *ep, struct as3310_request *req)
{
	void __iomem		*creg = ep->creg+0x2;
	volatile u16		csr = __raw_readw(creg);
	void __iomem	*dreg = ep->udc->udp_baseaddr+ep->index*4+0x20;
    volatile unsigned int i=0,j;
	unsigned	total, count, is_last;
    unsigned char *p1;
    u8  *buf;
    u16 txepstatus;
    __raw_writew(csr|USB_TX_AUTOSET,creg);
    csr = __raw_readw(creg);
    while (((csr&USB_TX_TXPKTRDY))&&(i<MAX_RETRY)) {
        udelay(200);
        csr = __raw_readw(creg);
        i++;
    }   

    total = req->req.length - req->req.actual;
    usb_dbg("total length:%d \n",total);
	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = total;
		is_last = (count < ep->ep.maxpacket) || !req->req.zero;
	}
    buf=req->req.buf + req->req.actual;
   // printk("buf %p\n",buf);
    setup_udc_dma(ep,1,(u32)req->req.dma+ req->req.actual,count);     
    csr|=USB_TxCSR_TxPktRdy;
    __raw_writew(csr,creg);
    csr = __raw_readw(creg);
    csr&=(~(USB_TXCSR_UnderRun));
    __raw_writew(csr,creg);//ep->creg+0x2

    req->req.actual += count;
	if (is_last)
    {

	       usb_dbg("%s %p in/%d%s\n", ep->ep.name, &req->req, count,
			" (done)");
        done(ep, req, 0);
    }
   
	return is_last;
}
#else


/* load fifo for an IN packet */
static int write_fifo(struct as3310_ep *ep, struct as3310_request *req)
{
	void __iomem		*creg = ep->creg+0x2;
	volatile u16		csr = __raw_readw(creg);
	void __iomem	*dreg = ep->udc->udp_baseaddr+ep->index*4+0x20;
	unsigned	total, count, is_last,count8,count32,end32;
    volatile unsigned int  * reg32;
    volatile unsigned char * reg8;
   if(csr&USB_TX_TXPKTRDY)
       return 0;
    total = req->req.length - req->req.actual;
    usb_dbg("total length:%d \n",total);
	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = total;
		is_last = (count < ep->ep.maxpacket) || !req->req.zero;
	}
    if (csr&USB_TXCSR_UnderRun) {
       csr&=(~(USB_TXCSR_UnderRun));
       __raw_writew(csr,creg);
    }
    end32=count/4;
    reg32=(volatile unsigned int  *)(req->req.buf + req->req.actual);
    
    for(count32=0;count32<end32;count32++)
    {
        *((volatile unsigned int  *)dreg)=*(reg32++);
    }
    count8=count32*4;
    reg8=(volatile unsigned char  *)(reg32);
    for(;count8<count;count8++)
    {
        *((volatile unsigned char  *)dreg)=*(reg8++);
    }

    csr|=USB_TxCSR_TxPktRdy;
    if (!count) {
        printk("0 packet\n");
    }
    __raw_writew(csr,creg);


    req->req.actual += count;
	if (is_last)
    {

        done(ep, req, 0);
    }
	return is_last;
}

#endif 

static void nuke(struct as3310_ep *ep, int status)
{
	struct as3310_request *req;

	// terminer chaque requete dans la queue
	ep->stopped = 1;
	if (list_empty(&ep->queue))
		return;

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct as3310_request, queue);
		done(ep, req, status);
	}
}

/*-------------------------------------------------------------------------*/

static int as3310_ep_enable(struct usb_ep *_ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct as3310_ep	*ep = container_of(_ep, struct as3310_ep, ep);
	struct as3310_udc	*dev = ep->udc;
	u16		maxpacket;
	u32		tmp;
	unsigned long	flags;
    usb_dbg("as3310_ep_enable:%s\n",ep->ep.name);
	if (!_ep || !ep
			|| !desc || ep->desc
			|| _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| (maxpacket = le16_to_cpu(desc->wMaxPacketSize)) == 0
			|| maxpacket > ep->maxpacket) {
        usb_dbg("maxpacket:%d ep->maxpacket:%d desc->bDescriptorType:%d\n",maxpacket,ep->maxpacket,desc->bDescriptorType)
        usb_dbg("_ep->name :%s, ep->desc %p,desc %p\n",_ep->name,ep->desc,desc);
		usb_dbg("bad ep or descriptor\n");
		return -EINVAL;
	}
    usb_dbg("desc wMaxPacketSize %d\n",desc->wMaxPacketSize);
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		usb_dbg("bogus device state\n");
		return -ESHUTDOWN;
	}

	tmp = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	switch (tmp) {
	case USB_ENDPOINT_XFER_CONTROL:
		usb_dbg("only one control endpoint\n");
		return -EINVAL;
	case USB_ENDPOINT_XFER_INT:
		usb_dbg("USB_ENDPOINT_XFER_INT\n");
		if (maxpacket > 1024)
			goto bogus_max;
		break;
	case USB_ENDPOINT_XFER_BULK:
		usb_dbg("USB_ENDPOINT_XFER_BULK\n");
		switch (maxpacket) {
		case 8:
		case 16:
		case 32:
        case 64:
        case 128:
        case 256:
        case 512:
        case 1024:
			goto ok;
		}
bogus_max:
		usb_dbg("bogus maxpacket %d\n", maxpacket);
		return -EINVAL;
	case USB_ENDPOINT_XFER_ISOC:
		if (!ep->is_pingpong) {
			usb_dbg("iso requires double buffering\n");
			return -EINVAL;
		}
		break;
	}

ok:
	local_irq_save(flags);

	/* initialize endpoint to match this descriptor */
	ep->is_in = (desc->bEndpointAddress & USB_DIR_IN) != 0;
	ep->is_iso = (tmp == USB_ENDPOINT_XFER_ISOC);
	ep->stopped = 0;

	ep->desc = desc;
	ep->ep.maxpacket = maxpacket;

	local_irq_restore(flags);
	return 0;
}

static int as3310_ep_disable (struct usb_ep * _ep)
{
	struct as3310_ep	*ep = container_of(_ep, struct as3310_ep, ep);
	unsigned long	flags;
    usb_dbg("as3310_ep_disable %s\n",ep->ep.name);
	if (ep == &ep->udc->ep[0])
		return -EINVAL;

	local_irq_save(flags);

	nuke(ep, -ESHUTDOWN);

	/* restore the endpoint's pristine config */
	ep->desc = NULL;
	ep->ep.maxpacket = ep->maxpacket;


	local_irq_restore(flags);
	return 0;
}

/*
 * this is a PIO-only driver, so there's nothing
 * interesting for request or buffer allocation.
 */

static struct usb_request *
as3310_ep_alloc_request(struct usb_ep *_ep, unsigned int gfp_flags)
{
	struct as3310_request *req;
    //usb_dbg("as3310_ep_alloc_request ep:%d\n",ep->index);
	req = kzalloc(sizeof (struct as3310_request), gfp_flags);
	if (!req)
		return NULL;
    req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void as3310_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct as3310_request *req;
    //usb_dbg("as3310_ep_free_request ep:%d\n",ep->index);
    req = container_of(_req, struct as3310_request, req);
	BUG_ON(!list_empty(&req->queue));
	kfree(req);

}

static void *as3310_ep_alloc_buffer(
	struct usb_ep *_ep,
	unsigned bytes,
	dma_addr_t *dma,
	gfp_t gfp_flags)
{
    void		*retval;
	struct as3310_ep	*ep;

	if (!_ep)
		return NULL;
	ep = container_of(_ep, struct as3310_ep, ep);
    *dma=DMA_ADDR_INVALID;
    //printk("alloc buffer %d\n",ep->index);
 #ifdef USE_DMA
	if (ep->index) {
    	static int	warned;
    	if (!warned && bytes < PAGE_SIZE) {
    		dev_warn(ep->udc->gadget.dev.parent,
    			"using dma_alloc_coherent for "
    			"small allocations wastes memory\n");
    		warned++;
        }
    	retval= dma_alloc_coherent(NULL,
    			bytes, dma, GFP_KERNEL);
        //printk("USB EP %d ADDR P: %p , Addr V %p\n size %d B\n",ep->index,*dma,retval,bytes);
        return retval;
	}

	retval = kmalloc(bytes, gfp_flags);
   	return retval;
 #else
    retval = kmalloc(bytes, gfp_flags);
    return retval;
 #endif
}

static DEFINE_SPINLOCK(buflock);
static LIST_HEAD(buffers);

struct free_record {
	struct list_head	list;
	struct device		*dev;
	unsigned		bytes;
	dma_addr_t		dma;
};

static void do_free(unsigned long ignored)
{
	spin_lock_irq(&buflock);
	while (!list_empty(&buffers)) {
		struct free_record	*buf;

		buf = list_entry(buffers.next, struct free_record, list);
		list_del(&buf->list);
		spin_unlock_irq(&buflock);

		dma_free_coherent(buf->dev, buf->bytes, buf, buf->dma);

		spin_lock_irq(&buflock);
	}
	spin_unlock_irq(&buflock);
}

static struct tasklet_struct deferred_free;

static DECLARE_TASKLET(deferred_free, do_free, 0);

static void as3310_ep_free_buffer(
	struct usb_ep *_ep,
	void *buf,
	dma_addr_t dma,
	unsigned bytes)
{
  	/* free memory into the right allocator */
	if (dma != DMA_ADDR_INVALID) {
        struct as3310_ep		*ep;
       	struct free_record	*rec = buf;
       	unsigned long		flags;
       
       	ep = container_of(_ep, struct as3310_ep, ep);
           usb_dbg("as3310_ep_free_buffer ep:%d\n",ep->index);
       	rec->dev = ep->udc->gadget.dev.parent;
       	rec->bytes = bytes;
       	rec->dma = dma;
       
       	spin_lock_irqsave(&buflock, flags);
       	list_add_tail(&rec->list, &buffers);
       	tasklet_schedule(&deferred_free);
       	spin_unlock_irqrestore(&buflock, flags);
	} else
		kfree(buf);
}

static int as3310_ep_queue(struct usb_ep *_ep,
			struct usb_request *_req, gfp_t gfp_flags)
{
	struct as3310_request	*req;
    struct as3310_ep	*ep;
    struct as3310_udc	*dev;
	int			status;
	unsigned long		flags;
    local_irq_save(flags);
	req = container_of(_req, struct as3310_request, req);
	ep  = container_of(_ep, struct as3310_ep, ep);
    if (!_req || !_req->complete
			|| !_req->buf || !list_empty(&req->queue)) {
		usb_dbg("invalid request\n");
		return -EINVAL;
	}
	if (!_ep || (!ep->desc && ep->ep.name != ep0name)) {
        if(!_ep)
        {
            usb_dbg("!_ep\n");
        }
        if(!ep->desc)
        {
            usb_dbg("!ep->desc\n");
        }
        usb_dbg("ep name:%s\n",ep->ep.name);
		usb_dbg("invalid ep\n");
		return -EINVAL;
	}
	dev = ep->udc;
    if(!dev)
    {
        usb_dbg("!dev\n");
    }
    if(!dev->driver)
    {
        usb_dbg("!dev->driver\n");
    }
    if(dev->gadget.speed == USB_SPEED_UNKNOWN)
    {
        usb_dbg("dev->gadget.speed:%d  == USB_SPEED_UNKNOWN:%d\n",dev->gadget.speed,USB_SPEED_UNKNOWN);
    }

	if (!dev || !dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
	usb_dbg("invalid device\n");
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	
    usb_dbg("as3310_ep_queue:flag1\n");

    usb_dbg("as3310_ep_queue:flag2\n");
	/* try to kickstart any empty and idle queue */
    usb_dbg("ep_stop %d\n",ep->stopped);
    usb_dbg("list_empty(&ep->queue):%d--%p\n",list_empty(&ep->queue),&ep->queue);
	if (list_empty(&ep->queue) && !ep->stopped) {
		int	is_ep0;
   
		is_ep0 = (ep->ep.name == ep0name);
		if (is_ep0) {
			if (!dev->req_pending) {
				status = -EINVAL;
				goto done;
			}
			/*
			 * defer changing CONFG until after the gadget driver
			 * reconfigures the endpoints.
			 */
			if (dev->wait_for_config_ack) {
    		    usb_dbg("toggle config\n");
                }		  
			if (req->req.length == 0) {
                ep0_in_status:
				PACKET("ep0 in/status\n");
                usb_dbg("ep0 in/status\n");
				status = 0;
 				dev->req_pending = 0;
				goto done;
			}
		}

		if (ep->is_in)
        {
            if(is_ep0)
            {
                 
                 status = write_fifo0(ep, req);
            }
            else
            {
                status = write_fifo(ep, req);
            }
        }
		else {
            if(is_ep0)
            {
                
                status = read_fifo0(ep, req);
            }
            else
            {
                
                status = read_fifo(ep, req);
            }
 
			/* IN/STATUS stage is otherwise triggered by irq */
			if (status && is_ep0)
				goto ep0_in_status;
		}
	} else
		status = 0;

	if (req && !status) {
        usb_dbg("req:%p && !status%d\n",req,status);
		list_add_tail (&req->queue, &ep->queue);
	}
done:
	local_irq_restore(flags);
	return (status < 0) ? status : 0;
}

static int as3310_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct as3310_ep	*ep;
	struct as3310_request	*req;
    ep = container_of(_ep, struct as3310_ep, ep);
    usb_dbg("as3310_ep_dequeue:%s\n",ep->ep.name);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req)
    {
        req = container_of(_req, struct as3310_request, req);
        done(ep, req, -ECONNRESET);
        return 0;
    }
	done(ep, req, -ECONNRESET);
	return 0;
}

static int as3310_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct as3310_ep	*ep = container_of(_ep, struct as3310_ep, ep);
	struct as3310_udc	*udc = ep->udc;
	u32 __iomem	*creg;
	u32		csr;
	unsigned long	flags;
	int		status = 0;

	if (!_ep || ep->is_iso || !ep->udc->clocked)
		return -EINVAL;

	creg = ep->creg;
	local_irq_save(flags);

	csr = __raw_readl(creg);

	/*
	 * fail with still-busy IN endpoints, ensuring correct sequencing
	 * of data tx then stall.  note that the fifo rx bytecount isn't
	 * completely accurate as a tx bytecount.
	 */
	if (ep->is_in && (!list_empty(&ep->queue) || (csr >> 16) != 0))
		status = -EAGAIN;
	else {
		csr |= CLR_FX;
		csr &= ~SET_FX;
		if (value) {
			csr |= AS3310_UDP_FORCESTALL;
			usb_dbg("halt %s\n", ep->ep.name);
		} else {
			as3310_usbc_writeb(udc, AS3310_UDP_RST_EP, ep->int_mask);
			as3310_usbc_writeb(udc, AS3310_UDP_RST_EP, 0);
			csr &= ~AS3310_UDP_FORCESTALL;
		}
		__raw_writel(csr, creg);
	}

	local_irq_restore(flags);
	return status;
}



/*-------------------------------------------------------------------------*/

static int as3310_get_frame(struct usb_gadget *gadget)
{
	struct as3310_udc *udc = to_udc(gadget);

	if (!to_udc(gadget)->clocked)
		return -EINVAL;
	return as3310_usbc_readb(udc, AS3310_UDP_FRM_NUM);
}

static int as3310_wakeup(struct usb_gadget *gadget)
{
	struct as3310_udc	*udc = to_udc(gadget);
	u32		glbstate;
	int		status = -EINVAL;
	unsigned long	flags;

	usb_dbg("%s\n", __FUNCTION__ );
	local_irq_save(flags);

	if (!udc->clocked || !udc->suspended)
		goto done;

	/* NOTE:  some "early versions" handle ESR differently ... */

	glbstate = as3310_usbc_readb(udc, AS3310_UDP_GLB_STAT);
	if (!(glbstate & AS3310_UDP_ESR))
		goto done;
	glbstate |= AS3310_UDP_ESR;
	as3310_usbc_writeb(udc, AS3310_UDP_GLB_STAT, glbstate);

done:
	local_irq_restore(flags);
	return status;
}

/* reinit == restore inital software state */   
//ok now!
static void udc_reinit(struct as3310_udc *udc)
{
	u32 i;

    usb_dbg("reinit\n");
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct as3310_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		ep->desc = NULL;
		ep->stopped = 0;
		ep->fifo_bank = 0;
		ep->ep.maxpacket = ep->maxpacket;
		ep->creg = (void __iomem *) udc->udp_baseaddr + AS3310_UDP_CSR(i);
		// initialiser une queue par endpoint
		INIT_LIST_HEAD(&ep->queue);
	}

    //reopen the interrupts
    as3310_usbc_writeb(udc,USB_IntrUSBE, 0xf7);
    as3310_usbc_writew(udc,USB_IntrTxE,  0xffff);
    as3310_usbc_writew(udc,USB_IntrRxE,  0xfffe);
}

static void stop_activity(struct as3310_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	int i;

	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->suspended = 0;

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct as3310_ep *ep = &udc->ep[i];
		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}
	if (driver)
		driver->disconnect(&udc->gadget);
    ep_init(udc);
	udc_reinit(udc);
}



/*
 * activate/deactivate link with host; minimize power usage for
 * inactive links by cutting clocks and transceiver power.
 */
static void pullup(struct as3310_udc *udc, int is_on)
{
	if (!udc->enabled || !udc->vbus)
		is_on = 0;
	usb_dbg("%sactive\n", is_on ? "" : "in");

	if (is_on) {
	usb_dbg("reset power 1ms!\n");

    } else {
		stop_activity(udc);

    }
}

/* vbus is here!  turn everything on that's ready */
static int as3310_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct as3310_udc	*udc = to_udc(gadget);
	unsigned long	flags;

	// Vusb_dbg("vbus %s\n", is_active ? "on" : "off");
	local_irq_save(flags);
	udc->vbus = (is_active != 0);
	if (udc->driver)
		pullup(udc, is_active);
	else
		pullup(udc, 0);
	local_irq_restore(flags);
	return 0;
}

static int as3310_pullup(struct usb_gadget *gadget, int is_on)
{
	struct as3310_udc	*udc = to_udc(gadget);
	unsigned long	flags;

	local_irq_save(flags);
	udc->enabled = is_on = !!is_on;
	pullup(udc, is_on);
	local_irq_restore(flags);
	return 0;
}

static int as3310_set_selfpowered(struct usb_gadget *gadget, int is_on)
{
	struct as3310_udc	*udc = to_udc(gadget);
	unsigned long	flags;

	local_irq_save(flags);
	udc->selfpowered = (is_on != 0);
	local_irq_restore(flags);
	return 0;
}

static const struct usb_gadget_ops as3310_udc_ops = {
	.get_frame		= as3310_get_frame,
	.wakeup			= as3310_wakeup,
	.set_selfpowered	= as3310_set_selfpowered,
	.vbus_session		= as3310_vbus_session,
	.pullup			= as3310_pullup,

	/*
	 * VBUS-powered devices may also also want to support bigger
	 * power budgets after an appropriate SET_CONFIGURATION.
	 */
	// .vbus_power		= as3310_vbus_power,
};

/*-------------------------------------------------------------------------*/

static int handle_ep(struct as3310_ep *ep)
{
	struct as3310_request	*req;
	void __iomem		*rxcreg = ep->creg+0x6;
	u16	rxcsr = __raw_readw(rxcreg);
    u8  is_ep0 = (ep->ep.name == ep0name);
   // printk("h%d\n",ep->index);
    usb_dbg("handle ep:%s\ncsr:%x\n",ep->ep.name,rxcsr);
    usb_dbg("ep %p ep->que%p\n",ep,ep->queue);
    usb_dbg("list_empty(&ep->queue):%d\n",list_empty(&ep->queue));
     //printk("handle ep:epname %s ep:%p que:%p\n",ep->ep.name,ep,&ep->queue);
	if (!list_empty(&ep->queue))
    {
  
		req = list_entry(ep->queue.next,
			struct as3310_request, queue);
        usb_dbg("ep %s req is not NULL\n",ep->ep.name);
    }
	else{
        req = NULL;
        usb_dbg("ep %s req is NULL\n",ep->ep.name);
    }

	if (ep->is_in) {
        usb_dbg("ep %s is in\n",ep->ep.name);
		//TODO:
		if (req)
        {
            if(is_ep0)
            {
            
            usb_dbg("%s write_fifo0\n",ep->ep.name);
            return write_fifo0(ep, req);
            }
            else
            {
               usb_dbg("%s write_fifo\n",ep->ep.name);
               return write_fifo(ep, req);
            }
        }

	} else {
        usb_dbg("ep %s is out\n",ep->ep.name);
      	if (req && (rxcsr & USB_RX_RXPKTRDY))
        {
            if(is_ep0)
            {
                usb_dbg("%s read_fifo0\n",ep->ep.name);
                return read_fifo0(ep, req);
            }
            else
            {
                usb_dbg("%s read_fifo\n",ep->ep.name);
                return read_fifo(ep,req);
            }
        }
			
	}
	return 0;
}

static void as3310_udc_tasklet(unsigned long data)
{
    struct as3310_ep *ep = (struct as3310_ep *) data;
    handle_ep(ep);
}

union setup {
	u8			raw[8];
	struct usb_ctrlrequest	r;
};

static void handle_setup(struct as3310_udc *udc, struct as3310_ep *ep, u16 csr)
{
	void __iomem	*creg = ep->creg;
    void __iomem    *csreg= ep->creg+2;
	void __iomem	*dreg = ep->creg + (AS3310_UDP_FDR(0) - AS3310_UDP_CSR(0));
   	unsigned	rxcount, i = 0;
	u16		tmp;
	union setup	pkt;
	int		status = 0;
	/* read and ack SETUP; hard-fail for bogus packets */
    USB_disk_on=1;
	rxcount = __raw_readw(creg+0x8);
	if (likely(rxcount == 8)) {
		while (rxcount--)
        {
            pkt.raw[i++] = __raw_readb(udc->udp_baseaddr+USB_FIFO0);
        }
		if (pkt.r.bRequestType & USB_DIR_IN) {
			ep->is_in = 1;
		} else {
			ep->is_in = 0;
		}
	} else {
        csr|=USB_CSR0_SERVICEDRXRDY;
        csr|=USB_CSR0_SENDSTALL;
        __raw_writew(csr,csreg);
		status = -EINVAL;
	}

	udc->wait_for_addr_ack = 0;
	udc->wait_for_config_ack = 0;
	ep->stopped = 0;
	if (unlikely(status != 0))
		goto stall;
#define w_index		le16_to_cpu(pkt.r.wIndex)
#define w_value		le16_to_cpu(pkt.r.wValue)
#define w_length	le16_to_cpu(pkt.r.wLength)
	/*
	 * A few standard requests get handled here, ones that touch
	 * hardware ... notably for device and endpoint features.
	 */
	udc->req_pending = 1;
	csr = __raw_readw(csreg);
	switch ((pkt.r.bRequestType << 8) | pkt.r.bRequest) {
  	case ((USB_TYPE_STANDARD|USB_RECIP_DEVICE) << 8)
			| USB_REQ_SET_ADDRESS:
        usb_dbg("flag1 \n");
        as3310udc_dbg_info(udc);
    	__raw_writew(csr | USB_CSR0_SERVICEDRXRDY|USB_CSR0_DATAEND, csreg);
		udc->addr = w_value;
		udc->wait_for_addr_ack = 1;
		udc->req_pending = 0;

		return;

	case ((USB_TYPE_STANDARD|USB_RECIP_DEVICE) << 8)
			| USB_REQ_SET_CONFIGURATION:
        usb_dbg("flag2 \n");
        if (pkt.r.wValue)
            udc->wait_for_config_ack = 1;
		else
			udc->wait_for_config_ack = 0;
		if (udc->wait_for_config_ack)
			usb_dbg("wait for config\n");
		/* CONFG is toggled later, if gadget driver succeeds */
		break;

	/*
	 * Hosts may set or clear remote wakeup status, and
	 * devices may report they're VBUS powered.
	 */
	case ((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_DEVICE) << 8)
			| USB_REQ_GET_STATUS:
		tmp = (udc->selfpowered << USB_DEVICE_SELF_POWERED);
		tmp |= (1 << USB_DEVICE_REMOTE_WAKEUP);
		usb_dbg("get device status\n");
		__raw_writeb(tmp, dreg);
		__raw_writeb(0, dreg);
		goto write_in;
		/* then STATUS starts later, automatically */
	case ((USB_TYPE_STANDARD|USB_RECIP_DEVICE) << 8)
			| USB_REQ_SET_FEATURE:
		if (w_value != USB_DEVICE_REMOTE_WAKEUP)
			goto stall;
		goto succeed;
	case ((USB_TYPE_STANDARD|USB_RECIP_DEVICE) << 8)
			| USB_REQ_CLEAR_FEATURE:
		goto succeed;

	/*
	 * Interfaces have no feature settings; this is pretty useless.
	 * we won't even insist the interface exists...
	 */
	case ((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_INTERFACE) << 8)
			| USB_REQ_GET_STATUS:
		PACKET("get interface status\n");
		__raw_writeb(0, dreg);
		__raw_writeb(0, dreg);
		goto write_in;
		/* then STATUS starts later, automatically */
	case ((USB_TYPE_STANDARD|USB_RECIP_INTERFACE) << 8)
			| USB_REQ_SET_FEATURE:
	case ((USB_TYPE_STANDARD|USB_RECIP_INTERFACE) << 8)
			| USB_REQ_CLEAR_FEATURE:
		goto stall;

	/*
	 * Hosts may clear bulk/intr endpoint halt after the gadget
	 * driver sets it (not widely used); or set it (for testing)
	 */
	case ((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_ENDPOINT) << 8)
			| USB_REQ_GET_STATUS:
		tmp = w_index & USB_ENDPOINT_NUMBER_MASK;
		ep = &udc->ep[tmp];
		if (tmp > NUM_ENDPOINTS || (tmp && !ep->desc))
			goto stall;

		if (tmp) {
			if ((w_index & USB_DIR_IN)) {
				if (!ep->is_in)
					goto stall;
			} else if (ep->is_in)
				goto stall;
		}
		PACKET("get %s status\n", ep->ep.name);
		tmp = 0;
		__raw_writeb(tmp, udc->udp_baseaddr+USB_FIFO0);
		__raw_writeb(0, udc->udp_baseaddr+USB_FIFO0);
		goto write_in;
		/* then STATUS starts later, automatically */
	case ((USB_TYPE_STANDARD|USB_RECIP_ENDPOINT) << 8)
			| USB_REQ_SET_FEATURE:
		tmp = w_index & USB_ENDPOINT_NUMBER_MASK;
		ep = &udc->ep[tmp];
		if (w_value != USB_ENDPOINT_HALT || tmp > NUM_ENDPOINTS)
			goto stall;
		if (!ep->desc || ep->is_iso)
			goto stall;
		if ((w_index & USB_DIR_IN)) {
			if (!ep->is_in)
				goto stall;
		} else if (ep->is_in)
			goto stall;
		goto succeed;
	case ((USB_TYPE_STANDARD|USB_RECIP_ENDPOINT) << 8)
			| USB_REQ_CLEAR_FEATURE:
		tmp = w_index & USB_ENDPOINT_NUMBER_MASK;
		ep = &udc->ep[tmp];
		if (w_value != USB_ENDPOINT_HALT || tmp > NUM_ENDPOINTS)
			goto stall;
		if (tmp == 0)
			goto succeed;
		if (!ep->desc || ep->is_iso)
			goto stall;
		if ((w_index & USB_DIR_IN)) {
			if (!ep->is_in)
				goto stall;
		} else if (ep->is_in)
			goto stall;
		if (!list_empty(&ep->queue))
			handle_ep(ep);
		goto succeed;
	}
#undef w_value
#undef w_index
#undef w_length

	/* pass request up to the gadget driver */
    csr|=USB_CSR0_SERVICEDRXRDY;
    __raw_writew(csr,csreg);
	if (udc->driver)
    {
       status = udc->driver->setup(&udc->gadget, &pkt.r);
    }
	else
		status = -ENODEV;
	if (status < 0) {
stall:
		usb_dbg("req %02x.%02x protocol STALL; stat %d\n",
				pkt.r.bRequestType, pkt.r.bRequest, status);
		udc->req_pending = 0;
	}
	return;

succeed:
	PACKET("ep0 in/status\n");
write_in:
    csr |= AS3310_UDP_TXPKTRDY;
	__raw_writel(csr, csreg);
	udc->req_pending = 0;
	return;
}

static void handle_ep0(struct as3310_udc *udc)
{
	struct  as3310_ep		*ep0 = &udc->ep[0];
	void    __iomem		    *creg = ep0->creg+0x2;
	u16     csr = __raw_readw(creg);
	struct as3310_request	*req;
	if (unlikely(csr & USB_CSR0_SENTSTALL)) {
		nuke(ep0, -EPROTO);
		udc->req_pending = 0;
        csr=csr&(~USB_CSR0_SENTSTALL);
        __raw_writew(csr, creg);
		csr = __raw_readw(creg);
	}
	if (csr & USB_CSR0_RxPktRdy) {
		nuke(ep0, 0);
		udc->req_pending = 0;
		handle_setup(udc, ep0, csr);
		return;
	}
	if (list_empty(&ep0->queue))
    {
        req = NULL;
    }
	else
    {
        req = list_entry(ep0->queue.next, struct as3310_request, queue);
    }
    if (udc->wait_for_addr_ack) {
        udc->wait_for_addr_ack = 0;
        mdelay(2);
        __raw_writeb(udc->addr,udc->udp_baseaddr);
        usb_dbg("address %d\n", udc->addr);
        }
}

static void vbus_err_isr(void)
{


}

static irqreturn_t as3310_udc_dma(int irq,void *_udc)
{
    struct as3310_udc		*udc = _udc;
    as3310_usbc_readw(udc,USB_DMA_INTR);
    complete(&(udc->complete_dma));
    return IRQ_HANDLED;
}

static irqreturn_t as3310_udc_irq (int irq, void *_udc)
{
	struct as3310_udc		*udc = _udc;
    u8          bytetmp;
	u8 status,DEVCTL_reg;
    u16 rxepstatus,txepstatus;
    usb_dbg("as3310_udc_irq \n");
	status    =as3310_usbc_readb(udc,USB_IntrUSB);
    rxepstatus=as3310_usbc_readw(udc,USB_IntrRx);
    txepstatus=as3310_usbc_readw(udc,USB_IntrTx);
    DEVCTL_reg=as3310_usbc_readb(udc,USB_DevCtl);


        if (status & USB_IntrUSB_Resume) {
            bytetmp=as3310_usbc_readb(udc,USB_Power);
            bytetmp&=(~(1<<2));
            as3310_usbc_writeb(udc,USB_Power,bytetmp);
        }

        if (status & USB_IntrUSB_SessReq) {
            if ((!(DEVCTL_reg&(1<<7)))&&(DEVCTL_reg&(1<<0))) {
                usb_dbg("session req\n");
            }
            
        }

        if (status & USB_IntrUSB_VBusErr)
        {
            if ((!(DEVCTL_reg&(1<<7)))&&(DEVCTL_reg&(1<<0))) {
                usb_dbg("Vbus error\n");
                vbus_err_isr();
            }
        }

        if (status & USB_IntrUSB_Suspend) {
            if(!(DEVCTL_reg&(1<<2)))
            {
                udc_reinit(udc);
                USB_disk_on=0;
            }
        }

        if (status & USB_IntrUSB_Conn) {
            if(DEVCTL_reg&2)
            {
                usb_dbg("connect\n");
            }
        }

        if (status &USB_IntrUSB_Discon) {
            usb_dbg("disconnect\n");
                #ifdef CONFIG_USB_HIGH_SPEED
                as3310_usbc_writeb(udc,USB_Power,0x00);
                mdelay(1);
                as3310_usbc_writeb(udc,USB_Power,0x60);
                #endif

        }

        if (status & USB_IntrUSB_SOF) {
            usb_dbg("sof\n");
        }
		/* USB reset irq:  not maskable */
		if (status & USB_IntrUSB_Reset) {
			usb_dbg("reset\n");
			udc->addr = 0;
			stop_activity(udc);
            #ifdef CONFIG_USB_HIGH_SPEED
            udc->gadget.speed = USB_SPEED_HIGH; 
            #else
            udc->gadget.speed = USB_SPEED_FULL;
            #endif
			udc->suspended = 0;
		  /*  as3310_usbc_writel(udc, AS3310_UDP_CSR(0),nt to switch out of slow clock
			 * mode into normal mode.
			 */
			if (udc->driver && udc->driver->resume)
				udc->driver->resume(&udc->gadget);

		/* endpoint IRQs are cleared by handling them */
		} else {
			int		i;
			unsigned	mask = 1;
			struct as3310_ep	*ep = &udc->ep[1];
			if (txepstatus & mask)
            {
                as3310_usbc_writeb(udc,USB_Index,0);
            	handle_ep0(udc);
            }
			for (i = 1; i < NUM_ENDPOINTS; i++) {                
				mask <<= 1;
				if (txepstatus & mask)
                {
                    handle_ep(ep);
                }
                ep++;
                if (rxepstatus & mask)
                {
                    handle_ep(ep);
                }
                ep++;
			}
		}
	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/
//ok now
static void nop_release(struct device *dev)
{
	/* nothing to free */
}


static struct usb_ep_ops as3310_ep_ops = {
	.enable		= as3310_ep_enable,
	.disable	= as3310_ep_disable,
	.alloc_request	= as3310_ep_alloc_request,
	.free_request	= as3310_ep_free_request,
	//.alloc_buffer	= as3310_ep_alloc_buffer,
	//.free_buffer	= as3310_ep_free_buffer,
	.queue		= as3310_ep_queue,
	.dequeue	= as3310_ep_dequeue,
	.set_halt	= as3310_ep_set_halt,
	// there's only imprecise fifo status reporting
};

//ok now
 struct as3310_udc controller = {
	.gadget = {
		.ops	= &as3310_udc_ops,
		.ep0	= &controller.ep[0].ep,        
		.name	= driver_name,
        #ifdef CONFIG_USB_HIGH_SPEED
        .is_dualspeed=1,
        #endif
		.dev	= {
			.bus_id = "gadget",
			.release = nop_release,
		}
	},
	.ep[0] = {
		.ep = {
			.name	= ep0name,
			.ops	= &as3310_ep_ops,
		},
        .index      = 0,
		.udc		= &controller,
		.maxpacket	= 64,
		.int_mask	= 1 << 0,
	},
	.ep[1] = {
		.ep = {
			.name	= "ep1in-bulk",
			.ops	= &as3310_ep_ops,
		},
        .index      = 1,
		.udc		= &controller,
		.is_pingpong	= 1,
		.maxpacket	= 512,
		.int_mask	= 1 << 1,
	},
	.ep[2] = {
		.ep = {
			.name	= "ep1out-bulk",
			.ops	= &as3310_ep_ops,
		},
        .index      = 2,
		.udc		= &controller,
		.is_pingpong	= 1,
		.maxpacket	= 512,
		.int_mask	= 1 << 2,
	},
	.ep[3] = {
		.ep = {
			/* could actually do bulk too */
			.name	= "ep2in-bulk",
			.ops	= &as3310_ep_ops,
		},
        .index      = 3,
		.udc		= &controller,
		.maxpacket	= 64,
		.int_mask	= 1 << 3,
	},
	.ep[4] = {
		.ep = {
			.name	= "ep2out-bulk",
			.ops	= &as3310_ep_ops,
		},
        .index      = 4,
		.udc		= &controller,
		.is_pingpong	= 1,
		.maxpacket	= 64,
		.int_mask	= 1 << 4,
	},
	/* ep6 and ep7 are also reserved (custom silicon might use them) */
};


/*-------------------------------------------------------------------------*/

static void as3310udc_shutdown(struct platform_device *dev)
{
	/* force disconnect on reboot */
	pullup(platform_get_drvdata(dev), 0);
}

//to be wrote
static void  as3310_hs_calibration(struct as3310_udc	*udc)
{
    u32 usb_tx;
    as3310_phy_writel(USB_TX+0x4,0x000f0000);
    as3310_phy_writel(USB_TX+0x4,0x00200000);
    mdelay(5);
    as3310_phy_writel(USB_TX+0x4,0x00000080);
    as3310_phy_writel(USB_TX+0x8,0x00000080);
    usb_tx=as3310_phy_readl(USB_TX);
    usb_dbg("DPUSB_TX:%x\n",usb_tx);
    while(((usb_tx&0x00800000)==0)&&((usb_tx&0x000f0000)!=0)){
        as3310_phy_writel(USB_TX,usb_tx-0x00010000);
        mdelay(5);
        as3310_phy_writel(USB_TX+0x4,0x00000080);
        as3310_phy_writel(USB_TX+0x8,0x00000080); 
        usb_tx=as3310_phy_readl(USB_TX);
        usb_dbg("USB_TX:%x ",usb_tx);
    }

    if(as3310_phy_readl(USB_TX)&0x00800000){
        usb_dbg("Calibration finish! 1\n");
    }else if((as3310_phy_readl(USB_TX)&0x000f0000)==0){
        usb_dbg("It has been the Maximum resistance for 45DP!.\n");
    }
    as3310_phy_writel(USB_TX+0x8,0x00200000);


    as3310_phy_writel(USB_TX+0x4,0x00000f00);
    as3310_phy_writel(USB_TX+0x4,0x00002000);
    mdelay(5);
    as3310_phy_writel(USB_TX+0x4,0x00000080);
    as3310_phy_writel(USB_TX+0x8,0x00000080);
    usb_tx=as3310_phy_readl(USB_TX);
    usb_dbg("DNUSB_TX:%x\n",usb_tx);
    while(((usb_tx&0x00800000)==0)&&((usb_tx&0x00000f00)!=0)){
        as3310_phy_writel(USB_TX,usb_tx-0x00000100);
        mdelay(5);
        as3310_phy_writel(USB_TX+0x4,0x00000080);
        as3310_phy_writel(USB_TX+0x8,0x00000080);  
        usb_tx=as3310_phy_readl(USB_TX);
        usb_dbg("USB_TX:%x",usb_tx);
    }

    if(as3310_phy_readl(USB_TX)&0x00800000){
        usb_dbg("Calibration finish!2\n");
    }else if((as3310_phy_readl(USB_TX)&0x00000f00)==0){
        usb_dbg("It has been the Maximum resistance for 45DN.\n");
    }
    as3310_phy_writel(USB_TX+0x8,0x00002000);
    as3310_phy_writel(USB_PWD,0x0);
}


//#define CONFIG_USB_HIGH_SPEED 1
//ok now
static void as3310_init_phy(struct as3310_udc	*udc)
{



    as3310_usbc_writeb(udc,USB_Power,0x00);
    mdelay(5);
    as3310_phy_writel(USB_CLK+8,0x00000004);

    as3310_clr_u32reg(HW_USBPHY_CTRL, GENERAL_sftrst);
    as3310_clr_u32reg(HW_USBPHY_CTRL, GENERAL_clkgate);

    #ifdef CONFIG_USB_HIGH_SPEED
    as3310_phy_writel(USB_PWD+8,0x7c00);
    #else
    as3310_phy_writel(USB_PWD,0x0);
    #endif
    
    as3310_phy_writel(USB_TX,0x0);
    as3310_phy_writel(USB_RX,0x0);
    as3310_phy_writel(USB_CTRL,0x0);
    as3310_phy_writel(USB_SYSCTRL,0x00004000);
    as3310_phy_writel(USB_PHY_ANALOG,0x11);

     /* Open power control clock gate */
    as3310_writel(0x40000000,0x80044000+8);
    /* set 5V detect and OTG_CMPS */
    as3310_writel((1<<16)+(1<<4),0x80044010+4);


    #ifdef CONFIG_USB_HIGH_SPEED
    as3310_hs_calibration(udc);
    #endif
    
    //otg init
    
    //as3310_phy_writel(USB_SYSCTRL+8,0x02000000);
    #ifdef CONFIG_USB_HIGH_SPEED
    as3310_usbc_writeb(udc,USB_Power,0x60);
    #else
    as3310_usbc_writeb(udc,USB_Power,0x40);
    #endif
}

#define USB_EP_MAX_PACKET_SIZE_SETV 6
#define USB_EP_MAX_PACKET_SIZE (8<<USB_EP_MAX_FIFO_SIZE_SETV)
#define USB_EP_MAX_FIFO_SIZE_SETV      USB_EP_MAX_PACKET_SIZE_SETV  

#define USB_EP_MAX_FIFO_SIZE           USB_EP_MAX_FIFO_SIZE_SETV

#define EP0_FIFO_OFFSET 128
#define USB_EP_TX_FIFO_ADDR(i) (EP0_FIFO_OFFSET +2*(i-1)*(8<<USB_EP_MAX_FIFO_SIZE_SETV))
#define USB_EP_RX_FIFO_ADDR(i) (EP0_FIFO_OFFSET +(2*i-1)*(8<<USB_EP_MAX_FIFO_SIZE_SETV))

static void ep_init(struct as3310_udc	*udc)
{
    unsigned char i;
    for(i=1;i<5;i++)
    {
        usb_dbg("ep_init set index %d \n",i);
        as3310_usbc_writeb(udc,USB_Index,i);
        as3310_usbc_writew(udc,USB_TxMaxP,USB_EP_MAX_PACKET_SIZE);
        as3310_usbc_writew(udc,USB_RxMaxP,USB_EP_MAX_PACKET_SIZE);
        as3310_usbc_writeb(udc,USB_TxFIFOsz,USB_EP_MAX_FIFO_SIZE_SETV);
        as3310_usbc_writeb(udc,USB_RxFIFOsz,USB_EP_MAX_FIFO_SIZE_SETV);//32 bytes
        as3310_usbc_writew(udc,USB_TxFIFOadd,USB_EP_TX_FIFO_ADDR(i)>>3);
        as3310_usbc_writew(udc,USB_RxFIFOadd,USB_EP_RX_FIFO_ADDR(i)>>3);//dynamic fifosize
    }
    //TODO: set the fifo address and fifo size in this function.
}

//probe is ok now



int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	struct as3310_udc	*udc = &controller;
	int		retval;
      usb_dbg("usb gadget_register_driver!\n");
	as3310_init_phy(udc);
	ep_init(udc);
	udc_reinit(udc);
	if (!driver
			|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->setup) {
		usb_dbg("bad parameter.\n");
		return -EINVAL;
	}

	if (udc->driver) {
		usb_dbg("UDC already has a gadget driver\n");
		return -EBUSY;
	}

	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	udc->gadget.dev.driver_data = &driver->driver;
	udc->enabled = 1;
	udc->selfpowered = 1;

	retval = driver->bind(&udc->gadget);
	if (retval) {
		usb_dbg("driver->bind() returned %d\n", retval);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;
		udc->gadget.dev.driver_data = NULL;
		udc->enabled = 0;
		udc->selfpowered = 0;
		return retval;
	}
	
       
	
	local_irq_disable();
	pullup(udc, 1);
	local_irq_enable();
	udc_reinit(udc);
	usb_dbg("bound to %s\n", driver->driver.name);
    
	return 0;
}
EXPORT_SYMBOL (usb_gadget_register_driver);

//ok now
int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	struct as3310_udc *udc = &controller;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	local_irq_disable();
	udc->enabled = 0;
	as3310_usbc_writew(udc, USB_IntrTxE, 0);
      as3310_usbc_writew(udc, USB_IntrRxE, 0);
	pullup(udc, 0);
	local_irq_enable();

	driver->unbind(&udc->gadget);
	udc->driver = NULL;

	usb_dbg("unbound from %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);

static int __devinit as3310udc_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	struct as3310_udc	*udc;
	int		retval;
	struct resource	*res;
  
    //as3310_phy_writel(0x8007c058,0x40000000);
	if (pdev->num_resources != 2) {
		usb_dbg("invalid num_resources");
		return -ENODEV;
	}
    init_MUTEX(&dma_udc_lock);
	if ((pdev->resource[0].flags != IORESOURCE_MEM)
			|| (pdev->resource[1].flags != IORESOURCE_IRQ)) {
		usb_dbg("invalid resource type");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	if (!request_mem_region(res->start,
			res->end - res->start + 1,
			driver_name)) {
		usb_dbg("someone's using UDC memory\n");
		return -EBUSY;
	}

    tasklet_enable(&udc_tasklet);
	/* init software state */


	udc = &controller;
	udc->gadget.dev.parent = dev;
	udc->pdev = pdev;
	udc->enabled = 0;
	udc->udp_baseaddr = ioremap(res->start, res->end - res->start + 1);
	if (!udc->udp_baseaddr) {
		release_mem_region(res->start, res->end - res->start + 1);
		return -ENOMEM;
	}
	ep_init(udc);
	udc_reinit(udc);

	retval = device_register(&udc->gadget.dev);
	if (retval < 0)
		goto fail0;



	/* request UDC irqs */
	udc->udp_irq = platform_get_irq(pdev, 0);
      usb_dbg("usb regist irq:%d\n",udc->udp_irq);
	if (request_irq(udc->udp_irq, as3310_udc_irq,
			IRQF_SHARED, driver_name, udc)) {
		usb_dbg("request irq %d failed\n", udc->udp_irq);
		retval = -EBUSY;
		goto fail1;
	}

   if(request_irq(INT_ASAP1820_USB_DMA,as3310_udc_dma,IRQF_SHARED,driver_name,udc)){
       usb_dbg("request irq %d failed\n",INT_ASAP1820_USB_DMA);
       retval= -EBUSY;
       goto fail1;
   }

	usb_dbg("USB no VBUS detection, assuming always-on\n");
	udc->vbus = 1;

	dev_set_drvdata(dev, udc);
	device_init_wakeup(dev, 1);
	INFO("%s version %s\n", driver_name, DRIVER_VERSION);
	return 0;

fail1:
	device_unregister(&udc->gadget.dev);
fail0:
	release_mem_region(res->start, res->end - res->start + 1);
	usb_dbg("%s probe failed, %d\n", driver_name, retval);
	return retval;
}

//ok now
static int __devexit as3310udc_remove(struct platform_device *pdev)
{
	struct as3310_udc *udc = platform_get_drvdata(pdev);
	struct resource *res;

	usb_dbg("remove\n");

	if (udc->driver)
		return -EBUSY;

	pullup(udc, 0);

	device_init_wakeup(&pdev->dev, 0);
	free_irq(udc->udp_irq, udc);
	device_unregister(&udc->gadget.dev);

	iounmap(udc->udp_baseaddr);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}


//pm ok now!
#ifdef CONFIG_PM
static int as3310udc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct as3310_udc *udc = platform_get_drvdata(pdev);
	int		wake = udc->driver && device_may_wakeup(&pdev->dev);

	/* Unless we can act normally to the host (letting it wake us up
	 * whenever it has work for us) force disconnect.  Wakeup requires
	 * PLLB for USB events (signaling for reset, wakeup, or incoming
	 * tokens) and VBUS irqs (on systems which support them).
	 */
	if ((!udc->suspended && udc->addr)
			|| !wake) {
		pullup(udc, 0);
		disable_irq_wake(udc->udp_irq);
	} else
		enable_irq_wake(udc->udp_irq);
	return 0;
}

static int as3310udc_resume(struct platform_device *pdev)
{
	struct as3310_udc *udc = platform_get_drvdata(pdev);

	/* maybe reconnect to host; if so, clocks on */
	pullup(udc, 1);
	return 0;
}
#else
#define	as3310udc_suspend	NULL
#define	as3310udc_resume	NULL
#endif

static struct platform_driver as3310_udc_driver = {
	.probe		= as3310udc_probe,
	.remove		= __devexit_p(as3310udc_remove),
	.shutdown	= as3310udc_shutdown,
	.suspend	= as3310udc_suspend,
	.resume		= as3310udc_resume,
	.driver		= {
		.name	= (char *) driver_name,
		.owner	= THIS_MODULE,
	},
};

static int __devinit udc_init_module(void)
{
	return platform_driver_register(&as3310_udc_driver);
}
module_init(udc_init_module);

static void __devexit udc_exit_module(void)
{
	platform_driver_unregister(&as3310_udc_driver);
}
module_exit(udc_exit_module);

MODULE_DESCRIPTION("AS3310 udc driver");
MODULE_AUTHOR("tony");
MODULE_LICENSE("GPL");
