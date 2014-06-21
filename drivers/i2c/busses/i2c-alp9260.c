/*
 * Support for ALPSCALE 9260 I2C chip
 *
 * Copyright (c) 2013 ALPSCALE Corporation.
 * Copyright (c) 2012 Synopsys. Inc.
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
 * 
 * WARNING: Due to the controller's inability to merely send DEVICEADDRESS+R/Wbit Byte without coming data,
 * ie the I2C_SMBUS_QUICK, while I2C_SMBUS_QUICK is mainly used(see i2c_detect_address()/i2c_probe_address )
 * to seek for the presence of the slave chip(s)(attached to the releated i2c_driver through i2c_driver->
 * i2c_client_address_data), and register the present slave client(s) to the releated i2c bus admin.  
 * so our I2C driver infrastructure doesn't support i2c_driver's probe/attach_adapter's way of seeking
 * i2c_client_address_data-like clients.
 * So, we should only register the client slave chip(s) infomation througth i2c_register_board_info(), not
 * the usual releated i2c_driver's i2c_client_address_data member.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <mach/i2c.h>
#include <mach/io.h>
#include <mach/pincontrol.h>


#define DRIVER_NAME	"ALP9260-I2C"
#define VERSION		"Version 0.1"
#define PLATFORM	"ALPSCALE9260-SOC"



enum alp_i2c_status {
	STATUS_IDLE = 0,
	STATUS_READ_START,
	STATUS_READ_IN_PROGRESS,
	STATUS_READ_SUCCESS,
	STATUS_WRITE_START,
	STATUS_WRITE_SUCCESS,
	STATUS_XFER_ABORT,
	STATUS_STANDBY
};

enum alp_i2c_xfer_kind {
    SINGLE_READ = 0,
    SINGLE_WRITE,
    WRITE_ADDRESSING_THEN_READ
};

/**
 * struct alp_i2c_private	- private I2C bus data
 * @adap: core i2c layer adapter information
 * @dev: device reference for power management
 * @base: register base
 * @speed: speed mode for this port
 * @complete: completion object for transaction wait
 * @abort: reason for last abort
 * @rx_buf: pointer into working receive buffer
 * @rx_buf_len: receive buffer length 
 * @rxtriggerneedtimes: the remaining bytes to read 
 * @status: adapter state machine
 * @msg: the message we are currently processing 
 * @lock: transaction serialization 
 * @i2cmoduleindex: which I2C module is being used 
 * @irqnum: the IRQ number designated for this used I2C module 
 *
 * We allocate one of these per device we discover, it holds the core
 * i2c layer objects and the data we need to track privately.
 */
struct alp_i2c_private {
	struct i2c_adapter adap;
	struct device *dev;
	void *base; //phys
	int speed;
	struct completion complete;
	int abort;
	u8 *rx_buf;
	int rx_buf_len;
    int rxtriggerneedtimes;
	volatile enum alp_i2c_status status;
	struct i2c_msg *msg;
	struct mutex lock;
    int i2cmoduleindex;//adap->nr, which I2C module;
    int irqnum;
    enum alp_i2c_xfer_kind xferkind;
    int ackpollround;
};


#define NUM_SPEEDS		2

/*CNT registers values when ic_clk = 12Mhz*/
const uint16_t HCNT[NUM_SPEEDS] = {
		 0x34,  0x6 	
	};
const uint16_t LCNT[NUM_SPEEDS] = {
		 0x3b,  0xf 
	};


int TX_BUFFER_DEPTH = 8;	  //Depth of the transmit buffer
int RX_BUFFER_DEPTH = 8;	  //Depth of receive buffer

int ACK_POLLING_NUM = 0x100;  //mainly used for EEPROM's ACKNOWLEDGE POLLING

/* Control register */
#define IC_CON			0x00
#define SLV_DIS			(1 << 6)	/* Disable slave mode */
#define RESTART			(1 << 5)	/* Send a Restart condition */
#define	ADDR_10BIT		(1 << 4)	/* 10-bit addressing */
#define	STANDARD_MODE		(1 << 1)	/* standard mode */
#define FAST_MODE		(2 << 1)	/* fast mode */
#define HIGH_MODE		(3 << 1)	/* high speed mode */
#define	MASTER_EN		(1 << 0)	/* Master mode */

/* Target address register */
#define IC_TAR			0x04
#define IC_TAR_10BIT_ADDR	(1 << 12)	/* 10-bit addressing */
#define IC_TAR_SPECIAL		(1 << 11)	/* Perform special I2C cmd */
#define IC_TAR_GC_OR_START	(1 << 10)	/* 0: Gerneral Call Address */
						/* 1: START BYTE */
/* Slave Address Register */
#define IC_SAR			0x08		/* Not used in Master mode */

/* High Speed Master Mode Code Address Register */
#define IC_HS_MADDR		0x0c

/* Rx/Tx Data Buffer and Command Register */
#define IC_DATA_CMD		0x10
#define IC_RD			(1 << 8)	/* 1: Read 0: Write */

/* Standard Speed Clock SCL High Count Register */
#define IC_SS_SCL_HCNT		0x14

/* Standard Speed Clock SCL Low Count Register */
#define IC_SS_SCL_LCNT		0x18

/* Fast Speed Clock SCL High Count Register */
#define IC_FS_SCL_HCNT		0x1c

/* Fast Spedd Clock SCL Low Count Register */
#define IC_FS_SCL_LCNT		0x20

/* High Speed Clock SCL High Count Register */
#define IC_HS_SCL_HCNT		0x24

/* High Speed Clock SCL Low Count Register */
#define IC_HS_SCL_LCNT		0x28

/* Interrupt Status Register */
#define IC_INTR_STAT		0x2c		/* Read only */
#define R_GEN_CALL		(1 << 11)
#define R_START_DET		(1 << 10)
#define R_STOP_DET		(1 << 9)
#define R_ACTIVITY		(1 << 8)
#define R_RX_DONE		(1 << 7)
#define	R_TX_ABRT		(1 << 6)
#define R_RD_REQ		(1 << 5)
#define R_TX_EMPTY		(1 << 4)
#define R_TX_OVER		(1 << 3)
#define	R_RX_FULL		(1 << 2)
#define	R_RX_OVER		(1 << 1)
#define R_RX_UNDER		(1 << 0)

/* Interrupt Mask Register */
#define IC_INTR_MASK		0x30		/* Read and Write */
#define M_GEN_CALL		(1 << 11)
#define M_START_DET		(1 << 10)
#define M_STOP_DET		(1 << 9)
#define M_ACTIVITY		(1 << 8)
#define M_RX_DONE		(1 << 7)
#define	M_TX_ABRT		(1 << 6)
#define M_RD_REQ		(1 << 5)
#define M_TX_EMPTY		(1 << 4)
#define M_TX_OVER		(1 << 3)
#define	M_RX_FULL		(1 << 2)
#define	M_RX_OVER		(1 << 1)
#define M_RX_UNDER		(1 << 0)

/* Raw Interrupt Status Register */
#define IC_RAW_INTR_STAT	0x34		/* Read Only */
#define GEN_CALL		(1 << 11)	/* General call */
#define START_DET		(1 << 10)	/* (RE)START occurred */
#define STOP_DET		(1 << 9)	/* STOP occurred */
#define ACTIVITY		(1 << 8)	/* Bus busy */
#define RX_DONE			(1 << 7)	/* Not used in Master mode */
#define	TX_ABRT			(1 << 6)	/* Transmit Abort */
#define RD_REQ			(1 << 5)	/* Not used in Master mode */
#define TX_EMPTY		(1 << 4)	/* TX FIFO <= threshold */
#define TX_OVER			(1 << 3)	/* TX FIFO overflow */
#define	RX_FULL			(1 << 2)	/* RX FIFO >= threshold */
#define	RX_OVER			(1 << 1)	/* RX FIFO overflow */
#define RX_UNDER		(1 << 0)	/* RX FIFO empty */

/* Receive FIFO Threshold Register */
#define IC_RX_TL		0x38

/* Transmit FIFO Treshold Register */
#define IC_TX_TL		0x3c

/* Clear Combined and Individual Interrupt Register */
#define IC_CLR_INTR		0x40
#define CLR_INTR		(1 << 0)

/* Clear RX_UNDER Interrupt Register */
#define IC_CLR_RX_UNDER		0x44
#define CLR_RX_UNDER		(1 << 0)

/* Clear RX_OVER Interrupt Register */
#define IC_CLR_RX_OVER		0x48
#define CLR_RX_OVER		(1 << 0)

/* Clear TX_OVER Interrupt Register */
#define IC_CLR_TX_OVER		0x4c
#define CLR_TX_OVER		(1 << 0)

#define IC_CLR_RD_REQ		0x50

/* Clear TX_ABRT Interrupt Register */
#define IC_CLR_TX_ABRT		0x54
#define CLR_TX_ABRT		(1 << 0)
#define IC_CLR_RX_DONE		0x58

/* Clear ACTIVITY Interrupt Register */
#define IC_CLR_ACTIVITY		0x5c
#define CLR_ACTIVITY		(1 << 0)

/* Clear STOP_DET Interrupt Register */
#define IC_CLR_STOP_DET		0x60
#define CLR_STOP_DET		(1 << 0)

/* Clear START_DET Interrupt Register */
#define IC_CLR_START_DET	0x64
#define CLR_START_DET		(1 << 0)

/* Clear GEN_CALL Interrupt Register */
#define IC_CLR_GEN_CALL		0x68
#define CLR_GEN_CALL		(1 << 0)

/* Enable Register */
#define IC_ENABLE		0x6c
#define ENABLE			(1 << 0)

/* Status Register */
#define IC_STATUS		0x70		/* Read Only */
#define STAT_SLV_ACTIVITY	(1 << 6)	/* Slave not in idle */
#define STAT_MST_ACTIVITY	(1 << 5)	/* Master not in idle */
#define STAT_RFF		(1 << 4)	/* RX FIFO Full */
#define STAT_RFNE		(1 << 3)	/* RX FIFO Not Empty */
#define STAT_TFE		(1 << 2)	/* TX FIFO Empty */
#define STAT_TFNF		(1 << 1)	/* TX FIFO Not Full */
#define STAT_ACTIVITY		(1 << 0)	/* Activity Status */

/* Transmit FIFO Level Register */
#define IC_TXFLR		0x74		/* Read Only */
#define TXFLR			(1 << 0)	/* TX FIFO level */

/* Receive FIFO Level Register */
#define IC_RXFLR		0x78		/* Read Only */
#define RXFLR			(1 << 0)	/* RX FIFO level */

/* Transmit Abort Source Register */
#define IC_TX_ABRT_SOURCE	0x80
#define ABRT_SLVRD_INTX		(1 << 15)
#define ABRT_SLV_ARBLOST	(1 << 14)
#define ABRT_SLVFLUSH_TXFIFO	(1 << 13)
#define	ARB_LOST		(1 << 12)
#define ABRT_MASTER_DIS		(1 << 11)
#define ABRT_10B_RD_NORSTRT	(1 << 10)
#define ABRT_SBYTE_NORSTRT	(1 << 9)
#define ABRT_HS_NORSTRT		(1 << 8)
#define ABRT_SBYTE_ACKDET	(1 << 7)
#define ABRT_HS_ACKDET		(1 << 6)
#define ABRT_GCALL_READ		(1 << 5)
#define ABRT_GCALL_NOACK	(1 << 4)
#define ABRT_TXDATA_NOACK	(1 << 3)
#define ABRT_10ADDR2_NOACK	(1 << 2)
#define ABRT_10ADDR1_NOACK	(1 << 1)
#define ABRT_7B_ADDR_NOACK	(1 << 0)

/* Enable Status Register */
#define IC_ENABLE_STATUS	0x9c
#define IC_EN			(1 << 0)	/* I2C in an enabled state */

/* Component Parameter Register 1*/
#define IC_COMP_PARAM_1		0xf4
#define APB_DATA_WIDTH		(0x3 << 0)



/**
 * alp_i2c_disable - Disable I2C controller
 * @adap: struct pointer to i2c_adapter
 *
 * Return Value:
 * 0		success
 * -ETIMEDOUT	if i2c cannot be disabled within the given time
 *
 * I2C bus state should be checked prior to disabling the hardware. If bus is
 * not in idle state, an errno is returned. Write "0" to IC_ENABLE to disable
 * I2C controller.
 */
static int alp_i2c_disable(struct i2c_adapter *adap)
{
	struct alp_i2c_private *i2c = i2c_get_adapdata(adap);
	int err = 0;
	int count = 0;
	int ret1, ret2;
	static const u16 delay[NUM_SPEEDS] = {100, 25};

	/* Set IC_ENABLE to 0 */
	as3310_writel(0, i2c->base + IC_ENABLE);

	/* Check if device is busy */
	dev_dbg(&adap->dev, "mrst i2c disable\n");
	while ((ret1 = as3310_readl(i2c->base + IC_ENABLE_STATUS) & 0x1)
		|| (ret2 = as3310_readl(i2c->base + IC_STATUS) & 0x1)) {
		udelay(delay[i2c->speed]);
		as3310_writel(0, i2c->base + IC_ENABLE);
		dev_dbg(&adap->dev, "i2c is busy, count is %d speed %d\n",
			count, i2c->speed);
		if (count++ > 10) {
			err = -ETIMEDOUT;
			break;
		}
	}

	/* Clear all interrupts */
	as3310_readl(i2c->base + IC_CLR_INTR);
	as3310_readl(i2c->base + IC_CLR_STOP_DET);
	as3310_readl(i2c->base + IC_CLR_START_DET);
	as3310_readl(i2c->base + IC_CLR_ACTIVITY);
	as3310_readl(i2c->base + IC_CLR_TX_ABRT);
	as3310_readl(i2c->base + IC_CLR_RX_OVER);
	as3310_readl(i2c->base + IC_CLR_RX_UNDER);
	as3310_readl(i2c->base + IC_CLR_TX_OVER);
	as3310_readl(i2c->base + IC_CLR_RX_DONE);
	as3310_readl(i2c->base + IC_CLR_GEN_CALL);

	/* Disable all interupts */
	as3310_writel(0x0, i2c->base + IC_INTR_MASK);

	return err;
}


/**
 * alp_i2c_pininit - i2c pin assignment.
 * 
 */
void alp_i2c_pininit(struct alp_i2c_private *i2c)
{
#if (defined(CONFIG_SOC_CAMERA_ASM9260)||defined(CONFIG_SOUND_ASM9260_SND_OSS)||defined(CONFIG_TOUCHSCREEN_FT5X06)||defined(CONFIG_SND_ASM9260_SOC_IIS_WM8731))
   if(i2c->i2cmoduleindex == 1) {   
	  set_pin_mux(17,2,7);
	  set_pin_mux(17,3,7);
#else
#if !defined(CONFIG_FB_ASM9260)
   if(i2c->i2cmoduleindex == 1) {  
      set_pin_mux(1,6,7);
	  set_pin_mux(1,7,7);  
#else     
    if(i2c->i2cmoduleindex == 1) {
        ;  
#error LCD and I2C1-EEPROM PINMUX conflict
#endif
#endif
   }else{
	  set_pin_mux(9, 0, 7);
	  set_pin_mux(9, 1, 7);
   }
}


/**
 * alp_i2c_hwinit - Initialize the I2C hardware registers
 * @i2c: private I2C bus data
 *
 * This function will be called in alp_i2c_probe() before device
 * registration. 
 *
 * Return Values:
 * 0		success 
 * -EINVAL  improper setting
 * -ETIMEDOUT	i2c cannot be disabled
 *
 * I2C should be disabled prior to other register operation. If failed, an
 * errno is returned. Mask and Clear all interrpts, this should be done at
 * first.  Set common registers which will not be modified during normal
 * transfers, including: control register, FIFO threshold and clock freq.
 * Check APB data width at last.
 */
static int alp_i2c_hwinit(struct alp_i2c_private *i2c)
{
	int err;
    int sourceclock;//the ic_clk(ie, pclk, due to "IC_CLK_TYPE = 0" configuration), in khz unit 
    uint16_t hcnt,lcnt;     

    if(i2c->i2cmoduleindex == 1) {    
       as3310_writel(1<<23, HW_PRESETCTRL0+4);
       as3310_writel(1<<23, HW_AHBCLKCTRL0+4);
    }else{        
       as3310_writel(1<<22, HW_PRESETCTRL0+4);
       as3310_writel(1<<22, HW_AHBCLKCTRL0+4);
    }

    /*pin assignment--I2C0/1*/
    alp_i2c_pininit(i2c);

	/* Disable i2c first */
	err = alp_i2c_disable(&i2c->adap);
	if (err)
		return err;

	/*
	 * Setup clock frequency and speed mode
	 * Enable restart condition,
	 * enable master FSM, disable slave FSM,
	 * use target address when initiating transfer
	 */

	as3310_writel((i2c->speed + 1) << 1 | SLV_DIS | RESTART | MASTER_EN,
		i2c->base + IC_CON);

    /*timing setting*/
    if( as3310_readl(HW_MAINCLKSEL)&0x01 ) {
        sourceclock = as3310_readl(HW_SYSPLLCTRL)*1000;
    }else{
        sourceclock = 12000;
    }

    dev_dbg(&i2c->adap.dev, "sourceclock: %dkHZ\n", sourceclock);

    if( as3310_readl(HW_CPUCLKDIV)&0xff ) {
        sourceclock /= (int)(as3310_readl(HW_CPUCLKDIV)&0xff);
    }else{
        return -EINVAL;
    }

    dev_dbg(&i2c->adap.dev, "CPUCLKDIV:%d\n", (int)(as3310_readl(HW_CPUCLKDIV)&0xff));

    if( as3310_readl(HW_SYSAHBCLKDIV)&0xff ) {
        sourceclock /= (int)(as3310_readl(HW_SYSAHBCLKDIV)&0xff);
    }else{
        return -EINVAL;
    }

    dev_dbg(&i2c->adap.dev, "SYSAHBCLKDIV:%d\n", (int)(as3310_readl(HW_SYSAHBCLKDIV)&0xff));

    if(sourceclock >= 12000){
	  	hcnt = (uint16_t)(((int)HCNT[i2c->speed]+8)*(sourceclock/12000)-8);
        lcnt = (uint16_t)(((int)LCNT[i2c->speed]+1)*(sourceclock/12000)-1);
	 }
	 else {
	    if(i2c->speed == 1)
			i2c->speed = 0;	  //according to DW_apb_i2c databook, when ic_clk <= 6Mhz, it's impossible to set 400kHz

		if( (sourceclock==6000)||(sourceclock==4000)||(sourceclock==3000) ){
	 		hcnt =  (uint16_t)(((int)HCNT[i2c->speed]+8)/(12000/sourceclock)-8);
			lcnt =  (uint16_t)(((int)LCNT[i2c->speed]+1)/(12000/sourceclock)-1);
		}else{
            dev_err(&i2c->adap.dev, "I2C clock setting failed!\n");
			return -EINVAL;	 //according to DW_apb_i2c databook, when ic_clk <= 2Mhz, it's impossible to set even 100kHz
		}
	 } 

    dev_dbg(&i2c->adap.dev, "hcnt:%d,lcnt:%d\n", hcnt, lcnt);

	as3310_writel(hcnt, i2c->base + (IC_SS_SCL_HCNT + (i2c->speed << 3)));
	as3310_writel(lcnt, i2c->base + (IC_SS_SCL_LCNT + (i2c->speed << 3)));

	/* Set tranmit & receive FIFO threshold to zero */
	as3310_writel(0x0, i2c->base + IC_RX_TL);
	as3310_writel(0x0, i2c->base + IC_TX_TL);

	return 0;
}

/**
 * alp_i2c_func - Return the supported three I2C operations.
 * @adapter: i2c_adapter struct pointer 
 *  
 * note: I2C_FUNC_10BIT_ADDR should also be supported, but I 
 * don't know the detail, so not this time. 
 */
static u32 alp_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/**
 * alp_i2c_address_neq - To check if the addresses for different i2c messages are equal. 
 * @p1: first i2c_msg
 * @p2: second i2c_msg
 *
 * Return Values:
 * 0	 if addresses are equal
 * 1	 if not equal
 *
 * Within a single transfer, the I2C client may need to send its address more
 * than once. So a check if the addresses match is needed.
 */
static inline bool alp_i2c_address_neq(const struct i2c_msg *p1,
				       const struct i2c_msg *p2)
{
	if (p1->addr != p2->addr)
		return 1;
	if ((p1->flags ^ p2->flags) & I2C_M_TEN)
		return 1;
	return 0;
}

/**
 * alp_i2c_abort - To handle transfer abortions and print error
 * messages. 
 * @i2c: i2c_adapter private info pointer
 *
 * By reading register IC_TX_ABRT_SOURCE, various transfer errors can be
 * distingushed. At present, no circumstances have been found out that
 * multiple errors would be occurred simutaneously, so we simply use the
 * register value directly.
 *
 * At last the error bits are cleared. (Note clear ABRT_SBYTE_NORSTRT bit need
 * a few extra steps)
 */
static void alp_i2c_abort(struct alp_i2c_private *i2c)
{
	/* Read about source register */
	int abort = i2c->abort;
    struct i2c_adapter *adap = &i2c->adap;

	/* Single transfer error check:
	 * According to databook, TX/RX FIFOs would be flushed when
	 * the abort interrupt occurred.
	 */
	if (abort & ABRT_MASTER_DIS)
		dev_err(&adap->dev, "i2c abort-ABRT_MASTER_DIS: "
               "initiate master operation with master mode disabled.\n");
	if (abort & ABRT_10B_RD_NORSTRT)
		dev_err(&adap->dev, "i2c abort-ABRT_10B_RD_NORSTRT: "
               "RESTART disabled and master sent READ cmd in 10-bit addressing.\n");

	if (abort & ABRT_SBYTE_NORSTRT) {
		dev_err(&adap->dev, "i2c abort-ABRT_SBYTE_NORSTRT: "
               "RESTART disabled and user is trying to send START byte.\n");
		as3310_writel(~ABRT_SBYTE_NORSTRT, i2c->base + IC_TX_ABRT_SOURCE);
		as3310_writel(RESTART, i2c->base + IC_CON);
		as3310_writel(~IC_TAR_SPECIAL, i2c->base + IC_TAR);
	}

	if (abort & ABRT_SBYTE_ACKDET)
		dev_err(&adap->dev, "i2c abort-ABRT_SBYTE_ACKDET: "
               "START byte was not acknowledged.\n");
	if (abort & ABRT_TXDATA_NOACK)
		dev_dbg(&adap->dev, "i2c abort-ABRT_TXDATA_NOACK: "
			   "No acknowledgement received from slave.\n");
	if (abort & ABRT_10ADDR2_NOACK)
		dev_dbg(&adap->dev, "i2c abort-ABRT_10ADDR2_NOACK: "
                "The 2nd address byte of the 10-bit address was not acknowledged.\n");
	if (abort & ABRT_10ADDR1_NOACK)
		dev_dbg(&adap->dev, "i2c abort-ABRT_10ADDR1_NOACK: "
                "The 1st address byte of 10-bit address was not acknowledged.\n");
	if (abort & ABRT_7B_ADDR_NOACK)
		dev_dbg(&adap->dev, "i2c abort-ABRT_7B_ADDR_NOACK: "
                "I2C slave device not acknowledged.\n");

	/* Clear TX_ABRT bit */
	as3310_readl(i2c->base + IC_CLR_TX_ABRT);
	i2c->status = STATUS_XFER_ABORT;
}

/**
 * xfer_read - Internal function to implement master read transfer.
 * @adap: i2c_adapter struct pointer
 * @buf: buffer in i2c_msg
 * @length: number of bytes to be read
 *
 * Return Values:
 * 0		if the read transfer succeeds
 * -ETIMEDOUT	if read timeout
 * -EIO	 if a transfer abort occurred
 *
 * For every byte, a "READ" command will be loaded into IC_DATA_CMD prior to
 * data transfer. The actual "read" operation will be performed if an RX_FULL
 * interrupt occurred.
 *
 * Note there may be two interrupt signals captured, one should read
 * IC_RAW_INTR_STAT to separate between errors and actual data.
 */
static int xfer_read(struct i2c_adapter *adap, unsigned char *buf, int length, enum alp_i2c_xfer_kind readkind)
{
	struct alp_i2c_private *i2c = i2c_get_adapdata(adap);
	int i;
	int err;
    char *sourceaddr;


	INIT_COMPLETION(i2c->complete);

	as3310_readl(i2c->base + IC_CLR_INTR);
	as3310_writel(0x0040, i2c->base + IC_INTR_MASK);

    /*Internal addressign first, linke eeprom "Random Read"*/
    if(readkind == WRITE_ADDRESSING_THEN_READ) {
        sourceaddr = buf;
        i2c->status = STATUS_WRITE_START;
        for( i=0; (i<length)&&(i2c->status == STATUS_WRITE_START); i++ ) {
            /*never send more data beyond the TX buffer limit*/
            if( (as3310_readl(i2c->base+IC_TXFLR)&((TX_BUFFER_DEPTH<<1)-1))==TX_BUFFER_DEPTH ) {
                i--;                
            }else{
                as3310_writel((unsigned int)(*sourceaddr++), i2c->base + IC_DATA_CMD);
			if(i==0)
			{				
				as3310_writel(0x0050, i2c->base + IC_INTR_MASK);
			}
            }
        }
        
        /*Then, wait for TX_EMPTY*/
        while( (i2c->status == STATUS_WRITE_START)&&( (as3310_readl(i2c->base+IC_TXFLR)&((TX_BUFFER_DEPTH<<1)-1))!=0 ) ) {
            ;
        }

        while( (as3310_readl(i2c->base+IC_STATUS)&STAT_ACTIVITY)||(i2c->status == STATUS_WRITE_START) ) {
           ;
        }

        if(i2c->status == STATUS_XFER_ABORT) {
            return -EIO;
        }

        i2c->msg++;
        buf = i2c->msg->buf;
        length = i2c->msg->len;
    }


	i2c->status = STATUS_READ_START;
	as3310_writel(0x0044, i2c->base + IC_INTR_MASK);

    /*just trigger the first FIFO read, transfer RX_BUFFER_DEPTH or all, the remaining finished in i2c_isr_read*/
    i2c->rxtriggerneedtimes = length;
    i2c->rxtriggerneedtimes -= ( (length>RX_BUFFER_DEPTH)?RX_BUFFER_DEPTH:length );
    for( i=0; (i<RX_BUFFER_DEPTH)&&(i<length); i++) {
        as3310_writel(IC_RD, i2c->base + IC_DATA_CMD);
    }

	i2c->status = STATUS_READ_START;
	err = wait_for_completion_interruptible_timeout(&i2c->complete, HZ); //wait for 1 second
	if (!err) {
		dev_err(&adap->dev, "I2C-read: " "Timeout for ACK from I2C slave device\n");
		alp_i2c_hwinit(i2c);
		return -ETIMEDOUT;
	}
    
    while( as3310_readl(i2c->base+IC_STATUS)&STAT_ACTIVITY ) {
        ;
    }

	if (i2c->status == STATUS_READ_SUCCESS)
		return 0;
	else
		return -EIO;

}

/**
 * xfer_write - Internal function to implement master write transfer.
 * @adap: i2c_adapter struct pointer
 * @buf: buffer in i2c_msg
 * @length: number of bytes to be read
 *
 * Return Values:
 * 0	if the read transfer succeeds
 * -EIO	if a transfer abort occurred
 *
 * For every byte, a "WRITE" command will be loaded into IC_DATA_CMD prior to
 * data transfer.
 *
 * Note there may be two interrupt signals captured, one should read
 * IC_RAW_INTR_STAT to separate between errors and actual data. 
 * Due to Tx FIFO is limited to TX_BUFFER_DEPTH, and there is 
 * likely no way to implement interrupt-drived write, DMA may 
 * do, but not this time. So, we only use the ugly POLLING 
 * write. 
 */
static int xfer_write(struct i2c_adapter *adap,
		      unsigned char *buf, int length)
{
	struct alp_i2c_private *i2c = i2c_get_adapdata(adap);
	int i = length;
    char *sourceaddr = buf;


    /*our conroller can't implement I2C_SMBUS_QUICK, due to the inability to merely send DEVICEADDRESS+R/Wbit Byte without coming data */
    if(length == 0) {
        return -EOPNOTSUPP;
    }

	as3310_readl(i2c->base + IC_CLR_INTR);
	as3310_writel(0x0040, i2c->base + IC_INTR_MASK);

	i2c->status = STATUS_WRITE_START;
    
    while( (i--)&&(i2c->status == STATUS_WRITE_START) ) {
        /*never send more data beyond the TX buffer limit*/
        if( (as3310_readl(i2c->base+IC_TXFLR)&((TX_BUFFER_DEPTH<<1)-1))==TX_BUFFER_DEPTH ) {
            i++;
            continue;
        }
        as3310_writel((unsigned int)(*sourceaddr++), i2c->base + IC_DATA_CMD);
		
	   if((i+1)==length)
	   {		
		 as3310_writel(0x0050, i2c->base + IC_INTR_MASK);
	   }
	   
    }


    /*Then, wait for TX_EMPTY*/
    while( (i2c->status == STATUS_WRITE_START)&&( (as3310_readl(i2c->base+IC_TXFLR)&((TX_BUFFER_DEPTH<<1)-1))!=0 ) ) {
        ;
    }


    while( (as3310_readl(i2c->base+IC_STATUS)&STAT_ACTIVITY)||(i2c->status == STATUS_WRITE_START) ) {
        ;
    }
    
        
    if(i2c->status==STATUS_XFER_ABORT) {
        /*Consider the EEPROM's ACKNOWLEDGE POLLING need*/
        if(i2c->abort & ABRT_7B_ADDR_NOACK) {
            dev_dbg(&adap->dev, "I2C-WRITE: not ACKed for slave address, chip may busy\n");            
            return -EAGAIN;
        }else{        
		    alp_i2c_hwinit(i2c);
            dev_err(&adap->dev, "I2C-WRITE: io error\n");
		    return -EIO;
        }
	} else {
		return 0;
	}
}

static int alp_i2c_setup(struct i2c_adapter *adap,  struct i2c_msg *pmsg)
{
	struct alp_i2c_private *i2c = i2c_get_adapdata(adap);
	int err;
	u32 reg;
	u32 bit_mask;
	u32 mode;

	/* Disable device first */
	err = alp_i2c_disable(adap);
	if (err) {
		dev_err(&adap->dev,
			"Cannot disable i2c controller, timeout\n");
		return err;
	}

	mode = (1 + i2c->speed) << 1;
	/* set the speed mode */
	reg = as3310_readl(i2c->base + IC_CON);
	if ((reg & 0x06) != mode) {
		dev_dbg(&adap->dev, "set mode %d\n", i2c->speed);
		as3310_writel((reg & ~0x6) | mode, i2c->base + IC_CON);
	}

	reg = as3310_readl(i2c->base + IC_CON);
	/* use 7-bit addressing */
	if (pmsg->flags & I2C_M_TEN) {
		if ((reg & ADDR_10BIT) != ADDR_10BIT) {
			dev_dbg(&adap->dev, "set i2c 10 bit address mode\n");
			as3310_writel(reg | ADDR_10BIT, i2c->base + IC_CON);
		}
	} else {
		if ((reg & ADDR_10BIT) != 0x0) {
			dev_dbg(&adap->dev, "set i2c 7 bit address mode\n");
			as3310_writel(reg & ~ADDR_10BIT, i2c->base + IC_CON);
		}
	}
	/* enable restart conditions */
	reg = as3310_readl(i2c->base + IC_CON);
	if ((reg & RESTART) != RESTART) {
		dev_dbg(&adap->dev, "enable restart conditions\n");
		as3310_writel(reg | RESTART, i2c->base + IC_CON);
	}

	/* enable master FSM */
	reg = as3310_readl(i2c->base + IC_CON);
	dev_dbg(&adap->dev, "ic_con reg is 0x%x\n", reg);
	as3310_writel(reg | MASTER_EN, i2c->base + IC_CON);
	if ((reg & SLV_DIS) != SLV_DIS) {
		dev_dbg(&adap->dev, "enable master FSM\n");
		as3310_writel(reg | SLV_DIS, i2c->base + IC_CON);
		dev_dbg(&adap->dev, "ic_con reg is 0x%x\n", reg);
	}

	/* use target address when initiating transfer */
	reg = as3310_readl(i2c->base + IC_TAR);
	bit_mask = IC_TAR_SPECIAL | IC_TAR_GC_OR_START;

	if ((reg & bit_mask) != 0x0) {
		dev_dbg(&adap->dev,
	 "WR: use target address when intiating transfer, i2c_tx_target\n");
		as3310_writel(reg & ~bit_mask, i2c->base + IC_TAR);
	}

	/* set target address to the I2C slave address */
	dev_dbg(&adap->dev,
		"set target address to the I2C slave address, addr is %x\n",
			pmsg->addr);
	as3310_writel(pmsg->addr | (pmsg->flags & I2C_M_TEN ? IC_TAR_10BIT_ADDR : 0),
		i2c->base + IC_TAR);

    as3310_writel(0x0, i2c->base + IC_RX_TL);
	as3310_writel(0x0, i2c->base + IC_TX_TL);

	/* Enable I2C controller */
	as3310_writel(ENABLE, i2c->base + IC_ENABLE);

	return 0;
}

/**
 * alp_i2c_xfer - Main master transfer routine.
 * @adap: i2c_adapter struct pointer
 * @pmsg: i2c_msg struct pointer
 * @num: number of i2c_msg, never more than TWO, only support R,
 *     W, W+R.
 *
 * Return Values:
 * +		number of messages transferred 
 * -1       not ready 
 * -ETIMEDOUT	If cannot disable I2C controller or read IC_STATUS
 * -EINVAL	If the address in i2c_msg is invalid
 *
 * This function will be registered in i2c-core and exposed to external
 * I2C clients.
 * 1. Disable I2C controller
 * 2. Unmask three interrupts: RX_FULL, TX_EMPTY, TX_ABRT
 * 3. Check if address in i2c_msg is valid
 * 4. Enable I2C controller
 * 5. Perform real transfer (call xfer_read or xfer_write)
 * 6. Wait until the current transfer is finished (check bus state)
 * 7. Mask and clear all interrupts 
 *  
 *  
 * NOTE: We should not limit the i2c_msg number to TWO for each 
 * time, but if so, users must be fully aware of the timing
 * condition between READ/WRITE, writing may fail because of no 
 * waiting, reading may fail(guess) because of waiting too long,
 * and the STOP accurs, breaking the RE-start condition. 
 *  
 * A litte due to the conroller's inability to manually send or 
 * not send the STOP signal. 
 *  
 * By the way, TWO msgs is enough, a testimony is that all 
 * smbus-xfer can be converted within TWO i2c_msg, see 
 * i2c_smbus_xfer_emulated for more. 
 */ 
static int alp_i2c_xfer(struct i2c_adapter *adap,
			 struct i2c_msg *pmsg,
			 int num)
{
	struct alp_i2c_private *i2c = i2c_get_adapdata(adap);
	int i, err = 0;


	/* if number of messages equal 0*/
	if (num == 0)
		return 0;


	mutex_lock(&i2c->lock);
	dev_dbg(&adap->dev, "alp_i2c_xfer, process %d msg(s)\n", num);
	dev_dbg(&adap->dev, "slave address is %x\n", pmsg->addr);


	if (i2c->status != STATUS_IDLE) {
		dev_err(&adap->dev, "Adapter %d in transfer/standby\n",
								adap->nr);
		mutex_unlock(&i2c->lock);
		return -1;
	}


	for (i = 1; i < num; i++) {
		/* Message address equal? */
		if (unlikely(alp_i2c_address_neq(&pmsg[0], &pmsg[i]))) {
			dev_err(&adap->dev, "Invalid address in msg[%d]\n", i);
			mutex_unlock(&i2c->lock);
			return -EINVAL;
		}
	}

	if (alp_i2c_setup(adap, pmsg)) {
		mutex_unlock(&i2c->lock);
		return -EINVAL;
	}


    if(num == 1) {
        if(pmsg[0].flags & I2C_M_RD) {
            i2c->xferkind = SINGLE_READ;
        }else{
            i2c->xferkind = SINGLE_WRITE;
        }
    }else if(num == 2){
        if( ((pmsg[0].flags & I2C_M_RD)==0)&&(pmsg[1].flags & I2C_M_RD) ) {
            i2c->xferkind = WRITE_ADDRESSING_THEN_READ;
        }else{            
			return -EINVAL;
        }
    }else{
        dev_err(&adap->dev, "usage(like, ioctl) error: i2c_msg num never more than TWO!\n");
        dev_err(&adap->dev, "usage supported: single I2C_M_RD/I2C_M_WR, combined I2C_M_WR+I2C_M_RD pair.\n");
        return -EINVAL;
    }


#if 0
    printk( "IC_CON:%8x\n", as3310_readl(i2c->base+IC_CON) );
    printk( "IC_TAR:%8x\n", as3310_readl(i2c->base+IC_TAR) );
    printk( "IC_SS_SCL_HCNT:%8x\n", as3310_readl(i2c->base+IC_SS_SCL_HCNT) );
    printk( "IC_SS_SCL_LCNT:%8x\n", as3310_readl(i2c->base+IC_SS_SCL_LCNT) );
    printk( "IC_INTR_STAT:%8x\n", as3310_readl(i2c->base+IC_INTR_STAT) );
    printk( "IC_INTR_MASK:%8x\n", as3310_readl(i2c->base+IC_INTR_MASK) );
    printk( "IC_RX_TL:%8x\n", as3310_readl(i2c->base+IC_RX_TL) );
    printk( "IC_TX_TL:%8x\n", as3310_readl(i2c->base+IC_TX_TL) );
    printk( "IC_ENABLE:%8x\n", as3310_readl(i2c->base+IC_ENABLE) );

    printk( "IC_STATUS:%8x\n", as3310_readl(i2c->base+IC_STATUS) );
    printk( "IC_TXFLR:%8x\n", as3310_readl(i2c->base+IC_TXFLR) );
    printk( "IC_RXFLR:%8x\n", as3310_readl(i2c->base+IC_RXFLR) );
    printk( "IC_ENABLE_STATUS:%8x\n", as3310_readl(i2c->base+IC_ENABLE_STATUS) );

#endif

    i2c->ackpollround = 0;

	for (i = 0; i < num; i++) {
		i2c->msg = pmsg;
		i2c->status = STATUS_IDLE;
		/* Read or Write */
		if ( (pmsg->flags & I2C_M_RD)&&(i2c->xferkind == SINGLE_READ) ) {
			dev_dbg(&adap->dev, "single I2C_M_RD\n");
			err = xfer_read(adap, pmsg->buf, pmsg->len, SINGLE_READ);
		} else if( ( (pmsg->flags&I2C_M_RD)==0 )&&(i2c->xferkind == SINGLE_WRITE) ) {
			dev_dbg(&adap->dev, "single I2C_M_WR\n");
            do{
                mdelay(2);
                err = xfer_write(adap, pmsg->buf, pmsg->len);
                mdelay(4);
            }while ( (err==-EAGAIN) && ((i2c->ackpollround++)<ACK_POLLING_NUM) );
            if(i2c->ackpollround>=ACK_POLLING_NUM) {
                err = -EIO;
                dev_err(&adap->dev, "I2C write polling timeout!\n");
            }
		}else{
			dev_dbg(&adap->dev, "I2C_M_WR+I2C_M_RD pair\n");
			err = xfer_read(adap, pmsg->buf, pmsg->len, WRITE_ADDRESSING_THEN_READ );
            i++;
        }
		if (err < 0){
			break;
        }
		dev_dbg(&adap->dev, "msg[%d] transfer complete\n", i);
		pmsg++;		/* next message */
	}

	/* Disable interrupts */
	as3310_writel(0x0000, i2c->base + IC_INTR_MASK);
	/* Clear all interrupts */
	as3310_readl(i2c->base + IC_CLR_INTR);

	i2c->status = STATUS_IDLE;
	mutex_unlock(&i2c->lock);
    
    if(err<0) {
        return err;
    }else
        return num;	
}


static void i2c_isr_read(struct alp_i2c_private *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	int rx_num;
	u32 len;
	u8 *buf;


	if (!(msg->flags & I2C_M_RD))
		return;

	if (i2c->status != STATUS_READ_IN_PROGRESS) {
		len = msg->len;
		buf = msg->buf;
	} else {
		len = i2c->rx_buf_len;
		buf = i2c->rx_buf;
	}

	rx_num = as3310_readl(i2c->base + IC_RXFLR);


	for (; len > 0 && rx_num > 0; len--, rx_num--)
		*buf++ = (u8)as3310_readl(i2c->base + IC_DATA_CMD);

	if (len > 0) {
		i2c->status = STATUS_READ_IN_PROGRESS;
		i2c->rx_buf_len = len;
		i2c->rx_buf = buf;
	} else
		i2c->status = STATUS_READ_SUCCESS;
    

    if(i2c->rxtriggerneedtimes>0) {
        while( (i2c->rxtriggerneedtimes>0)&&((as3310_readl(i2c->base+IC_RXFLR)&((RX_BUFFER_DEPTH<<1)-1))<RX_BUFFER_DEPTH)&&((i2c->rx_buf_len-i2c->rxtriggerneedtimes)<RX_BUFFER_DEPTH) ) {
            as3310_writel(IC_RD, i2c->base + IC_DATA_CMD);
            i2c->rxtriggerneedtimes -= 1;
        }
    }

	return;
}

static irqreturn_t alp_i2c_isr(int this_irq, void *dev)
{
    int abortnotify = 0;
	struct alp_i2c_private *i2c = dev;
	u32 stat = as3310_readl(i2c->base + IC_INTR_STAT);


    if (!stat)
		return IRQ_NONE;


	dev_dbg(&i2c->adap.dev, "%s, stat = 0x%x\n", __func__, stat);
	stat &= 0x54;

	if (i2c->status != STATUS_WRITE_START &&
	    i2c->status != STATUS_READ_START &&
	    i2c->status != STATUS_READ_IN_PROGRESS)
		goto err;

	if (stat & TX_ABRT)
		i2c->abort = as3310_readl(i2c->base + IC_TX_ABRT_SOURCE);

	as3310_readl(i2c->base + IC_CLR_INTR);

	if (stat & TX_ABRT) {
        if( (i2c->xferkind==WRITE_ADDRESSING_THEN_READ)&&(i2c->status == STATUS_READ_START) ) {
            abortnotify = 1;
        }
		alp_i2c_abort(i2c);
		goto exit;
	}

	if (stat & RX_FULL) {
		i2c_isr_read(i2c);
		goto exit;
	}

	if (stat & TX_EMPTY) {
		if (as3310_readl(i2c->base + IC_STATUS) & 0x4)
			i2c->status = STATUS_WRITE_SUCCESS;
	}

exit:
	if (i2c->status == STATUS_READ_SUCCESS ||
	    i2c->status == STATUS_WRITE_SUCCESS ||
	    i2c->status == STATUS_XFER_ABORT) {
		/* Clear all interrupts */
		as3310_readl(i2c->base + IC_CLR_INTR);
		/*disable interrupts */
		as3310_writel(0, i2c->base + IC_INTR_MASK);


        if( (i2c->status == STATUS_READ_SUCCESS)||(abortnotify == 1) ) {
            complete(&i2c->complete);
        }	
       	
	}
err:
	return IRQ_HANDLED;
}

static struct i2c_algorithm alp_i2c_algorithm = {
	.master_xfer	= alp_i2c_xfer,
	.functionality	= alp_i2c_func,
};



/**
 * alp_i2c_probe - I2C controller initialization routine
 * @pdev: I2C controller related platformdevice.
 *
 * Return Values:
 * 0		success
 * -ENODEV	If cannot allocate platform resource
 * -ENOMEM	If the register base remapping failed, or
 *		if kzalloc failed
 *
 * Initialization steps: 
 * 1. Request for device memory region 
 * 2. Fill in the struct members of alp_i2c_private 
 * 3. Call alp_i2c_hwinit() for hardware initialization 
 * 4. Register the IRQ line
 * 5. Register I2C adapter in i2c-core 
 */
static int alp_i2c_probe(struct platform_device *pdev)
{
    struct as9260_i2c_hw_data *i2chwdata;
	struct resource *res;
	struct alp_i2c_private *mrst;	
	int err, busnum;
    int irq;

	dev_dbg(&pdev->dev, "Get into probe function for I2C\n");

	/* Determine the address of the I2C area */    
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENODEV;
		goto exit;
	}

	if (!request_mem_region(res->start, res->end-res->start, pdev->name)) {
		err = -EBUSY;
		goto exit;
	}

	/* Allocate the per-device data structure, alp_i2c_private */
	mrst = kzalloc(sizeof(struct alp_i2c_private), GFP_KERNEL);
	if (mrst == NULL) {
		dev_err(&pdev->dev, "can't allocate I2C bus private\n");
		err = -ENOMEM;
		goto fail1;
	}

    mrst->i2cmoduleindex = pdev->id;
	/* Initialize struct members */
	snprintf(mrst->adap.name, sizeof(mrst->adap.name),
		"Alpscale 9260 I2C-%d at 0x:%8x", pdev->id, IO_ADDRESS(res->start));
	mrst->adap.owner = THIS_MODULE;
	mrst->adap.algo = &alp_i2c_algorithm;
	mrst->adap.dev.parent = &pdev->dev;
	mrst->dev = &pdev->dev;
	mrst->base = (void *)res->start;
	mrst->abort = 0;
	mrst->rx_buf_len = 0;
	mrst->status = STATUS_IDLE;

    i2chwdata = pdev->dev.platform_data;
	mrst->speed = i2chwdata->speedmode; //I2C speed set here;
    mrst->adap.class = i2chwdata->class; //class for client probe, user also can substitute i2c_register_board_info for this kind of client-add.

    platform_set_drvdata(pdev, mrst);
	i2c_set_adapdata(&mrst->adap, mrst);

	mrst->adap.nr = busnum = pdev->id;
	dev_dbg(&pdev->dev, "I2C%d\n", busnum);

	/* Initialize i2c controller */
	err = alp_i2c_hwinit(mrst);
	if (err < 0) {
		dev_err(&pdev->dev, "I2C interface initialization failed\n");
		goto fail2;
	}

	mutex_init(&mrst->lock);
	init_completion(&mrst->complete);

	/* Clear all interrupts */
	as3310_readl(mrst->base + IC_CLR_INTR);
    /* Disable all interrupts*/
	as3310_writel(0x0000, mrst->base + IC_INTR_MASK);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = -ENXIO;
		goto fail2;
	}
	err = request_irq(irq, alp_i2c_isr, 0,
				pdev->name, mrst);

	if (err) {          	
        dev_err(&pdev->dev, "request irq:%d fail!\n", irq);
		goto fail2;
	}else{
        mrst->irqnum = irq;
    }

	/* Adapter registration */
	err = i2c_add_numbered_adapter(&mrst->adap);
	if (err) {
		 dev_err(&pdev->dev, "Adapter %s registration failed\n",
			mrst->adap.name);
		goto fail3;
	}

	return 0;

fail3:
	free_irq(mrst->irqnum, mrst);
fail2:
    platform_set_drvdata(pdev, NULL);
	kfree(mrst);
fail1:
	release_mem_region(res->start, res->end-res->start);
exit:
	return err;
}

static int alp_i2c_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct alp_i2c_private *mrst = platform_get_drvdata(pdev);

	alp_i2c_disable(&mrst->adap);
	if (i2c_del_adapter(&mrst->adap))
		dev_err(&pdev->dev, "Failed to delete i2c adapter");

	free_irq(mrst->irqnum, mrst);
	platform_set_drvdata(pdev, NULL);
	kfree(mrst);
	release_mem_region(res->start, res->end-res->start);

    return 0;
}


static struct platform_driver alp_i2c_driver = {
	.probe		= alp_i2c_probe,
	.remove		= alp_i2c_remove,
    .driver     ={
        .name = "as9260_i2c",
        .owner = THIS_MODULE,
    },
};

static int __init alp_i2c_init(void)
{
	return platform_driver_register(&alp_i2c_driver);
}

static void __exit alp_i2c_exit(void)
{
	platform_driver_unregister(&alp_i2c_driver);
    return;
}

module_init(alp_i2c_init);
module_exit(alp_i2c_exit);

MODULE_DESCRIPTION("I2C driver for Alpscale 9260 Platform");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);
