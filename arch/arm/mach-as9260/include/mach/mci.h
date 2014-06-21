
/*
 *  linux/drivers/mmc/mci.h - 
 *
 *  Copyright (C) 2013 Alpscale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct clk;

//FIXME: DMA Resource management ?!
#define AS9260SDI_DMA 1
#define AS9260_RESP_ERR 1<<15
#define AS9260_RESP_TIMEOUT 1<<14
#define AS9260_DATA_TIMEOUT 1<<12
#define AS9260_DATA_CRC 1<<13
#define AS9260_RECV_TIMEOUT 1<<11
#define AS9260_RECV_OVERFLOW 1<<9

#define AS9260_SSP_ERRORS           (AS9260_RECV_OVERFLOW|AS9260_RECV_TIMEOUT|AS9260_DATA_CRC|AS9260_RESP_ERR|AS9260_RESP_TIMEOUT|AS9260_DATA_TIMEOUT)

#define AS9260_CTRL0_RUN 1<<29
#define AS9260_CTRL0_IGN_CRC 1<<26
#define AS9260_CTRL0_READ 1<<25
#define AS9260_CTRL0_DATA_XFER 1<<24
#define AS9260_CTRL0_BUS4 1<<22
#define AS9260_CTRL0_WAITIRQ 1<<21
#define AS9260_CTRL0_WAITFCMD 1<<20
#define AS9260_CTRL0_LONG_RESP 1<<19
#define AS9260_CTRL0_GETRSP 1<<17
#define AS9260_CTRL0_ENABLE 1<<16
#define AS9260_CTRL0_XFER 0xffff 

#define AS9260_SSP_CTRL1_SDIIRQ                 (1<<31)
#define AS9260_SSP_CTRL1_SDIIRQ_EN              (1<<30)
#define AS9260_SSP_CTRL1_RESP_ERRIRQ            (1<<29)
#define AS9260_SSP_CTRL1_RESP_ERRIRQ_EN         (1<<28)
#define AS9260_SSP_CTRL1_RESP_TIMEOUTIRQ        (1<<27)
#define AS9260_SSP_CTRL1_RESP_TIMEOUTIRQ_EN     (1<<26)
#define AS9260_SSP_CTRL1_DATA_TIMEOUT_IRQ       (1<<25)
#define AS9260_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN    (1<<24)
#define AS9260_SSP_CTRL1_DATA_CRCIRQ            (1<<23)
#define AS9260_SSP_CTRL1_DATA_CRCIRQ_EN         (1<<22)
#define AS9260_SSP_CTRL1_XMIT_IRQ               (1<<21)
#define AS9260_SSP_CTRL1_XMIT_IRQ_EN            (1<<20)
#define AS9260_SSP_CTRL1_RECV_IRQ               (1<<19)
#define AS9260_SSP_CTRL1_RECV_IRQ_EN            (1<<18)
#define AS9260_SSP_CTRL1_RECV_TIMEOUTIRQ        (1<<17)
#define AS9260_SSP_CTRL1_RECV_TIMEOUTIRQ_EN     (1<<16)
#define AS9260_SSP_CTRL1_RECV_OVERFLWIRQ        (1<<15)
#define AS9260_SSP_CTRL1_RECV_OVERFLWIRQ_EN     (1<<14)
#define AS9260_SSP_CTRL1_RECV_DMA_ENABLE        (1<<13)

#define AS9260_SSP_CTRL1_IRQ_ERR (AS9260_SSP_CTRL1_SDIIRQ|AS9260_SSP_CTRL1_RESP_ERRIRQ|AS9260_SSP_CTRL1_RESP_TIMEOUTIRQ|AS9260_SSP_CTRL1_DATA_TIMEOUT_IRQ|AS9260_SSP_CTRL1_DATA_CRCIRQ|AS9260_SSP_CTRL1_RECV_TIMEOUTIRQ|AS9260_SSP_CTRL1_RECV_OVERFLWIRQ)


struct as9260_mmc_data{
    u8 det_pin;
    unsigned slot_b:1;
    unsigned wire4:1;
    u8  wp_pin;
    u8  vcc_pin;
};


enum as9260sdi_waitfor {
	COMPLETION_NONE,
	COMPLETION_CMDSENT,
	COMPLETION_RSPFIN,
	COMPLETION_XFERFINISH,
	COMPLETION_XFERFINISH_RSPFIN,
};

