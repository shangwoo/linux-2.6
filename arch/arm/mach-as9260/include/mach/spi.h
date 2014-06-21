/*
 *  arch/arm/mach-as9260/include/mach/spi.h - 
 *
 *  Copyright (C) 2013 Alpscale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AS9260_SPI_HW_H
#define AS9260_SPI_HW_H


#include <mach/dma.h>
/***************************************************
 *             SPI controller releated             *
 ***************************************************/
typedef enum { NO_QUAD, DO_QUAD } QuadAbility;

/*the spi controller common data*/
struct alp_spi_info {
	unsigned int		 num_cs;	/* total chipselects */
	int			 bus_num;       /* bus number to use. */
    DMAmodule dma_module;
    DMAchannel dma_channel;
    int tx_handshake_interface;
    int rx_handshake_interface;  
    QuadAbility quadSupport;
};


#endif
