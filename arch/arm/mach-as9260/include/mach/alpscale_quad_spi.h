/*
 *  mach/alpscale_quad_spi.h - 
 *
 *  Copyright (C) 2013 Alpscale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Common 
 */

#ifndef AS3310_SPI_QUAD_H
#define AS3310_SPI_QUAD_H

#include <linux/spi/spi_bitbang.h>
#include <linux/mtd/mtd.h>

/***********************************************************************************************************/
/*identify the QUAD-USAGE, based on the controller, SPI FLASH, and CONFIG_ALPSCALE_SPI_FLASH_QUAD_FUNC*/
typedef enum { QUADDESELECTED, QUADSELECTED } QuadFunctionSwitch;
/************************************************************************************************************/



/***************************************************
 *        QUAD SPI controller specific             *
 ***************************************************/
/*the QUAD SPI controller private data structure, ugly seen here, but it must be exported to spi/spi_flash_alp9260.c for quadPinSwitch, I've no ideas.*/
struct alp9260_quad_spi {
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

    /*quad pin switchFunction*/
    void (*quadPinSwitch)(QuadFunctionSwitch);
};


typedef enum { NOQUAD, DOQUAD } QuadFunc;
#define	CMD_SIZE		4


/****************************************************
 *         SPI flash sys bin releated               *
 ****************************************************/
struct spi_flash_info {
	char		*name;

	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32		jedec_id;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		flags;
#define	SECT_4K		0x01		/* OPCODE_BE_4K works uniformly */

    QuadFunc quad;
};

struct alpSpiFlash_data {
	struct spi_device	*spi;
	struct mutex		lock;
	struct bin_attribute	bin;
    struct spi_flash_info *flashInfo;
	u8			erase_opcode;  //always OPCODE_BE_4K for the momnet.
    int sectorSize;
	u8	command[CMD_SIZE+1];  //dummy byte considered
    char *tempBuffer;            //4K Bytes buffer for read-modify-write usage.
    QuadFunctionSwitch quadFuction; //indicate whether we use the QUAD function for chip data content
    int quadForThis;        //indicate whether the current spi_message needs QUAD triggering. Status(polling) needs none. 0, no; 1, yes.
    int spiTransferQuad[2];  //indicate whether the current spi_message does QUAD. For the moment, just check two transfer, seems no need for more. 0, none; 1, yes;
    int spiTransferIndex;
};


/****************************************************
 *            SPI flash MTD releated                *
 ****************************************************/

struct mtd_flash_info {
	char		*name;

	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32		jedec_id;
	u16             ext_id;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		flags;
#define	SECT_4K		0x01		/* OPCODE_BE_4K works uniformly */

    QuadFunc quad;
};


struct asm9260_mtd_spi_flash {
	struct spi_device	*spi;
	struct mutex		lock;
	struct mtd_info		mtd;
	unsigned		partitioned:1;
	u8			erase_opcode;
	u8			command[CMD_SIZE + 1];
    QuadFunctionSwitch quadFuction; //indicate whether we use the QUAD function for chip data content
    int quadForThis;        //indicate whether the current spi_message needs QUAD triggering. Status(polling) needs none. 0, no; 1, yes.
    int spiTransferQuad[2];  //indicate whether the current spi_message does QUAD. For the moment, just check two transfer, seems no need for more. 0, none; 1, yes;
    int spiTransferIndex;
};

#endif
