/*
 * Support for ALPSCALE 9260 SPI flash chips
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
 * 
 * Warning:
 * 1.this time we just support the SECT_4K erase function as a compromise
 * between writing-flexibility and writing-efficiency.
 * 2.We don't support spi_transfer.speed_hz for CLOCK-switch, and 03h/0Bh switch.
 * 3.SST SPI flash write support maybe added later.
 * 4.Due to our controller's bug, when read, we must provide 4-bytes aligned address
 *   and length.
 * 5.Writing protection problems not considered yet.
 * 
 * Note: /sys/bus/spi/devices/spiX.Y/SPIContent access implementation. Use dd, hexdump commands.
 * 
 * changelog:
 *          1. 02/04/2013, created.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#if 1
#include <linux/delay.h>
#endif

#include <mach/spi.h>
#include <mach/alpscale_quad_spi.h>

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR1	0x05	/* Read status register1 */
#define OPCODE_RDSR2    0X35    /* Read status register2 */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte or 2 byte */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define OPCODE_FAST_READ_QUAD_OUTPUT    0x6b      /*Fast read quad output with dummy byte.*/
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define OPCODE_QUAD_PP  0x32    /* Quad input page program*/
#define	OPCODE_BE_4K		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

/*due to the usage of asm9260 spi controller's DMA functon, we check the CACHE-LINE*/
#define CACHE_LINE_BYTES 32


/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_COUNT	100000
#define FLASH_PAGESIZE	256
#define SECTOR_4K_SIZE  4096


/****************************************************************************/

/* NOTE: double check command sets and memory organization when you add
 * more flash chips. 
 * Warning: we always use SECT_4K erase solution when write. 
 */
static struct spi_flash_info __devinitdata alp_spi_flash_data [] = {

    /*Chingis--PM25*/
    {"pm25lv080", 0x7f9d13, 64*1024, 128, SECT_4K, NOQUAD}, //sorry, I assume it's QuadAbility due to the lack of the datasheet.

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25Q64BV", 0xef4017, 64*1024, 128, SECT_4K, DOQUAD},

    /*ESMT*/    
	{ "f25l04pa", 0x8c3013, 64*1024, 8, SECT_4K, NOQUAD},
};



/***********************************
 *   Internal helper functions     *
 ***********************************/
union {
    u8 val;
    u32 dummy[CACHE_LINE_BYTES/sizeof(u32)];
} __attribute__ ((aligned(CACHE_LINE_BYTES)))  u_sr;

/*
 * Read the status register1 or register2 indicated by statusIndex, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct alpSpiFlash_data *flash, int statusIndex)
{
	ssize_t retval;
	u8 code;

    if( statusIndex == 0 ) {
        code = OPCODE_RDSR1;
    }else{
        code = OPCODE_RDSR2;
    }

    /*must be 4 bytes aligned.*/
    if( (u32)(&u_sr.val)&0x03 ) {
        dev_err(&flash->spi->dev, "%s not 4 bytes aligned.\n", __func__);
        return -EINVAL;
    }

	retval = spi_write_then_read(flash->spi, &code, 1, &u_sr.val, 4);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return u_sr.val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct alpSpiFlash_data *flash, u8 val)
{
	flash->command[0] = OPCODE_WRSR;
	flash->command[1] = val;

	return spi_write(flash->spi, flash->command, 2);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct alpSpiFlash_data *flash)
{
	u8	code = OPCODE_WREN;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}


/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct alpSpiFlash_data *flash)
{
	int count;
	int sr;

	/* one chip guarantees max 5 msec wait here after page writes,
	 * but potentially three seconds (!) after page erase.
	 */
	for (count = 0; count < MAX_READY_WAIT_COUNT; count++) {
		if ((sr = read_sr(flash,0)) < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

		/* REVISIT sometimes sleeping would be best */
	}

	return 1;
}


/*
 * Erase one sector(SECT_4K) of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_4ksector(struct alpSpiFlash_data *flash, u32 offset)
{
    if(offset&(flash->sectorSize-1)) {
        dev_err(&flash->spi->dev, "%s not aligned at 0x%08x\n", __func__, offset);
        return 1;
    }

	dev_dbg(&flash->spi->dev, "%s %dKiB at 0x%08x\n",
			__func__,
			flash->flashInfo->sector_size*flash->flashInfo->n_sectors/ 1024, offset);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 2;

	/* Send write enable, then erase commands. */
	if(write_enable(flash))
        return 3;

	/* Set up command buffer. */
	flash->command[0] = flash->erase_opcode;
	flash->command[1] = offset >> 16;
	flash->command[2] = offset >> 8;
	flash->command[3] = offset;

	if(spi_write(flash->spi, flash->command, CMD_SIZE)){
        dev_err(&flash->spi->dev, "%s trigger failed at 0x%08x\n", __func__, offset);
        return 4;
    }else{
        return 0;
    }

}




/*Not appropriate for all QUAD SPI Flash*/
int alp_spi_flash_quad_cmd_switch(struct alpSpiFlash_data *flash, QuadFunctionSwitch quadSwitch){
    u8 quad_spi_status[4];
    struct spi_transfer t[2];
    struct spi_message m;


    quad_spi_status[0] = read_sr(flash, 0);
    quad_spi_status[1] = read_sr(flash, 1);

    if( quadSwitch == QUADSELECTED ) {
        quad_spi_status[1] = quad_spi_status[1]|0x02;
    }else{
        quad_spi_status[1] = quad_spi_status[1]&0xfd;
    }

    /*now set up the CMD*/
    spi_message_init(&m);
    memset(t, 0, (sizeof t));


	t[0].tx_buf = flash->command;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = quad_spi_status;
	t[1].len = 2;
	spi_message_add_tail(&t[1], &m);


    /* Set up the write data buffer. */
	flash->command[0] = OPCODE_WRSR;

    if(spi_sync(flash->spi, &m)){
        dev_err(&flash->spi->dev, "%s sync failed!\n", __func__);
        return -EIO;
    }


    if( (m.actual_length-1)==2 ){
        return 0;
    }else{
        return -EIO;
    }
}


/*switch to QUAD, like QUAD command sending, pin switching. */
int alp_spi_flash_quadfunc_switch(struct alpSpiFlash_data *flash, QuadFunctionSwitch funcSwitch)
{
    /* Wait until previous write command finished. */
	if (wait_till_ready(flash)) {
		return -EBUSY;
	}

    if( funcSwitch == QUADDESELECTED ) {        
        /*now, pin switch to normal*/
        ((struct alp9260_quad_spi *)spi_master_get_devdata(flash->spi->master))->quadPinSwitch(QUADDESELECTED);
    }

    /*send enable first for QUAD cmd switch*/
	if(write_enable(flash))
        return -1;
 
      
    /*now QUAD switch CMD*/
    if(alp_spi_flash_quad_cmd_switch(flash, funcSwitch))
        return -EIO;


    /* Wait till previous status modifying is over.*/
	if (wait_till_ready(flash)) {
		return -EBUSY;
	}
    
    if(funcSwitch == QUADSELECTED) {    
        /*now, pin switch to QUAD*/
        ((struct alp9260_quad_spi *)spi_master_get_devdata(flash->spi->master))->quadPinSwitch(QUADSELECTED);
    }

    return 0;
}



/********************************* 
 *  sysfs interface functions    *
 *********************************/
/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static ssize_t alp_spi_flash_bin_read(struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t from, size_t len)
{
    struct device		*dev;
	struct alpSpiFlash_data	*flash;
	struct spi_transfer t[2];
	struct spi_message m;

    dev = container_of(kobj, struct device, kobj);
    flash = dev_get_drvdata(dev);

    /*first clear the QUAD mark*/
    if(flash->quadFuction == QUADSELECTED) {
        flash->quadForThis = 0;
        flash->spiTransferIndex = 0;
        memset(flash->spiTransferQuad, 0, sizeof(flash->spiTransferQuad));
    }

    if( ((u32)buf&0x03)||((u32)len&0x03) ) {
        dev_err(dev, "%s not 4 bytes aligned.\n", __func__);
        return -EINVAL;
    }


	dev_dbg(&flash->spi->dev, "%s %s 0x%08x, len %zd\n",
			__func__, "from",
			(u32)from, len);

	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > flash->flashInfo->sector_size*flash->flashInfo->n_sectors)
		return -EINVAL;

      
    if(flash->quadFuction == QUADSELECTED) {

        if( alp_spi_flash_quadfunc_switch(flash, QUADSELECTED) ){
            dev_err(dev, "%s QUADSELECTED failed!\n", __func__);
            return -EPERM;
        }
        /*specify quad later*/
        flash->quadForThis = 0;
    }


	spi_message_init(&m);
	memset(t, 0, (sizeof t));


	t[0].tx_buf = flash->command;
    if(flash->quadFuction == QUADSELECTED) {
        t[0].len = CMD_SIZE + 1;
    }else{
        t[0].len = CMD_SIZE;
    }
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);


	mutex_lock(&flash->lock);

	/* Wait till previous write/erase is done. NO QUAD */    
    if(flash->quadFuction == QUADSELECTED) {        
        flash->quadForThis = 0;
    }
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		mutex_unlock(&flash->lock);

        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
        }
		return -EBUSY;
	}

	/* FIXME, no switch to OPCODE_FAST_READ.  It's required for higher
	 * clocks; and at this writing, every chip this driver handles
	 * supports that opcode.
	 */

	/* Set up the write data buffer. */
    if(flash->quadFuction == QUADSELECTED) {
        flash->command[0] = OPCODE_FAST_READ_QUAD_OUTPUT;
        /*now set the spiTransferQuad and spiTransferIndex according to the chosen CMD*/
        flash->spiTransferIndex = 0;
        flash->spiTransferQuad[0] = 0;
        flash->spiTransferQuad[1] = 1;
    }else{        
        flash->command[0] = OPCODE_NORM_READ;
    }
	flash->command[1] = from >> 16;
	flash->command[2] = from >> 8;
	flash->command[3] = from;
    /*dummy byte for 6Bh*/
    if(flash->quadFuction == QUADSELECTED ) {  
        flash->command[4] = 0;
    }


    if(flash->quadFuction == QUADSELECTED) {
         /*now set QUAD for the following.*/
         flash->quadForThis = 1;
         flash->spiTransferIndex = 0;
    }
	if(spi_sync(flash->spi, &m)){
        dev_err(dev, "%s failed at 0x%08x, len %zd\n", __func__, (u32)from, len);
        mutex_unlock(&flash->lock);

        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
        }
        return -EIO;
    }

    mutex_unlock(&flash->lock);

    if(flash->quadFuction == QUADSELECTED) {             
        flash->quadForThis = 0;
        if( alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED) ){
            dev_err(dev, "%s QUADDESELECTED failed!\n", __func__);
            return -EPERM;
        }
    }


    if( flash->quadFuction == QUADSELECTED ) {
        return (m.actual_length - ( CMD_SIZE + 1));
    }else{
    	return (m.actual_length - CMD_SIZE);
    }
}

/*
 * Helper function for unaligned page write access to read targeted page buffer when partial modification needed.
 */
static int alp_spi_flash_read_4ksector(struct alpSpiFlash_data *flash, char *buf, unsigned offset){
     
	struct spi_transfer t[2];
	struct spi_message m;


    /*first clear the QUAD mark*/
    if(flash->quadFuction == QUADSELECTED) {
        flash->quadForThis = 0;
        flash->spiTransferIndex = 0;
        memset(flash->spiTransferQuad, 0, sizeof(flash->spiTransferQuad));
    }

    if( (offset%flash->sectorSize)||((u32)buf&0x03)||(flash->sectorSize&0x03) ) {
        dev_err(&flash->spi->dev, "%s not aligned.\n", __func__);
        return -EINVAL;
    }
 

    if(flash->quadFuction == QUADSELECTED) {

        if( alp_spi_flash_quadfunc_switch(flash, QUADSELECTED) ){
            dev_err(&flash->spi->dev, "%s QUADSELECTED failed!\n", __func__);
            return -EPERM;
        }
        /*specify quad later*/
        flash->quadForThis = 0;
    }


	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
    if(flash->quadFuction == QUADSELECTED) {
        t[0].len = CMD_SIZE + 1;
    }else{
        t[0].len = CMD_SIZE;
    }
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = flash->sectorSize;
	spi_message_add_tail(&t[1], &m);


	/* Wait till previous write/erase is done. */
    if(flash->quadFuction == QUADSELECTED) {        
        flash->quadForThis = 0;
    }
	if (wait_till_ready(flash)) {

        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
        }
		return -EBUSY;
	}

	/* FIXME, no switch to OPCODE_FAST_READ.  It's required for higher
	 * clocks; and at this writing, every chip this driver handles
	 * supports that opcode.
	 */

	/* Set up the write data buffer. */
    if(flash->quadFuction == QUADSELECTED) {
        flash->command[0] = OPCODE_FAST_READ_QUAD_OUTPUT;
        /*now set the spiTransferQuad and spiTransferIndex according to the chosen CMD*/
        flash->spiTransferIndex = 0;
        flash->spiTransferQuad[0] = 0;
        flash->spiTransferQuad[1] = 1;
    }else{        
        flash->command[0] = OPCODE_NORM_READ;
    }
	flash->command[1] = offset >> 16;
	flash->command[2] = offset >> 8;
	flash->command[3] = offset;
    /*dummy byte for 6Bh*/
    if(flash->quadFuction == QUADSELECTED ) {  
        flash->command[4] = 0;
    }


    if(flash->quadFuction == QUADSELECTED) {
        /*now set QUAD for the following.*/
        flash->quadForThis = 1;
        flash->spiTransferIndex = 0;
    }
	if(spi_sync(flash->spi, &m)){
        dev_err(&flash->spi->dev, "%s failed at 0x%08x\n", __func__, (u32)offset);
        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
        }
        return -EIO;
    }


   if(flash->quadFuction == QUADSELECTED) {             
        flash->quadForThis = 0;
        if( alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED) ){
            dev_err(&flash->spi->dev, "%s QUADDESELECTED failed!\n", __func__);
            return -EPERM;
        }
    }

	return 0;
}


/*
 * Helper function for sector(SECT_4K) aligned programming within a sector. 
 * return positive value of the bytes to write, else, a negative error code. 
 */
static ssize_t alp_spi_flash_sector_program(struct alpSpiFlash_data *flash, char *buf, unsigned offset, size_t len){
    u32 page_offset, page_size;
    struct spi_transfer t[2];
    struct spi_message m;
    ssize_t xferedNum = 0;

	
    /*first clear the QUAD mark*/
    flash->quadForThis = 0;
    flash->spiTransferIndex = 0;
    memset(flash->spiTransferQuad, 0, sizeof(flash->spiTransferQuad));


    /*len within int range, no overflow.*/
    if( (offset%flash->sectorSize)||(len>flash->sectorSize)||(len<=0) ) {
        return -EINVAL;
    }

    /*erase the current SECT_4K sector*/
    if(erase_4ksector(flash,  offset&(~(flash->sectorSize-1))))
       return -1;


	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].len = CMD_SIZE;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	spi_message_add_tail(&t[1], &m);


	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		return -EBUSY;
	}


    if(flash->quadFuction == QUADSELECTED) {

        if( alp_spi_flash_quadfunc_switch(flash, QUADSELECTED) ){
            dev_err(&flash->spi->dev, "%s QUADSELECTED failed!\n", __func__);
            return -EPERM;
        }
        /*specify it later.*/
        flash->quadForThis = 0;
    }


	if(write_enable(flash)){
        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
        }
        return -1;
    }

    /* Set up the opcode in the write buffer. */
    if(flash->quadFuction == QUADSELECTED) {
        flash->command[0] = OPCODE_QUAD_PP;
        /*now set the spiTransferQuad and spiTransferIndex according to the chosen CMD*/
        flash->spiTransferIndex = 0;
        flash->spiTransferQuad[0] = 0;
        flash->spiTransferQuad[1] = 1;
    }else{
        flash->command[0] = OPCODE_PP;
    }
	flash->command[1] = offset >> 16;
	flash->command[2] = offset >> 8;
	flash->command[3] = offset;

    /* what page do we start with? */
	page_offset = offset % FLASH_PAGESIZE;

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= FLASH_PAGESIZE) {
		t[1].len = len;
     
        if(flash->quadFuction == QUADSELECTED) {
            /*now set QUAD for the following.*/
            flash->quadForThis = 1;
            flash->spiTransferIndex = 0;
        }

		if(spi_sync(flash->spi, &m)){
            if(flash->quadFuction == QUADSELECTED) {
                flash->quadForThis = 0;
                alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
            }
            return -EIO;
        }

		return len;
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		page_size = FLASH_PAGESIZE - page_offset;

		t[1].len = page_size;
        if(flash->quadFuction == QUADSELECTED) {
            /*now set QUAD for the following.*/
            flash->quadForThis = 1;
            flash->spiTransferIndex = 0;
        }

		if(spi_sync(flash->spi, &m)){
            if(flash->quadFuction == QUADSELECTED) {
                flash->quadForThis = 0;
                alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
            }
            return -EIO;
        }

        xferedNum =  m.actual_length - CMD_SIZE;

		/* write everything in PAGESIZE chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > FLASH_PAGESIZE)
				page_size = FLASH_PAGESIZE;

			/* write the next page to flash */
			flash->command[1] = (offset + i) >> 16;
			flash->command[2] = (offset + i) >> 8;
			flash->command[3] = (offset + i);

			t[1].tx_buf = buf + i;
			t[1].len = page_size;

            /*no need to QUAD for the following.*/
            flash->quadForThis = 0;

			if(wait_till_ready(flash)){
                if(flash->quadFuction == QUADSELECTED) {
                    flash->quadForThis = 0;
                    alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
                }
                return -EBUSY;
            }

			if(write_enable(flash)){
                if(flash->quadFuction == QUADSELECTED) {
                    flash->quadForThis = 0;
                    alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
                }
                return -1;
            }


            if(flash->quadFuction == QUADSELECTED) {
                /*now set QUAD for the following.*/
                flash->quadForThis = 1;
                flash->spiTransferIndex = 0;
            }

            if(spi_sync(flash->spi, &m)){
                if(flash->quadFuction == QUADSELECTED) {
                    flash->quadForThis = 0;
                    alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
                }
                return -EIO;
            }
			
			xferedNum += m.actual_length - CMD_SIZE;
		}
	}

    if(flash->quadFuction == QUADSELECTED) {             
        flash->quadForThis = 0;
        if( alp_spi_flash_quadfunc_switch(flash, QUADDESELECTED) ){
            dev_err(&flash->spi->dev, "%s QUADDESELECTED failed!\n", __func__);
            return -EPERM;
        }
    }

	return xferedNum;
}




/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static ssize_t alp_spi_flash_bin_write(struct kobject *kobj, struct bin_attribute *attr,
         char *buf, loff_t to, size_t len)
{
    struct device		*dev;
	struct alpSpiFlash_data	*flash;
	u32 page_offset;
    int xfercurnum = 0;
    size_t originalLen = len;


	dev = container_of(kobj, struct device, kobj);
    flash = dev_get_drvdata(dev);

	dev_dbg(dev, "%s %s 0x%08x, len %zd\n",
			__func__, "to",
			(u32)to, len);

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > flash->flashInfo->sector_size*flash->flashInfo->n_sectors)
		return -EINVAL;


	mutex_lock(&flash->lock);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		mutex_unlock(&flash->lock);
		return  -EBUSY;
	}


	/* sector alignment check */
	page_offset = to&(flash->sectorSize-1);

    if(page_offset!=0) {
        /*start address not sector aligned*/
        if(alp_spi_flash_read_4ksector(flash, flash->tempBuffer, to&(~(flash->sectorSize-1)))) {
            mutex_unlock(&flash->lock);
            return -EIO;
        }

        /*modify the targeted portion within the current sector*/
        if(page_offset+len<=flash->sectorSize) {
            xfercurnum = len;
        }else{
            xfercurnum = flash->sectorSize-page_offset;
        }
        
        memcpy(flash->tempBuffer+page_offset, buf, xfercurnum);

        if( alp_spi_flash_sector_program(flash, flash->tempBuffer, to&(~(flash->sectorSize-1)), flash->sectorSize)!=flash->sectorSize ) {
            mutex_unlock(&flash->lock);
            return -EIO;
        }

        buf+=xfercurnum;
        to+=xfercurnum;
        len-=xfercurnum;
    }

    /*Following sector aligned programming*/
    while( len ) {
        /*start address not sector aligned*/
        if(alp_spi_flash_read_4ksector(flash, flash->tempBuffer, to&(~(flash->sectorSize-1)))) {
            mutex_unlock(&flash->lock);
            return -EIO;
        }

        /*modify the targeted portion within the current sector*/
        xfercurnum = (len<=flash->sectorSize)?len:flash->sectorSize;

        memcpy(flash->tempBuffer, buf, xfercurnum);

        if( alp_spi_flash_sector_program(flash, flash->tempBuffer, to&(~(flash->sectorSize-1)), flash->sectorSize)!=flash->sectorSize ) {
            mutex_unlock(&flash->lock);
            return -EIO;
        }

        buf+=xfercurnum;
        to+=xfercurnum;
        len-=xfercurnum;
    }

	mutex_unlock(&flash->lock);
    return originalLen;
}



union{
    u8	id[3];
    u32 dummy[CACHE_LINE_BYTES/sizeof(u32)];
}__attribute__ ((aligned(CACHE_LINE_BYTES)))  u_id;;

/*
 * Some SPI FLASH chips support EXTENDED DEVICE INFO, we may consider it later.
 */
static struct spi_flash_info *__devinit jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = OPCODE_RDID;	
	u32			jedec;
	struct spi_flash_info	*info;


    if( (u32)(&u_id.id)&0x03 ) {
        dev_err(&spi->dev, "%s not 4 bytes aligned.\n", __func__);
        return NULL;
    }

	tmp = spi_write_then_read(spi, &code, 1, &u_id.id[0], 4);
	if (tmp < 0) {
		dev_dbg(&spi->dev, "error %d reading JEDEC ID\n",
			tmp);
		return NULL;
	}
	jedec = u_id.id[0];
	jedec = jedec << 8;
	jedec |= u_id.id[1];
	jedec = jedec << 8;
	jedec |= u_id.id[2];


	for (tmp = 0, info = alp_spi_flash_data;
			tmp < ARRAY_SIZE(alp_spi_flash_data);
			tmp++, info++) {
		if (info->jedec_id == jedec) {
			return info;
		}
	}
	dev_err(&spi->dev, "unrecognized JEDEC id %06x\n", jedec);
	return NULL;
}



/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports.
 */
static int __devinit alp_spi_flash_probe(struct spi_device *spi)
{
	struct flash_platform_data	*data;
	struct alpSpiFlash_data		*flash;
	struct spi_flash_info		*info;
	unsigned			i;
	int			err;
    int         doQuad = 0;

	/* Platform data helps sort out which chip type we have, as
	 * well as how this board partitions it.  If we don't have
	 * a chip ID, try the JEDEC id commands; they'll work for most
	 * newer chips, even if we don't recognize the particular chip.
	 */
    data = spi->dev.platform_data;
	if (data && data->type) {
		for (i = 0, info = alp_spi_flash_data;
				i < ARRAY_SIZE(alp_spi_flash_data);
				i++, info++) {
			if (strcmp(data->type, info->name) == 0)
				break;
		}

		/* unrecognized chip? */
		if (i == ARRAY_SIZE(alp_spi_flash_data)) {
			dev_dbg(&spi->dev, "unrecognized id %s\n",
					data->type);
			info = NULL;

		/* recognized; is that chip really what's there? */
		} else if (info->jedec_id) {
			struct spi_flash_info	*chip = jedec_probe(spi);

			if (!chip || chip != info) {
				dev_warn(&spi->dev, "found %s, expected %s\n",
						chip ? chip->name : "UNKNOWN",
						info->name);
				info = NULL;
			}
		}
	} else
		info = jedec_probe(spi);


	if (!info)
		return -ENODEV;

    
    /*now, start the SPI FLASH CHIP's QUAD function selection check!*/
    #ifdef CONFIG_ALPSCALE_SPI_FLASH_QUAD_FUNC
        if( info->quad == DOQUAD ) {
            if( ((struct alp9260_quad_spi *)spi_master_get_devdata(spi->master))->pdata->quadSupport == DOQUAD ) {
                doQuad = 1;
                dev_info(&spi->dev,"QUAD selected!\n");
            }
            else
            {
                dev_err(&spi->dev, "%s QUAD chip not supported on this controller!\n",  __func__);
                return -EOPNOTSUPP;
            }
        }else{
            dev_err(&spi->dev, "%s QUAD not supported on this chip!\n",  __func__);
            return -EOPNOTSUPP;
        }
    #else
        doQuad = 0;
    #endif


	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash){
		return -ENOMEM;
    }
    
    /*record the QUAD selection*/
    if (doQuad == 0) {
        flash->quadFuction = QUADDESELECTED;
    }else{
        flash->quadFuction = QUADSELECTED;
    }
    flash->quadForThis = 0;
        
    flash->tempBuffer = kzalloc(SECTOR_4K_SIZE, GFP_KERNEL);
    if(!flash->tempBuffer) {
		kfree(flash);
		return -ENOMEM;
    }
    
    flash->flashInfo = info;
	flash->spi = spi;
	mutex_init(&flash->lock);
	dev_set_drvdata(&spi->dev, flash);

    /*FIXME: fixed to prefer "small sector" erase for the moment. */
	flash->erase_opcode = OPCODE_BE_4K;
    flash->sectorSize = SECTOR_4K_SIZE;

	/*
	 * Atmel serial flash tend to power up
	 * with the software protection bits set
	 */
	if (info->jedec_id >> 16 == 0x1f) {
		write_enable(flash);
		write_sr(flash, 0);
	}


	dev_info(&spi->dev, "%s (%d Kbytes)\n", info->name,
			flash->flashInfo->sector_size*flash->flashInfo->n_sectors/ 1024);


   /* Export the EEPROM bytes through sysfs, since that's convenient.
	 * Default to root-only access to the data; EEPROMs often hold data
	 * that's sensitive for read and/or write, like ethernet addresses,
	 * security codes, board-specific manufacturing calibrations, etc.
	 */
	flash->bin.attr.name = "SPIContent";
	flash->bin.attr.mode = S_IRUSR|S_IWUSR;
	flash->bin.read = alp_spi_flash_bin_read;
    flash->bin.write = alp_spi_flash_bin_write;
	flash->bin.size = flash->flashInfo->sector_size*flash->flashInfo->n_sectors;

	err = sysfs_create_bin_file(&spi->dev.kobj, &flash->bin);
	if (err){        
		kfree(flash->tempBuffer);
        kfree(flash);
        return err;
    }

    printk("@@@@@@@@@%s finished\n", __func__);

	return 0;
}


static int __devexit alp_spi_flash_remove(struct spi_device *spi)
{
	struct alpSpiFlash_data		*flash;

	flash = dev_get_drvdata(&spi->dev);
	sysfs_remove_bin_file(&spi->dev.kobj, &flash->bin);
	kfree(flash->tempBuffer);
    kfree(flash);
	return 0;
}


/*-------------------------------------------------------------------------*/

static struct spi_driver alpscale_spi_flash_driver = {
	.driver = {
		.name		= "alpscale_spi_flash",
		.owner		= THIS_MODULE,
	},
	.probe		= alp_spi_flash_probe,
	.remove		= __devexit_p(alp_spi_flash_remove),
};

static int __init alp_spi_flash_init(void)
{
	return spi_register_driver(&alpscale_spi_flash_driver);
}
module_init(alp_spi_flash_init);

static void __exit alp_spi_flash_exit(void)
{
	spi_unregister_driver(&alpscale_spi_flash_driver);
}
module_exit(alp_spi_flash_exit);

MODULE_DESCRIPTION("Alpscale 9260 driver for most SPI FLASHes");
MODULE_LICENSE("GPL");
