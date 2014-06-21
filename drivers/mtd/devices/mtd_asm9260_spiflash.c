/*
 * MTD SPI driver for SPI serial flash chips
 *
 * Author: Chendd
 *
 * Copyright (c) 2014, Alpscale.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <asm/memory.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <mach/spi.h>
#include <mach/alpscale_quad_spi.h>


#define FLASH_PAGESIZE		256

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR1	0x05	/* Read status register1 */
#define OPCODE_RDSR2    0X35    /* Read status register2 */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
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

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_COUNT	100000

#ifdef CONFIG_ASM9260_SPI_FLASH_USE_FAST_READ
#define OPCODE_READ 	OPCODE_FAST_READ
#define FAST_READ_DUMMY_BYTE 1
#else
#define OPCODE_READ 	OPCODE_NORM_READ
#define FAST_READ_DUMMY_BYTE 0
#endif

#ifdef CONFIG_MTD_PARTITIONS
#define	mtd_has_partitions()	(1)
#else
#define	mtd_has_partitions()	(0)
#endif

/****************************************************************************/



static inline struct asm9260_mtd_spi_flash *mtd_to_spi_flash(struct mtd_info *mtd)
{
	return container_of(mtd, struct asm9260_mtd_spi_flash, mtd);
}

/****************************************************************************/

/*due to the usage of asm9260 spi controller's DMA functon, we check the CACHE-LINE*/
#define CACHE_LINE_BYTES 32

/*
 * Internal helper functions
 */
union {
    u8 val;
    u32 dummy[CACHE_LINE_BYTES/sizeof(u32)];
} __attribute__ ((aligned(CACHE_LINE_BYTES)))  spi_flash_sr;

/*
 * Read the status register1 or register2 indicated by statusIndex, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct asm9260_mtd_spi_flash *flash, int statusIndex)
{
	ssize_t retval;
	u8 code;

    if( statusIndex == 0 ) {
        code = OPCODE_RDSR1;
    }else{
        code = OPCODE_RDSR2;
    }

	/*must be 4 bytes aligned.*/
	if( (u32)(&spi_flash_sr.val)&0x03 ) {
	dev_err(&flash->spi->dev, "%s not 4 bytes aligned.\n", __func__);
	return -EINVAL;
	}

	retval = spi_write_then_read(flash->spi, &code, 1, &spi_flash_sr.val, 4);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return spi_flash_sr.val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct asm9260_mtd_spi_flash *flash, u8 val)
{
	flash->command[0] = OPCODE_WRSR;
	flash->command[1] = val;

	return spi_write(flash->spi, flash->command, 2);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct asm9260_mtd_spi_flash *flash)
{
	u8	code = OPCODE_WREN;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}


/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct asm9260_mtd_spi_flash *flash)
{
	int count;
	int sr;

	/* one chip guarantees max 5 msec wait here after page writes,
	 * but potentially three seconds (!) after page erase.
	 */
	for (count = 0; count < MAX_READY_WAIT_COUNT; count++) {
		if ((sr = read_sr(flash, 0)) < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

		/* REVISIT sometimes sleeping would be best */
	}

	return 1;
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct asm9260_mtd_spi_flash *flash)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s: %s %lldKiB\n",
			flash->spi->dev.bus_id, __func__,
			flash->mtd.size / 1024);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = OPCODE_CHIP_ERASE;

	spi_write(flash->spi, flash->command, 1);

	return 0;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct asm9260_mtd_spi_flash *flash, u32 offset)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s: %s %dKiB at 0x%08x\n",
			flash->spi->dev.bus_id, __func__,
			flash->mtd.erasesize / 1024, offset);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = flash->erase_opcode;
	flash->command[1] = offset >> 16;
	flash->command[2] = offset >> 8;
	flash->command[3] = offset;

	spi_write(flash->spi, flash->command, CMD_SIZE);

	return 0;
}

/****************************************************************************/

/*
 * MTD implementation
 */

/*
 * Erase an address range on the flash chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int mtd_spi_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct asm9260_mtd_spi_flash *flash = mtd_to_spi_flash(mtd);
	u32 addr,len;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %lld\n",
			flash->spi->dev.bus_id, __func__, "at",
			(u32)instr->addr, instr->len);

	addr = instr->addr;
	len = instr->len;

	/* sanity checks */
	if (instr->addr + instr->len > flash->mtd.size)
		return -EINVAL;

	if ((addr % mtd->erasesize) != 0
			|| (len % mtd->erasesize) != 0) {
		return -EINVAL;
	}

	mutex_lock(&flash->lock);

	/* whole-chip erase? */
	if (len == flash->mtd.size && erase_chip(flash)) {
		instr->state = MTD_ERASE_FAILED;
		mutex_unlock(&flash->lock);
		return -EIO;

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using OPCODE_SE instead of OPCODE_BE_4K.  We may have set up
	 * to use "small sector erase", but that's not always optimal.
	 */

	/* "sector"-at-a-time erase */
	} else {

		while (len) {
			if (erase_sector(flash, addr)) {
				instr->state = MTD_ERASE_FAILED;
				mutex_unlock(&flash->lock);
				return -EIO;
			}

			addr += mtd->erasesize;
			len -= mtd->erasesize;
		}
	}

	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}


/*Not appropriate for all QUAD SPI Flash*/
int mtd_spi_flash_quad_cmd_switch(struct asm9260_mtd_spi_flash *flash, QuadFunctionSwitch quadSwitch){
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
int mtd_spi_flash_quadfunc_switch(struct asm9260_mtd_spi_flash *flash, QuadFunctionSwitch funcSwitch)
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
    if(mtd_spi_flash_quad_cmd_switch(flash, funcSwitch))
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



/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int mtd_spi_flash_read(struct mtd_info *mtd, loff_t from, size_t length,
	size_t *retlen, u_char *buffer)
{
	struct asm9260_mtd_spi_flash *flash = mtd_to_spi_flash(mtd);
	struct spi_transfer t[2];
	struct spi_message m;
	u_char *buf;
	int flag = 0;
	size_t len = length;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
			flash->spi->dev.bus_id, __func__, "from",
			(u32)from, len);


        /*first clear the QUAD mark*/
    if(flash->quadFuction == QUADSELECTED) {
        flash->quadForThis = 0;
        flash->spiTransferIndex = 0;
        memset(flash->spiTransferQuad, 0, sizeof(flash->spiTransferQuad));
    }


	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > flash->mtd.size)
		return -EINVAL;

	if (virt_addr_valid(buffer) && !(len & 0x3))
		buf = buffer;
	else
	{
		flag = 1;
		len = ALIGN(len, 4);
		buf = (u_char *)kzalloc(len, GFP_KERNEL);
	}


    if(flash->quadFuction == QUADSELECTED) {

        if( mtd_spi_flash_quadfunc_switch(flash, QUADSELECTED) ){
            dev_err(&flash->spi->dev, "%s QUADSELECTED failed!\n", __func__);
            return -EPERM;
        }
        /*specify quad later*/
        flash->quadForThis = 0;
    }


	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	/* NOTE:
	 * OPCODE_FAST_READ (if available) is faster.
	 * Should add 1 byte DUMMY_BYTE.
	 */
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

	/* Byte count starts at zero. */
	if (retlen)
		*retlen = 0;

	mutex_lock(&flash->lock);



    /* Wait till previous write/erase is done. NO QUAD */    
    if(flash->quadFuction == QUADSELECTED) {        
        flash->quadForThis = 0;
    }
	/* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		mutex_unlock(&flash->lock);
        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
        }
		return 1;
	}

	/* FIXME switch to OPCODE_FAST_READ.  It's required for higher
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
        dev_err(&flash->spi->dev, "%s failed at 0x%08x, len %zd\n", __func__, (u32)from, len);
        mutex_unlock(&flash->lock);

        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
        }
        return -EIO;
    }
    
    mutex_unlock(&flash->lock);

    if(flash->quadFuction == QUADSELECTED) {             
        flash->quadForThis = 0;
        if( mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED) ){
            dev_err(&flash->spi->dev, "%s QUADDESELECTED failed!\n", __func__);
            return -EPERM;
        }
    }

     if( flash->quadFuction == QUADSELECTED ) {
         *retlen = m.actual_length - CMD_SIZE - 1 - (len - length);
     }else{
         *retlen = m.actual_length - CMD_SIZE - (len - length);
     }
	

	if (flag == 1)
	{
		memcpy(buffer, buf, *retlen);
		kfree(buf);
	}

	return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int mtd_spi_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buffer)
{
	struct asm9260_mtd_spi_flash *flash = mtd_to_spi_flash(mtd);
	u32 page_offset, page_size;
	struct spi_transfer t[2];
	struct spi_message m;
	u_char *buf;
	int flag = 0;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
			flash->spi->dev.bus_id, __func__, "to",
			(u32)to, len);


    /*first clear the QUAD mark*/
    if(flash->quadFuction == QUADSELECTED) {
          flash->quadForThis = 0;
          flash->spiTransferIndex = 0;
          memset(flash->spiTransferQuad, 0, sizeof(flash->spiTransferQuad));
    }


	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > flash->mtd.size)
		return -EINVAL;

	if (virt_addr_valid(buffer))
		buf = (u_char *)buffer;
	else
	{
		flag = 1;
		buf = (u_char *)kzalloc(len, GFP_KERNEL);
		memcpy(buf, buffer, len);
	}


    if(flash->quadFuction == QUADSELECTED) {

        if( mtd_spi_flash_quadfunc_switch(flash, QUADSELECTED) ){
            dev_err(&flash->spi->dev, "%s QUADSELECTED failed!\n", __func__);
            return -EPERM;
        }
        /*specify quad later*/
        flash->quadForThis = 0;
    }

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].len = CMD_SIZE;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	spi_message_add_tail(&t[1], &m);

	mutex_lock(&flash->lock);


    /* Wait till previous write/erase is done. NO QUAD */    
    if(flash->quadFuction == QUADSELECTED) {        
        flash->quadForThis = 0;
    }	
	if (wait_till_ready(flash)) {
		mutex_unlock(&flash->lock);
        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
        }
		return 1;
	}

    if(write_enable(flash)){
        mutex_unlock(&flash->lock);
        if(flash->quadFuction == QUADSELECTED) {
            flash->quadForThis = 0;
            mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
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
	flash->command[1] = to >> 16;
	flash->command[2] = to >> 8;
	flash->command[3] = to;

	/* what page do we start with? */
	page_offset = to % FLASH_PAGESIZE;

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= FLASH_PAGESIZE) {
		t[1].len = len;
        
        if(flash->quadFuction == QUADSELECTED) {
            /*now set QUAD for the following.*/
            flash->quadForThis = 1;
            flash->spiTransferIndex = 0;
        }

        if(spi_sync(flash->spi, &m)){
            mutex_unlock(&flash->lock);
            if(flash->quadFuction == QUADSELECTED) {
                flash->quadForThis = 0;
                mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
            }
            return -EIO;
        }

		*retlen = m.actual_length - CMD_SIZE;
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
            mutex_unlock(&flash->lock);
            if(flash->quadFuction == QUADSELECTED) {
                flash->quadForThis = 0;
                mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
            }
            return -EIO;
        }

		*retlen = m.actual_length - CMD_SIZE;

		/* write everything in PAGESIZE chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > FLASH_PAGESIZE)
				page_size = FLASH_PAGESIZE;

			/* write the next page to flash */
			flash->command[1] = (to + i) >> 16;
			flash->command[2] = (to + i) >> 8;
			flash->command[3] = (to + i);

			t[1].tx_buf = buf + i;
			t[1].len = page_size;

            /*no need to QUAD for the following.*/
            flash->quadForThis = 0;

            if(wait_till_ready(flash)){
                mutex_unlock(&flash->lock);
                if(flash->quadFuction == QUADSELECTED) {
                    flash->quadForThis = 0;
                    mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
                }
                return -EBUSY;
            }

			if(write_enable(flash)){
                mutex_unlock(&flash->lock);
                if(flash->quadFuction == QUADSELECTED) {
                    flash->quadForThis = 0;
                    mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
                }
                return -1;
            }

            if(flash->quadFuction == QUADSELECTED) {
                /*now set QUAD for the following.*/
                flash->quadForThis = 1;
                flash->spiTransferIndex = 0;
            }

            if(spi_sync(flash->spi, &m)){
                mutex_unlock(&flash->lock);
                if(flash->quadFuction == QUADSELECTED) {
                    flash->quadForThis = 0;
                    mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED);
                }
                return -EIO;
            }

			if (retlen)
				*retlen += m.actual_length - CMD_SIZE;
		}
	}

	mutex_unlock(&flash->lock);

    if(flash->quadFuction == QUADSELECTED) {             
        flash->quadForThis = 0;
        if( mtd_spi_flash_quadfunc_switch(flash, QUADDESELECTED) ){
            dev_err(&flash->spi->dev, "%s QUADDESELECTED failed!\n", __func__);
            return -EPERM;
        }
    }

	if (flag == 1)
		kfree(buf);

	return 0;
}


/****************************************************************************/

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static struct mtd_flash_info __devinitdata spi_flash_ids [] = {

	/* Atmel -- some are (confusingly) marketed as "DataFlash" */
	{ "at25fs010",  0x1f6601, 0, 32 * 1024, 4, SECT_4K, },
	{ "at25fs040",  0x1f6604, 0, 64 * 1024, 8, SECT_4K, },

	{ "at25df041a", 0x1f4401, 0, 64 * 1024, 8, SECT_4K, },
	{ "at25df641",  0x1f4800, 0, 64 * 1024, 128, SECT_4K, },

	{ "at26f004",   0x1f0400, 0, 64 * 1024, 8, SECT_4K, },
	{ "at26df081a", 0x1f4501, 0, 64 * 1024, 16, SECT_4K, },
	{ "at26df161a", 0x1f4601, 0, 64 * 1024, 32, SECT_4K, },
	{ "at26df321",  0x1f4701, 0, 64 * 1024, 64, SECT_4K, },

	/* Spansion -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{ "s25sl004a", 0x010212, 0, 64 * 1024, 8, },
	{ "s25sl008a", 0x010213, 0, 64 * 1024, 16, },
	{ "s25sl016a", 0x010214, 0, 64 * 1024, 32, },
	{ "s25sl032a", 0x010215, 0, 64 * 1024, 64, },
	{ "s25sl064a", 0x010216, 0, 64 * 1024, 128, },
        { "s25sl12800", 0x012018, 0x0300, 256 * 1024, 64, },
	{ "s25sl12801", 0x012018, 0x0301, 64 * 1024, 256, },

	/* SST -- large erase sizes are "overlays", "sectors" are 4K */
	{ "sst25vf040b", 0xbf258d, 0, 64 * 1024, 8, SECT_4K, },
	{ "sst25vf080b", 0xbf258e, 0, 64 * 1024, 16, SECT_4K, },
	{ "sst25vf016b", 0xbf2541, 0, 64 * 1024, 32, SECT_4K, },
	{ "sst25vf032b", 0xbf254a, 0, 64 * 1024, 64, SECT_4K, },

	/* ST Microelectronics -- newer production may have feature updates */
	{ "m25p05",  0x202010,  0, 32 * 1024, 2, },
	{ "m25p10",  0x202011,  0, 32 * 1024, 4, },
	{ "m25p20",  0x202012,  0, 64 * 1024, 4, },
	{ "m25p40",  0x202013,  0, 64 * 1024, 8, },
	{ "m25p80",  0x202014,  0, 64 * 1024, 16, },
	{ "m25p16",  0x202015,  0, 64 * 1024, 32, },
	{ "m25p32",  0x202016,  0, 64 * 1024, 64, },
	{ "m25p64",  0x202017,  0, 64 * 1024, 128, },
	{ "m25p128", 0x202018, 0, 256 * 1024, 64, },

	{ "m45pe80", 0x204014,  0, 64 * 1024, 16, },
	{ "m45pe16", 0x204015,  0, 64 * 1024, 32, },

	{ "m25pe80", 0x208014,  0, 64 * 1024, 16, },
	{ "m25pe16", 0x208015,  0, 64 * 1024, 32, SECT_4K, },

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25x10", 0xef3011, 0, 64 * 1024, 2, SECT_4K, },
	{ "w25x20", 0xef3012, 0, 64 * 1024, 4, SECT_4K, },
	{ "w25x40", 0xef3013, 0, 64 * 1024, 8, SECT_4K, },
	{ "w25x80", 0xef3014, 0, 64 * 1024, 16, SECT_4K, },
	{ "w25x16", 0xef3015, 0, 64 * 1024, 32, SECT_4K, },
	{ "w25x32", 0xef3016, 0, 64 * 1024, 64, SECT_4K, },
	{ "w25x64", 0xef3017, 0, 64 * 1024, 128, SECT_4K, DOQUAD},
	{ "w25q128", 0xef4018, 0, 64 * 1024, 256, 0, DOQUAD},
};

static struct mtd_flash_info *__devinit jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = OPCODE_RDID;
	u8			id[8];
	u32			jedec;
	u16                     ext_jedec;
	struct mtd_flash_info	*info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spi_write_then_read(spi, &code, 1, id, 4);
	if (tmp < 0) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: error %d reading JEDEC ID\n",
			spi->dev.bus_id, tmp);
		return NULL;
	}

	jedec = id[0];
	jedec = jedec << 8;
	jedec |= id[1];
	jedec = jedec << 8;
	jedec |= id[2];

	ext_jedec = id[3] << 8 | id[4];

	for (tmp = 0, info = spi_flash_ids;
			tmp < ARRAY_SIZE(spi_flash_ids);
			tmp++, info++) {
		if (info->jedec_id == jedec) {
			if (info->ext_id != 0 && info->ext_id != ext_jedec)
				continue;
			return info;
		}
	}
	dev_err(&spi->dev, "unrecognized JEDEC id %06x\n", jedec);
	return NULL;
}


/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit mtd_spi_flash_probe(struct spi_device *spi)
{
	struct flash_platform_data	*data;
	struct asm9260_mtd_spi_flash			*flash;
	struct mtd_flash_info		*info;
	unsigned			i;
    int         doQuad = 0;

	/* Platform data helps sort out which chip type we have, as
	 * well as how this board partitions it.  If we don't have
	 * a chip ID, try the JEDEC id commands; they'll work for most
	 * newer chips, even if we don't recognize the particular chip.
	 */
	data = spi->dev.platform_data;
	if (data && data->type) {
		for (i = 0, info = spi_flash_ids;
				i < ARRAY_SIZE(spi_flash_ids);
				i++, info++) {
			if (strcmp(data->type, info->name) == 0)
				break;
		}

		/* unrecognized chip? */
		if (i == ARRAY_SIZE(spi_flash_ids)) {
			DEBUG(MTD_DEBUG_LEVEL0, "%s: unrecognized id %s\n",
					spi->dev.bus_id, data->type);
			info = NULL;

		/* recognized; is that chip really what's there? */
		} else if (info->jedec_id) {
			struct mtd_flash_info	*chip = jedec_probe(spi);
                       
			if (!chip || chip != info) {
				dev_warn(&spi->dev, "found %s, expected %s\n",
						chip ? chip->name : "UNKNOWN",
						info->name);
				info = NULL;
			}else{
                 printk("------ chip: %s, info: %s\n", chip->name, info->name);
            }
		}
	} else
		info = jedec_probe(spi);

	if (!info)
		return -ENODEV;

        /*now, start the SPI FLASH CHIP's QUAD function selection check!*/
    #ifdef CONFIG_MTD_ASM9260_SPI_FLASH_QUAD_FUNC
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
	if (!flash)
		return -ENOMEM;

    /*record the QUAD selection*/
    if (doQuad == 0) {
        //printk("--------1\n");
        flash->quadFuction = QUADDESELECTED;
    }else{
        //printk("--------2\n");
        flash->quadFuction = QUADSELECTED;
    }
    flash->quadForThis = 0;

	flash->spi = spi;
	mutex_init(&flash->lock);
	dev_set_drvdata(&spi->dev, flash);

	/*
	 * Atmel serial flash tend to power up
	 * with the software protection bits set
	 */

	if (info->jedec_id >> 16 == 0x1f) {
		write_enable(flash);
		write_sr(flash, 0);
	}

	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = spi->dev.bus_id;

	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.writesize = 1;
	flash->mtd.flags = MTD_CAP_NORFLASH;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd.erase = mtd_spi_flash_erase;
	flash->mtd.read = mtd_spi_flash_read;
	flash->mtd.write = mtd_spi_flash_write;

	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K) {
		flash->erase_opcode = OPCODE_BE_4K;
		flash->mtd.erasesize = 4096;
	} else {
		flash->erase_opcode = OPCODE_SE;
		flash->mtd.erasesize = info->sector_size;
	}

	dev_info(&spi->dev, "%s (%lld Kbytes)\n", info->name,
			flash->mtd.size / 1024);

	DEBUG(MTD_DEBUG_LEVEL1,
		"mtd .name = %s, .size = 0x%.8llx (%lluMiB) "
			".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->mtd.name,
		flash->mtd.size, flash->mtd.size / (1024*1024),
		flash->mtd.erasesize, flash->mtd.erasesize / 1024,
		flash->mtd.numeraseregions);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			DEBUG(MTD_DEBUG_LEVEL1,
				"mtd.eraseregions[%d] = { .offset = 0x%.8llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, flash->mtd.eraseregions[i].offset,
				flash->mtd.eraseregions[i].erasesize,
				flash->mtd.eraseregions[i].erasesize / 1024,
				flash->mtd.eraseregions[i].numblocks);


	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	 */
	if (mtd_has_partitions()) {
		struct mtd_partition	*parts = NULL;
		int			nr_parts = 0;

#ifdef CONFIG_MTD_CMDLINE_PARTS
		static const char *part_probes[] = { "cmdlinepart", NULL, };

		nr_parts = parse_mtd_partitions(&flash->mtd,
				part_probes, &parts, 0);
#endif

		if (nr_parts <= 0 && data && data->parts) {
			parts = data->parts;
			nr_parts = data->nr_parts;
		}

		if (nr_parts > 0) {
			for (i = 0; i < nr_parts; i++) {
				DEBUG(MTD_DEBUG_LEVEL2, "partitions[%d] = "
					"{.name = %s, .offset = 0x%.8x, "
						".size = 0x%.8x (%uKiB) }\n",
					i, parts[i].name,
					parts[i].offset,
					parts[i].size,
					parts[i].size / 1024);
			}
			flash->partitioned = 1;

			return add_mtd_partitions(&flash->mtd, parts, nr_parts);
		}
	} else if (data->nr_parts)
		dev_warn(&spi->dev, "ignoring %d default partitions on %s\n",
				data->nr_parts, data->name);

	return add_mtd_device(&flash->mtd) == 1 ? -ENODEV : 0;
}


static int __devexit mtd_spi_flash_remove(struct spi_device *spi)
{
	struct asm9260_mtd_spi_flash	*flash = dev_get_drvdata(&spi->dev);
	int		status;

	/* Clean up MTD stuff. */
	if (mtd_has_partitions() && flash->partitioned)
		status = del_mtd_partitions(&flash->mtd);
	else
		status = del_mtd_device(&flash->mtd);
	if (status == 0)
		kfree(flash);
	return 0;
}


static struct spi_driver mtd_spi_flash_driver = {
	.driver = {
		.name	= "mtd_spi_flash",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= mtd_spi_flash_probe,
	.remove	= __devexit_p(mtd_spi_flash_remove),

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};


static int mtd_spi_flash_init(void)
{
	return spi_register_driver(&mtd_spi_flash_driver);
}


static void mtd_spi_flash_exit(void)
{
	spi_unregister_driver(&mtd_spi_flash_driver);
}


module_init(mtd_spi_flash_init);
module_exit(mtd_spi_flash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chendd");
MODULE_DESCRIPTION("MTD SPI driver for SPI flash chips");
