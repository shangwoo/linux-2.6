/* Linux driver for Mixed NAND Flash Translation Layer  */
/* (c) 2007 Alpscale, Inc.                              */
/* Author: He Yong <hoffer@sjtu.org>                    */
/*         David Woodhouse <dwmw2@infradead.org>        */
/* $Id: nftlcore_2k.c,v 0.12 2007/09/24 15:38:21        */

/*
  The contents of this file are distributed under the GNU General
  Public License version 2. The author places no additional
  restrictions of any kind on it.
 */

#define PRERELEASE

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/hdreg.h>

#include <linux/kmod.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nftl_2k.h>
#include <linux/mtd/blktrans.h>

#ifdef CONFIG_NFTL2K_DEBUG
    #define NFTL2K_CORE_DEBUG    1
#else // CONFIG_NFTL2K_DEBUG
    #define NFTL2K_CORE_DEBUG    0
#endif //CONFIG_NFTL2K_DEBUG

#ifdef CONFIG_NFTL2K_ABORT_TEST
    #define TEST_POWEROFF_DEBUG
#endif //CONFIG_NFTL2K_POWEROFF_TEST

#define MAX_LOOPS 10000
#define BYTES_PER_LINE 32
#define ABORT_WIRTE_BACK_SIGNAL 13310
#define WIRTE_BACK_LP_OK 0  /* wirte back last page ok */
/**
 * nftl2k_dbg_hexdump - dump a buffer.
 * @ptr: the buffer to dump
 * @size: buffer size which must be multiple of 4 bytes
 */
void nftl2k_dbg_hexdump(const void *ptr, int size)
{
	int i, k = 0, rows, columns;
	const uint8_t *p = ptr;

	size = ALIGN(size, 4);
	rows = size/BYTES_PER_LINE + size % BYTES_PER_LINE;
	for (i = 0; i < rows; i++) {
		int j;

	   // cond_resched();
		columns = min(size - k, BYTES_PER_LINE) / 4;
		if (columns == 0)
			break;
		printk( "%5d:  ", i * BYTES_PER_LINE);
		for (j = 0; j < columns; j++) {
			int n, N;

			N = size - k > 4 ? 4 : size - k;
			for (n = 0; n < N; n++)
				printk("%02x", p[k++]);
			printk(" ");
		}
		printk("\n");
	}
}

void nftl2k_dbg_nftl2k_oob_dump(struct nftl2k_oob * oob_struct){
    printk("Page status: 0x%02x 0x%02x\n",oob_struct->b.Status,oob_struct->b.Status1);
    printk("VUN: 0x%04x 0x%04x\n",oob_struct->u.a.VirtUnitNum,oob_struct->u.a.SpareVirtUnitNum);
    printk("REPL: 0x%04x 0x%04x\n",oob_struct->u.a.ReplUnitNum,oob_struct->u.a.SpareReplUnitNum);
    printk("EraseMark: 0x%04x 0x%04x\n",oob_struct->u.b.EraseMark,oob_struct->u.b.EraseMark1);
}

/**
 * nftl2k_dbg_hexdump - dump a buffer.
 * @ptr: the buffer to dump
 * @size: buffer size which must be multiple of 4 bytes
 */
void nftl2k_dbg_MediaHeader_dump(const struct NFTL2KMediaHeader * header)
{
	printk("NFTL2K MediaHeader dump:\n");
	printk("vol_id    %s\n",   header->DataOrgID);
	printk("FirstPhysicalEUN %d\n",   header->FirstPhysicalEUN);
	printk("FormattedSize %Ld\n",   header->FormattedSize);
	printk("NumEraseUnits %d\n",   header->NumEraseUnits);
	printk("UnitSizeFactor %d\n",   header->UnitSizeFactor);
}

void nftl2k_dbg_EUNtable_dump(struct NFTL2Krecord *nftl2k){
    int block;
    u16 LastGoodBlock;

    for (block = 0; block < nftl2k->nb_blocks ; block++) {
        LastGoodBlock = nftl2k->EUNtable[block];
        if (LastGoodBlock >= nftl2k->nb_blocks) continue;
        printk("\nVUN: (%d)",block);
        while(LastGoodBlock < nftl2k->nb_blocks) {
            printk("->%d",LastGoodBlock);
            LastGoodBlock = nftl2k->ReplUnitTable[LastGoodBlock];
        }
    }
    printk("\n");
}

#ifdef CONFIG_NFTL2K_WEAR_LEVELING
void nftl2k_dbg_WearInfo_dump(struct NFTL2Krecord *nftl2k)
{
    int i;
	printk("NFTL2K WearInfo dump:\n");
    for (i=0;i<nftl2k->nb_blocks;i++) {
        printk(" %d",nftl2k->WearInfoTable[i]);
        if ((i&0x7)==7) {printk("\n"); }
    }
}
#endif

void nftl2k_bbt_inc(struct NFTL2Krecord *nftl2k,int vuc,int block);

/* we must write back any cached data now */
static int nftl2k_flush(struct mtd_blktrans_dev *dev){

	struct NFTL2Krecord *nftl2k = (void *)dev;
    /*
    dprintk(
        "\n==============================\n"
        "===    Flush Write Cache   ===\n"
        "==============================\n"
        );
        */
    /* Write Back Cached Data if needed */
    if (nftl2k->write_buf_status > 0) {
        int max_try;
        max_try = 20;
        while (max_try--) {
            if (nftl2k_writeback_lastpage(nftl2k) != WIRTE_BACK_LP_OK){
                printk("!!! Warning !! AFTL Writeback Last Page Fail in nftl2k_flush()!!!\n");
                mdelay(100);
            }
            else break;
        }
    }
    return 0;
}

static int nftl2k_release(struct mtd_blktrans_dev *dev){
// #if NFTL2K_CORE_DEBUG
// 	struct NFTL2Krecord *nftl2k = (void *)dev;
// #endif

    // dprintk("\n===    NFTL Release    ===\n" );
    /* Flush Cached Data if needed */
    nftl2k_flush(dev);
// #if((NFTL2K_CORE_DEBUG)&&(defined(CONFIG_NFTL2K_WEAR_LEVELING)))
//     nftl2k_dbg_WearInfo_dump(nftl2k);
// #endif
// #if NFTL2K_CORE_DEBUG
//     nftl2k_dbg_EUNtable_dump(nftl2k);
// #endif
    return 0;
}


void format_mtd(struct NFTL2Krecord *nftl2k,struct mtd_info *mtd){
    int total_block;
	struct erase_info instr;
    struct NFTL2KMediaHeader NFTLHeader;
    int retlen;
	struct nftl2k_oob oob;
    u64 reserved_spaces;

    /* this is ugly , because we can not handle 64 bit div yet -- He Yong*/
    total_block = ((u_int32_t)(mtd->size >> 9)) / (mtd->erasesize>>9); // (mtd->size / mtd->erasesize);

    /* Erase all Good Blocks in this Partition */
    memset((char *)&instr,0,sizeof(struct erase_info));
    instr.mtd = mtd;
    do{
        instr.addr = instr.fail_addr;
        if (instr.state == MTD_ERASE_FAILED) {
            instr.addr += mtd->erasesize;
            dprintk("NFTL2K:Format Failed at 0x%09Lx continue at 0x%09Lx.\n"
                   ,instr.fail_addr,instr.addr);
        }
        instr.len = mtd->size - instr.addr;
        mtd->erase(mtd, &instr);
    }
    while ((instr.state == MTD_ERASE_FAILED)&&(instr.fail_addr + mtd->erasesize < mtd->size));

    /* ===== Prepare media header ======= */
    /* 
    count for reserved blocks, 
    x : 'CONFIG_NFTL_2K_RESERVED_BLOCK_PRECENT' which configed in 'menuconfig'
    x/128 of total blocks will be reserved, 
    x = 3 typicaly 
    -- He Yong
    */
    reserved_spaces = (mtd->size>>7)*CONFIG_NFTL_2K_RESERVED_BLOCK_PRECENT + 2*mtd->erasesize;
    strcpy(NFTLHeader.DataOrgID,"ANAND");
    NFTLHeader.FirstPhysicalEUN = 1;
    NFTLHeader.FormattedSize = mtd->size - reserved_spaces;
    NFTLHeader.NumEraseUnits = total_block - 1;
    NFTLHeader.UnitSizeFactor = 0;

    /* write to boot record block, block 0 */
    memset(nftl2k->write_buf,0,nftl2k->page_size);
    memcpy(nftl2k->write_buf,(char *)&NFTLHeader,sizeof(struct NFTL2KMediaHeader));

    memset(&oob, 0xff, sizeof(struct nftl2k_oob));
    oob.b.Status = oob.b.Status1 = SECTOR_USED;
    oob.u.a.ReplUnitNum = oob.u.a.SpareReplUnitNum = 0xF123;
    oob.u.a.VirtUnitNum = oob.u.a.SpareVirtUnitNum = 0xF321;
    oob.u.b.EraseMark = oob.u.b.EraseMark1 = ERASE_MARK;

    nftl2k_write(mtd, 0, nftl2k->page_size, &retlen, nftl2k->write_buf,(char *)&oob);

}

/* 
this gvar 'is_this_a_timeout_write_back' is a signal send for mtd->nand,
which indicate that this write ops is a cache write back.
if the device is busy, delay this writeback,
because it is highly possible that another write ops is been doing.

this is ugly and may need improvement using locks.
if it is a diable_irq mode, use gvar is safe.
-- He Yong
 
*/
#define TIME_OUT_WB 1
#define NORMAL_WB 0
extern volatile int is_this_a_timeout_write_back;

static void nftl2k_timer(unsigned long data){
    int res;
	struct NFTL2Krecord *nftl2k = (struct NFTL2Krecord *) data;

    if (nftl2k->write_buf_status > 0) {
       // printk("Execute Time-out Page Write Back.\n");
        printk("^");
        is_this_a_timeout_write_back = TIME_OUT_WB;
        res = nftl2k_writeback_lastpage(nftl2k);
        is_this_a_timeout_write_back = NORMAL_WB;

        if (res == -ABORT_WIRTE_BACK_SIGNAL) { 
           // printk("Page Write Back aborted!!!\n");
            printk("~");
            /* write this page back again, if time out x seconds */
            mod_timer(&nftl2k->timer, jiffies + WRITE_BACK_TIMEOUT_SECONDS*100);
        }
    }
}

static void nftl2k_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct NFTL2Krecord *nftl2k;
	unsigned long temp;
    char * endp;
    int format_part = 0;
    int i;

	if (mtd->type != MTD_NANDFLASH)
		return;
	/* OK, this is moderately ugly.  But probably safe.  Alternatives? */
	if (memcmp(mtd->name, "NFTL2K", 6))
		return;

	if (!mtd->block_isbad) {
		printk(KERN_ERR
               "NFTL2K Need mtd->block_isbad(); in NAND subsystem.\n");
		return;
	}

	DEBUG(MTD_DEBUG_LEVEL1, "NFTL2K: add_mtd for %s\n", mtd->name);

	nftl2k = kzalloc(sizeof(struct NFTL2Krecord), GFP_KERNEL);

	if (!nftl2k) {
		printk(KERN_WARNING "NFTL2K: out of memory for data structures\n");
		return;
	}

	nftl2k->mbd.mtd = mtd;
	nftl2k->mbd.devnum = -1;

	nftl2k->mbd.tr = tr;

    /* see if we need format nftl partition */

    for (i = 0; i < 128 ;i++) {
        if ((saved_command_line[i]==' ')&&(strncmp("nftlfmt=",&saved_command_line[i+1],8)==0)) {
            format_part = simple_strtol(&saved_command_line[i+9], &endp, 10);
        }
    }

    if (format_part) {
        printk("NFTL: Format Mtd partition:[%d %s]\n",mtd->index,mtd->name);
        format_mtd(nftl2k,mtd);
    }

    if (NFTL2K_mount(nftl2k) < 0) {
	printk(KERN_WARNING "NFTL2K: could not mount device\n");
	kfree(nftl2k);
	return;
    }

	setup_timer(&nftl2k->timer, nftl2k_timer, (unsigned long)nftl2k);

	/* OK, it's a new one. Set up all the data structures. */

	/* Calculate geometry */
	nftl2k->cylinders = 1024;
	nftl2k->heads = 16;

	temp = nftl2k->cylinders * nftl2k->heads;
	nftl2k->sectors = nftl2k->mbd.size / temp;
	if (nftl2k->mbd.size % temp) {
		nftl2k->sectors++;
		temp = nftl2k->cylinders * nftl2k->sectors;
		nftl2k->heads = nftl2k->mbd.size / temp;

		if (nftl2k->mbd.size % temp) {
			nftl2k->heads++;
			temp = nftl2k->heads * nftl2k->sectors;
			nftl2k->cylinders = nftl2k->mbd.size / temp;
		}
	}

	if (nftl2k->mbd.size != nftl2k->heads * nftl2k->cylinders * nftl2k->sectors) {
		/*
		  Oh no we don't have
		   mbd.size == heads * cylinders * sectors
		*/
		printk(KERN_WARNING "NFTL2K: cannot calculate a geometry to "
		       "match size of 0x%lx.\n", nftl2k->mbd.size);
		printk(KERN_WARNING "NFTL2K: using C:%d H:%d S:%d "
			"(== 0x%lx sects)\n",
			nftl2k->cylinders, nftl2k->heads , nftl2k->sectors,
			(long)nftl2k->cylinders * (long)nftl2k->heads *
			(long)nftl2k->sectors );
	}

    /*init globe vars*/
    nftl2k->write_buf_status = nftl2k->read_buf_status = 0; 
    nftl2k->Last_Write_EUN = nftl2k->Last_Write_VUC = nftl2k->Last_Read_VUC = nftl2k->Last_Pre_Block = BLOCK_NIL;
    nftl2k->Last_Write_Page = nftl2k->Last_Read_Page = 0xff;
    //nftl2k->swtich_wrtie_vuc_target = 1;

	if (add_mtd_blktrans_dev(&nftl2k->mbd)) {
		kfree(nftl2k->ReplUnitTable);
		kfree(nftl2k->EUNtable);
		kfree(nftl2k);
		return;
	}
#ifdef PSYCHO_DEBUG
	printk(KERN_INFO "NFTL2K: Found new nftl2k%c\n", nftl2k->mbd.devnum + 'a');
#endif
}

static void nftl2k_remove_dev(struct mtd_blktrans_dev *dev)
{
	struct NFTL2Krecord *nftl2k = (void *)dev;

	DEBUG(MTD_DEBUG_LEVEL1, "NFTL2K: remove_dev (i=%d)\n", dev->devnum);

	del_mtd_blktrans_dev(dev);
	kfree(nftl2k->ReplUnitTable);
	kfree(nftl2k->EUNtable);
	kfree(nftl2k);
}

/*
 * Read oob data from flash
 */
int nftl2k_read_oob(struct mtd_info *mtd, loff_t offs, size_t len,
		  size_t *retlen, uint8_t *buf)
{
	struct mtd_oob_ops ops;
	int res;
    ftl_ECC ecc;
    int result;
    struct nftl2k_oob * oob_struct = (struct nftl2k_oob *)buf;

    if (len > mtd->ecclayout->oobavail) {
        len = mtd->ecclayout->oobavail;
    }

	ops.mode = MTD_OOB_AUTO;  /* AUTO use of nand oob space in case MTD layer used some bytes*/
	ops.ooboffs = offs & (mtd->writesize - 1);
	ops.ooblen = len;
	ops.len = 0;
	ops.oobbuf = buf;
	ops.datbuf = NULL;

    #if 0    /* This is for debug */
    dprintk( "nftl2k_read_oob: "
          "ops.ooblen = [%d],offs [0x%09Lx]\n"
          ,ops.ooblen,offs);
    #endif    /* This is for debug */

	res = mtd->read_oob(mtd, offs & ~(mtd->writesize - 1), &ops);
	*retlen = ops.oobretlen;

    if (((offs & ((loff_t)mtd->erasesize - 1)) == 0)&&(len >= sizeof(struct nftl2k_oob))) {

        if ((oob_struct->u.a.VirtUnitNum | oob_struct->u.a.SpareVirtUnitNum) == 0xffff) {
            /* it's like a free block, skip ecc check */ 
            return res;
        }
        /* first page of block, so we need do ecc for uci */ 
        // printk("nftl2k_read_oob Do ECC Check...");

        /* bci status 2 byte + size of uci 16 byte = 18 byte , generate 3 byte of ecc code */
        ftl_ECCCalculate((unsigned char *)&oob_struct->b.Status,
                    sizeof(struct nftl2k_uci)+2,
                    &ecc);
        result =
            ftl_ECCCorrect((unsigned char *)&oob_struct->b.Status,
                              sizeof(struct nftl2k_uci)+2,
                              &oob_struct->b.ecc, &ecc);
        switch(result){
            case 0: 
                /* No error */ 
            //    printk("OK\n");
                break;
            case 1: 
                /* error Fixed */ 
            //    printk("Fixed\n");
                break;
            case -1:
                /* error unFixed */ 
                printk("ECC <Fail>\n Error Offs = 0x%09Lx, len = %d, Block = %d\n"
                       ,offs,len,((int)offs>>9)/ ((int)(mtd->erasesize>>9)));

                nftl2k_dbg_hexdump(buf,sizeof(struct nftl2k_oob));
                nftl2k_dbg_nftl2k_oob_dump(oob_struct);
                break;
            default:
                /* error unKnown */ 
                printk("<unKnown State>\n");
        }
    }

	return res;
}


/*
 * Write data and oob to flash
 * return should >= 0
 */
int nftl2k_write(struct mtd_info *mtd, loff_t offs, size_t len,
		      size_t *retlen, uint8_t *buf, uint8_t *oob)
{
	struct mtd_oob_ops ops;
	int res;
    struct nftl2k_oob * oob_struct = (struct nftl2k_oob *)oob;

	ops.mode = MTD_OOB_AUTO;  /* AUTO use of nand oob space in case MTD layer used some bytes*/
	ops.ooboffs = 0;
	ops.ooblen = mtd->ecclayout->oobavail;
	ops.oobbuf = oob;
	ops.datbuf = buf;
	ops.len = len;

    if ((offs & ((loff_t)mtd->erasesize - 1)) == 0) {
        /* first page of block, so we need do ecc for uci */ 
        /* bci status 2 byte + size of uci 16 byte = 18 byte , generate 3 byte of ecc code */
        ftl_ECCCalculate((unsigned char *)&oob_struct->b.Status,
                    sizeof(struct nftl2k_uci)+2,
                    &oob_struct->b.ecc);
    }

	res = mtd->write_oob(mtd, offs & ~(mtd->writesize - 1), &ops);
	*retlen = ops.retlen;
	return res;
}

/* NFTL2K_findfreeblock: Find a free Erase Unit on the NFTL2K partition. This function is used
 *	when the give Virtual Unit Chain
 * this function may directly affect wear-levelings
 */
static u16 NFTL2K_findfreeblock(struct NFTL2Krecord *nftl2k, int desperate )
{
	/* For a given Virtual Unit Chain: find or create a free block and
	   add it to the chain */
	/* We're passed the number of the last EUN in the chain, to save us from
	   having to look it up again */
	u16 pot = nftl2k->LastFreeEUN;
	int silly = nftl2k->nb_blocks;

#ifdef CONFIG_NFTL2K_WEAR_LEVELING
    int available_pots;
    u16 lowest_level_pot;
    u16 lowest_level;
#endif //CONFIG_WEAR_LEVEL_DEBUG

	/* Normally, we force a fold to happen before we run out of free blocks completely */
	if (!desperate && nftl2k->numfreeEUNs < 2) {
		// dprintk("NFTL2K_findfreeblock: there are too few free EUNs\n");
		return 0xffff;
	}

#ifdef CONFIG_NFTL2K_WEAR_LEVELING
    available_pots = 0;
    lowest_level_pot = BLOCK_NIL;
    lowest_level = BLOCK_NIL;
#endif //CONFIG_WEAR_LEVEL_DEBUG

	/* Scan for a free block */
	do {
		if ((nftl2k->ReplUnitTable[pot] == BLOCK_FREE)
            &&(nftl2k->Last_Write_EUN != pot)) {
            /* 
            we must protect write cached target blocks, 
            now, these blocks seems to be free 
            */
            #ifdef CONFIG_NFTL2K_WEAR_LEVELING
            if (available_pots == 0) { 
                lowest_level = nftl2k->WearInfoTable[pot]; 
                lowest_level_pot = pot; 
                }

            /* found a free EUN*/
            available_pots++;

            /* Check if it has the lowest Level*/
            if (nftl2k->WearInfoTable[pot] < lowest_level) {
                lowest_level_pot = pot;
                lowest_level = nftl2k->WearInfoTable[pot]; 
            }

            /* 
            with in how many pots should we stop searching
            we set 1/128 of total blocks, and at least WEAR_LEVEL_LEAST_CMP_POTS pots
            */
            if (available_pots> (WEAR_LEVEL_LEAST_CMP_POTS + (nftl2k->nb_blocks>>7))) {
                goto _pot_found;
            }
            #else
                        
			nftl2k->LastFreeEUN = pot;
			nftl2k->numfreeEUNs--;
			return pot;

            #endif //CONFIG_WEAR_LEVEL_DEBUG
		}

		/* This will probably point to the MediaHdr unit itself,
		   right at the beginning of the partition. But that unit
		   (and the backup unit too) should have the UCI set
		   up so that it's not selected for overwriting */
		if (++pot > nftl2k->lastEUN)
			pot = le16_to_cpu(nftl2k->MediaHdr.FirstPhysicalEUN);

		if (!silly--) {
			printk("Argh! No free blocks found! LastFreeEUN = %d, "
			       "FirstEUN = %d\n", nftl2k->LastFreeEUN,
			       le16_to_cpu(nftl2k->MediaHdr.FirstPhysicalEUN));
			return 0xffff;
		}
	} while (pot != nftl2k->LastFreeEUN);

#ifdef CONFIG_NFTL2K_WEAR_LEVELING
_pot_found:
    nftl2k->LastFreeEUN = lowest_level_pot;
    nftl2k->numfreeEUNs--;
    nftl2k->WearInfoTable[lowest_level_pot]++;
	return lowest_level_pot;
#else
	return 0xffff;
#endif //CONFIG_WEAR_LEVEL_DEBUG
}

/*
Copy the [start_page - end_page] datas in srcEUN to the targetEUN
copyback do not copy the end_page itself, leave it free
*/
int NFTL2K_CopyBack(struct NFTL2Krecord *nftl2k, u16 thisVUC, u16 LastEUN,
                    u16 srcEUN, u16 targetEUN,int start_page,int end_page ){
	struct mtd_info *mtd = nftl2k->mbd.mtd;
    int page;
    int ret;
	size_t retlen;
	struct nftl2k_oob oob;
    //struct nftl2k_oob tmp_oob;

    dprintk("NFTL2K_CopyBack: VUC %d from EUN %d to EUN %d page (%d - %d), LastEUN = %d\n",
           thisVUC,srcEUN,targetEUN,start_page,end_page-1,LastEUN);

    memset(&oob, 0xff, sizeof(struct nftl2k_oob));
    oob.b.Status = oob.b.Status1 = SECTOR_USED;

    for (page = start_page; page < end_page; page++) {
        /* read the data */
        ret = mtd->read(mtd, (nftl2k->EraseSize * srcEUN) + (page * nftl2k->page_size),
                nftl2k->page_size, &retlen, nftl2k->movebuf);
        if (ret < 0 && ret != -EUCLEAN) {
         //   printk("NFTL2K_CopyBack: ECC Error 1st:\n");
         //   nftl2k_dbg_hexdump(movebuf, nftl2k->page_size);
            ret = mtd->read(mtd, (nftl2k->EraseSize * srcEUN)
                    + (page * nftl2k->page_size), nftl2k->page_size, &retlen, nftl2k->movebuf);
          //  printk("NFTL2K_CopyBack: ECC Error 2nd:\n");
          //  nftl2k_dbg_hexdump(movebuf, nftl2k->page_size);
            if (ret < 0){
                printk("NFTL2K_CopyBack: Read Error Twice, Fail.\n");
               // nftl2k_bbt_inc(nftl2k,thisVUC,BlockMap[page]);
            }
            
        }

        oob.b.free_page_pos = oob.b.free_page_pos1 = page+1;            

        if (unlikely( (page == 0) || (page == nftl2k->page_per_block - 1))){
            /*this page is the most important one, because it has uci info*/
            oob.u.a.ReplUnitNum = oob.u.a.SpareReplUnitNum = LastEUN;  /* this may be a HEAD block */
            oob.u.a.VirtUnitNum = oob.u.a.SpareVirtUnitNum = cpu_to_le16(thisVUC);
            oob.u.b.EraseMark = oob.u.b.EraseMark1 = ERASE_MARK;/* not wear-info here */
        }
        ret = nftl2k_write(nftl2k->mbd.mtd, (nftl2k->EraseSize * targetEUN) +
               (page * nftl2k->page_size), nftl2k->page_size, &retlen, nftl2k->movebuf, (char *)&oob);
        if (ret < 0) {
            printk("nftl2k_write() Fail, in %s\n",__FUNCTION__);
        }
        #ifdef TEST_POWEROFF_DEBUG
        mdelay(1000);
        #endif
    }
    return page;
}

/*
Fold the [startpage - endpage] datas in VUC from the [startEUN - secondlastEUN] to the targetEUN
fold_page_data do not copy the stop_page itself, leave it free
*/

void NFTL2K_fold_page_data(struct NFTL2Krecord *nftl2k, u16 thisVUC,
u16 secondlastEUN, u16 targetEUN, int startpage, int endpage){

  struct mtd_info *mtd = nftl2k->mbd.mtd;
  u16 thisEUN;
  char thisffpage,thispage;
  struct nftl2k_oob oob;
  size_t retlen;
  int ret;

  thisEUN = secondlastEUN; // copy from the target EUN's previous EUN

  dprintk("fold_page_data in VUC %d, to EUN %d Page (%d-%d)\n",
          thisVUC,targetEUN,startpage,endpage-1);

  while(thisEUN!=BLOCK_HEAD){
      thisffpage = nftl2k->FirstFreePageTable[thisEUN]; // get page position of thisEUN 
      if(thisffpage > startpage){
          if(thisffpage >= endpage){ /// thisEUN's page data is enough 
              NFTL2K_CopyBack(nftl2k, thisVUC, BLOCK_HEAD, thisEUN, targetEUN, startpage,endpage );
              nftl2k->FirstFreePageTable[targetEUN] = endpage; // set targetEUN page position
              break;
          }
          else{   // thisEUN's page data is not enough, continue searching for the previous one
              NFTL2K_CopyBack(nftl2k,thisVUC,BLOCK_HEAD,thisEUN,targetEUN,startpage,thisffpage);
              nftl2k->FirstFreePageTable[targetEUN] = thisffpage; // set targetEUN page position
              startpage = thisffpage;
          }
      }
      thisEUN = nftl2k->ReverseUnitTable[thisEUN];
  }
  
  if (thisEUN == BLOCK_HEAD) {

      // there's not enough valid data for the target page of targetEUN, we must fill ZEROES 
      memset(nftl2k->movebuf,0x0,nftl2k->page_size);
      dprintk("no valid data in VUC %d, fill 0 to EUN %d Page (%d-%d)\n",
              thisVUC,targetEUN,startpage,endpage-1);

      oob.b.Status = oob.b.Status1 = SECTOR_USED; // this page do not content valid data
      oob.u.a.ReplUnitNum = oob.u.a.SpareReplUnitNum = BLOCK_HEAD;  // this is a HEAD block 
      oob.u.a.VirtUnitNum = oob.u.a.SpareVirtUnitNum = thisVUC;

      for ( thispage = startpage ; thispage < endpage ; thispage++){

          oob.b.free_page_pos = oob.b.free_page_pos1 = thispage+1;  
          ret = nftl2k_write(mtd,(nftl2k->EraseSize * targetEUN)+(thispage * nftl2k->page_size),
                        nftl2k->page_size, &retlen, nftl2k->movebuf, (char*)&oob);
          if (ret < 0) {
              printk("nftl2k_write() Fail, in %s\n",__FUNCTION__);
          }
      }
      nftl2k->FirstFreePageTable[targetEUN] = endpage; // set targetEUN page position 
  }
}

/*
this sometimes may tale a long time, the music will be paused for about a second
so, it needs improvement!
*/

u16 NFTL2K_foldchain (struct NFTL2Krecord *nftl2k, unsigned thisVUC)
{
	unsigned int thisEUN;
	unsigned int targetEUN,tailEUN;
    int chain_len;

    /* write back last package if needed */
    if (nftl2k->write_buf_status > 0) {
        int max_try;
        max_try = 20;
        while (max_try--) {
            if (nftl2k_writeback_lastpage(nftl2k) != WIRTE_BACK_LP_OK){
                printk("!!! Warning !! AFTL Writeback Last Page Fail in NFTL2K_foldchain()!!!\n");
                mdelay(100);
            }
            else break;
        }
    }

	thisEUN = nftl2k->EUNtable[thisVUC];

	if (thisEUN >= nftl2k->nb_blocks) {
		printk(KERN_WARNING "Trying to fold non-existent "
		       "Virtual Unit Chain %d!\n", thisVUC);
		return BLOCK_NIL;
	}

    chain_len = 0;

    while (thisEUN < nftl2k->nb_blocks){
        chain_len++;
        tailEUN = thisEUN;
        thisEUN = nftl2k->ReplUnitTable[thisEUN];
    }

	thisEUN = nftl2k->EUNtable[thisVUC];

	if (chain_len < 2 ) {
		printk("Trying to fold a single EUN chain.\n");
		return thisEUN;
	}

    /* ==== check for inplace folding if possible ======= */
    if (nftl2k->FirstFreePageTable[tailEUN] < nftl2k->page_per_block) {
        /* ===== in place folding ========== */
        targetEUN = tailEUN;
        dprintk("#### Inplace Folding chain %d into unit %d ####\n",thisVUC, targetEUN);
        NFTL2K_fold_page_data(nftl2k,thisVUC, nftl2k->ReverseUnitTable[tailEUN],
               targetEUN, nftl2k->FirstFreePageTable[tailEUN], nftl2k->page_per_block);
        #ifdef TEST_POWEROFF_DEBUG
        printk("#### Inplace folding chain done. ####\n");
        #endif
    }
    else{
        /* 
        the last block is full, this should not happen normally,
        the only case is the system crashed when free previous blocks juring folding
        this can be repared by doing folding again
        */
        printk("!!! Warning !!! the last block is full when folding chain %d.\n", thisVUC);
        targetEUN = tailEUN;
        //return BLOCK_NIL;
    }


	/* OK. We've moved the whole lot into the new block. Now we have to free the original blocks. */

	/* 
    If we crash now, we get confused. However, this will not influent the normal operation
    this could be repared by doing folding again.
	*/
	thisEUN = nftl2k->EUNtable[thisVUC];

	/* For each block in the old chain (except the targetEUN of course),
	   free it and make it available for future use */
    #ifdef TEST_POWEROFF_DEBUG
    printk("#### Format blocks after folding chain. ####\n");
    #endif
	while ((thisEUN <= nftl2k->lastEUN) && (thisEUN != targetEUN)) {
		unsigned int EUNtmp;

		EUNtmp = nftl2k->ReplUnitTable[thisEUN];

		if (NFTL2K_formatblock(nftl2k, thisEUN) < 0) {
			/* could not erase : mark block as reserved
			 */
			nftl2k->ReplUnitTable[thisEUN] = BLOCK_RESERVED;
			nftl2k->ReverseUnitTable[thisEUN] = BLOCK_RESERVED;
		} else {

        //printk("->%d",thisEUN);
    
			/* correctly erased : mark it as free */
			nftl2k->ReplUnitTable[thisEUN] = BLOCK_FREE;
			nftl2k->ReverseUnitTable[thisEUN] = BLOCK_FREE;
			nftl2k->numfreeEUNs++;
            nftl2k->FirstFreePageTable[thisEUN] = 0;
		}
		thisEUN = EUNtmp;
        #ifdef TEST_POWEROFF_DEBUG
        mdelay(1000);
        #endif
	}
    #ifdef TEST_POWEROFF_DEBUG
    printk("#### Format blocks done. ####\n");
    #endif    
        
    //printk("\n");

	/* Make this the new start of chain for thisVUC */
	nftl2k->ReplUnitTable[targetEUN] = BLOCK_NIL;
	nftl2k->ReverseUnitTable[targetEUN] = BLOCK_HEAD;
	nftl2k->EUNtable[thisVUC] = targetEUN;

	return targetEUN;
}


/*
when a read/write/erase fail on a nand block, call this function once.
a NFTL2K_BBT_MARKBAD_THRESHOLD will be set to decide whether we are consider
this block to be bad. 
if entries exceed NFTL2K_BBT_MARKBAD_THRESHOLD from a same block,
we mark this block bad, in physical.

*/
void nftl2k_bbt_inc(struct NFTL2Krecord *nftl2k,int vuc,int block){
    int block_mask = block & NFTL2K_BBT_MASK;
    int next_block, pre_block;

    nftl2k->nftl2k_bbt[block_mask]++;
    printk("nftl2k: block (%d) fail already (%d) times\n", block,nftl2k->nftl2k_bbt[block_mask]);

    if (nftl2k->nftl2k_bbt[block_mask] >= NFTL2K_BBT_MARKBAD_THRESHOLD){
        /* error exceed limits, mark this block as bad */
        nftl2k->mbd.mtd->block_markbad(nftl2k->mbd.mtd, block * nftl2k->EraseSize);
        printk("nftl2k: Marked block (%d) as bad\n", block);
        nftl2k->nftl2k_bbt[block_mask] = 0;

        next_block = nftl2k->ReplUnitTable[block];
        pre_block = nftl2k->ReverseUnitTable[block];

        if (nftl2k->EUNtable[vuc] == block){ /* first logical block */
            if (pre_block != BLOCK_HEAD) {
                printk("nftl2k: confused.. head block %d has a previous block %d\n", block,pre_block);
            }
            /* ok let's update table */
            if (next_block < nftl2k->nb_blocks ) {
                nftl2k->EUNtable[vuc] = next_block;
                nftl2k->ReverseUnitTable[next_block] = BLOCK_HEAD;
            }
            else{
                nftl2k->EUNtable[vuc] = BLOCK_NIL;
            }
        }
        else{
            /* ok let's update table */
            if (next_block < nftl2k->nb_blocks ) {
                nftl2k->ReplUnitTable[pre_block] = next_block;
                nftl2k->ReverseUnitTable[next_block] = pre_block;
            }
            else{
                nftl2k->ReplUnitTable[pre_block] = BLOCK_NIL;
            }
        }

        nftl2k->ReplUnitTable[block]    = BLOCK_RESERVED;
        nftl2k->ReverseUnitTable[block] = BLOCK_RESERVED;

        NFTL2K_foldchain (nftl2k,vuc);
    }
}

static u16 NFTL2K_makefreeblock( struct NFTL2Krecord *nftl2k ,u16 * foldVUC)
{
	/* This is the part that needs some cleverness applied.
	   For now, I'm doing the minimum applicable to actually
	   get the thing to work.
	   Wear-levelling and other clever stuff needs to be implemented
	   and we also need to do some assessment of the results when
	   the system loses power half-way through the routine.
	*/
	u16 LongestChain = 0;
	u16 ChainLength = 0, thislen;
	u16 chain, EUN;
    * foldVUC = 0xffff;

	for (chain = 0; chain < ((u_int32_t)le32_to_cpu(nftl2k->MediaHdr.FormattedSize>>9)) / ((u_int32_t)nftl2k->EraseSize>>9); chain++) {
		EUN = nftl2k->EUNtable[chain];
		thislen = 0;

		while (EUN <= nftl2k->lastEUN) {
			thislen++;
			//printk("VUC %d reaches len %d with EUN %d\n", chain, thislen, EUN);
			EUN = nftl2k->ReplUnitTable[EUN] & 0x7fff;
			if (thislen > 0xff00) {
				printk("Endless loop in Virtual Chain %d: Unit %x\n",
				       chain, EUN);
			}
			if (thislen > 0xff10) {
				/* Actually, don't return failure. Just ignore this chain and
				   get on with it. */
				thislen = 0;
				break;
			}
		}

		if (thislen > ChainLength) {
			//printk("New longest chain is %d with length %d\n", chain, thislen);
			ChainLength = thislen;
			LongestChain = chain;
		}
        if ((thislen > CONFIG_NFTL_2K_FOLD_THRESHOLD)&&(CONFIG_NFTL_2K_FOLD_THRESHOLD)){
            /* exceed the threshold break */
            break;
        }
	}

	if (ChainLength < 2) {
		printk(KERN_WARNING "No Virtual Unit Chains available for folding. "
		       "Failing request\n");
		return 0xffff;
	}

    * foldVUC = LongestChain;
	return NFTL2K_foldchain (nftl2k, LongestChain);
}

/* NFTL2K_findwriteunit: Return the unit number into which we can write
                       for this block. Make it available if it isn't already
*/
static inline u16 NFTL2K_findwriteunit(struct NFTL2Krecord *nftl2k, 
              unsigned block, u16 * LastBlockNum)
{
	u16 lastEUN,secondlastEUN;
	u16 thisVUC = block / ((int)nftl2k->EraseSize >>9);
	//struct mtd_info *mtd = nftl2k->mbd.mtd;
	unsigned int writeEUN;
    struct nftl2k_oob oob;
	int silly;
    int chain_length;
    int page_pos;
    int retlen;
    int page;
    u16 foldedVUC, folded_targetEUN;
    char * temp_buf;
    int ret;

    temp_buf = nftl2k->movebuf;

	page_pos = ((block>> (nftl2k->page_shift-9) ) & (nftl2k->page_per_block -1));
    //page_pos = ((block>>2) & (nftl2k->page_per_block -1));
    secondlastEUN = BLOCK_NIL;
    * LastBlockNum = lastEUN = BLOCK_HEAD;
    chain_length = 0; /* count for folding , if length exceed limits we do folding */


	/* This condition catches the 0x[7f]fff cases, as well as
	   being a sanity check for past-end-of-media access
	*/
	writeEUN = nftl2k->EUNtable[thisVUC];
	silly = MAX_LOOPS;
	while (writeEUN <= nftl2k->lastEUN) {
        /* save last used EUN No. */
        * LastBlockNum = writeEUN;

        secondlastEUN = lastEUN;
        lastEUN = writeEUN;

        if (nftl2k->FirstFreePageTable[lastEUN] <= page_pos) break;
		/* Skip to next block in chain */
		writeEUN = nftl2k->ReplUnitTable[writeEUN];
        chain_length++;

		if (!silly--) {
			printk(KERN_WARNING "Infinite loop in Virtual Unit Chain 0x%x\n",
			       thisVUC);
			return 1;
		}
	}

    if (lastEUN <= nftl2k->nb_blocks) {
        /* not an empty chain */
        if (nftl2k->FirstFreePageTable[lastEUN] <= page_pos) {
            if (nftl2k->FirstFreePageTable[lastEUN] == page_pos) {
                /*
                yes, we can write directly to the lastEUN
                */
                return lastEUN;
            }
            /*
            ok, we need to do some copys
            */
            if (secondlastEUN <= nftl2k->lastEUN) {
                /* has valid data */
                #ifdef TEST_POWEROFF_DEBUG
                printk("#### Write unit found in a chain. Do some copys. ####\n");
                #endif
                NFTL2K_CopyBack(nftl2k, thisVUC, secondlastEUN, secondlastEUN, 
                                lastEUN,  nftl2k->FirstFreePageTable[lastEUN], page_pos );
          //      NFTL2K_fold_page_data(nftl2k,thisVUC,secondlastEUN,lastEUN, 
          //          nftl2k->FirstFreePageTable[lastEUN], page_pos);
                #ifdef TEST_POWEROFF_DEBUG
                printk("#### Copys done. ####\n");
                #endif
            }
            else{
                /*
                we have only one data block, so we need to fill all ZERO 
                from 'nftl2k->FirstFreePageTable[lastEUN]' to 'page_pos'
                */
                #ifdef TEST_POWEROFF_DEBUG
                printk("#### Write unit found in a chain. Write data 0. ####\n");
                #endif
                memset(temp_buf, 0, nftl2k->page_size);
                memset(&oob, 0xff, sizeof(struct nftl2k_oob));
                oob.b.Status = oob.b.Status1 = SECTOR_USED;

                for (page = nftl2k->FirstFreePageTable[lastEUN]; page < page_pos; page++) {
                    /* read the data */
                    oob.b.free_page_pos = oob.b.free_page_pos1 = page+1;            

                    ret = nftl2k_write(nftl2k->mbd.mtd, (nftl2k->EraseSize * lastEUN) +
                           (page * nftl2k->page_size), nftl2k->page_size, &retlen, temp_buf, (char *)&oob);
                    if (ret < 0) {
                        printk("nftl2k_write() Fail, in %s\n",__FUNCTION__);
                        #ifdef TEST_POWEROFF_DEBUG
                        mdelay(1000);
                        #endif
                    }
                }
                #ifdef TEST_POWEROFF_DEBUG
                printk("#### Data 0 written. ####\n");
                #endif
            }
            /* update free page table */
            nftl2k->FirstFreePageTable[lastEUN] = page_pos;
            return lastEUN;
        }
        else{
            dprintk("VUC %d lastEUN %d's FirstFreePageTable is %d, can not write Page %d\n",
                    thisVUC,lastEUN,nftl2k->FirstFreePageTable[lastEUN],page_pos);
            /*
            no, we need to copy all data from lastEUN to a new target block;
            */
        }
    }

	/* OK. We didn't find one in the existing chain, or there
	   is no existing chain. */

	/* Try to find an already-free block */
	writeEUN = NFTL2K_findfreeblock(nftl2k, 0);

	if (writeEUN >= nftl2k->nb_blocks ) {
		/* That didn't work - there were no free blocks just
		   waiting to be picked up. We're going to have to fold
		   a chain to make room.
		*/

		//printk("Write to VirtualUnitChain %d, calling makefreeblock()\n", thisVUC);
        folded_targetEUN = NFTL2K_makefreeblock(nftl2k,&foldedVUC);

        if (foldedVUC == thisVUC){
             /* 
             if the folded chain is what we are now writing , 
             the lastEUN should changed to the folding target EUN
             */
             lastEUN = folded_targetEUN;
         }

		if (folded_targetEUN >= nftl2k->nb_blocks ) {
			/* Ouch. This should never happen - we should
			   always be able to make some room somehow.
			   If we get here, we've allocated more storage
			   space than actual media, or our makefreeblock
			   routine is missing something.
			*/
			printk(KERN_WARNING "Cannot make free space.\n");
			return BLOCK_NIL;
		}

		/* find free block again, this time we should have a empty block */
        writeEUN = NFTL2K_findfreeblock(nftl2k, 0);
	}

    dprintk("VUC %d find a new ENU %d to write Page %d\n",thisVUC,writeEUN,page_pos);

	/* We've found a free block. Insert it into the chain. */
    if (lastEUN <= nftl2k->nb_blocks) {
        /* not an empty chain  */
        if (page_pos > 0) {
            /* we need to copy page0 - page_pos data to the new block */
            #ifdef TEST_POWEROFF_DEBUG
            printk("#### A new write unit found. Do some copys. ####\n");
            #endif
            NFTL2K_CopyBack(nftl2k, thisVUC, lastEUN,
                    lastEUN, writeEUN,  0, page_pos );
            #ifdef TEST_POWEROFF_DEBUG
            printk("#### Copys done. ####\n");
            #endif
        }

        /* update links */
        nftl2k->ReplUnitTable[lastEUN] = writeEUN;
        nftl2k->ReplUnitTable[writeEUN] = BLOCK_NIL;
        nftl2k->ReverseUnitTable[writeEUN] = lastEUN;
    }
    else{
        /* this is a new EUN in a new VUC */
        if (page_pos != 0) {
           /* make the link at page 0 */
            #ifdef TEST_POWEROFF_DEBUG
            printk("#### A new write unit found. Write data 0. ####\n");
            #endif
            memset(&oob, 0xff, sizeof(struct nftl2k_oob));
            memset(temp_buf, 0, nftl2k->page_size);
            oob.b.Status = oob.b.Status1 = SECTOR_USED; // this page do not content valid data

            for (page = 0; page < page_pos; page++) {
                /* read the data */
                oob.b.free_page_pos = oob.b.free_page_pos1 = page+1;            

                if (unlikely(page == 0)){
                    /*this page is the most important one, because it has uci info*/
                    oob.u.a.ReplUnitNum = oob.u.a.SpareReplUnitNum = BLOCK_HEAD;  /* this is a HEAD block */
                    oob.u.a.VirtUnitNum = oob.u.a.SpareVirtUnitNum = cpu_to_le16(thisVUC);
                    oob.u.b.EraseMark = oob.u.b.EraseMark1 = ERASE_MARK;/* not wear-info here */
                    dprintk("findwriteunit: new block VUC(%d) block (%d)->(%d)\n",
                           thisVUC,writeEUN,BLOCK_HEAD);
                }

                ret = nftl2k_write(nftl2k->mbd.mtd, (nftl2k->EraseSize * writeEUN) +
                       (page * nftl2k->page_size), nftl2k->page_size, &retlen, temp_buf, (char *)&oob);
                if (ret < 0) {
                    printk("nftl2k_write() Fail, in %s\n",__FUNCTION__);
                }
                #ifdef TEST_POWEROFF_DEBUG
                mdelay(1000);
                #endif
            }
            #ifdef TEST_POWEROFF_DEBUG
            printk("#### Data 0 written. ####\n");
            #endif       
        }           

        /* update links */
        nftl2k->EUNtable[thisVUC] = writeEUN;
        nftl2k->ReplUnitTable[writeEUN] = BLOCK_NIL;
        nftl2k->ReverseUnitTable[writeEUN] = BLOCK_HEAD;
    }

    /* update free page table */
    nftl2k->FirstFreePageTable[writeEUN] = page_pos;
	return writeEUN;

}

u16 NFTL2K_find_read_EUN(struct NFTL2Krecord *nftl2k, u16 thisVUC, int page_pos){

	int thisEUN;
    u16 readEUN;
    int silly;

	/* Scan to find the Erase Unit which holds the actual data for each
	   2048-byte page within the Chain. stored in BlockMap[]
	*/
    thisEUN = nftl2k->EUNtable[thisVUC];
    readEUN = thisEUN;
	silly = MAX_LOOPS;
	while (thisEUN <= nftl2k->lastEUN ) {

        if (nftl2k->FirstFreePageTable[thisEUN] > page_pos) {
            readEUN = thisEUN;
        }

		thisEUN = nftl2k->ReplUnitTable[thisEUN];

        if (!silly--) {
            printk(KERN_WARNING "Infinite loop in Virtual Unit Chain 0x%x\n",
                   thisVUC);
            return readEUN;
        }
	}

    return readEUN;
}

int nftl2k_writeback_lastpage(struct NFTL2Krecord *nftl2k){

	struct nftl2k_oob oob;
    size_t retlen;
	struct mtd_info *mtd = nftl2k->mbd.mtd;
    int res;
    u16 readEUN,thisEUN,tempEUN;
    char * tmp_buf;
    int sector;

    tmp_buf = nftl2k->movebuf;

    memset((char *)&oob, 0xff, sizeof(struct nftl2k_oob));
    oob.b.Status = oob.b.Status1 = SECTOR_USED;
    oob.b.free_page_pos = oob.b.free_page_pos1 = nftl2k->Last_Write_Page+1;

    /* check if the write target page is the first free page in the block */
    if( nftl2k->FirstFreePageTable[nftl2k->Last_Write_EUN] != nftl2k->Last_Write_Page){
        printk ("nftl2k_writeback_lastpage: get confused the FirstFreePageTable[%d] = %d, "
                "but the target write page is %d. what should i do??\n",nftl2k->Last_Write_EUN,
                nftl2k->FirstFreePageTable[nftl2k->Last_Write_EUN],nftl2k->Last_Write_Page
                );
    }
    /*
    check if we need read back the org page data
    
    */
    if (unlikely(nftl2k->write_buf_status !=  ((1<<(nftl2k->page_size/512)) - 1)) ) {
        // dprintk("+");
        /* yes we need do that */
        readEUN = NFTL2K_find_read_EUN(nftl2k,nftl2k->Last_Write_VUC,nftl2k->Last_Write_Page);

        if (readEUN < nftl2k->nb_blocks) {
            res = mtd->read(mtd, (readEUN * nftl2k->EraseSize) + (nftl2k->Last_Write_Page*nftl2k->page_size)
                                , nftl2k->page_size, &retlen, tmp_buf);
            // to check if someone else has holding the nand chip,
            // if so, wait until it finishes
            if (res == -ABORT_WIRTE_BACK_SIGNAL) {  return -ABORT_WIRTE_BACK_SIGNAL;  }

            if (res < 0 && res != -EUCLEAN) {
                res = mtd->read(mtd, (readEUN * nftl2k->EraseSize) + (nftl2k->Last_Write_Page*nftl2k->page_size)
                                , nftl2k->page_size, &retlen, tmp_buf);
                if (res < 0){
                    printk(KERN_WARNING
                           "nftl2k_writeback_lastpage(): Cannot read page %d in VUC %d, EUN %d, offs 0x%9Lx\n",
                           nftl2k->Last_Write_Page,nftl2k->Last_Write_VUC,readEUN,
                            (readEUN * nftl2k->EraseSize) + (nftl2k->Last_Write_Page*nftl2k->page_size) );
                    //nftl2k_bbt_inc(nftl2k,nftl2k->Last_Write_VUC,readEUN);
                    return -EIO;
                }
            }

        }
        else{
            /* */
            memset(tmp_buf,0x0,nftl2k->page_size);
        }

        for (sector = 0 ; sector < (1<<(nftl2k->page_shift-9)); sector++ ) {
            if ((nftl2k->write_buf_status & (1<<sector)) == 0) { // none cached
                memcpy(nftl2k->write_buf+ (sector*512),tmp_buf+(sector*512),512);  }
        }

    }
    #if 0
    else{
        printk("-");
    }
    #endif
    
    if (unlikely(nftl2k->Last_Write_Page == 0)) {
        /* the cached page is page-0, so we must write the uci info */
        oob.u.a.ReplUnitNum = oob.u.a.SpareReplUnitNum = nftl2k->Last_Pre_Block;
        oob.u.a.VirtUnitNum = oob.u.a.SpareVirtUnitNum = nftl2k->Last_Write_VUC;
        oob.u.b.EraseMark = oob.u.b.EraseMark1 = ERASE_MARK;
     //   dprintk("writeback_lastpage: new block VUC(%d) block (%d)->(%d)\n",
     //          nftl2k->Last_Write_VUC,nftl2k->Last_Write_EUN,nftl2k->Last_Pre_Block);
    }

    /* 
    if the block is written full, 
    free the previous blocks, and mark this block as a "Late HEAD Block" in the last page
    */
    if (nftl2k->Last_Write_Page == nftl2k->page_per_block - 1) {

        thisEUN = nftl2k->ReverseUnitTable[nftl2k->Last_Write_EUN];

        while ( thisEUN < nftl2k->nb_blocks ) {
            tempEUN = nftl2k->ReverseUnitTable[thisEUN];
            if (NFTL2K_formatblock(nftl2k, thisEUN) < 0) {
                // could not erase : mark block as reserved
                nftl2k->ReplUnitTable[thisEUN] = BLOCK_RESERVED;
                nftl2k->ReverseUnitTable[thisEUN] = BLOCK_RESERVED;
            } 
            else {// correctly erased : mark it as free 
                nftl2k->ReplUnitTable[thisEUN] = BLOCK_FREE;
                nftl2k->ReverseUnitTable[thisEUN] = BLOCK_FREE;
                nftl2k->numfreeEUNs++;
                nftl2k->FirstFreePageTable[thisEUN] = 0;
            }
            thisEUN = tempEUN;
        }

        // === Make this the new start of chain for thisVUC 
        nftl2k->ReverseUnitTable[nftl2k->Last_Write_EUN] = BLOCK_HEAD;
        nftl2k->EUNtable[nftl2k->Last_Write_VUC] = nftl2k->Last_Write_EUN;

        oob.u.a.ReplUnitNum = oob.u.a.SpareReplUnitNum = BLOCK_HEAD;
    }

    /* write that page */
    res = nftl2k_write(nftl2k->mbd.mtd, (nftl2k->Last_Write_EUN * nftl2k->EraseSize) 
                       + (nftl2k->Last_Write_Page*nftl2k->page_size),
           nftl2k->page_size, &retlen, (char *)nftl2k->write_buf, (char *)&oob);

    /* links are already done at NFTL2K_findwriteunit() */

    if (res == -ABORT_WIRTE_BACK_SIGNAL) { 
        return -ABORT_WIRTE_BACK_SIGNAL;
        dprintk("Get ABORT_WIRTE_BACK_SIGNAL!!!\n")
    }
    if (res < 0) {
        printk("nftl2k_write() Fail, in %s\n",__FUNCTION__);
    }

    nftl2k->FirstFreePageTable[nftl2k->Last_Write_EUN]++;
    nftl2k->write_buf_status = 0;
#if 1
    dprintk("wb_page: VUC %d ENU %d Page %d, FFtable = %d\n",
           nftl2k->Last_Write_VUC,nftl2k->Last_Write_EUN,nftl2k->Last_Write_Page,
            nftl2k->FirstFreePageTable[nftl2k->Last_Write_EUN]);
#endif
//    #if NFTL2K_CORE_DEBUG   /* This is for debug */
//    if (nftl2k->Last_Write_Page == 0) {
//        nftl2k_dbg_EUNtable_dump(nftl2k);
//    }
//    #endif    /* This is for debug */
    
    return WIRTE_BACK_LP_OK;
}

/* ======================================================== */
/* =========== Actual NFTL2K access routines ============== */
/* ======================================================== */

static int nftl2k_writeblock(struct mtd_blktrans_dev *mbd, unsigned long block,
			   char *buffer)
{
	struct NFTL2Krecord *nftl2k = (void *)mbd;
	//struct mtd_info *mtd = nftl2k->mbd.mtd;
	u16 writeEUN;
	u16 thisVUC = block / ((int)nftl2k->EraseSize >> 9);
	//u64 pageofs = ((block>>2) * 2048) & (nftl2k->EraseSize -1);
    int sector_in_page = ((1<<(nftl2k->page_shift-9)) - 1) & block;
	u64 pageofs = ((block>> (nftl2k->page_shift-9) ) * nftl2k->page_size) & (nftl2k->EraseSize -1);
	//size_t retlen;
    u16 LastBlockNum;
    int do_cache_ops = 0;

    /* write this page back if time out x seconds */
    mod_timer(&nftl2k->timer, jiffies + WRITE_BACK_TIMEOUT_SECONDS*100);

    //printk(">");
    dprintk("Write Req LBA %d, VUC %d Page %d\n",
             block,thisVUC,((int)(pageofs>>nftl2k->page_shift)&(nftl2k->page_per_block-1)));

    /* check if this page is already cached */
    if (nftl2k->write_buf_status > 0) {
        /* already cached */
        if ((nftl2k->Last_Write_VUC == thisVUC)&&
            ( nftl2k->Last_Write_Page == (pageofs>>nftl2k->page_shift))){
            /* we visit the same page, so do cached write*/
            do_cache_ops = 1;
        }
        else{
            /* we visit the different page, see if we need to write back the last one */
            int max_try;
            max_try = 20;
            while (max_try--) {
                if (nftl2k_writeback_lastpage(nftl2k) != WIRTE_BACK_LP_OK){
                    printk("!!! Warning !! AFTL Writeback Last Page Fail in nftl2k_writeblock()!!!\n");
                    mdelay(100);
                }
                else break;
            }
        }
    }

    if(!do_cache_ops){
        /* it is not a cached write, so find which block should we write to */
        writeEUN = NFTL2K_findwriteunit(nftl2k, block, &LastBlockNum);

        // dprintk("Not a cached write dev_block %d, find writeEUN = %d\n", block,writeEUN);

        if (writeEUN > nftl2k->nb_blocks) {
            printk(KERN_WARNING
                   "NFTL2K_writeblock(): Cannot find block to write to\n");
            /* If we _still_ haven't got a block to use, we're screwed */
            return 1;
        }
        
        nftl2k->Last_Write_EUN = writeEUN;
        nftl2k->Last_Pre_Block = LastBlockNum;
        nftl2k->Last_Write_VUC = thisVUC;
        nftl2k->Last_Write_Page = (int)(pageofs>>nftl2k->page_shift);
    }

    #if  0  /* This is for debug */
    dprintk("Cached write VUC %d block %d\n",
             thisVUC,(block & ((nftl2k->EraseSize / 512)-1)) );
    #endif    /* This is for debug */
    
    /* write target dev_block data in the page buffer, we don't handle cache write now */
    memcpy(nftl2k->write_buf + (sector_in_page*512),buffer,512);

    /* mark this buffer as write cached */
    nftl2k->write_buf_status |= (1<<sector_in_page);

    if (unlikely((nftl2k->Last_Read_VUC == thisVUC)&&( nftl2k->Last_Read_Page == nftl2k->Last_Write_Page ))){
        /* we visit the same page, so update the read cache */
        memcpy(nftl2k->read_buf + (sector_in_page*512),buffer,512);
        return 0;
    }
	else return 0;
}

static int nftl2k_readblock(struct mtd_blktrans_dev *mbd, unsigned long block,
			  char *buffer)
{
	struct NFTL2Krecord *nftl2k = (void *)mbd;
	struct mtd_info *mtd = nftl2k->mbd.mtd;
	u16 lastgoodEUN;
    loff_t ptr;
    int res;
	u16 thisVUC = block / ((int)nftl2k->EraseSize >> 9);
	//u16 thisEUN = nftl2k->EUNtable[block / ((int)nftl2k->EraseSize >> 9)];
	//u64 pageofs = ((block>>2) * 2048) & (nftl2k->EraseSize -1);
    int sector_in_page = ((1<<(nftl2k->page_shift-9)) - 1) & block;
	u64 pageofs = ((block>> (nftl2k->page_shift-9) ) * nftl2k->page_size) & (nftl2k->EraseSize -1);
    
    int do_cache_ops = 0;

    /*
     check if this page is already cached in write buffer 
     if so, read the sector in write buffer and return
     */
    if (unlikely((nftl2k->Last_Write_VUC == thisVUC)&&( (int)(pageofs>>nftl2k->page_shift) == nftl2k->Last_Write_Page ))){
        if ((1<<sector_in_page) & nftl2k->write_buf_status) {
            /* if the target page is already cached in write_buf */
            memcpy(buffer,nftl2k->write_buf + (sector_in_page*512),512);
            return 0;
        }
    }

    /* check if this page is already cached */
    if (nftl2k->read_buf_status > 0) {
        /* already cached */
        if ((nftl2k->Last_Read_VUC == thisVUC)&&( nftl2k->Last_Read_Page == (int)(pageofs>>nftl2k->page_shift))){
            /* we visit the same page, so do cached write*/
            do_cache_ops = 1;
        }
    }

    /* read cache if this page is cached*/
    if (do_cache_ops) {
    #if  0  /* This is for debug */
        dprintk("Cached Read VUC %d dev_block %d Page %d\n",
                 thisVUC,(block & ((nftl2k->EraseSize / 512)-1)),(int)(pageofs>>nftl2k->page_shift));
    #endif    /* This is for debug */
        memcpy(buffer,nftl2k->read_buf + (sector_in_page*512),512);
        return 0;
    }

    /* do read */
    lastgoodEUN = NFTL2K_find_read_EUN(nftl2k, thisVUC, (int)(pageofs>>nftl2k->page_shift) );

	if (lastgoodEUN > nftl2k->nb_blocks) {
		/* the requested block is not on the media, return all 0x00 */
      //  dprintk("Read a non-data VUC %d block %d\n", 
      //          thisVUC,(block & ((nftl2k->EraseSize / 512)-1)));
		memset(buffer, 0, 512);
	} else {
		size_t retlen;

    #if  1  /* This is for debug */
        dprintk("Non-Cached Read VUC %d block %d in ENU %d\n", 
                thisVUC,(block & ((int)(nftl2k->EraseSize / 512)-1)),lastgoodEUN);
    #endif    /* This is for debug */

		ptr = (lastgoodEUN * nftl2k->EraseSize) + pageofs;
		res = mtd->read(mtd, ptr, nftl2k->page_size, &retlen, nftl2k->read_buf);
        if (res < 0 && res != -EUCLEAN){
            res = mtd->read(mtd, ptr, nftl2k->page_size, &retlen, nftl2k->read_buf); // re-try
            if (res < 0 ){
            printk(KERN_WARNING
                   "NFTL2K_readblock(): Cannot read page %d in VUC %d, EUN %d, offs 0x%9Lx\n",
                   (int)(pageofs>>nftl2k->page_shift),thisVUC,lastgoodEUN,(u64)ptr);
            //nftl2k_bbt_inc(nftl2k,thisVUC,lastgoodEUN);
            return -EIO;
            }
        }
        memcpy(buffer,nftl2k->read_buf + (sector_in_page*512),512);

        /* update current page after read data */
        nftl2k->Last_Read_VUC = thisVUC;
        nftl2k->Last_Read_Page = (int)(pageofs>>nftl2k->page_shift);
        nftl2k->read_buf_status = PAGE_READ_CACHED;

	}
	return 0;
}

static int nftl2k_getgeo(struct mtd_blktrans_dev *dev,  struct hd_geometry *geo)
{
	struct NFTL2Krecord *nftl2k = (void *)dev;

	geo->heads = nftl2k->heads;
	geo->sectors = nftl2k->sectors;
	geo->cylinders = nftl2k->cylinders;

	return 0;
}

/****************************************************************************
 *
 * Module stuff
 *
 ****************************************************************************/


static struct mtd_blktrans_ops nftl2k_tr = {
	.name		= "nftl2k",
	.major		= NFTL2K_MAJOR,
	.part_bits	= NFTL2K_PARTN_BITS,
	.blksize 	= 512,
	.flush		= nftl2k_flush,
	.release	= nftl2k_release,
	.getgeo		= nftl2k_getgeo,
	.readsect	= nftl2k_readblock,
	.writesect	= nftl2k_writeblock,
	.add_mtd	= nftl2k_add_mtd,
	.remove_dev	= nftl2k_remove_dev,
	.owner		= THIS_MODULE,
};

extern char nftl2kmountrev[];

static int __init init_nftl2k(void)
{
	printk(KERN_INFO "NFTL2K driver: nftl2kcore.c $Revision: 0.98 $, nftl2kmount.c %s\n", nftl2kmountrev);

	return register_mtd_blktrans(&nftl2k_tr);
}

static void __exit cleanup_nftl2k(void)
{
	deregister_mtd_blktrans(&nftl2k_tr);
}

module_init(init_nftl2k);
module_exit(cleanup_nftl2k);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("He Yong <hoffer@sjtu.org>");
MODULE_DESCRIPTION("Support code for Big Page NAND Flash Translation Layer");

