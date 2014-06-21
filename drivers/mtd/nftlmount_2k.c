/*
 * NFTL2K mount code with extensive checks
 * for Mixed NAND Flash Translation Layer 
 * (c) 2007 Alpscale, Inc.                             
 * Author: He Yong <hoffer@sjtu.org>                   
 * $Id: nftlmount_2k.c,v 0.12 2007/09/24 15:38:21       
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

 /* 
 ====================   Usage   ======================
 
 1, make a MTD partition, name start with "NFTL2K"
     e.g. at mtdparts= ...,64M@256M(NFTL2K),...
 2, before first mount a NFTL2K partition, you must format it.
    add "nftlfmt=1" in kernel cmdline before "mtdparts=..."
    remove "nftlfmt=1" or set "nftlfmt=0" after a successful format
 3, in most cases, you will find /dev/nftl2ka at your dev directory
    after udev is initialized. now you can do FDISK or format
 4, format your own partitions
    (take FAT as an example:)
    /$mkdosfs /dev/nftl2ka
 5, mount your fs
    (take FAT as an example:)
    mount -t vfat /dev/nftl2ka /mnt/fat
 
 */


#include <linux/kernel.h>
#include <asm/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nftl_2k.h>

#define MAX_LOOPS 10000

char nftl2kmountrev[]="$Revision: 0.12 $";


/* find_boot_record: Find the NFTL2K Media Header and its Spare copy which contains the
 *	various device information of the NFTL2K partition and Bad Unit Table. Update
 *	the ReplUnitTable[] table accroding to the Bad Unit Table. ReplUnitTable[]
 *	is used for management of Erase Unit in other routines in nftl2k.c and nftl2kmount.c
 */
static int find_boot_record(struct NFTL2Krecord *nftl2k)
{
	struct nftl2k_oob page_oob;
	unsigned int block, boot_record_count = 0;
	size_t retlen;
	struct NFTL2KMediaHeader *mh = &nftl2k->MediaHdr;
	struct mtd_info *mtd = nftl2k->mbd.mtd;
	unsigned int i;
    int aftl_formated;
    char * databuf_empty;

    aftl_formated = 0;
    databuf_empty = nftl2k->movebuf;
        /* Assume logical EraseSize == physical erasesize for starting the scan.
	   We'll sort it out later if we find a MediaHeader which says otherwise */
	nftl2k->EraseSize = nftl2k->mbd.mtd->erasesize;
	nftl2k->page_size = nftl2k->mbd.mtd->writesize;

	nftl2k->page_shift = (nftl2k->page_size>>11) + 10; // 2KB -> 11, 4KB -> 12, other wise wrong!! check later

    if (( nftl2k->page_size != (1<<nftl2k->page_shift)) || (nftl2k->page_size > MAX_PAGE_SIZE )){
        printk("NFTL2K ERROR!!!! find_boot_record() Wrong Page Size %d and page shift %d.\n",nftl2k->page_size,nftl2k->page_shift);
        return -1; /* Error */
    }

    nftl2k->nb_blocks = (((u_int32_t)(nftl2k->mbd.mtd->size >> 9)) / ((u_int32_t)nftl2k->EraseSize>>9)) - 1;
    nftl2k->page_per_block = nftl2k->mbd.mtd->erasesize / nftl2k->page_size;

	nftl2k->MediaUnit = BLOCK_NIL;
	nftl2k->SpareMediaUnit = BLOCK_NIL;

	/* search for a valid boot record */
	for (block = 0; block < nftl2k->nb_blocks; block++) {
		int ret;

    __find_media_header_again:

		/* Check for ANAND header first. Then can whinge if it's found but later
		   checks fail */
		ret = mtd->read(mtd, block * nftl2k->EraseSize, nftl2k->page_size,
				&retlen, nftl2k->read_buf);
        if (ret < 0 && ret != -EUCLEAN) {
            ret = mtd->read(mtd, block * nftl2k->EraseSize, nftl2k->page_size,
				&retlen, nftl2k->read_buf);
            if (ret < 0){
                printk("NFTL2K: find_boot_record() Failed to Read ANAND Header Twice.\n");
            }
        }
		/* We ignore ret in case the ECC of the MediaHeader is invalid
		   (which is apparently acceptable) */
		if (retlen != nftl2k->page_size) {
			static int warncount = 5;

			if (warncount) {
				printk(KERN_WARNING "Block read at 0x%09Lx of mtd%d failed: %d\n",
				       block * nftl2k->EraseSize, nftl2k->mbd.mtd->index, ret);
				if (!--warncount)
					printk(KERN_WARNING "Further failures for this block will not be printed\n");
			}
			continue;
		}

		if (retlen < 6 || memcmp(nftl2k->read_buf, "ANAND", 6)) {
			/* ANAND\0 not found. Continue */

            /* no valid header */
            memset (databuf_empty, 0xff , nftl2k->page_size);
            if (memcmp(nftl2k->read_buf,databuf_empty,nftl2k->page_size)) {
                /* no valid header */
                printk("NFTL2K: No Media Header at Block %d\n",block);
                continue;
            }
            else{
                /* empty header, we format it */
                if (aftl_formated == 1) {
                    printk("AFTL: Can not Format AFTL\n");
                    return -1;
                }
                printk("NFTL2K: Empty Media Header at Block %d, Format it.\n",block);
                format_mtd(nftl2k,mtd);
                aftl_formated = 1;
                goto __find_media_header_again;
            }
#if 0
			printk(KERN_DEBUG "ANAND header not found at 0x%x in mtd%d\n",
			       block * nftl2k->EraseSize, nftl2k->mbd.mtd->index);
#endif
			continue;
		}

		/* To be safer with BIOS, also use erase mark as discriminant */
		if ((ret = nftl2k_read_oob(mtd, block * nftl2k->EraseSize,sizeof(struct nftl2k_oob), &retlen,
					 (char *)&page_oob) < 0)) {
			printk(KERN_WARNING "ANAND header found at 0x%09Lx in mtd%d, but OOB data read failed (err %d)\n",
			       block * nftl2k->EraseSize, nftl2k->mbd.mtd->index, ret);
			continue;
		}

		/* OK, we like it. */
		/* This is the first we've seen. Copy the media header structure into place */
		memcpy(mh, nftl2k->read_buf, sizeof(struct NFTL2KMediaHeader));
        nftl2k_dbg_MediaHeader_dump(mh);

		/* Do some sanity checks on it */
		nftl2k->nb_boot_blocks = le16_to_cpu(mh->FirstPhysicalEUN);
		if ((nftl2k->nb_boot_blocks + 2) >= nftl2k->nb_blocks) {
			printk(KERN_NOTICE "NFTL2K Media Header sanity check failed:\n");
			printk(KERN_NOTICE "nb_boot_blocks (%d) + 2 > nb_blocks (%d)\n",
			       nftl2k->nb_boot_blocks, nftl2k->nb_blocks);
			return -1;
		}

		nftl2k->numvunits = ((u32)(le32_to_cpu(mh->FormattedSize) >> 9)) / ((u32)nftl2k->EraseSize>>9);
		if (nftl2k->numvunits > (nftl2k->nb_blocks - nftl2k->nb_boot_blocks - 2)) {
			printk(KERN_NOTICE "NFTL2K Media Header sanity check failed:\n");
			printk(KERN_NOTICE "numvunits (%d) > nb_blocks (%d) - nb_boot_blocks(%d) - 2\n",
			       nftl2k->numvunits, nftl2k->nb_blocks, nftl2k->nb_boot_blocks);
			return -1;
		}

		nftl2k->mbd.size  = nftl2k->numvunits * (nftl2k->EraseSize / 512);

		/* If we're not using the last sectors in the device for some reason,
		   reduce nb_blocks accordingly so we forget they're there */
		nftl2k->nb_blocks = le16_to_cpu(mh->NumEraseUnits) + le16_to_cpu(mh->FirstPhysicalEUN);

		/* XXX: will be suppressed */
		nftl2k->lastEUN = nftl2k->nb_blocks - 1;

		/* memory alloc */
		nftl2k->EUNtable = kmalloc(nftl2k->nb_blocks * sizeof(u16), GFP_KERNEL);
		if (!nftl2k->EUNtable) {
			printk(KERN_NOTICE "NFTL2K: allocation of EUNtable failed\n");
			return -ENOMEM;
		}

		nftl2k->ReplUnitTable = kmalloc(nftl2k->nb_blocks * sizeof(u16), GFP_KERNEL);
		if (!nftl2k->ReplUnitTable) {
			kfree(nftl2k->EUNtable);
			printk(KERN_NOTICE "NFTL2K: allocation of ReplUnitTable failed\n");
			return -ENOMEM;
		}

        /* add for reverse temp storage */
		nftl2k->ReverseUnitTable = kmalloc(nftl2k->nb_blocks * sizeof(u16), GFP_KERNEL);
		if (!nftl2k->ReverseUnitTable) {
			kfree(nftl2k->EUNtable);
			kfree(nftl2k->ReplUnitTable);
			printk(KERN_NOTICE "NFTL2K: allocation of ReverseUnitTable failed\n");
			return -ENOMEM;
		}

		nftl2k->FirstFreePageTable = kmalloc(nftl2k->nb_blocks * sizeof(char), GFP_KERNEL);
		if (!nftl2k->FirstFreePageTable) {
			printk(KERN_NOTICE "NFTL2K: allocation of FirstFreePageTable failed\n");
			return -ENOMEM;
		}

    #ifdef CONFIG_NFTL2K_WEAR_LEVELING
        /* give statistics about wear-info */
        nftl2k->WearInfoTable = kmalloc(nftl2k->nb_blocks * sizeof(u16), GFP_KERNEL);
        if (!nftl2k->WearInfoTable) {
            printk(KERN_NOTICE "NFTL2K: allocation of WearInfoTable failed\n");
            return -ENOMEM;
        }
        memset(nftl2k->WearInfoTable,0x0,nftl2k->nb_blocks * sizeof(u16));
    #endif

		/* mark the bios blocks (blocks before NFTL2K MediaHeader) as reserved */
		for (i = 0; i < nftl2k->nb_boot_blocks; i++){
			nftl2k->ReplUnitTable[i] = BLOCK_RESERVED;
			nftl2k->ReverseUnitTable[i] = BLOCK_RESERVED;
        }
		/* mark all remaining blocks as potentially containing data */
		for (; i < nftl2k->nb_blocks; i++) {
			nftl2k->ReplUnitTable[i] = BLOCK_NOTEXPLORED;
			nftl2k->ReverseUnitTable[i] = BLOCK_NOTEXPLORED;
		}

		/* Mark this boot record (NFTL2K MediaHeader) block as reserved */
		nftl2k->ReplUnitTable[block] = BLOCK_RESERVED;
		nftl2k->ReverseUnitTable[block] = BLOCK_RESERVED;

		/* read the Bad Erase Unit Table and modify ReplUnitTable[] accordingly */
        printk("NFTL2K: Scan for Bad Blocks:\n");
		for (i = 0; i < nftl2k->nb_blocks; i++) {
			if (nftl2k->mbd.mtd->block_isbad(nftl2k->mbd.mtd, i * nftl2k->EraseSize)){
				nftl2k->ReplUnitTable[i] = BLOCK_RESERVED;
				nftl2k->ReverseUnitTable[i] = BLOCK_RESERVED;
                printk("(%d)\t",i);
            }
		}
        printk("\n");
        memset(nftl2k->nftl2k_bbt,0x0,NFTL2K_BBT_SIZE);
        memset(nftl2k->FirstFreePageTable,0x0,nftl2k->nb_blocks * sizeof(char));

		nftl2k->MediaUnit = block;
		boot_record_count++;
        /* why should we check every block ??? found a valid one is OK -- hoffer*/
        return boot_record_count?0:-1;

	} /* foreach (block) */

	return boot_record_count?0:-1;
}

static int memcmpb(void *a, int c, int n)
{
	int i;
	for (i = 0; i < n; i++) {
		if (c != ((unsigned char *)a)[i])
			return 1;
	}
	return 0;
}

/* check_free_sector: check if a sector oob is actually FREE*/
static int check_free_oob_sectors(struct NFTL2Krecord *nftl2k, unsigned int address, int len)
{
	u8 buf[nftl2k->mbd.mtd->oobsize];
	struct mtd_info *mtd = nftl2k->mbd.mtd;
	size_t retlen;
	int i;

	for (i = 0; i < len; i += nftl2k->page_size) {
			if(nftl2k_read_oob(mtd, address, mtd->ecclayout->oobavail,
					 &retlen, buf) < 0)
				return -1;
			if (memcmpb(buf, 0xff, mtd->ecclayout->oobavail) != 0)
				return -1;
		address += nftl2k->page_size;
	}

	return 0;
}

/* NFTL2K_format: format a Erase Unit by erasing ALL Erase Zones in the Erase Unit and
 *              Update NFTL2K metadata. Each erase operation is checked with check_free_sectors
 *
 * Return: 0 when succeed, -1 on error.
 *
 *  ToDo: 1. Is it neceressary to check_free_sector after erasing ??
 */
int NFTL2K_formatblock(struct NFTL2Krecord *nftl2k, int block)
{
	struct erase_info *instr = &nftl2k->instr;
	struct mtd_info *mtd = nftl2k->mbd.mtd;

    /* bad block */
	if(nftl2k->mbd.mtd->block_isbad(nftl2k->mbd.mtd, block * nftl2k->EraseSize)) {
		printk("nftl2k Warning: try to format a bad block %d\n", block);
        return 0;
    }

    printk(" (%d)", block);

	memset(instr, 0, sizeof(struct erase_info));
	instr->mtd = nftl2k->mbd.mtd;
	instr->addr = block * nftl2k->EraseSize;
	instr->len = nftl2k->EraseSize;

	/* XXX: use async erase interface, XXX: test return code */
	mtd->erase(mtd, instr);

	if (instr->state == MTD_ERASE_FAILED) {
		printk("Error while formatting block %d\n", block);
		goto fail;
	}
	return 0;
fail:
	/* could not format, update the bad block table (caller is responsible
	   for setting the ReplUnitTable to BLOCK_RESERVED on failure) */
	nftl2k->mbd.mtd->block_markbad(nftl2k->mbd.mtd, instr->addr);
	return -1;
}


/* calc_chain_lenght: Walk through a Virtual Unit Chain and estimate chain length */
static int calc_chain_length(struct NFTL2Krecord *nftl2k, unsigned int first_block)
{
	unsigned int length = 0, block = first_block;

	for (;;) {
		length++;
		/* avoid infinite loops, although this is guaranted not to
		   happen because of the previous checks */
		if (length >= nftl2k->nb_blocks) {
			printk("nftl2k: length too long %d !\n", length);
			break;
		}

		block = nftl2k->ReplUnitTable[block];
		if (!(block == BLOCK_NIL || block == BLOCK_NOTEXPLORED || block < nftl2k->nb_blocks))
			printk("incorrect ReplUnitTable[] : %d\n", block);
		if (block == BLOCK_NIL || block == BLOCK_NOTEXPLORED || block >= nftl2k->nb_blocks)
			break;
	}
	return length;
}

/* format_chain: Format an invalid Virtual Unit chain. It frees all the Erase Units in a
 *	Virtual Unit Chain, i.e. all the units are disconnected.
 *
 *	It is not stricly correct to begin from the first block of the chain because
 *	if we stop the code, we may see again a valid chain if there was a first_block
 *	flag in a block inside it. But is it really a problem ?
 *
 * FixMe: Figure out what the last statesment means. What if power failure when we are
 *	in the for (;;) loop formatting blocks ??
 */
static void format_chain(struct NFTL2Krecord *nftl2k, unsigned int first_block)
{
	unsigned int block = first_block, block1;

	// dprintk("Formatting chain at block %d\n", first_block);

	for (;;) {
		block1 = nftl2k->ReplUnitTable[block];

		if (NFTL2K_formatblock(nftl2k, block) < 0) {
			/* cannot format !!!! Mark it as Bad Unit */
			nftl2k->ReplUnitTable[block] = BLOCK_RESERVED;
			nftl2k->ReverseUnitTable[block] = BLOCK_RESERVED;
		} else {
			nftl2k->ReplUnitTable[block] = BLOCK_FREE;
			nftl2k->ReverseUnitTable[block] = BLOCK_FREE;
		}

		/* goto next block on the chain */
		block = block1;

		if (!(block == BLOCK_NIL || block == BLOCK_NOTEXPLORED || block < nftl2k->nb_blocks))
			printk("incorrect ReplUnitTable[] : %d\n", block);
		if (block == BLOCK_NIL || block == BLOCK_NOTEXPLORED || block >= nftl2k->nb_blocks)
			break;
	}
   // dprintk("\n");
}

int NFTL2K_mount(struct NFTL2Krecord *s)
{
	int i;
	unsigned int first_logical_block, logical_block, pre_block, nb_erases, erase_mark,tmp_pre_block;
	unsigned int block, first_block;
	int chain_length, do_format_chain;
	struct nftl2k_oob page_oob;
	struct mtd_info *mtd = s->mbd.mtd;
	size_t retlen;
    int tmp_block;

	unsigned int writeEUN;
	int silly2;
    struct nftl2k_oob oob;
    char status;

	/* search for NFTL2K MediaHeader and Spare NFTL2K Media Header */
	if (find_boot_record(s) < 0) {
		printk("Could not find valid boot record\n");
		return -1;
	}

	/* init the logical to physical table and inverse table */
	for (i = 0; i < s->nb_blocks; i++) {
		s->EUNtable[i] = BLOCK_NIL;
	}

	/* first pass : explore each block chain */
	first_logical_block = 0;
	for (first_block = 0; first_block < s->nb_blocks; first_block++) {
		/* if the block was not already explored, we can look at it */
		if (s->ReverseUnitTable[first_block] == BLOCK_NOTEXPLORED) {
			block = first_block;
			chain_length = 0;
			do_format_chain = 0;

			for (;;) {
				/* read the block header. If error, we format the chain */
                if (nftl2k_read_oob(mtd, block * s->EraseSize,
                    sizeof(struct nftl2k_oob), &retlen, (char *)&page_oob) < 0)                {
					   // s->ReplUnitTable[block] = BLOCK_NIL;
					    s->ReverseUnitTable[block] = BLOCK_NIL;
					    do_format_chain = 1;
                        printk("check oob fail at block %d when explore all\n",block);
                        break;
				}

				logical_block = le16_to_cpu ((page_oob.u.a.VirtUnitNum | page_oob.u.a.SpareVirtUnitNum));
				pre_block = le16_to_cpu ((page_oob.u.a.ReplUnitNum | page_oob.u.a.SpareReplUnitNum));
				nb_erases = le32_to_cpu (page_oob.u.b.WearInfo);
				erase_mark = le16_to_cpu ((page_oob.u.b.EraseMark | page_oob.u.b.EraseMark1));

                // dprintk("EUN(%d): L(%d) R(%d)\n", block,logical_block,pre_block);


                if (page_oob.u.a.VirtUnitNum != page_oob.u.a.SpareVirtUnitNum) {
                    printk("VUN No. confused for physical block %d, 1:0x%04x 2:0x%04x\n",
                           block, page_oob.u.a.VirtUnitNum,page_oob.u.a.SpareVirtUnitNum);
                }
                if (page_oob.u.a.ReplUnitNum != page_oob.u.a.SpareReplUnitNum) {
                    printk("REP No. confused for physical block %d, 1:0x%04x 2:0x%04x\n",
                           block, page_oob.u.a.ReplUnitNum,page_oob.u.a.SpareReplUnitNum);
                }
                if (page_oob.u.b.EraseMark != page_oob.u.b.EraseMark1) {
                    printk("EraseMark No. confused for physical block %d, 1:0x%04x 2:0x%04x\n",
                           block, page_oob.u.b.EraseMark,page_oob.u.b.EraseMark1);
                    if (erase_mark != 0xffff) {
                        /* if not free, treated as correct mark */
                        erase_mark = ERASE_MARK;
                    }
                }

				/* invalid/free block test */
				if (erase_mark != ERASE_MARK || logical_block >= s->nb_blocks) {
					if (chain_length == 0) {
						/* if not currently in a chain, we can handle it safely */
						if (check_free_oob_sectors(s, block* s->EraseSize,s->page_size) < 0) {
							/* not really free: format it */
                            printk("check_free_oob_sectors() fail at block %d, addr 0x%09Lx\n",block,block* s->EraseSize);
                            printk("Formatting block %d\n", block);
							if (NFTL2K_formatblock(s, block) < 0) {
								/* could not format: reserve the block */
								s->ReplUnitTable[block] = BLOCK_RESERVED;
								s->ReverseUnitTable[block] = BLOCK_RESERVED;
                                //dprintk("R(%d) ", block);
							} else {
								s->ReplUnitTable[block] = BLOCK_FREE;
								s->ReverseUnitTable[block] = BLOCK_FREE;
                                //dprintk("F(%d) ", block);
							}
						} else {
							/* free block: mark it */
							s->ReplUnitTable[block] = BLOCK_FREE;
							s->ReverseUnitTable[block] = BLOCK_FREE;
                            //dprintk("F(%d) ", block);
						}
						/* directly examine the next block. */
						goto examine_ReplUnitTable;
					} else {
						/* the block was in a chain : this is bad. We
						   must format all the chain */
						printk("Block %d: free but referenced in chain %d, last relp block is %d\n",
						       block, first_logical_block,first_block);
					   // s->ReplUnitTable[block] = BLOCK_NIL;
						s->ReverseUnitTable[block] = BLOCK_NIL;
						do_format_chain = 1;
						break;
					}
				}

				/* we accept only first blocks here */
				if (chain_length == 0) {
			   // 	/* this block is not the first block in chain :
			   // 	   ignore it, it will be included in a chain
			   // 	   later, or marked as not explored */
			   // 	if (!is_first_block)
			   // 		goto examine_ReplUnitTable;
					first_logical_block = logical_block;
				} else {
					if (logical_block != first_logical_block) {
						printk("Block %d: incorrect logical block: %d expected: %d\n",
						       block, logical_block, first_logical_block);
						/* the chain is incorrect : we must format it,
						   but we need to read it completly */
						do_format_chain = 1;
                        break;
					}
				}


                /*
                if we reached here, we found another valid block contents data
                so, check the 'pre_block' of this block

                there're 3 possibilities marked as A,B,C

                */


				chain_length++;

				if (pre_block == BLOCK_HEAD) {
                    /* 
                    A:  this block is the head of the chain
                        check if the EUNtable has already locate a physical block                   
                    */
                    if (s->EUNtable[logical_block] < s->nb_blocks){
                        /*
                        A-1,    already has a valid block, this is bad
                                maybe caused by power failure in folding
                                need refold the chain
                        */
                        printk("NFTL:HEAD Block %d's logical block number %d already has an EUNblock %d\n",
                               block,logical_block,s->EUNtable[logical_block]);
                        do_format_chain = FORMAT_CHAIN_IN_FOLDING;
                    }
                    else{
                        /* 
                        A-2,    ok, it is good, it seems to be a Head Block 
                                we mark the EUNtable
                        */
                        s->EUNtable[logical_block] = block;
                        s->ReverseUnitTable[block] = BLOCK_HEAD;
                    }
					break;
				} 
                else if (pre_block >= s->nb_blocks) {
                    /* 
                    B:  this block's 'pre_block' is an invalid block
                        we are confused and must format this chain
                    */
					printk("Block %d: referencing invalid block %d\n",
					       block, pre_block);
					do_format_chain = 1;
					s->ReverseUnitTable[block] = BLOCK_NIL;
					//s->ReplUnitTable[block] = BLOCK_NIL;
					break;
				} 
                else {             
                    /*
                    C:  the previous block No. is a normal block, there are 2 possibilities
                    
                    C-1,    the previous block is already freed and may be used in other chains
                            we can check the block's last page cui, if 'ReplUnitNum' == BLOCK_HEAD
                            means this block is fully used, and the previous block MUST be freed already
                            so if this happens, we mark this block as the HEAD_BLOCK of this VUC
                        
                    */
                    nftl2k_read_oob(mtd, ((block+1) * s->EraseSize) - (s->page_size),
                        sizeof(struct nftl2k_oob), &retlen, (char *)&page_oob);   

                    tmp_pre_block = le16_to_cpu ((page_oob.u.a.ReplUnitNum | page_oob.u.a.SpareReplUnitNum));

                  //  nftl2k_dbg_hexdump((char *)&page_oob,sizeof(struct nftl2k_oob));
                  //  printk("Block %d's last page Used = 0x%02x PreB = %d\n", block, page_oob.b.Status, tmp_pre_block);

                    if (tmp_pre_block ==  BLOCK_HEAD){
                        //finish this chain
                        if (s->EUNtable[logical_block] < s->nb_blocks) {
                            printk("Late HEAD Block %d's in VUC %d already has an EUNblock %d\n"
                                   ,block, logical_block, s->EUNtable[logical_block] );
                            do_format_chain = 1;
                            s->ReverseUnitTable[block] = BLOCK_NIL;
                            break;
                        }
                        //dprintk("EUN(%d) is a Late HEAD Block VUC(%d)\n", block,logical_block);
                        s->EUNtable[logical_block] = block;
                        s->ReverseUnitTable[block] = BLOCK_HEAD;
                        break;                                
                    }

                    /*
                    C-2,    the block last page's 'ReplUnitNum' != BLOCK_HEAD
                            means this block is not fully used, and it MUST be a middle-EUN of a VUC
                            so check the previous block's ReverseUnitTable[]
                         
                    */

                    if (s->ReverseUnitTable[pre_block] != BLOCK_NOTEXPLORED) {

					    /* C-2-a,   the previous block is already in a chain
                                    so we must check if it is a valid one
                        */
					    if ((s->ReverseUnitTable[pre_block] != BLOCK_HEAD) &&
                            (s->ReverseUnitTable[pre_block] > s->nb_blocks))
                            {
                            /* C-2-a-1, the previous block is an invalid one
                                        we are confused and must format this chain
                            */
					    	printk("Block %d: previous block %d is an invalid one, which referring %d (0x%04x)\n",
					    	       block, pre_block,s->ReverseUnitTable[pre_block],s->ReverseUnitTable[pre_block]);
					    	do_format_chain = 1;
					    	//s->ReplUnitTable[block] = BLOCK_NIL;
                            s->ReverseUnitTable[block] = BLOCK_NIL;
					    } else {
                            /* C-2-a-2, the previous block is a valid one
                                        check if the ReplUnitTable has already locate a physical block
                            */
                            if (s->ReplUnitTable[pre_block] < s->nb_blocks){
                                /*
                                C-2-a-2-a,  already has a valid block, this is bad
                                            maybe caused by power failure in folding
                                            need refold the chain
                                */
                                printk("NFTL:Block %d's previous block %d already has a replace-block %d\n",
                                       block,pre_block,s->ReplUnitTable[pre_block]);
                                do_format_chain = FORMAT_CHAIN_MULTI;
                            }
                            else{
                                /* 
                                C-2-a-2-b,  we may found another part of a chain
                                            check if the chain can reach head 
                                */
                                silly2 = 1000;
                                tmp_block = pre_block;
                                while ((tmp_block != BLOCK_HEAD)&&(silly2 > 0)) {
                                    //dprintk("->%d",tmp_block);
                                    tmp_block = s->ReverseUnitTable[tmp_block] ;
                                    silly2--;
                                }

                                if (silly2 == 0) {
                                    printk("NFTL:Infinite Loop for EUN %d in VUC %d\n",
                                           block,logical_block);
                                    do_format_chain = 1;
                                    break;
                                }
                                /* 
                                    ok, it is good, we found another part of a chain 
                                */
                                s->ReverseUnitTable[block] = pre_block;
                                s->ReplUnitTable[pre_block] = block;
                            }
					    }
					    break;
                    } else {
					    /* C-2-b,   the previous block has not been explored
                                    this is OK, continue to complete the chain
                        */
					    s->ReverseUnitTable[block] = pre_block;
					    s->ReplUnitTable[pre_block] = block;
					    block = pre_block;
                    }
                }
			}

			/* the chain was completely explored. Now we can decide
			   what to do with it */

            if (do_format_chain == FORMAT_CHAIN_MULTI) {
                /* two chain pointed to one block, format the shorter one */
				unsigned int first_block0,first_block1, chain_to_format, chain_length0,chain_length1;
                first_block0 = block;
                first_block1 = s->ReplUnitTable[pre_block];

                /* XXX: what to do if same length ? */
                chain_length0 = calc_chain_length(s, first_block0);
                chain_length1 = calc_chain_length(s, first_block1);

                if (chain_length0 > chain_length1) {
                    /* format the previous one, change link to this one */
                    chain_to_format = first_block1;
                    s->ReplUnitTable[pre_block] = block;
                    s->ReverseUnitTable[block] = pre_block;
                }
                else{
                    /* format the this one */
                    chain_to_format = first_block0;
                }
                format_chain(s, chain_to_format);

            }
            else if (do_format_chain == FORMAT_CHAIN_IN_FOLDING) {
                /* two chain pointed to one logical entry, format the shorter one */
				unsigned int first_block0,first_block1, chain_to_format, chain_length0,chain_length1;
                first_block0 = block;
                first_block1 = s->EUNtable[logical_block];

                /* XXX: what to do if same length ? */
                chain_length0 = calc_chain_length(s, first_block0);
                chain_length1 = calc_chain_length(s, first_block1);

                if (chain_length0 > chain_length1) {
                    /*  format the previous one, change link to this one */
                    chain_to_format = first_block1;
                    s->EUNtable[logical_block] = block;
                    s->ReverseUnitTable[block] = BLOCK_HEAD;
                }
                else{
                    /* format the this one */
                    chain_to_format = first_block0;
                }
                format_chain(s, chain_to_format);
            }
			else if (do_format_chain) {
				/* 
                invalid chain : format it.
                because it is a reverse search, 
                so format chain start from the current block 
                */
				format_chain(s, block);
			} else {
			  /* now handle the case where we find two chains at the
			     same virtual address : we select the longer one,
			     because the shorter one is the one which was being
			     folded if the folding was not done in place */
			}
		}
	examine_ReplUnitTable:;
	}

	/* second pass to format unreferenced blocks  and init free block count */
	s->numfreeEUNs = 0;
	s->LastFreeEUN = le16_to_cpu(s->MediaHdr.FirstPhysicalEUN);

	for (block = 0; block < s->nb_blocks; block++) {
		if (s->ReverseUnitTable[block] == BLOCK_NOTEXPLORED) {
			printk("Unreferenced block %d, formatting it\n", block);
            printk("Formatting block %d\n", block);
			if (NFTL2K_formatblock(s, block) < 0){
				s->ReplUnitTable[block] = BLOCK_RESERVED;
                s->ReverseUnitTable[block] = BLOCK_RESERVED;
            }
			else{
				s->ReplUnitTable[block] = BLOCK_FREE;
                s->ReverseUnitTable[block] = BLOCK_FREE;
            }
		}
		if (s->ReverseUnitTable[block] == BLOCK_FREE) {
			s->numfreeEUNs++;
			s->LastFreeEUN = block;
		}
	}

	for (block = 0; block < s->nb_blocks; block++) {
		if (s->ReplUnitTable[block] == BLOCK_NOTEXPLORED) {
            /* it must be the Tail of one valid chain */
            s->ReplUnitTable[block] = BLOCK_NIL;
		}
	}

    /*
    init FirstFreePageTable[] , and search and fold all vuc that exceed limits
    */  
	for (writeEUN = 0; writeEUN < s->nb_blocks; writeEUN++) {

        if ((s->ReverseUnitTable[writeEUN] < s->nb_blocks)||(s->ReverseUnitTable[writeEUN] == BLOCK_HEAD)) {
        //=== init FreePageOffsTable[], use binary search argorithmn ===
            //=== a valid EUN ===
            int page_i;
            int last_used;
            int last_free;
            last_used = 0; last_free = s->page_per_block;
            page_i = s->page_per_block-1; // search from the last page
            do  {

			    nftl2k_read_oob(mtd, (writeEUN * s->EraseSize) +
			    	      (page_i * s->page_size), sizeof(struct nftl2k_oob) , &retlen,
			    	      (char *)&oob);
            
			    status = oob.b.Status | oob.b.Status1;

                if (status == SECTOR_FREE) {  last_free = page_i;  }
                else{ last_used = page_i;  } // page is used 

                if (last_used + 1 == last_free ) break;  // we found the first free page
                
                page_i = (last_free + last_used)/2;  // next for seach                
            } while(1);

            s->FirstFreePageTable[writeEUN] = last_free;
        }
    }

    //nftl2k_dbg_EUNtable_dump(s);

	return 0;
}

