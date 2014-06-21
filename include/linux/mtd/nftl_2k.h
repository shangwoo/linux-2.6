/* Linux driver for Mixed NAND Flash Translation Layer  */
/* (c) 2007 Alpscale, Inc.                              */
/* Author: He Yong <hoffer@sjtu.org>                    */

#ifndef __MTD_NFTL2K_H__
#define __MTD_NFTL2K_H__

#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>
#include <linux/timer.h>
#include <linux/interrupt.h>

#include <mtd/nftl_2k-user.h>

//#define CONFIG_NFTL2K_WEAR_LEVELING 1
#define WEAR_LEVEL_LEAST_CMP_POTS   4  /* at least 4 block to compare wear-info */
    
#define MAX_PAGE_SIZE 4096  /* this is the maximun flash page size , 4KB */

/* these info are used in ReplUnitTable */
#define BLOCK_FREE         0xffff /* free block */
#define BLOCK_NIL          0xfffe /* last block of a chain */
#define BLOCK_NOTEXPLORED  0xfffd /* non explored block, only used during mounting */
#define BLOCK_RESERVED     0xfffc /* bios block or bad block */
#define BLOCK_HEAD         0xfffb /* this block is a head block in the reverse chain -- hoffer */

#define FORMAT_THIS_CHAIN           1
#define FORMAT_CHAIN_IN_FOLDING     2
#define FORMAT_CHAIN_MULTI          3

#define PAGE_READ_CACHED           1
#define PAGE_WRITE_CACHED          2
#define PAGE_CACHE_INVALID         (-1)

#define NFTL2K_BBT_SIZE     1024
#define NFTL2K_BBT_MASK     0x3ff
#define NFTL2K_BBT_MARKBAD_THRESHOLD     3

#define CACHE_SECTOR_0  (1<<0)
#define CACHE_SECTOR_1  (1<<1)
#define CACHE_SECTOR_2  (1<<2)
#define CACHE_SECTOR_3  (1<<3)

#define WRITE_BACK_TIMEOUT_SECONDS 2

typedef struct NFTL2Krecord {
	struct mtd_blktrans_dev mbd;
	__u16 MediaUnit, SpareMediaUnit;
	__u64 EraseSize;
	struct NFTL2KMediaHeader MediaHdr;
	int usecount;
	unsigned char heads;
	unsigned char sectors;
	unsigned short cylinders;
	__u16 numvunits;
	__u16 lastEUN;                  /* should be suppressed */
	__u16 numfreeEUNs;
	__u16 LastFreeEUN; 		/* To speed up finding a free EUN */
	int head,sect,cyl;
	__u16 *EUNtable; 		/* [numvunits]: First EUN for each virtual unit  */
	__u16 *ReplUnitTable; 		/* [numEUNs]: ReplUnitNumber for each */
	__u16 *ReverseUnitTable; 		/* [numEUNs]: ReverseUnitNumber for each */
	char *FirstFreePageTable; 		/* [numEUNs]: first free page in each EUN */
        unsigned int nb_blocks;		/* number of physical blocks */
        unsigned int nb_boot_blocks;	/* number of blocks used by the bios */
        struct erase_info instr;
	struct nand_ecclayout oobinfo;

    int page_per_block;         /* physical page in  physical block , 64 or 128*/
    int page_size;              /* physical page size */
    int page_shift;             /* (1<<page_shift) == page_size */

    /* For Cached Read/Write Support */
    int read_buf_status; /* 1:read data, 2:write data, -1:invalid data*/
    int write_buf_status; /* 1:sector-0, 2:sector-1, 4:sector-2, 8:sector-3,,, 0: none-cached*/
    u16 Last_Read_VUC;
    u16 Last_Write_VUC; /* Last Visited VUC (Write Only)   */
    char Last_Read_Page;
    char Last_Write_Page; /* Last Visited Page (Write Only)   */
    u16 Last_Write_EUN;   /* Last target physical Block */
    u16 Last_Pre_Block;   /* Last previous physical Block */
   // int cached_is_new_block;

   // int swtich_wrtie_vuc_target; /* 0: last write vuc == current target vuc, 1: current target vuc changed  */

    /* timer */
	struct timer_list timer;

    /* bad block hash table */
	char nftl2k_bbt[NFTL2K_BBT_SIZE]; /* 1k */

    #ifdef CONFIG_NFTL2K_WEAR_LEVELING
    /* give statistics about wear-info */
	__u16 *WearInfoTable; 		/* [numEUNs]: wear-info */
    #endif
    
    /* read/write use the same buffer to do cached rw*/
    char read_buf[MAX_PAGE_SIZE];  /* page read buff */
    char write_buf[MAX_PAGE_SIZE];  /* page write buff */
    char movebuf[MAX_PAGE_SIZE]; /* page copy-back buffer */

}__NFTL2Krecord;

void format_mtd(struct NFTL2Krecord *nftl2k,struct mtd_info *mtd);
int NFTL2K_mount(struct NFTL2Krecord *s);
int NFTL2K_formatblock(struct NFTL2Krecord *s, int block);
int nftl2k_writeback_lastpage(struct NFTL2Krecord *nftl2k);
int nftl2k_write(struct mtd_info *mtd, loff_t offs, size_t len,
		      size_t *retlen, uint8_t *buf, uint8_t *oob);
int nftl2k_write_oob(struct mtd_info *mtd, loff_t offs, size_t len,
			  size_t *retlen, uint8_t *buf);
int nftl2k_read_oob(struct mtd_info *mtd, loff_t offs, size_t len,
		  size_t *retlen, uint8_t *buf);
int nftl2k_read_oob_place(struct mtd_info *mtd, loff_t offs, size_t len,
		  size_t *retlen, uint8_t *buf);

void nftl2k_dbg_hexdump(const void *ptr, int size);
void nftl2k_dbg_MediaHeader_dump(const struct NFTL2KMediaHeader * header);
void nftl2k_dbg_EUNtable_dump(struct NFTL2Krecord *nftl2k);
int search_and_fold_chain(struct NFTL2Krecord *nftl2k,int lenth_limits);
u16 NFTL2K_foldchain (struct NFTL2Krecord *nftl2k, unsigned thisVUC);
//void NFTL2K_fold_page_data(struct NFTL2Krecord *nftl2k, u16 thisVUC,u16 secondlastEUN, u16 targetEUN, int startpage, int endpage);
void NFTL2K_fold_page_data(struct NFTL2Krecord *nftl2k, u16 thisVUC,u16 secondlastEUN, u16 targetEUN, int start_page, int stop_page);
#ifdef CONFIG_NFTL2K_DEBUG
    #define dprintk(msg...)	if (1) { dbg_printf(KERN_DEBUG "NFTL2K: " msg); }
#else // CONFIG_NFTL2K_DEBUG
    #define dprintk(msg...)	{}
#endif //CONFIG_NFTL2K_DEBUG

#define le16_2_cpu(x) (x)
#define le32_2_cpu(x) (x)

#ifndef NFTL2K_MAJOR
#define NFTL2K_MAJOR 93
#endif

#define MAX_NFTL2KS 16
#define MAX_SECTORS_PER_UNIT 128  /* one block may have 128 pages */
#define MAX_CHAIN_LENGTH     256  /* max chain length */
#define NFTL2K_PARTN_BITS 4

#endif /* __MTD_NFTL2K_H__ */
