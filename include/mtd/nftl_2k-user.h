/*
 * Linux driver for Mixed NAND Flash Translation Layer  
 * (c) 2007 Alpscale, Inc.                              
 * Author: He Yong <hoffer@sjtu.org>                    
 *
 * Parts of NFTL headers shared with userspace
 *
 */

#ifndef __MTD_NFTL2K_USER_H__
#define __MTD_NFTL2K_USER_H__

#include "ftl_ecc.h"

/* Block Control Information */

struct nftl2k_bci {
	ftl_ECC ecc;
	uint8_t free_page_pos;
	uint8_t free_page_pos1;
	uint8_t reserved;
	uint8_t Status;
	uint8_t Status1;
}__attribute__((packed));

/* Unit Control Information */

/* Hoffer:
   in order to get rid of partially page programming, we'd better place whole uci in the first page
   for 2K page support, we can use 2-28 => 26 bytes of free oob space, that's enough 
  */

struct nftl2k_uci0 {
	uint16_t VirtUnitNum;
	uint16_t ReplUnitNum;
	uint16_t SpareVirtUnitNum;
	uint16_t SpareReplUnitNum;
} __attribute__((packed));

struct nftl2k_uci1 {
	uint32_t WearInfo;
	uint16_t EraseMark;
	uint16_t EraseMark1;
} __attribute__((packed));

struct nftl2k_uci2 {
        uint16_t FoldMark;
        uint16_t FoldMark1;
   /* uint32_t unused; */
} __attribute__((packed));

struct nftl2k_uci {
	struct nftl2k_uci0 a;   /*  8byte */
	struct nftl2k_uci1 b;   /*  8byte */
 //   struct nftl2k_uci2 c;   /*  4byte */
} __attribute__((packed));

struct nftl2k_oob {
	struct nftl2k_bci b;    /*  8byte */
	struct nftl2k_uci u;    /*  16byte */
};

/* NFTL2K Media Header */

struct NFTL2KMediaHeader {
	char DataOrgID[6];            /* "ANAND" */
	uint16_t NumEraseUnits;       /* number of EUNs */
	uint16_t FirstPhysicalEUN;    /* number of bios blocks */
	uint64_t FormattedSize;       /* size of all VUCs */
	unsigned char UnitSizeFactor; /* 0 */
} __attribute__((packed));

#define MAX_ERASE_ZONES (8192 - 512)

#define ERASE_MARK 0x3c69
//#define ERASE_MARK 0x69
#define SECTOR_FREE 0xff
#define SECTOR_USED 0x55
#define SECTOR_IGNORE 0x11
#define SECTOR_DELETED 0x00

#define FOLD_MARK_IN_PROGRESS 0x5555

#define ZONE_GOOD 0xff
#define ZONE_BAD_ORIGINAL 0
#define ZONE_BAD_MARKED 7


#endif /* __MTD_NFTL2K_USER_H__ */
