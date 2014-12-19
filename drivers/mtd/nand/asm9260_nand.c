/*
 * NAND controller driver for Alphascale ASM9260, which is probably
 * based on Evatronix NANDFLASH-CTRL IP (version unknown)
 *
 * Copyright (C), 2007-2013, Alphascale Tech. Co., Ltd.
 * 		  2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#define ASM9260_ECC_STEP		512

#define mtd_to_priv(m)	container_of(m, struct asm9260_nand_priv, mtd)

#define HW_CMD				0x00
#define BM_CMD_CMD2_S			24
#define BM_CMD_CMD1_S			16
#define BM_CMD_CMD0_S			8
/* 0 - ADDR0, 1 - ADDR1 */
#define BM_CMD_ADDR1			BIT(7)
/* 0 - PIO, 1 - DMA */
#define BM_CMD_DMA			BIT(6)
#define BM_CMD_CMDSEQ_S			0
/* FIXME: some description for SEQ? */
#define  SEQ1				0x21 /* 6'b100001 */
#define  SEQ2				0x22 /* 6'b100010 */
#define  SEQ4				0x24 /* 6'b100100 */
#define  SEQ5				0x25 /* 6'b100101 */
#define  SEQ6				0x26 /* 6'b100110 */
#define  SEQ7				0x27 /* 6'b100111 */
#define  SEQ9				0x29 /* 6'b101001 */
#define  SEQ10				0x2a /* 6'b101010 */
#define  SEQ11				0x2b /* 6'b101011 */
#define  SEQ15				0x2f /* 6'b101111 */
#define  SEQ0				0x00 /* 6'b000000 */
#define  SEQ3				0x03 /* 6'b000011 */
#define  SEQ8				0x08 /* 6'b001000 */
#define  SEQ12				0x0c /* 6'b001100 */
#define  SEQ13				0x0d /* 6'b001101 */
#define  SEQ14				0x0e /* 6'b001110 */
#define  SEQ16				0x30 /* 6'b110000 */
#define  SEQ17				0x15 /* 6'b010101 */
#define  SEQ18				0x32 /* 6'h110010 */

#define HW_CTRL				0x04
#define BM_CTRL_DIS_STATUS		BIT(23)
#define BM_CTRL_READ_STAT		BIT(22)
#define BM_CTRL_SMALL_BLOCK_EN		BIT(21)
#define BM_CTRL_ADDR_CYCLE1_S		18
#define  ADDR_CYCLE_0			0x0
#define  ADDR_CYCLE_1			0x1
#define  ADDR_CYCLE_2			0x2
#define  ADDR_CYCLE_3			0x3
#define  ADDR_CYCLE_4			0x4
#define  ADDR_CYCLE_5			0x5
#define BM_CTRL_ADDR1_AUTO_INCR		BIT(17)
#define BM_CTRL_ADDR0_AUTO_INCR		BIT(16)
#define BM_CTRL_WORK_MODE		BIT(15)
#define BM_CTRL_PORT_EN			BIT(14)
#define BM_CTRL_LOOKU_EN		BIT(13)
#define BM_CTRL_IO_16BIT		BIT(12)
/* Overwrite BM_CTRL_PAGE_SIZE with HW_DATA_SIZE */
#define BM_CTRL_CUSTOM_PAGE_SIZE	BIT(11)
#define BM_CTRL_PAGE_SIZE_S		8
#define  PAGE_SIZE_256B			0x0
#define  PAGE_SIZE_512B			0x1
#define  PAGE_SIZE_1024B		0x2
#define  PAGE_SIZE_2048B		0x3
#define  PAGE_SIZE_4096B		0x4
#define  PAGE_SIZE_8192B		0x5
#define  PAGE_SIZE_16384B		0x6
#define  PAGE_SIZE_32768B		0x7
#define BM_CTRL_BLOCK_SIZE_S		6
#define  BLOCK_SIZE_32P			0x0
#define  BLOCK_SIZE_64P			0x1
#define  BLOCK_SIZE_128P		0x2
#define  BLOCK_SIZE_256P		0x3
#define BM_CTRL_ECC_EN			BIT(5)
#define BM_CTRL_INT_EN			BIT(4)
#define BM_CTRL_SPARE_EN		BIT(3)
/* same values as BM_CTRL_ADDR_CYCLE1_S */
#define BM_CTRL_ADDR_CYCLE0_S		0

#define HW_STATUS			0x08
#define	BM_CTRL_NFC_BUSY		BIT(8)
/* MEM1_RDY (BIT1) - MEM7_RDY (BIT7) */
#define	BM_CTRL_MEM0_RDY		BIT(0)

#define HW_INT_MASK			0x0c
#define HW_INT_STATUS			0x10
#define BM_INT_FIFO_ERROR		BIT(12)
/* MEM1_RDY (BIT5) - MEM7_RDY (BIT11) */
#define BM_INT_MEM0_RDY			BIT(4)
#define BM_INT_ECC_TRSH_ERR		BIT(3)
#define BM_INT_ECC_FATAL_ERR		BIT(2)
#define BM_INT_CMD_END			BIT(1)

#define HW_ECC_CTRL			0x14
/* bits per 512 bytes */
#define	BM_ECC_CAP_S			5
/* FIXME: reduce all this defines */
#define  ECC_CAP_2			0x0
#define  ECC_CAP_4			0x1
#define  ECC_CAP_6			0x2
#define  ECC_CAP_8			0x3
#define  ECC_CAP_10			0x4
#define  ECC_CAP_12			0x5
#define  ECC_CAP_14			0x6
#define  ECC_CAP_16			0x7
#define BM_ECC_ERR_THRESHOLD_S		8
/* FIXME: reduce all this defines */
#define  ECC_THRESHOLD_0		0x0
#define  ECC_THRESHOLD_1		0x1
#define  ECC_THRESHOLD_2		0x2
#define  ECC_THRESHOLD_3		0x3
#define  ECC_THRESHOLD_4		0x4
#define  ECC_THRESHOLD_5		0x5
#define  ECC_THRESHOLD_6		0x6
#define  ECC_THRESHOLD_7		0x7
#define  ECC_THRESHOLD_8		0x8
#define  ECC_THRESHOLD_9		0x9
#define  ECC_THRESHOLD_10		0xa
#define  ECC_THRESHOLD_11		0xb
#define  ECC_THRESHOLD_12		0xc
#define  ECC_THRESHOLD_13		0xd
#define  ECC_THRESHOLD_14		0xe
#define  ECC_THRESHOLD_15		0xf
#define BM_ECC_ERR_OVER			BIT(2)
/* Uncorrected error. */
#define BM_ECC_ERR_UNC			BIT(1)
/* Corrected error. */
#define BM_ECC_ERR_CORRECT		BIT(0)

#define HW_ECC_OFFSET			0x18
#define HW_ADDR0_0			0x1c
#define HW_ADDR1_0			0x20
#define HW_ADDR0_1			0x24
#define HW_ADDR1_1			0x28
#define HW_SPARE_SIZE			0x30
#define HW_DMA_ADDR			0x64
#define HW_DMA_CNT			0x68

#define HW_DMA_CTRL			0x6c
#define BM_DMA_CTRL_START		BIT(7)
/* 0 - to device; 1 - from device */
#define BM_DMA_CTRL_FROM_DEVICE		BIT(6)
/* 0 - software maneged; 1 - scatter-gather */
#define BM_DMA_CTRL_SG			BIT(5)
#define BM_DMA_CTRL_BURST_S		2
#define  DMA_BURST_INCR4		0x0
#define  DMA_BURST_STREAM		0x1
#define  DMA_BURST_SINGLE		0x2
#define  DMA_BURST_INCR			0x3
#define  DMA_BURST_INCR8		0x4
#define  DMA_BURST_INCR16		0x5
#define BM_DMA_CTRL_ERR			BIT(1)
#define BM_DMA_CTRL_RDY			BIT(0)

#define HW_MEM_CTRL			0x80
#define	BM_MEM_CTRL_WP_STATE_MASK	0xff00
#define	BM_MEM_CTRL_UNWPn(x)		(1 << ((x) + 8))
#define BM_MEM_CTRL_CEn(x)		(((x) & 7) << 0)

/* BM_CTRL_CUSTOM_PAGE_SIZE should be set */
#define HW_DATA_SIZE			0x84
#define HW_READ_STATUS			0x88
#define HW_TIM_SEQ_0			0x8c
#define HW_TIMING_ASYN			0x90
#define HW_TIMING_SYN			0x94

#define HW_FIFO_DATA			0x98
#define HW_TIME_MODE			0x9c
#define HW_FIFO_INIT			0xb0
/*
 * Counter for ecc related errors.
 * For each 512 byte block it has 5bit counter.
 */
#define HW_ECC_ERR_CNT			0xb8

#define HW_TIM_SEQ_1			0xc8

struct asm9260_nand_priv {
	struct device		*dev;
	struct mtd_info		mtd;
	struct nand_chip	nand;

	struct clk		*clk;
	struct clk		*clk_ahb;

	void __iomem *base;
	int irq_done;

	u32 read_cache;
	int read_cache_cnt;
	u32 cmd_cache;
	u32 ctrl_cache;
	u32 mem_status_mask;
	u32 page_cache;

	unsigned int ecc_cap;
	unsigned int ecc_threshold;
	unsigned int spare_size;
};

/*2KB--4*512B, correction ability: 4bit--7Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_64 = {
	.eccbytes = 4 * 7,
	.eccpos =  {
			36, 37, 38, 39, 40, 41, 42,
			43, 44, 45, 46, 47, 48, 49,
			50, 51, 52, 53, 54, 55, 56,
			57, 58, 59, 60, 61, 62, 63},
	.oobfree = {{2, 34}}
};

/*4KB--8*512B, correction ability: 6bit--10Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_128 = {
	.eccbytes = 8 * 10,
	.eccpos = {
			48, 49, 50,	51, 52, 53, 54, 55, 56, 57,
			58, 59, 60, 61, 62, 63,	64, 65, 66, 67,
			68, 69, 70, 71, 72, 73, 74, 75, 76, 77,
			78, 79, 80, 81, 82, 83, 84, 85, 86, 87,

			88, 89, 90, 91, 92,	93, 94, 95, 96, 97,
			98, 99, 100, 101, 102, 103, 104, 105, 106, 107,
			108, 109, 110, 111, 112, 113, 114, 115, 116, 117,
			118, 119, 120, 121, 122, 123, 124, 125, 126, 127},
	.oobfree = {
			{.offset = 2,
			 .length = 46}}
};

/*4KB--8*512B, correction ability: 14bit--23Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_218 = {
	.eccbytes = 8 * 23,
	.eccpos = {
			34, 35, 36, 37, 38, 39, 40, 41,
			42, 43, 44, 45, 46, 47, 48, 49,
			50,	51, 52, 53, 54, 55, 56, 57,
			58, 59, 60, 61, 62, 63,	64, 65,
			66, 67, 68, 69, 70, 71, 72, 73,
			74, 75, 76, 77, 78, 79, 80, 81,
			82, 83, 84, 85, 86, 87, 88, 89,
			90, 91, 92,	93, 94, 95, 96, 97,
			98, 99, 100, 101, 102, 103, 104, 105,
			106, 107, 108, 109, 110, 111, 112, 113,
			114, 115, 116, 117, 118, 119, 120, 121,
			122, 123, 124, 125, 126, 127, 128, 129,
			130, 131, 132, 133, 134, 135, 136, 137,
			138, 139, 140, 141, 142, 143, 144, 145,
			146, 147, 148, 149, 150, 151, 152, 153,
			154, 155, 156, 157, 158, 159, 160, 161,
			162, 163, 164, 165, 166, 167, 168, 169,
			170, 171, 172, 173, 174, 175, 176, 177,
			178, 179, 180, 181, 182, 183, 184, 185,
			186, 187, 188, 189, 190, 191, 192, 193,
			194, 195, 196, 197, 198, 199, 200, 201,
			202, 203, 204, 205, 206, 207, 208, 209,
			210, 211, 212, 213, 214, 215, 216, 217},
	.oobfree = {
			{.offset = 2,
			 .length = 32}}
};

/*4KB--8*512B, correction ability: 14bit--23Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_224 = {
	.eccbytes = 8 * 23,
	.eccpos = {
			40, 41, 42, 43, 44, 45, 46, 47,
			48, 49, 50,	51, 52, 53, 54, 55,
			56, 57,	58, 59, 60, 61, 62, 63,
			64, 65, 66, 67, 68, 69, 70, 71,

			72, 73, 74, 75, 76, 77, 78, 79,
			80, 81, 82, 83, 84, 85, 86, 87,
			88, 89, 90, 91, 92,	93, 94, 95,
			96, 97, 98, 99, 100, 101, 102, 103,

			104, 105, 106, 107, 108, 109, 110, 111,
			112, 113, 114, 115, 116, 117, 118, 119,
			120, 121, 122, 123, 124, 125, 126, 127,
			128, 129, 130, 131, 132, 133, 134, 135,

			136, 137, 138, 139, 140, 141, 142, 143,
			144, 145, 146, 147, 148, 149, 150, 151,
			152, 153, 154, 155, 156, 157, 158, 159,
			160, 161, 162, 163, 164, 165, 166, 167,

			168, 169, 170, 171, 172, 173, 174, 175,
			176, 177, 178, 179, 180, 181, 182, 183,
			184, 185, 186, 187, 188, 189, 190, 191,
			192, 193, 194, 195, 196, 197, 198, 199,

			200, 201, 202, 203, 204, 205, 206, 207,
			208, 209, 210, 211, 212, 213, 214, 215,
			216, 217, 218, 219, 220, 221, 222, 223},
	.oobfree = {
			{.offset = 2,
			 .length = 38}}
};


/*8KB--16*512B, correction ability: 8bit--13Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_256 = {
	.eccbytes = 16*13,
	.eccpos =  {
			48, 49, 50,	51, 52, 53, 54, 55, 56, 57,	58, 59, 60, 61, 62, 63,	
			64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 
			80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92,	93, 94, 95, 
			96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 

			112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
			128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
			144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
			160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,

			176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
			192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207,
			208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223,
			224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,

			240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255},
	.oobfree = {{2, 46}}
};

/*8KB--16*512B, correction ability: 14bit--23Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_436 = {
	.eccbytes = 16*23,
	.eccpos =  {
			68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 
			84, 85, 86, 87, 88, 89, 90, 91, 92,	93, 94, 95, 96, 97, 98, 99, 
			100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
			116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131,

			132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147,
			148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163,
			164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
			180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195,

			196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211,
			212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227,
			228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243,
			244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259,

			260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275,
			276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291,
			292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307,
			308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323,

			324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339,
			340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355,
			356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371,
			372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387,

			388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403,
			404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419,
			420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431, 432, 433, 434, 435},
	.oobfree = {{2, 66}}
};

/*8KB--16*512B, correction ability: 16bit--26Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_448 = {
	.eccbytes = 16*26,
	.eccpos =  {
			32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
			48, 49, 50,	51, 52, 53, 54, 55, 56, 57,	58, 59, 60, 61, 62, 63,
			64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
			80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92,	93, 94, 95,

			96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
			112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
			128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
			144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,

			160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
			176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
			192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207,
			208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223,

			224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
			240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255,
			256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271,
			272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287,

			288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303,
			304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319,
			320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335,
			336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351,

			352, 353, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 366, 367,
			368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383,
			384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398 ,399,
			400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415,

			416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431,
			432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447},
	.oobfree = {{2, 30}}
};

/**
 * struct ecc_info - ASAP1826T ECC INFO Structure
 * @ecc_cap:	The ECC module correction ability.
 * @ecc_threshold:		The acceptable errors level
 * @ecc_bytes_per_sector:		ECC bytes per sector
 */
struct ecc_info {
	int ecc_cap;
	int ecc_threshold;
	int ecc_bytes_per_sector;
};

/*
*	ECC info list
*
*	ecc_cap, ecc_threshold, ecc bytes per sector
*/
struct ecc_info ecc_info_table[8] = {
	{ECC_CAP_2, ECC_THRESHOLD_2, 4},
	{ECC_CAP_4, ECC_THRESHOLD_4, 7},
	{ECC_CAP_6, ECC_THRESHOLD_6, 10},
	{ECC_CAP_8, ECC_THRESHOLD_8, 13},
	{ECC_CAP_10, ECC_THRESHOLD_10, 17},
	{ECC_CAP_12, ECC_THRESHOLD_12, 20},
	{ECC_CAP_14, ECC_THRESHOLD_14, 23},
	{ECC_CAP_16, ECC_THRESHOLD_15, 26},
};

static void asm9260_reg_rmw(struct asm9260_nand_priv *priv,
		u32 reg_offset, u32 set, u32 clr)
{
	u32 val;

	val = ioread32(priv->base + reg_offset);
	val &= ~clr;
	val |= set;
	iowrite32(val, priv->base + reg_offset);
}

static void asm9260_select_chip(struct mtd_info *mtd, int chip)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	if (chip == -1)
		iowrite32(BM_MEM_CTRL_WP_STATE_MASK, priv->base + HW_MEM_CTRL);
	else
		iowrite32(BM_MEM_CTRL_UNWPn(chip) | BM_MEM_CTRL_CEn(chip),
			  priv->base + HW_MEM_CTRL);
}

static void asm9260_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
}

/* TODO: 3 commands are supported by HW. 3-d can be used for TWO PLANE. */
static void asm9260_nand_cmd_prep(struct asm9260_nand_priv *priv,
		u8 cmd0, u8 cmd1, u8 cmd2, u8 seq)
{
	priv->cmd_cache  = (cmd0 << BM_CMD_CMD0_S) | (cmd1 << BM_CMD_CMD1_S);
	priv->cmd_cache |= seq << BM_CMD_CMDSEQ_S;
}

static dma_addr_t asm9260_nand_dma_set(struct mtd_info *mtd, void *buf,
		enum dma_data_direction dir, size_t size)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	dma_addr_t dma_addr;

	dma_addr = dma_map_single(priv->dev, buf, size, dir);
	if (dma_mapping_error(priv->dev, dma_addr)) {
		dev_err(priv->dev, "dma_map_single failed!\n");
		return dma_addr;

	}

	iowrite32(dma_addr, priv->base + HW_DMA_ADDR);
	iowrite32(size, priv->base + HW_DMA_CNT);
	iowrite32(BM_DMA_CTRL_START
		  | (dir == DMA_FROM_DEVICE ? BM_DMA_CTRL_FROM_DEVICE : 0)
			/* TODO: check different DMA_BURST_INCR16 settings */
		  | (DMA_BURST_INCR16 << BM_DMA_CTRL_BURST_S),
		  priv->base + HW_DMA_CTRL);
	return dma_addr;
}

/* complete command request */
static void asm9260_nand_cmd_comp(struct mtd_info *mtd, int dma)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	int timeout;
	u32 cmd;

	if (!priv->cmd_cache)
		return;

	if (dma) {
		priv->cmd_cache |= BM_CMD_DMA;
		priv->irq_done = 0;
		/* FIXME: should we allow all MEM* device? */
		iowrite32(BM_INT_MEM0_RDY, priv->base + HW_INT_MASK);
	}

	iowrite32(priv->cmd_cache, priv->base + HW_CMD);
	cmd = priv->cmd_cache;
	priv->cmd_cache = 0;

	if (dma) {
		struct nand_chip *nand = &priv->nand;

		/* FIXME: change timeout value */
		timeout = wait_event_timeout(nand->controller->wq,
				priv->irq_done, 1 * HZ);
		if (timeout <= 0) {
			dev_info(priv->dev,
					"Request 0x%08x timed out\n", cmd);
			/* TODO: Do something useful here? */
			/* FIXME: if we have problems on DMA or PIO, we need to
			 * reset NFC. On asm9260 it is possible only with global
			 * reset register. How can we use it here? */
		}
	} else
		nand_wait_ready(mtd);
}

static int asm9260_nand_dev_ready(struct mtd_info *mtd)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	u32 tmp;

	tmp = ioread32(priv->base + HW_STATUS);

	/* FIXME: use define instead of 0x1 */
	return (!(tmp & BM_CTRL_NFC_BUSY) &&
			(tmp & 0x1));
}

static void asm9260_nand_ctrl(struct asm9260_nand_priv *priv, u32 set)
{
	iowrite32(priv->ctrl_cache | set, priv->base + HW_CTRL);
}

static void asm9260_nand_set_addr(struct asm9260_nand_priv *priv,
		u32 row_addr, u32 column)
{
	u32 addr[2];

	addr[0] = (column & 0xffff) | (0xffff0000 & (row_addr << 16));
	addr[1] = (row_addr >> 16) & 0xff;

	iowrite32(addr[0], priv->base + HW_ADDR0_0);
	iowrite32(addr[1], priv->base + HW_ADDR0_1);
}

static void asm9260_nand_command_lp(struct mtd_info *mtd,
		unsigned int command, int column, int page_addr)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	switch (command) {
	case NAND_CMD_RESET:
		asm9260_nand_cmd_prep(priv, NAND_CMD_RESET, 0, 0, SEQ0);
		asm9260_nand_cmd_comp(mtd, 0);
		break;

	case NAND_CMD_READID:
		iowrite32(1, priv->base + HW_FIFO_INIT);
		iowrite32((ADDR_CYCLE_1 << BM_CTRL_ADDR_CYCLE1_S)
			| BM_CTRL_CUSTOM_PAGE_SIZE
			| (PAGE_SIZE_4096B << BM_CTRL_PAGE_SIZE_S)
			| (BLOCK_SIZE_32P << BM_CTRL_BLOCK_SIZE_S)
			| BM_CTRL_INT_EN
			| (ADDR_CYCLE_1 << BM_CTRL_ADDR_CYCLE0_S),
			priv->base + HW_CTRL);

		iowrite32(8, priv->base + HW_DATA_SIZE);
		iowrite32(column, priv->base + HW_ADDR0_0);
		asm9260_nand_cmd_prep(priv, NAND_CMD_READID, 0, 0, SEQ1);

		priv->read_cache_cnt = 0;
		break;

	case NAND_CMD_READOOB:
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	case NAND_CMD_READ0:
		iowrite32(1, priv->base + HW_FIFO_INIT);

		if (column == 0) {
			asm9260_nand_ctrl(priv, 0);
			iowrite32(priv->spare_size, priv->base + HW_SPARE_SIZE);
		} else if (column == mtd->writesize) {
			asm9260_nand_ctrl(priv, BM_CTRL_CUSTOM_PAGE_SIZE);
			iowrite32(mtd->oobsize, priv->base + HW_SPARE_SIZE);
			iowrite32(mtd->oobsize, priv->base + HW_DATA_SIZE);
		} else {
			dev_err(priv->dev, "Couldn't support the column\n");
			break;
		}

		asm9260_nand_set_addr(priv, page_addr, column);

		asm9260_nand_cmd_prep(priv, NAND_CMD_READ0,
				NAND_CMD_READSTART, 0, SEQ10);

		priv->read_cache_cnt = 0;
		break;
	case NAND_CMD_SEQIN:
		iowrite32(1, priv->base + HW_FIFO_INIT);

		if (column == 0) {
			priv->page_cache = page_addr;
			asm9260_nand_ctrl(priv, 0);
			iowrite32(priv->spare_size, priv->base + HW_SPARE_SIZE);
		} else if (column == mtd->writesize) {
			asm9260_nand_ctrl(priv, BM_CTRL_CUSTOM_PAGE_SIZE);
			iowrite32(mtd->oobsize, priv->base + HW_DATA_SIZE);
		}

		asm9260_nand_set_addr(priv, page_addr, column);

		asm9260_nand_cmd_prep(priv, NAND_CMD_SEQIN, NAND_CMD_PAGEPROG,
			0, SEQ12);

		break;
	case NAND_CMD_STATUS:
		iowrite32(1, priv->base + HW_FIFO_INIT);
		asm9260_nand_ctrl(priv, BM_CTRL_CUSTOM_PAGE_SIZE);

		/*
		 * Workaround for status bug.
		 * Instead of SEQ4 we need to use SEQ1 here, which will
		 * send cmd with address. For this case we need to make sure
		 * ADDR == 0.
		 */
		asm9260_nand_set_addr(priv, 0, 0);
		iowrite32(4, priv->base + HW_DATA_SIZE);
		asm9260_nand_cmd_prep(priv, NAND_CMD_STATUS, 0, 0, SEQ1);

		priv->read_cache_cnt = 0;
		break;

	case NAND_CMD_ERASE1:
		asm9260_nand_set_addr(priv, page_addr, column);

		asm9260_nand_ctrl(priv, 0);

		/*
		 * Prepare and send command now. We don't need to split it in
		 * two stages.
		 */
		asm9260_nand_cmd_prep(priv, NAND_CMD_ERASE1, NAND_CMD_ERASE2,
				0, SEQ14);
		asm9260_nand_cmd_comp(mtd, 0);
		break;
	default:
		break;
	}
}


/**
 * We can't read less then 32 bits on HW_FIFO_DATA. So, to make
 * read_byte and read_word happy, we use sort of cached 32bit read.
 * Note: expected values for size should be 1 or 2 bytes.
 */
static u32 asm9260_nand_read_cached(struct mtd_info *mtd, int size)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	u8 tmp;

	if ((priv->read_cache_cnt <= 0) || (priv->read_cache_cnt > 4))
	{
		asm9260_nand_cmd_comp(mtd, 0);
		priv->read_cache = ioread32(priv->base + HW_FIFO_DATA);
		priv->read_cache_cnt = 4;
	}

	tmp = priv->read_cache >> (8 * (4 - priv->read_cache_cnt));
	priv->read_cache_cnt -= size;

	return tmp;
}

static u8 asm9260_nand_read_byte(struct mtd_info *mtd)
{
	return 0xff & asm9260_nand_read_cached(mtd, 1);
}

static u16 asm9260_nand_read_word(struct mtd_info *mtd)
{
	return 0xffff & asm9260_nand_read_cached(mtd, 2);
}

static void asm9260_nand_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	dma_addr_t dma_addr;
	int dma_ok;

	if (len & 0x3) {
		dev_err(priv->dev, "Unsupported length (%x)\n", len);
		return;
	}

	dma_addr = asm9260_nand_dma_set(mtd, buf, DMA_FROM_DEVICE, len);
	dma_ok = !(dma_mapping_error(priv->dev, dma_addr));
	printk("%s:%i len:%i dma:%x\n", __func__, __LINE__, len, dma_ok);
	asm9260_nand_cmd_comp(mtd, dma_ok);

	if (dma_ok) {
		dma_sync_single_for_cpu(priv->dev, dma_addr, len, DMA_FROM_DEVICE);
		dma_unmap_single(priv->dev, dma_addr, len, DMA_FROM_DEVICE);
		return;
	}

	/* fall back to pio mode */
	len >>= 2;
	ioread32_rep(priv->base + HW_FIFO_DATA, buf, len);
}

static void asm9260_nand_write_buf(struct mtd_info *mtd,
		const u8 *buf, int len)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	dma_addr_t dma_addr;
	int dma_ok;

	if (len & 0x3) {
		dev_err(priv->dev, "Unsupported length (%x)\n", len);
		return;
	}

	dma_addr = asm9260_nand_dma_set(mtd, buf, DMA_TO_DEVICE, len);
	dma_ok = !(dma_mapping_error(priv->dev, dma_addr));
	if (dma_ok)
		dma_sync_single_for_device(priv->dev, dma_addr, len,
				DMA_TO_DEVICE);
	asm9260_nand_cmd_comp(mtd, dma_ok);

	if (dma_ok) {
		dma_unmap_single(priv->dev, dma_addr, len, DMA_TO_DEVICE);
		return;
	}

	/* fall back to pio mode */
	len >>= 2;
	iowrite32_rep(priv->base + HW_FIFO_DATA, buf, len);
}

static int asm9260_nand_write_page_raw(struct mtd_info *mtd,
		struct nand_chip *chip, const u8 *buf,
		int oob_required)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	chip->write_buf(mtd, buf, mtd->writesize);
	if (oob_required)
		chip->ecc.write_oob(mtd, chip, priv->page_cache &
				chip->pagemask);
	return 0;
}

static int asm9260_nand_write_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, const u8 *buf,
		int oob_required)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	asm9260_reg_rmw(priv, HW_CTRL, BM_CTRL_ECC_EN, 0);
	chip->ecc.write_page_raw(mtd, chip, buf, oob_required);

	return 0;
}

static unsigned int asm9260_nand_count_ecc(struct asm9260_nand_priv *priv)
{
	u32 tmp, i, count, maxcount = 0;

	/* FIXME: this layout was tested only on 2048byte NAND.
	 * NANDs with bigger page size should use more registers. */
	tmp = ioread32(priv->base + HW_ECC_ERR_CNT);
	for (i = 0; i < 4; i++) {
		count = 0x1f & (tmp >> (5 * i));
		maxcount = max_t(unsigned int, maxcount, count);
	}

	return count;
}

static int asm9260_nand_read_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, u8 *buf,
		int oob_required, int page)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	u8 *temp_ptr;
	u32 status, max_bitflips = 0;

	temp_ptr = buf;

	/* enable ecc */
	asm9260_reg_rmw(priv, HW_CTRL, BM_CTRL_ECC_EN, 0);
	chip->read_buf(mtd, temp_ptr, mtd->writesize);

	status = ioread32(priv->base + HW_ECC_CTRL);
	status = 0;

	if (status & BM_ECC_ERR_UNC) {
		mtd->ecc_stats.failed++;
		max_bitflips = 10;
	} else if (status & BM_ECC_ERR_CORRECT) {
		max_bitflips = asm9260_nand_count_ecc(priv);
		mtd->ecc_stats.corrected += max_bitflips;
	}

	if (oob_required)
		chip->ecc.read_oob(mtd, chip, page);

	printk("%s:%i oob:%i: bad:%i\n", __func__, __LINE__, oob_required,
			max_bitflips);
	return max_bitflips;
}

static irqreturn_t asm9260_nand_irq(int irq, void *device_info)
{
	struct asm9260_nand_priv *priv = device_info;
	struct nand_chip *nand = &priv->nand;
	u32 status;

	status = ioread32(priv->base + HW_INT_STATUS);
	if (!status)
		return IRQ_NONE;

	iowrite32(0, priv->base + HW_INT_MASK);
	iowrite32(0, priv->base + HW_INT_STATUS);
        priv->irq_done = 1;
        wake_up(&nand->controller->wq);

        return IRQ_HANDLED;
}

static void __init asm9260_nand_init_chip(struct nand_chip *nand_chip)
{
	nand_chip->select_chip	= asm9260_select_chip;
	nand_chip->cmd_ctrl	= asm9260_cmd_ctrl;
	nand_chip->cmdfunc	= asm9260_nand_command_lp;
	nand_chip->read_byte	= asm9260_nand_read_byte;
	nand_chip->read_word	= asm9260_nand_read_word;
	nand_chip->read_buf	= asm9260_nand_read_buf;
	nand_chip->write_buf	= asm9260_nand_write_buf;

	nand_chip->dev_ready	= asm9260_nand_dev_ready;
	nand_chip->chip_delay	= 100;
	nand_chip->options		|= NAND_NO_SUBPAGE_WRITE;

	nand_chip->ecc.mode	= NAND_ECC_HW;

	nand_chip->ecc.write_page	= asm9260_nand_write_page_hwecc;
	nand_chip->ecc.write_page_raw	= asm9260_nand_write_page_raw;
	nand_chip->ecc.read_page	= asm9260_nand_read_page_hwecc;
}

static void __init asm9260_nand_cached_config(struct asm9260_nand_priv *priv)
{
	struct nand_chip *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;
	u32 addr_cycles, col_cycles, block_shift;

	/* FIXME: remove it or replace it */
	/* FIXME: these complete part need fixing  */
	block_shift = __ffs(mtd->erasesize) - nand->page_shift;
	col_cycles  = 2;
	addr_cycles = col_cycles +
		(((mtd->size >> mtd->writesize) > 65536) ? 3 : 2);

	priv->mem_status_mask = BM_CTRL_MEM0_RDY;
	//priv->ctrl_cache = BM_CTRL_READ_STAT
	priv->ctrl_cache = addr_cycles << BM_CTRL_ADDR_CYCLE1_S
		| ((nand->page_shift - 8) & 0x7) << BM_CTRL_PAGE_SIZE_S
		| ((block_shift - 5) & 0x3) << BM_CTRL_BLOCK_SIZE_S
		| BM_CTRL_INT_EN
		| addr_cycles << BM_CTRL_ADDR_CYCLE0_S;

	iowrite32(priv->ecc_threshold << BM_ECC_ERR_THRESHOLD_S
			| priv->ecc_cap << BM_ECC_CAP_S,
			priv->base + HW_ECC_CTRL);
	iowrite32(mtd->writesize + priv->spare_size,
			priv->base + HW_ECC_OFFSET);

}

static unsigned long __init clk_get_cyc_from_ns(struct clk *clk,
		unsigned long ns)
{
	unsigned int cycle;

	cycle = NSEC_PER_SEC / clk_get_rate(clk);
	return DIV_ROUND_CLOSEST(ns, cycle);
}

static void __init asm9260_nand_timing_config(struct asm9260_nand_priv *priv)
{
	struct nand_chip *nand = &priv->nand;
	const struct nand_sdr_timings *time;
	u32 twhr, trhw, trwh, trwp, tadl, tccs, tsync, trr, twb;
	int mode;

	mode = nand->onfi_timing_mode_default;
	time = onfi_async_timing_mode_to_sdr_timings(mode);
	if (IS_ERR_OR_NULL(time)) {
		dev_err(priv->dev, "Can't get onfi_timing_mode: %i\n", mode);
		return;
	}

	trwh = clk_get_cyc_from_ns(priv->clk, time->tWH_min / 1000);
	trwp = clk_get_cyc_from_ns(priv->clk, time->tWP_min / 1000);

	iowrite32((trwh << 4) | (trwp), priv->base + HW_TIMING_ASYN);

	twhr = clk_get_cyc_from_ns(priv->clk, time->tWHR_min / 1000);
	trhw = clk_get_cyc_from_ns(priv->clk, time->tRHW_min / 1000);
	tadl = clk_get_cyc_from_ns(priv->clk, time->tADL_min / 1000);
	/* tCCS - change read/write collumn. Time between last cmd and data. */
	tccs = clk_get_cyc_from_ns(priv->clk,
			(time->tCLR_min + time->tCLH_min + time->tRC_min)
			/ 1000);

	iowrite32((twhr << 24) | (trhw << 16)
		| (tadl << 8) | (tccs), priv->base + HW_TIM_SEQ_0);

	trr = clk_get_cyc_from_ns(priv->clk, time->tRR_min / 1000);
	tsync = 0; /* ??? */
	twb = clk_get_cyc_from_ns(priv->clk, time->tWB_max / 1000);
	iowrite32((tsync << 16) | (trr << 9) | (twb),
			priv->base + HW_TIM_SEQ_1);
}

static int __init asm9260_ecc_cap_select(struct asm9260_nand_priv *priv,
		int nand_page_size, int nand_oob_size)
{
	int ecc_bytes = 0;
	int i;

	for (i=(ARRAY_SIZE(ecc_info_table) - 1); i>=0; i--)
	{
		if ((nand_oob_size - ecc_info_table[i].ecc_bytes_per_sector
					* (nand_page_size >> 9)) > (28 + 2))
		{
			priv->ecc_cap =
				ecc_info_table[i].ecc_cap;
			priv->ecc_threshold =
				ecc_info_table[i].ecc_threshold;
			ecc_bytes = ecc_info_table[i].ecc_bytes_per_sector
				* (nand_page_size >> 9);
			break;
		}
	}

	return ecc_bytes;
}

#if 0
static int __init asm9260_nand_ecc_conf(struct asm9260_nand_priv *priv)
{
	struct device_node *np = priv->dev->of_node;
	struct nand_chip *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;
	int ecc_strength;

	ecc_strength = of_get_nand_ecc_strength(np);
	if (ecc_strength < nand->ecc_strength_ds) {
		/* Let's check if ONFI can help us. */
		if (nand->ecc_strength_ds <= 0) {
			/* No ONFI and no DT - it is bad. */
			dev_err(priv->dev,
					"ecc_strength is not set by DT or ONFI. Please set nand-ecc-strength in DT.\n");
			return -EINVAL;
		} else if (nand->ecc_step_ds == ASM9260_ECC_STEP)
			ecc_strength = nand->ecc_strength_ds;
		else if (nand->ecc_step_ds != ASM9260_ECC_STEP) {
			dev_err(priv->dev, "FIXME: calculate ecc_strength!\n");
			return -EINVAL;
		}
	} else if (ecc_strength == 0) {
		
	}
#endif

static void __init asm9260_nand_ecc_conf(struct asm9260_nand_priv *priv)
{
	struct nand_chip *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;

	printk("strenght: %i %i\n", nand->ecc_strength_ds, nand->ecc_step_ds);
	printk("timeing: %i \n", nand->onfi_timing_mode_default);

	if (nand->ecc.mode == NAND_ECC_HW) {
		/* ECC is calculated for the whole page (1 step) */
		nand->ecc.size = mtd->writesize;

		/* set ECC page size and oob layout */
		switch (mtd->writesize) {
			case 2048:
				nand->ecc.bytes  =
					asm9260_ecc_cap_select(priv, 2048,
							mtd->oobsize);
				nand->ecc.layout = &asm9260_nand_oob_64;
				nand->ecc.strength = 4;
				break;

			case 4096:
				nand->ecc.bytes =
					asm9260_ecc_cap_select(priv, 4096,
							mtd->oobsize);

				if (mtd->oobsize == 128) {
					nand->ecc.layout =
						&asm9260_nand_oob_128;
					nand->ecc.strength = 6;
				} else if (mtd->oobsize == 218) {
					nand->ecc.layout =
						&asm9260_nand_oob_218;
					nand->ecc.strength = 14;
				} else if (mtd->oobsize == 224) {
					nand->ecc.layout =
						&asm9260_nand_oob_224;
					nand->ecc.strength = 14;
				} else
					dev_err(priv->dev, "Unsupported Oob size [%d].\n",
							mtd->oobsize);

				break;

			case 8192:
				nand->ecc.bytes =
					asm9260_ecc_cap_select(priv, 8192,
							mtd->oobsize);

				if (mtd->oobsize == 256) {
					nand->ecc.layout =
						&asm9260_nand_oob_256;
					nand->ecc.strength = 8;
				} else if (mtd->oobsize == 436) {
					nand->ecc.layout =
						&asm9260_nand_oob_436;
					nand->ecc.strength = 14;
				} else if (mtd->oobsize == 448) {
					nand->ecc.layout =
						&asm9260_nand_oob_448;
					nand->ecc.strength = 16;
				} else
					dev_err(priv->dev, "Unsupported Oob size [%d].\n",
							mtd->oobsize);
				break;

			default:
				dev_err(priv->dev, "Unsupported Page size [%d].\n",
						mtd->writesize);
				break;
		}
	}

	priv->spare_size = mtd->oobsize - nand->ecc.bytes;
}

static int __init asm9260_nand_get_dt_clks(struct asm9260_nand_priv *priv)
{
	struct device_node *np = priv->dev->of_node;
	int clk_idx = 0, err;

	priv->clk = of_clk_get(np, clk_idx);
	if (IS_ERR(priv->clk))
		goto out_err;

	/* configure AHB clock */
	clk_idx = 1;
	priv->clk_ahb = of_clk_get(np, clk_idx);
	if (IS_ERR(priv->clk_ahb))
		goto out_err;

	err = clk_prepare_enable(priv->clk_ahb);
	if (err)
		dev_err(priv->dev, "Failed to enable ahb_clk!\n");

	err = clk_set_rate(priv->clk, clk_get_rate(priv->clk_ahb));
	if (err)
		dev_err(priv->dev, "Failed to set rate!\n");

	err = clk_prepare_enable(priv->clk);
	if (err)
		dev_err(priv->dev, "Failed to enable clk!\n");

	return 0;
out_err:
	dev_err(priv->dev, "%s: Failed to get clk (%i)\n", __func__, clk_idx);
	return 1;
}

static int __init asm9260_nand_probe(struct platform_device *pdev)
{
	struct asm9260_nand_priv *priv;
	struct nand_chip *nand;
	struct mtd_info *mtd;
	struct device_node *np = pdev->dev.of_node;
	int ret;
	unsigned int irq;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct asm9260_nand_priv),
			GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "Allocation filed!\n");
		return -ENOMEM;
	}

	priv->base = of_io_request_and_map(np, 0, np->full_name);
        if (!priv->base) {
		dev_err(&pdev->dev, "Unable to map resource!\n");
		return -EINVAL;
	}

	priv->dev = &pdev->dev;
	nand = &priv->nand;
	nand->priv = priv;

	platform_set_drvdata(pdev, priv);
	mtd = &priv->mtd;
	mtd->priv = nand;
	mtd->owner = THIS_MODULE;
	mtd->name = dev_name(&pdev->dev);

	priv->read_cache_cnt = 0;
        priv->irq_done = 0;

	/* FIXME: add more dt options? for example chip number? */
	if (asm9260_nand_get_dt_clks(priv))
		return -ENODEV;

	irq = irq_of_parse_and_map(np, 0);
	if (!irq)
		return -ENODEV;

	iowrite32(0, priv->base + HW_INT_MASK);
	ret = devm_request_irq(priv->dev, irq, asm9260_nand_irq,
				IRQF_ONESHOT | IRQF_SHARED,
				dev_name(&pdev->dev), priv);

	asm9260_nand_init_chip(nand);


	/* first scan to find the device and get the page size */
	if (nand_scan_ident(mtd, 1, NULL)) {
		dev_err(&pdev->dev, "scan_ident filed!\n");
		return -ENXIO;
	}

	asm9260_nand_timing_config(priv);
	asm9260_nand_ecc_conf(priv);
	asm9260_nand_cached_config(priv);

	/* second phase scan */
	if (nand_scan_tail(mtd)) {
		dev_err(&pdev->dev, "scan_tail filed!\n");
		return -ENXIO;
	}


	ret = mtd_device_parse_register(mtd, NULL,
			&(struct mtd_part_parser_data) {
				.of_node = pdev->dev.of_node,
			},
			NULL, 0);

	return ret;
}


static int asm9260_nand_remove(struct platform_device *pdev)
{
	struct asm9260_nand_priv *priv = platform_get_drvdata(pdev);

	nand_release(&priv->mtd);

	return 0;
}

static const struct of_device_id asm9260_nand_match[] =
{
	{
		.compatible   = "alphascale,asm9260-nand",
	},
	{},
};
MODULE_DEVICE_TABLE(of, asm9260_nand_match);

static struct platform_driver asm9260_nand_driver = {
	.probe		= asm9260_nand_probe,
	.remove		= asm9260_nand_remove,
	.driver		= {
		.name	= "asm9260_nand",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(asm9260_nand_match),
	},
};

module_platform_driver(asm9260_nand_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ASM9260 NAND driver");
