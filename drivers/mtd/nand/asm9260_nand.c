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
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <asm/hardware/iomd.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/bitops.h>
#include <linux/of_platform.h>


// timing parameters
#define  TITC  0x0
#define  TWHR  0x6//0x6
#define  TRHW  0x6//0x6
#define  TADL  0x0
#define  TCCS  0x0

#define  TWH   0x8//0x8
#define  TWP   0x8//0x8

#define  TCAD  0x0

// cmd parameters
//ok

// seq parameter
#define  SEQ1     0x21   // 6'b100001
#define  SEQ2     0x22   // 6'b100010
#define  SEQ4     0x24   // 6'b100100
#define  SEQ5     0x25   // 6'b100101
#define  SEQ6     0x26   // 6'b100110
#define  SEQ7     0x27   // 6'b100111
#define  SEQ9     0x29   // 6'b101001
#define  SEQ10    0x2A   // 6'b101010
#define  SEQ11    0x2B   // 6'b101011
#define  SEQ15    0x2F   // 6'b101111
#define  SEQ0     0x00   // 6'b000000
#define  SEQ3     0x03   // 6'b000011
#define  SEQ8     0x08   // 6'b001000
#define  SEQ12    0x0C   // 6'b001100
#define  SEQ13    0x0D   // 6'b001101
#define  SEQ14    0x0E   // 6'b001110
#define  SEQ16    0x30   // 6'b110000
#define  SEQ17    0x15   // 6'b010101
#define  SEQ18    0x32   // 6'h110010

// cmd register
#define  ADDR_SEL_0    0x0
#define  ADDR_SEL_1    0x1

#define  INPUT_SEL_BIU  0x0
#define  INPUT_SEL_DMA  0x1

// control register parameter
#define  DISABLE_STATUS    1
#define  EN_STATUS         0

#define  RNB_SEL           0
#define  NO_RNB_SEL        1

#define  BIG_BLOCK_EN      0
#define  SMALL_BLOCK_EN    1

#define  LOOKUP_EN         1
#define  LOOKUP_DIS        0

#define  WORK_MODE_ASYNC   0
#define  WORK_MODE_SYNC    1

#define  PROT_EN           1
#define  PROT_DIS          0

#define  IO_WIDTH_8        0
#define  IO_WIDTH_16       1

#define  DATA_SIZE_FULL_PAGE  0
#define  DATA_SIZE_CUSTOM     1

#define  PAGE_SIZE_256B        0x0
#define  PAGE_SIZE_512B        0x1
#define  PAGE_SIZE_1024B       0x2
#define  PAGE_SIZE_2048B       0x3
#define  PAGE_SIZE_4096B       0x4
#define  PAGE_SIZE_8192B       0x5
#define  PAGE_SIZE_16384B      0x6
#define  PAGE_SIZE_32768B      0x7
#define  PAGE_SIZE_0B          0x0

#define  BLOCK_SIZE_32P        0x0
#define  BLOCK_SIZE_64P        0x1
#define  BLOCK_SIZE_128P       0x2
#define  BLOCK_SIZE_256P       0x3

#define  ECC_DIS          0
#define  ECC_EN           1

#define  INT_DIS          0
#define  INT_EN           1

#define  SPARE_DIS        0
#define  SPARE_EN         1

#define  ADDR0_AUTO_INCR_DIS  0
#define  ADDR0_AUTO_INCR_EN   1

#define  ADDR1_AUTO_INCR_DIS  0
#define  ADDR1_AUTO_INCR_EN   1

#define  ADDR_CYCLE_0      0x0
#define  ADDR_CYCLE_1      0x1
#define  ADDR_CYCLE_2      0x2
#define  ADDR_CYCLE_3      0x3
#define  ADDR_CYCLE_4      0x4
#define  ADDR_CYCLE_5      0x5

//generic_seq_ctrl
#define  CMD0_EN      0x1
#define  CMD0_DIS     0x0

#define  ADDR0_EN     0x1
#define  ADDR0_DIS    0x0

#define  CMD1_EN      0x1
#define  CMD1_DIS     0x0

#define  ADDR1_EN     0x1
#define  ADDR1_DIS    0x0

#define  CMD2_EN      0x1
#define  CMD2_DIS     0x0

#define  CMD3_EN      0x1
#define  CMD3_DIS     0x0

#define  ADDR2_EN     0x1
#define  ADDR2_DIS    0x0 

#define  DEL_DIS_ALL  0x0
#define  DEL_EN_ALL   0x3
#define  DEL_EN_0     0x1
#define  DEL_EN_1     0x2

#define  DATA_EN      0x1
#define  DATA_DIS     0x0

#define  COL_ADDR_EN  0x1
#define  COL_ADDR_DIS 0x0

// int_mask register
#define  FIFO_ERROR_DIS  0
#define  FIFO_ERROR_EN   1

#define  MEM7_RDY_DIS    0
#define  MEM7_RDY_EN     1

#define  MEM6_RDY_DIS    0
#define  MEM6_RDY_EN     1

#define  MEM5_RDY_DIS    0
#define  MEM5_RDY_EN     1

#define  MEM4_RDY_DIS    0
#define  MEM4_RDY_EN     1

#define  MEM3_RDY_DIS    0
#define  MEM3_RDY_EN     1

#define  MEM2_RDY_DIS    0
#define  MEM2_RDY_EN     1

#define  MEM1_RDY_DIS    0
#define  MEM1_RDY_EN     1

#define  MEM0_RDY_DIS    0
#define  MEM0_RDY_EN     1

#define  ECC_TRSH_ERR_DIS  0
#define  ECC_TRSH_ERR_EN   1

#define  ECC_FATAL_ERR_DIS 0
#define  ECC_FATAL_ERR_EN  1

#define  CMD_END_INT_DIS   0
#define  CMD_END_INT_EN    1

#define  PROT_INT_DIS   0
#define  PROT_INT_EN    1

// dma ctrl register
#define  DMA_START_EN   0x1
#define  DMA_START_DIS  0x0

#define  DMA_DIR_WRITE  0x0
#define  DMA_DIR_READ   0x1

#define  DMA_MODE_SFR   0x0
#define  DMA_MODE_SG    0x1

#define  DMA_BURST_INCR4   0x0
#define  DMA_BURST_STREAM  0x1
#define  DMA_BURST_SINGLE  0x2
#define  DMA_BURST_INCR    0x3
#define  DMA_BURST_INCR8   0x4
#define  DMA_BURST_INCR16  0x5

//ecc ctrl register
#define  ECC_WORD_POS_SPARE  1
#define  ECC_WORD_POS_DATA   0

#define  ECC_THRESHOLD_0     0x0
#define  ECC_THRESHOLD_1     0x1
#define  ECC_THRESHOLD_2     0x2
#define  ECC_THRESHOLD_3     0x3
#define  ECC_THRESHOLD_4     0x4
#define  ECC_THRESHOLD_5     0x5
#define  ECC_THRESHOLD_6     0x6
#define  ECC_THRESHOLD_7     0x7
#define  ECC_THRESHOLD_8     0x8
#define  ECC_THRESHOLD_9     0x9
#define  ECC_THRESHOLD_10    0xA
#define  ECC_THRESHOLD_11    0xB
#define  ECC_THRESHOLD_12    0xC
#define  ECC_THRESHOLD_13    0xD
#define  ECC_THRESHOLD_14    0xE
#define  ECC_THRESHOLD_15    0xF

#define  ECC_CAP_2    0x0
#define  ECC_CAP_4    0x1
#define  ECC_CAP_6    0x2
#define  ECC_CAP_8    0x3
#define  ECC_CAP_10   0x4
#define  ECC_CAP_12   0x5
#define  ECC_CAP_14   0x6
#define  ECC_CAP_16   0x7

// boot parameter
#define  BOOT_REQ      0x1

#define	ASM9260T_NAND_WP_STATE_MASK		0xFF00
#define	ASM9260T_NAND_CTRL_BUSY			(1UL << 8)
#define	ASM9260T_NAND_DEV0_READY		(1UL << 0)
#define	ASM9260T_NAND_DMA_READY			0x00000001
#define	ASM9260T_NAND_DMA_ERROR			0x00000002

#define NAND_CMD_CMD2				24
#define NAND_CMD_CMD1				16
#define NAND_CMD_CMD0				8
#define NAND_CMD_ADDR_SEL			7
#define NAND_CMD_INPUT_SEL			6
#define NAND_CMD_CMDSEQ				0

#define NAND_CTRL_DIS_STATUS		23
#define NAND_CTRL_RNB_SEL			22
#define NAND_CTRL_SMALL_BLOCK_EN	21
#define NAND_CTRL_ADDR_CYCLE1		18
#define NAND_CTRL_ADDR1_AUTO_INCR	17
#define NAND_CTRL_ADDR0_AUTO_INCR	16
#define NAND_CTRL_WORK_MODE			15
#define NAND_CTRL_PORT_EN			14
#define NAND_CTRL_LOOKU_EN			13
#define NAND_CTRL_IO_WIDTH			12
#define NAND_CTRL_CUSTOM_SIZE_EN	11
#define NAND_CTRL_PAGE_SIZE			8
#define NAND_CTRL_BLOCK_SIZE		6
#define NAND_CTRL_ECC_EN			5
#define NAND_CTRL_INT_EN			4
#define NAND_CTRL_SPARE_EN			3
#define NAND_CTRL_ADDR_CYCLE0		0

#define NAND_DMA_CTRL_START			7
#define NAND_DMA_CTRL_DIR			6
#define NAND_DMA_CTRL_MODE			5
#define NAND_DMA_CTRL_BURST			2
#define NAND_DMA_CTRL_ERR			1
#define NAND_DMA_CTRL_READY			0

#define ASM9260T_NAND_CLK_EN		0x00000400
#define	ASM9260T_NAND_CLK_DIV		0x00000008




#define HW_CMD		0x00
#define HW_CTRL		0x04
#define HW_STATUS	0x08

#define HW_INT_MASK	0x0c
#define HW_INT_STATUS	0x10
#define BM_INT_FIFO_ERROR	BIT(12)
/* FIXME: do we need MEM1_RDY (BIT5) - MEM7_RDY (BIT11) */
#define BM_INT_MEM0_RDY		BIT(4)
#define BM_INT_ECC_TRSH_ERR	BIT(3)
#define BM_INT_ECC_FATAL_ERR	BIT(2)
#define BM_INT_CMD_END		BIT(1)

#define HW_ECC_CTRL	0x14
#define	NAND_ECC_CAP			5
#define NAND_ECC_ERR_THRESHOLD		8
#define BM_ECC_ERR_OVER		BIT(2)
/* Uncorrected error. */
#define BM_ECC_ERR_UNC		BIT(1)
/* Corrected error. */
#define BM_ECC_ERR_CORRECT	BIT(0)

#define HW_ECC_OFFSET	0x18
#define HW_ADDR0_0	0x1c
#define HW_ADDR1_0	0x20
#define HW_ADDR0_1	0x24
#define HW_ADDR1_1	0x28
#define HW_SPARE_SIZE	0x30
#define HW_DMA_ADDR	0x64
#define HW_DMA_CNT	0x68
#define HW_DMA_CTRL	0x6c
#define HW_MEM_CTRL	0x80
#define HW_DATA_SIZE	0x84
#define HW_READ_STATUS	0x88
#define HW_TIM_SEQ_0	0x8c
#define HW_TIMING_ASYN	0x90
#define HW_TIMING_SYN	0x94

#define HW_FIFO_DATA	0x98
#define HW_TIME_MODE	0x9c
#define HW_FIFO_INIT	0xb0
/*
 * Counter for ecc related errors.
 * For each 512 byte block it has 5bit counter.
 */
#define HW_ECC_ERR_CNT	0xb8

#define HW_TIM_SEQ_1	0xc8

u32 reg_list[][4] = {
	{ 0, 0, 0, 0},
	{ 0, 0, 0, 4},
//	{ 0x4, 0, 0, 4},
	{ 0x8, 0, 0, 4},
	{ 0xc, 0, 0, 4},
	{ 0x10, 0, 0, 4},
//	{ 0x14, 0, 0, 4},
	{ 0x18, 0, 0, 4},
//	{ 0x1c, 0, 0, 4}, //address
	{ 0x20, 0, 0, 4},
	{ 0x24, 0, 0, 4},
	{ 0x28, 0, 0, 4},
	{ 0x2c, 0, 0, 4},
	{ 0x30, 0, 0, 4},
	{ 0x34, 0, 0, 4},
	{ 0x38, 0, 0, 4},
	{ 0x3c, 0, 0, 4},
	{ 0x40, 0, 0, 4},
	{ 0x44, 0, 0, 4},
	{ 0x48, 0, 0, 4},
	{ 0x4c, 0, 0, 4},
	{ 0x50, 0, 0, 4},
	{ 0x54, 0, 0, 4},
	{ 0x58, 0, 0, 4},
	{ 0x5c, 0, 0, 4},
	{ 0x60, 0, 0, 4},
	{ 0x64, 0, 0, 4},
	{ 0x68, 0, 0, 4},
	{ 0x6c, 0, 0, 4},
	{ 0x70, 0, 0, 4},
	{ 0x74, 0, 0, 4},
	{ 0x78, 0, 0, 4},
	{ 0x7c, 0, 0, 4},
	{ 0x80, 0, 0, 4},
	{ 0x84, 0, 0, 4},
	{ 0x88, 0, 0, 4},
	{ 0x8c, 0, 0, 4},
	{ 0x90, 0, 0, 4},
	{ 0x94, 0, 0, 4},
	//{ 0x98, 0, 0, 4}, //fifo data
	{ 0x9c, 0, 0, 4},
	{ 0xa0, 0, 0, 4},
	{ 0xa4, 0, 0, 4},
	{ 0xa8, 0, 0, 4},
	{ 0xac, 0, 0, 4},
	{ 0xb0, 0, 0, 4},
	{ 0xb4, 0, 0, 4},
	{ 0xb8, 0, 0, 4},
	{ 0xbc, 0, 0, 4},
	{ 0xc0, 0, 0, 4},
	{ 0xc4, 0, 0, 4},
	{ 0xc8, 0, 0, 4},
	{ 0xcc, 0, 0, 4},
	{ 0xd0, 0, 0, 4},
	{ 0xd4, 0, 0, 4},
	{ 0xd8, 0, 0, 4},
	{ 0xdc, 0, 0, 4},
	{ 0xe0, 0, 0, 4},
	{ 0xe4, 0, 0, 4},
	{ 0xe8, 0, 0, 4},
	{ 0xec, 0, 0, 4},
	{ 0xf0, 0, 0, 4},
	{ 0xf4, 0, 0, 4},
	{ 0xf8, 0, 0, 4},
	{ 0xfc, 0, 0, 4},
};

struct asm9260_nand_priv {
	struct device		*dev;
	struct mtd_info		mtd;
	struct nand_chip	nand;

	struct clk		*clk;
	struct clk		*clk_ahb;

	unsigned int block_shift;

	void __iomem *base;
	/* fall back dma buffer */
	void __iomem *bdma_virt;
	dma_addr_t bdma_phy;
	int bdma;
	wait_queue_head_t wq;
	int irq_done;

	u32 read_cache;
	int read_cache_cnt;
	u32 cmd_cache;
	u32 mem_status_mask;

	unsigned int addr_cycles;
	unsigned int col_cycles;
	unsigned int ecc_cap;
	unsigned int ecc_threshold;
	unsigned int spare_size;
};

#define mtd_to_priv(m)	container_of(m, struct asm9260_nand_priv, mtd)

static void asm9260_reg_snap(struct asm9260_nand_priv *priv)
{
	int a, b;
	b = reg_list[0][1] ? 2 : 1;
	reg_list[0][b] = 1;
	/* grub all needed regs */
	for (a = 1; a < ARRAY_SIZE(reg_list); a++) {
		if (reg_list[a][3] == 1)
			reg_list[a][b] = readb(priv->base + reg_list[a][0]);
		else if (reg_list[a][3] == 2)
			reg_list[a][b] = readw(priv->base + reg_list[a][0]);
		else if (reg_list[a][3] == 4)
			reg_list[a][b] = readl(priv->base + reg_list[a][0]);
		else
			printk("-- wrong lenght\n");
	}
	/* if we have two version, compare them */
	if (reg_list[0][1] && reg_list[0][2]) {
		for (a = 1; a < ARRAY_SIZE(reg_list); a++) {
			if (reg_list[a][1] != reg_list[a][2])
				printk("-- reg %02x: %08x %s %08x\n",
						reg_list[a][0],
						reg_list[a][1],
						b == 1 ? "<" : ">",
						reg_list[a][2]);
		}
		if (b == 1)
			reg_list[0][2] = 0;
		else
			reg_list[0][1] = 0;
	}
}



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
		iowrite32(ASM9260T_NAND_WP_STATE_MASK, priv->base + HW_MEM_CTRL);
	else {
		iowrite32(ASM9260T_NAND_WP_STATE_MASK | chip,
				priv->base + HW_MEM_CTRL);
		iowrite32((1 << (chip + 8)) ^ ioread32(priv->base + HW_MEM_CTRL),
				priv->base + HW_MEM_CTRL);	//clear WP reg
	}
}

static void asm9260_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
}

/* TODO: 3 commands are supported by HW. 3-d can be used for TWO PLANE. */
static void asm9260_nand_cmd_prep(struct asm9260_nand_priv *priv,
		u8 cmd0, u8 cmd1, u8 cmd2, u8 seq)
{
	priv->cmd_cache  = (cmd0 << NAND_CMD_CMD0) | (cmd1 << NAND_CMD_CMD1);
	priv->cmd_cache |= ADDR_SEL_0 << NAND_CMD_ADDR_SEL;
	priv->cmd_cache |= seq;
}

static dma_addr_t asm9260_nand_dma_set(struct mtd_info *mtd, void *buf,
		enum dma_data_direction dir, size_t size)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	dma_addr_t dma_addr;

	dma_addr = dma_map_single(priv->dev, buf, size, dir);
	if (dma_mapping_error(priv->dev, dma_addr)) {
		dev_err(priv->dev, "dma_map_single filed");
		return dma_addr;

	}

#if 0
if (!IS_ALIGNED((dma_addr_t) dma_addr, PAGE_SIZE)) {
		dma_unmap_single(priv->dev, dma_addr, size, dir);

		/* use fallback buffer */
		dma_addr =  priv->bdma_phy;

		if (dir == DMA_TO_DEVICE)
			memcpy_toio(priv->bdma_virt, buf, size);

		priv->bdma = 1;
	}
#endif


	iowrite32(dma_addr, priv->base + HW_DMA_ADDR);
	iowrite32(size, priv->base + HW_DMA_CNT);
	iowrite32((DMA_START_EN << NAND_DMA_CTRL_START)
		  | ((dir == DMA_FROM_DEVICE ? 1 : 0) << NAND_DMA_CTRL_DIR)
		  | (DMA_MODE_SFR << NAND_DMA_CTRL_MODE)
		  | (DMA_BURST_INCR16 << NAND_DMA_CTRL_BURST),
		  priv->base + HW_DMA_CTRL);
	return dma_addr;
}

static void asm9260_nand_dma_unset(struct mtd_info *mtd, void *buf,
		dma_addr_t dma_addr, enum dma_data_direction dir, size_t size)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);

	if (!priv->bdma) {
		dma_unmap_single(priv->dev, dma_addr, size, dir);
		return;
	}

	if (dir == DMA_FROM_DEVICE)
		memcpy_fromio(buf, priv->bdma_virt, size);
	priv->bdma = 0;
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
		priv->cmd_cache |= INPUT_SEL_DMA << NAND_CMD_INPUT_SEL;
		priv->irq_done = 0;
		/* FIXME Do we need CMD case? */
		iowrite32(BM_INT_MEM0_RDY, priv->base + HW_INT_MASK);
		//iowrite32(BM_INT_CMD_END | BM_INT_MEM0_RDY, priv->base + HW_INT_MASK);
	}

	iowrite32(priv->cmd_cache, priv->base + HW_CMD);
	cmd = priv->cmd_cache;
	priv->cmd_cache = 0;

	if (dma) {
		struct nand_chip *nand = &priv->nand;

		timeout = wait_event_timeout(nand->controller->wq,
				priv->irq_done, 10 * HZ);
		if (timeout <= 0) {
			dev_info(priv->dev,
                        	 "Request 0x%08x timed out\n", cmd);
			/* TODO: Do something useful here? */
		}
	} else
		nand_wait_ready(mtd);
}

static int asm9260_nand_dev_ready(struct mtd_info *mtd)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	u32 tmp;

	tmp = ioread32(priv->base + HW_STATUS);

	return (!(tmp & ASM9260T_NAND_CTRL_BUSY) &&
			(tmp & 0x1));
}

static void asm9260_nand_controller_config (struct mtd_info *mtd)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	struct nand_chip *nand = &priv->nand;

	iowrite32((NO_RNB_SEL << NAND_CTRL_RNB_SEL)
		| (priv->addr_cycles << NAND_CTRL_ADDR_CYCLE1)
		| (((nand->page_shift - 8) & 0x7) << NAND_CTRL_PAGE_SIZE)
		| (((priv->block_shift - 5) & 0x3) << NAND_CTRL_BLOCK_SIZE)
		| INT_EN << 4
		| (priv->addr_cycles),
		priv->base + HW_CTRL);
}

static void asm9260_nand_make_addr_lp(struct mtd_info *mtd,
		u32 row_addr, u32 column)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
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
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_ERASE2:
		return;

	case NAND_CMD_RESET:
		asm9260_nand_cmd_prep(priv, NAND_CMD_RESET, 0, 0, SEQ0);
		asm9260_nand_cmd_comp(mtd, 0);
		return;

	case NAND_CMD_READID:
		iowrite32(1, priv->base + HW_FIFO_INIT);
		iowrite32(
			(ADDR_CYCLE_1 << NAND_CTRL_ADDR_CYCLE1)
			| (DATA_SIZE_CUSTOM << NAND_CTRL_CUSTOM_SIZE_EN)
			| (PAGE_SIZE_4096B << NAND_CTRL_PAGE_SIZE)
			| (BLOCK_SIZE_32P << NAND_CTRL_BLOCK_SIZE)
			| INT_EN << 4
			| (ADDR_CYCLE_1),
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

		asm9260_nand_controller_config(mtd);

		if (column == 0) {
			iowrite32(
				(priv->ecc_threshold << NAND_ECC_ERR_THRESHOLD)
				| (priv->ecc_cap << NAND_ECC_CAP),
				priv->base + HW_ECC_CTRL);
			iowrite32(mtd->writesize + priv->spare_size,
					priv->base + HW_ECC_OFFSET);
			iowrite32(priv->spare_size, priv->base + HW_SPARE_SIZE);
		} else if (column == mtd->writesize) {
			asm9260_reg_rmw(priv, HW_CTRL,
				DATA_SIZE_CUSTOM << NAND_CTRL_CUSTOM_SIZE_EN, 0);
			iowrite32(mtd->oobsize, priv->base + HW_SPARE_SIZE);
			iowrite32(mtd->oobsize, priv->base + HW_DATA_SIZE);
		} else {
			dev_err(priv->dev, "Couldn't support the column\n");
			break;
		}

		asm9260_nand_make_addr_lp(mtd, page_addr, column);

		asm9260_nand_cmd_prep(priv, NAND_CMD_READ0,
				NAND_CMD_READSTART, 0, SEQ10);

		priv->read_cache_cnt = 0;
		break;
	case NAND_CMD_SEQIN:
		iowrite32(1, priv->base + HW_FIFO_INIT);
		asm9260_nand_controller_config(mtd);

		if (column == 0) {
			iowrite32(
				(priv->ecc_threshold << NAND_ECC_ERR_THRESHOLD)
				| (priv->ecc_cap << NAND_ECC_CAP),
				priv->base + HW_ECC_CTRL);
			iowrite32(mtd->writesize + priv->spare_size,
				priv->base + HW_ECC_OFFSET);
			iowrite32(priv->spare_size, priv->base + HW_SPARE_SIZE);
		} else if (column == mtd->writesize) {
			asm9260_reg_rmw(priv, HW_CTRL,
					DATA_SIZE_CUSTOM << NAND_CTRL_CUSTOM_SIZE_EN, 0);
			iowrite32(mtd->oobsize, priv->base + HW_DATA_SIZE);
		}

		asm9260_nand_make_addr_lp(mtd, page_addr, column);

		asm9260_nand_cmd_prep(priv, NAND_CMD_SEQIN, NAND_CMD_PAGEPROG,
			0, SEQ12);

		break;
	case NAND_CMD_STATUS:
		iowrite32(1, priv->base + HW_FIFO_INIT);
		asm9260_nand_controller_config(mtd);

		/*
		 * Workaround for status bug.
		 * Instead of SEQ4 we need to use SEQ1 here, which will
		 * send cmd with address. For this case we need to make sure
		 * ADDR == 0.
		 */
		asm9260_nand_make_addr_lp(mtd, 0, 0);
		asm9260_reg_rmw(priv, HW_CTRL,
				DATA_SIZE_CUSTOM << NAND_CTRL_CUSTOM_SIZE_EN, 0);
		iowrite32(4, priv->base + HW_DATA_SIZE);
		asm9260_nand_cmd_prep(priv, NAND_CMD_STATUS, 0, 0, SEQ1);

		priv->read_cache_cnt = 0;
		break;

	case NAND_CMD_ERASE1:
		asm9260_nand_make_addr_lp(mtd, page_addr, column);

		asm9260_nand_controller_config(mtd);

		/*
		 * Prepare and send command now. We don't need to split it in
		 * two stages.
		 */
		asm9260_nand_cmd_prep(priv, NAND_CMD_ERASE1, NAND_CMD_ERASE2,
				0, SEQ14);
		asm9260_nand_cmd_comp(mtd, 0);
		return;
	default:
		dev_err(priv->dev, "don't support this command : 0x%x!\n",
				command);
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
	asm9260_nand_cmd_comp(mtd, dma_ok);

	if (dma_ok) {
		dma_sync_single_for_cpu(priv->dev, dma_addr, len, DMA_FROM_DEVICE);
		asm9260_nand_dma_unset(mtd, buf, dma_addr,
				DMA_FROM_DEVICE, len);
		return;
	}

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
	if(dma_ok)
		dma_sync_single_for_device(priv->dev, dma_addr, len,
				DMA_TO_DEVICE);
	asm9260_nand_cmd_comp(mtd, dma_ok);

	if (dma_ok) {
		asm9260_nand_dma_unset(mtd, buf, dma_addr, DMA_TO_DEVICE, len);
		return;
	}

	len >>= 2;
	iowrite32_rep(priv->base + HW_FIFO_DATA, buf, len);
}

static int asm9260_nand_write_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, const u8 *buf,
		int oob_required)
{
	struct asm9260_nand_priv *priv = mtd_to_priv(mtd);
	u8 *temp_ptr;
	temp_ptr = (u8 *)buf;

	asm9260_reg_rmw(priv, HW_CTRL, ECC_EN << NAND_CTRL_ECC_EN, 0);
	chip->write_buf(mtd, temp_ptr, mtd->writesize);

	if (oob_required)
		chip->ecc.write_oob(mtd, chip, mtd->writesize);
	return 0;
}

static unsigned int asm9260_nand_count_ecc(struct asm9260_nand_priv *priv)
{
	u32 tmp, i, count = 0;

	/* FIXME: this layout was tested only on 2048byte NAND.
	 * NANDs with bigger page size should use more registers. */
	tmp = ioread32(priv->base + HW_ECC_ERR_CNT);
	for (i = 0; i < 4; i++)
		count += 0x1f & (tmp >> (5 * i));

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
	asm9260_reg_rmw(priv, HW_CTRL, ECC_EN << NAND_CTRL_ECC_EN, 0);
	chip->read_buf(mtd, temp_ptr, mtd->writesize);

	status = ioread32(priv->base + HW_ECC_CTRL);

	max_bitflips = asm9260_nand_count_ecc(priv);

	/* FIXME: do we need it for failed bit too? */
	if (status & BM_ECC_ERR_UNC)
		mtd->ecc_stats.failed += max_bitflips;
	else if (status & BM_ECC_ERR_CORRECT)
		mtd->ecc_stats.corrected += max_bitflips;

	if (oob_required)
		chip->ecc.read_oob(mtd, chip, page);

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

static void asm9260_nand_init_chip(struct nand_chip *nand_chip)
{
	nand_chip->select_chip = asm9260_select_chip;
	nand_chip->cmd_ctrl    = asm9260_cmd_ctrl;
	nand_chip->cmdfunc     = asm9260_nand_command_lp;
	nand_chip->read_byte   = asm9260_nand_read_byte;
	nand_chip->read_word   = asm9260_nand_read_word;
	nand_chip->read_buf    = asm9260_nand_read_buf;
	nand_chip->write_buf   = asm9260_nand_write_buf;

	nand_chip->dev_ready   = asm9260_nand_dev_ready;
	nand_chip->chip_delay  = 100;

	nand_chip->ecc.mode = NAND_ECC_HW;

	nand_chip->ecc.write_page	= asm9260_nand_write_page_hwecc;
	nand_chip->ecc.read_page	= asm9260_nand_read_page_hwecc;
}

static void asm9260_nand_timing_config(struct asm9260_nand_priv *priv)
{
	u32 twhr;
	u32 trhw;
	u32 trwh;
	u32 trwp;
	u32 tadl = 0;
	u32 tccs = 0;
	u32 tsync = 0;
	u32 trr = 0;
	u32 twb = 0;

	trwh = 1; //TWH;
	trwp = 1; //TWP;
	iowrite32((trwh << 4) | (trwp), priv->base + HW_TIMING_ASYN);

	twhr = 2;
	trhw = 4;
	iowrite32((twhr << 24) | (trhw << 16)
		| (tadl << 8) | (tccs), priv->base + HW_TIM_SEQ_0);

	iowrite32((tsync << 16) | (trr << 9) | (twb),
			priv->base + HW_TIM_SEQ_1);
}

static int asm9260_ecc_cap_select(struct asm9260_nand_priv *priv,
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

static void asm9260_nand_ecc_conf(struct asm9260_nand_priv *priv)
{
	struct nand_chip *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;

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

static int asm9260_nand_get_dt_clks(struct asm9260_nand_priv *priv)
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

static int asm9260_nand_alloc_dma(struct asm9260_nand_priv *priv,
		size_t size)
{
	priv->bdma_virt = dmam_alloc_coherent(priv->dev, size, &priv->bdma_phy,
			GFP_KERNEL);
	priv->bdma = 0;
	return 0;
}

static int asm9260_nand_probe(struct platform_device *pdev)
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

	if (asm9260_nand_get_dt_clks(priv))
		return -ENODEV;

	asm9260_nand_alloc_dma(priv, 8192 + 448);

	irq = irq_of_parse_and_map(np, 0);
	if (!irq)
		return -ENODEV;

	iowrite32(0, priv->base + HW_INT_MASK);
	ret = devm_request_irq(priv->dev, irq, asm9260_nand_irq,
				IRQF_ONESHOT, np->full_name, priv);
				//IRQF_SHARED, np->full_name, priv);

	asm9260_nand_init_chip(nand);

	asm9260_nand_timing_config(priv);

	/* first scan to find the device and get the page size */
	if (nand_scan_ident(mtd, 1, NULL)) {
		dev_err(&pdev->dev, "scan_ident filed!\n");
		return -ENXIO;
	}

	asm9260_nand_ecc_conf(priv);

	/* second phase scan */
	if (nand_scan_tail(mtd)) {
		dev_err(&pdev->dev, "scan_tail filed!\n");
		return -ENXIO;
	}

	/* FIXME: remove it or replace it */
	/* FIXME: these complete part need fixing  */
	priv->block_shift = __ffs(mtd->erasesize) - nand->page_shift;
	priv->col_cycles  = 2;
	priv->addr_cycles = priv->col_cycles +
		(((mtd->size >> mtd->writesize) > 65536) ? 3 : 2);

	priv->mem_status_mask = ASM9260T_NAND_DEV0_READY;

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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chen Dongdong <chendd@alphascale.cn>");
MODULE_DESCRIPTION("ASM9260 MTD NAND driver");
