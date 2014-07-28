/********************************************************
Copyright (C), 2007-2013, Alpscale Tech. Co., Ltd.
File name: asm9260_nand.c
Author: ChenDongdong    Version: 1.0    Date: 2012-04-17
Description: asm9260 nand driver.
History: 
1. Date: 2012-12-17		Version: 0.1
Author:	ChenDongdong
Description: Rewrite the NAND driver.
2. Date: 2012-12-21		Version: 0.2
Author:	ChenDongdong
Modification: Code is basically completed(for 4K), but the driver exist BUG.
3. Date: 2012-01-04		Version: 0.3
Author:	ChenDongdong
Modification: 	4K page size NAND is supported. 
				Only supports PIO mode of operation.
				Update yaffs2.
4. Date: 2012-01-06		Version: 0.4
Author:	ChenDongdong
Modification: 	Supports DMA operation.
5. Date: 2012-01-07		Version: 0.5
Author:	ChenDongdong
Modification: 	Supports NAND verify.
6. Date: 2012-01-08		Version: 0.6
Author:	ChenDongdong
Modification: 	Supports 2K page size NAND.
				Modify h-boot.
7. Date: 2012-01-09		Version: 0.7
Author:	ChenDongdong
Modification: 	Supports 8K page size NAND.
				Modify h-boot.
8. Date: 2012-01-10		Version: 1.0
Author:	ChenDongdong
Modification: 	Tidy up code.				

********************************************************/

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
#include <mach/system.h>
#include <mach/pincontrol.h>
#include <mach/dma.h>
#include <asm/hardware/iomd.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/uart_reg.h>
#include <mach/asm9260_nand.h>

// ================== Definitions ====================

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
#define  RESET                        0xFF
#define  SYNC_RESET                   0xFC

#define  READ_ID                      0x90

#define  READ_STATUS                  0x70
#define  READ_STATUS_ENHANCE          0x78

#define  CHANGE_WRITE_COLUMN          0x85
#define  CHANGE_ROW_ADDRESS           0x85

#define  READ_PAGE_1                  0x00
#define  READ_PAGE_2                  0x30

#define  PROGRAM_PAGE_1               0x80
#define  PROGRAM_PAGE_2               0x10

#define  PROGRAM_PAGE1                0x80

#define  WRITE_PAGE                   0x10
#define  WRITE_PAGE_CACHE             0x15
#define  WRITE_MULTIPLANE             0x11

#define  ERASE_BLOCK_1                0x60
#define  ERASE_BLOCK_2                0xD0



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

#define	NAND_ECC_CAP				5
#define NAND_ECC_ERR_THRESHOLD		8




struct asm9260_nand_regs{
	volatile uint32_t nand_command;          
	volatile uint32_t nand_control;          
	volatile uint32_t nand_status;           
	volatile uint32_t nand_int_mask;         
	volatile uint32_t nand_int_status;       
	volatile uint32_t nand_ecc_ctrl;         
	volatile uint32_t nand_ecc_offset;       
	volatile uint32_t nand_addr0_l;          
	volatile uint32_t nand_addr1_l;
	volatile uint32_t nand_addr0_h;          
	volatile uint32_t nand_addr1_h;          
	volatile uint32_t nand_rsvd0;
	volatile uint32_t nand_spare_size;    
	volatile uint32_t nand_rsvd1;
	volatile uint32_t nand_protect;
	volatile uint32_t nand_rsvd2;
	volatile uint32_t nand_lookup_en;        
	volatile uint32_t nand_lookup0;          
	volatile uint32_t nand_lookup1;          
	volatile uint32_t nand_lookup2;          
	volatile uint32_t nand_lookup3;          
	volatile uint32_t nand_lookup4;          
	volatile uint32_t nand_lookup5;          
	volatile uint32_t nand_lookup6;          
	volatile uint32_t nand_lookup7;          
	volatile uint32_t nand_dma_addr;         
	volatile uint32_t nand_dma_cnt;          
	volatile uint32_t nand_dma_ctrl;
	volatile uint32_t nand_rsvd3;
	volatile uint32_t nand_rsvd4;
	volatile uint32_t nand_rsvd5;
	volatile uint32_t nand_rsvd6;
	volatile uint32_t nand_mem_ctrl;         
	volatile uint32_t nand_data_size;        
	volatile uint32_t nand_read_status;      
	volatile uint32_t nand_time_seq_0;       
	volatile uint32_t nand_timings_asyn;     
	volatile uint32_t nand_timings_syn;      
	volatile uint32_t nand_fifo_data;        
	volatile uint32_t nand_time_mode;        
	volatile uint32_t nand_dma_addr_offset;  
	volatile uint32_t nand_rsvd7;
	volatile uint32_t nand_rsvd8;
	volatile uint32_t nand_rsvd9;
	volatile uint32_t nand_fifo_init;        
	volatile uint32_t nand_generic_seq_ctrl; 
	volatile uint32_t nand_err_cnt00;        
	volatile uint32_t nand_err_cnt01;        
	volatile uint32_t nand_err_cnt10;        
	volatile uint32_t nand_err_cnt11;        
	volatile uint32_t nand_time_seq_1;       
};

/*2KB--4*512B, correction ability: 4bit--7Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_64 = {
 	.eccbytes = 28,
 	.eccpos =  {
 			36, 37, 38, 39, 40, 41, 42,
			43, 44, 45, 46, 47, 48, 49,
			50, 51, 52, 53, 54, 55, 56,
			57, 58, 59, 60, 61, 62, 63},
 	.oobfree = {{2, 34}}
};

/*4KB--8*512B, correction ability: 6bit--10Byte ecc*/
static struct nand_ecclayout asm9260_nand_oob_128 = {
	.eccbytes = 8*10,
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
	.eccbytes = 8*23,  
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
	.eccbytes = 8*23,  
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

#define ASM9260_NAND_ERR_CORRECT	1
#define ASM9260_NAND_ERR_UNCORRECT	2
#define ASM9260_NAND_ERR_OVER		3

#define  NAND_BASE_ADDR         	0xF0600000

#ifdef CONFIG_MTD_NAND_ASM9260_DEBUG
#define DBG(x...) printk("ASM9260_NAND_DBG: " x)
#else
#define DBG(x...)
#endif

extern void set_pin_mux(int port,int pin,int mux_type);
extern void set_GPIO(int port,int pin);
static u_int8_t asm9260_nand_read_byte(struct mtd_info *mtd);
struct asm9260_nand_regs *nand_regs;
struct mtd_info *asm9260_mtd;
struct nand_chip *asm9260_nand;
static int read_cache_byte_cnt = 0;
static uint8_t read_cache[4] = {0};
static uint32_t *read_val = (uint32_t *)read_cache;
static uint8_t __attribute__((aligned(32))) NandAddr[32] = {0}; 
static uint32_t page_shift, block_shift, addr_cycles, row_cycles, col_cycles;
static uint32_t asm9260_nand_spare_data_size;
static uint32_t asm9260_nand_acceptable_err_level = 0;
static uint32_t asm9260_nand_ecc_correction_ability = 0;
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
static uint8_t *asm9260_nand_verify_buffer = NULL;
#endif
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
static uint8_t *asm9260_nand_dma_buf_virt;
static uint32_t asm9260_nand_dma_buf_phy;
static uint8_t *asm9260_nand_dma_read_buf_virt;
static uint8_t *asm9260_nand_dma_write_buf_virt;
static uint32_t asm9260_nand_dma_read_buf_phy;
static uint32_t asm9260_nand_dma_write_buf_phy;
#endif

#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif
#endif


static void asm9260_select_chip(struct mtd_info *mtd, int chip)
{
	if (chip == -1)
	{
		nand_regs->nand_mem_ctrl = ASM9260T_NAND_WP_STATE_MASK;		
	}
	else
	{
		nand_regs->nand_mem_ctrl = ASM9260T_NAND_WP_STATE_MASK | chip;
		nand_regs->nand_mem_ctrl = (1UL << (chip+8)) ^ (nand_regs->nand_mem_ctrl);	//clear WP reg
	}
}

static void asm9260_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	static int count = 0;
	printk("asm9260_cmd_ctrl count %d\n", ++count);
}

/*
* 等待NAND控制器ready
*/
static int asm9260_nand_controller_ready(void)
{
	int ret = 1;
	int waittime = 0;
	int timeout = 0x1000000;	

	while ((nand_regs->nand_status) & ASM9260T_NAND_CTRL_BUSY)
	{
		waittime++;
		if (waittime > timeout)
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

/*
* 等待NAND设备ready
*/
static int asm9260_nand_dev_ready(struct mtd_info *mtd)
{
	int ret = 1;
	int waittime = 0;
	int timeout = 0x1000000;

	while (!((nand_regs->nand_status) & ASM9260T_NAND_DEV0_READY))
	{
		waittime++;
		if (waittime > timeout)
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

#if 0
static int asm9260_nand_ecc_check(void)
{
	int ret = 0;
	uint32_t ecc_status = 0;
	
	ecc_status = nand_regs->nand_ecc_ctrl;
	if (ecc_status & 0x01)
	{
		DBG("NAND_ERR_CORRECT!!!\r\n");
		ret = ASM9260_NAND_ERR_CORRECT;
	}
	if (ecc_status & 0x02)
	{
		DBG("NAND_ERR_UNCORRECT!!!\r\n");
		ret = ASM9260_NAND_ERR_UNCORRECT;
	}
	if (ecc_status & 0x04)
	{
		DBG("NAND_ERR_OVER!!!\r\n");
		ret = ASM9260_NAND_ERR_OVER;
	}

	return ret;
}
#endif

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
/*
* 等待DMA传输完成
*/
static int asm9260_nand_dma_ready(void)
{
	int ret = 1;
	int waittime = 0;
	int timeout = 0x1000000;

	while(!(nand_regs->nand_dma_ctrl & ASM9260T_NAND_DMA_READY))
	{
		waittime++;
		if (waittime > timeout)
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

/*
* 检查DMA传输是否发生错误
*/
static int asm9260_nand_dma_error(void)
{
	int ret = 0;

	if ((nand_regs->nand_dma_ctrl & ASM9260T_NAND_DMA_ERROR) != 0)
	{
		return ASM9260T_NAND_DMA_ERROR;
	}
	
	return ret;
}
#endif

static void asm9260_nand_pin_mux(void)
{
	/*set pin assign*/
    set_pin_mux(11,0,5);
    set_pin_mux(11,1,5);
    set_pin_mux(11,2,5);
    set_pin_mux(11,3,5);
    set_pin_mux(11,4,5);
    set_pin_mux(11,5,5);
    set_pin_mux(11,6,5);
    set_pin_mux(12,0,5);
    set_pin_mux(12,1,5);
    set_pin_mux(12,2,5);
    set_pin_mux(12,3,5);
    set_pin_mux(12,4,5);
    set_pin_mux(12,5,5);
    set_pin_mux(12,6,5);
    set_pin_mux(12,7,5);
}

/*
* NAND芯片复位
*/
int asm9260_nand_reset(uint8_t nChip)
{
	nand_regs->nand_mem_ctrl = (ASM9260T_NAND_WP_STATE_MASK | nChip);
	nand_regs->nand_command  = (RESET << NAND_CMD_CMD0)
					        | (ADDR_SEL_0 << NAND_CMD_ADDR_SEL)
					        | (INPUT_SEL_BIU << NAND_CMD_INPUT_SEL)
					        | (SEQ0);
	

	return !asm9260_nand_dev_ready(asm9260_mtd);
}

/*
* 设置NAND控制器时序
*/
static int asm9260_nand_timing_config(void)
{
	int ret = 0;
	uint32_t twhr;
	uint32_t trhw;
	uint32_t trwh;
	uint32_t trwp;
	uint32_t tadl = 0;
	uint32_t tccs = 0;
	uint32_t tsync = 0;
	uint32_t trr = 0;
	uint32_t twb = 0;

	/*default config before read id*/

	nand_regs->nand_control = (ADDR_CYCLE_1 << NAND_CTRL_ADDR_CYCLE1) 		| (ADDR1_AUTO_INCR_DIS << NAND_CTRL_ADDR1_AUTO_INCR)
						   | (ADDR0_AUTO_INCR_DIS << NAND_CTRL_ADDR0_AUTO_INCR)	| (WORK_MODE_ASYNC << NAND_CTRL_WORK_MODE)
						   | (PROT_DIS << NAND_CTRL_PORT_EN) 			| (LOOKUP_DIS << NAND_CTRL_LOOKU_EN)
						   | (IO_WIDTH_8 << NAND_CTRL_IO_WIDTH) 		 	| (DATA_SIZE_CUSTOM << NAND_CTRL_CUSTOM_SIZE_EN)
						   | (PAGE_SIZE_4096B << NAND_CTRL_PAGE_SIZE) 		| (BLOCK_SIZE_32P << NAND_CTRL_BLOCK_SIZE)
						   | (ECC_DIS << NAND_CTRL_ECC_EN) 				| (INT_DIS << NAND_CTRL_INT_EN)
						   | (SPARE_DIS << NAND_CTRL_SPARE_EN) 			| (ADDR_CYCLE_1);
	

	// init timing registers
//	trwh = 8;
//	trwp = 8;

//	nand_regs->nand_timings_asyn = (trwh << 4) | (trwp);

//	ret = asm9260_nand_reset(0);
//	if (ret != 0)
//	{
//		return ret;
//	}
	
	trwh = 1; //TWH;
	trwp = 1; //TWP;
	nand_regs->nand_timings_asyn = (trwh << 4) | (trwp);

	twhr = 2;
	trhw = 4;
	nand_regs->nand_time_seq_0 = (twhr << 24) |
						         (trhw << 16) |
						         (tadl << 8)  |
						         (tccs);

	nand_regs->nand_time_seq_1 = (tsync << 16) |
						         (trr << 9) |
						         (twb);
	
	return ret;
}

static int asm9260_nand_inithw(uint8_t nChip)
{
	int ret = 0;

	/*open clk*/
    as3310_writel(ASM9260T_NAND_CLK_EN, HW_AHBCLKCTRL1 + 4);  // open nand pclk
    as3310_writel(ASM9260T_NAND_CLK_DIV, HW_NANDCLKDIV);      // set nand clk to 1/2 pclk

	asm9260_nand_pin_mux();				/*设置PIN Mux*/

	nand_regs->nand_mem_ctrl = (ASM9260T_NAND_WP_STATE_MASK |  nChip);
	nand_regs->nand_mem_ctrl = (1UL << (nChip+8)) ^ (nand_regs->nand_mem_ctrl);
	

	ret = asm9260_nand_timing_config();		/*设置NAND控制器时序*/
	if (ret != 0)
	{
		return ret;
	}
	
	ret = asm9260_nand_reset(nChip);	/*复位*/

	return ret;
}


/*
* 配置NAND控制器
*/

static void asm9260_nand_controller_config (struct mtd_info *mtd)
{
	static int count = 1;
	uint32_t chip_size   = mtd->size;
	uint32_t page_size   = mtd->writesize;

	if (count)
	{
		count = 0;
		page_shift = __ffs(page_size);
		block_shift = __ffs(mtd->erasesize) - page_shift;
		
		col_cycles  = 2;	//目前所有支持NAND的col_cycles都为2
		addr_cycles = col_cycles + (((chip_size >> page_size) > 65536) ? 3 : 2);
		row_cycles  = addr_cycles - col_cycles;
		DBG("page_shift: 0x%x.\n", page_shift);
		DBG("block_shift: 0x%x.\n", block_shift);
		DBG("col_cycles: 0x%x.\n", col_cycles);
		DBG("addr_cycles: 0x%x.\n", addr_cycles);
	}
	
	nand_regs->nand_control = (EN_STATUS << NAND_CTRL_DIS_STATUS)				| (NO_RNB_SEL << NAND_CTRL_RNB_SEL)
						    | (BIG_BLOCK_EN << NAND_CTRL_SMALL_BLOCK_EN)	 		| (addr_cycles << NAND_CTRL_ADDR_CYCLE1)
						    | (ADDR1_AUTO_INCR_DIS << NAND_CTRL_ADDR1_AUTO_INCR) 	| (ADDR0_AUTO_INCR_DIS << NAND_CTRL_ADDR0_AUTO_INCR)
					 	    | (WORK_MODE_ASYNC << NAND_CTRL_WORK_MODE)	 	| (PROT_DIS << NAND_CTRL_PORT_EN)
						    | (LOOKUP_DIS << NAND_CTRL_LOOKU_EN)			 	| (IO_WIDTH_8 << NAND_CTRL_IO_WIDTH)
						    | (DATA_SIZE_FULL_PAGE << NAND_CTRL_CUSTOM_SIZE_EN) 	| (((page_shift - 8) & 0x7) << NAND_CTRL_PAGE_SIZE)
						    | (((block_shift - 5) & 0x3) << NAND_CTRL_BLOCK_SIZE)	 | (ECC_EN<< NAND_CTRL_ECC_EN)
						    | (INT_DIS << NAND_CTRL_INT_EN)				 	| (SPARE_EN << NAND_CTRL_SPARE_EN)
						    | (addr_cycles);

}


/*******************************************************************************
*函数名:	asm9260_nand_make_addr_lp
*功能:		设置将要发送到控制器的地址，控制器将地址处理后发送给NAND。
*输入参数:	nPage--NAND页数
*			oob_flag--设置的如果是OOB的地址，该参数需为1，否则为0
*输出参数:	pAddr--设置后地址的保存处
*返回值:	无
*NOTE:		1、调用方需保证存放地址的内存，
*			该驱动申请了全局数组NandAddr[32]专用于设置地址的存放
*			2、擦除时，控制器会自动取行地址发给NAND，因此不需要进行专门的处理
*******************************************************************************/
static void asm9260_nand_make_addr_lp(struct mtd_info *mtd, uint32_t nPage, uint32_t nColumn, uint8_t *pAddr)
{
	int i = 0;
	uint32_t row_addr = nPage;

	//清空
	memset(pAddr, 0, 32);

	//设置列地址
	for (i=0; i<col_cycles; i++)
	{
		pAddr[i] = (uint8_t)(nColumn & 0xFF);
		nColumn = nColumn >> 8;
	}
	
	//设置行地址,其实就是nPage页号
	for (i = col_cycles; i < addr_cycles; i++)
	{
		pAddr[i] = (uint8_t)(row_addr & 0xFF);		//字节掩码
		row_addr = row_addr >> 8;				//字节位数
	}
}


/**
 * nand_command_lp - [DEFAULT] Send command to NAND large page device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices We dont have the separate regions as we have in the small page
 * devices.  We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
static void asm9260_nand_command_lp(struct mtd_info *mtd, unsigned int command, int column, int page_addr)
{
	uint32_t *addr = (uint32_t *)NandAddr;
	int ret;
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	static int flag = 0;
#endif

	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	ret = !asm9260_nand_dev_ready(0);
	if (ret)
		DBG("wait for device ready timeout.\n");
	
	switch (command)
	{
		case NAND_CMD_PAGEPROG:
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA

			if (flag == 1)
			{
				flag = 0;
				nand_regs->nand_dma_ctrl = (DMA_START_EN<<NAND_DMA_CTRL_START)
										 | (DMA_DIR_WRITE<<NAND_DMA_CTRL_DIR)
										 | (DMA_MODE_SFR<<NAND_DMA_CTRL_MODE)
										 | (DMA_BURST_INCR16<<NAND_DMA_CTRL_BURST);
				nand_regs->nand_command  = (PROGRAM_PAGE_2<<NAND_CMD_CMD1)
										 | (PROGRAM_PAGE_1<<NAND_CMD_CMD0)
										 | (ADDR_SEL_0<<NAND_CMD_ADDR_SEL)
										 | (INPUT_SEL_DMA<<NAND_CMD_INPUT_SEL)
										 | (SEQ12);

				break;
			}
#endif

		case NAND_CMD_CACHEDPROG:
		case NAND_CMD_ERASE2:
			break;
			
		case NAND_CMD_RESET:
			nand_regs->nand_command = (RESET << NAND_CMD_CMD0)
							        | (ADDR_SEL_0 << NAND_CMD_ADDR_SEL)
							        | (INPUT_SEL_BIU << NAND_CMD_INPUT_SEL)
							        | (SEQ0);
			break;

		case NAND_CMD_READID:
			nand_regs->nand_control   = (ADDR_CYCLE_1 << NAND_CTRL_ADDR_CYCLE1) 		| (ADDR1_AUTO_INCR_DIS << NAND_CTRL_ADDR1_AUTO_INCR)
									  | (ADDR0_AUTO_INCR_DIS << NAND_CTRL_ADDR0_AUTO_INCR)	| (WORK_MODE_ASYNC << NAND_CTRL_WORK_MODE)
									  | (PROT_DIS << NAND_CTRL_PORT_EN) 			| (LOOKUP_DIS << NAND_CTRL_LOOKU_EN)
									  | (IO_WIDTH_8 << NAND_CTRL_IO_WIDTH) 		 	| (DATA_SIZE_CUSTOM << NAND_CTRL_CUSTOM_SIZE_EN)
									  | (PAGE_SIZE_4096B << NAND_CTRL_PAGE_SIZE) 		| (BLOCK_SIZE_32P << NAND_CTRL_BLOCK_SIZE)
									  | (ECC_DIS << NAND_CTRL_ECC_EN) 				| (INT_DIS << NAND_CTRL_INT_EN)
									  | (SPARE_DIS << NAND_CTRL_SPARE_EN) 			| (ADDR_CYCLE_1); 
			nand_regs->nand_fifo_init = 1;	//reset FIFO
			nand_regs->nand_data_size = 8;	//ID 4 Bytes
			nand_regs->nand_addr0_l   = column;
			nand_regs->nand_command   = (READ_ID << NAND_CMD_CMD0)
									  | (ADDR_SEL_0 << NAND_CMD_ADDR_SEL)
									  | (INPUT_SEL_BIU << NAND_CMD_INPUT_SEL)
									  | (SEQ1);

			read_cache_byte_cnt = 0;
			
			break;

		case NAND_CMD_READ0:

			/*1、复位FIFO，配置NAND控制器*/
			nand_regs->nand_fifo_init = 1;
			
			asm9260_nand_controller_config(mtd);

			if (column == 0)
			{
				nand_regs->nand_ecc_ctrl = (asm9260_nand_acceptable_err_level << NAND_ECC_ERR_THRESHOLD) 
										| (asm9260_nand_ecc_correction_ability << NAND_ECC_CAP);
				nand_regs->nand_ecc_offset = mtd->writesize + asm9260_nand_spare_data_size;
				nand_regs->nand_spare_size = asm9260_nand_spare_data_size;
			}
			else if (column == mtd->writesize)
			{
				nand_regs->nand_control = ((nand_regs->nand_control) & (~(ECC_EN << NAND_CTRL_ECC_EN))) | (DATA_SIZE_CUSTOM<< NAND_CTRL_CUSTOM_SIZE_EN);
				nand_regs->nand_spare_size = mtd->oobsize;
				nand_regs->nand_data_size = mtd->oobsize;
			}
			else
			{
				printk("couldn't support the column\n");
				break;
			}

			/*2、选择chip，配置NAND地址*/
			asm9260_nand_make_addr_lp(mtd, page_addr, column, NandAddr);

			nand_regs->nand_addr0_l = addr[0];
			nand_regs->nand_addr0_h = addr[1];

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
			if (column == 0)
			{
				/*DMA配置，DMA有两种模式，其中SFR managed模式在单个DMA包情况下使用*/
				nand_regs->nand_dma_addr = asm9260_nand_dma_read_buf_phy;
				nand_regs->nand_dma_cnt  = mtd->writesize + asm9260_nand_spare_data_size;
				nand_regs->nand_dma_ctrl = (DMA_START_EN<<NAND_DMA_CTRL_START)
										 | (DMA_DIR_READ<<NAND_DMA_CTRL_DIR)
										 | (DMA_MODE_SFR<<NAND_DMA_CTRL_MODE)
										 | (DMA_BURST_INCR16<<NAND_DMA_CTRL_BURST);
				nand_regs->nand_command  = (READ_PAGE_2<<NAND_CMD_CMD1)
										 | (READ_PAGE_1<<NAND_CMD_CMD0)
										 | (ADDR_SEL_0<<NAND_CMD_ADDR_SEL)
										 | (INPUT_SEL_DMA<<NAND_CMD_INPUT_SEL)
										 | (SEQ10);
				break;
			}
#endif

			/*3、发起命令*/
			nand_regs->nand_command = (READ_PAGE_2<<NAND_CMD_CMD1)
								    | (READ_PAGE_1<<NAND_CMD_CMD0)
								    | (ADDR_SEL_0<<NAND_CMD_ADDR_SEL)
								    | (INPUT_SEL_BIU<<NAND_CMD_INPUT_SEL)
								    | (SEQ10);
			
			read_cache_byte_cnt = 0;
			break;
		case NAND_CMD_SEQIN:

			/*1、复位FIFO，配置NAND控制器*/
			nand_regs->nand_fifo_init = 1;
			asm9260_nand_controller_config(mtd);

			if (column == 0)
			{
				nand_regs->nand_ecc_ctrl = (asm9260_nand_acceptable_err_level << NAND_ECC_ERR_THRESHOLD) | (asm9260_nand_ecc_correction_ability << NAND_ECC_CAP);
				nand_regs->nand_ecc_offset = mtd->writesize + asm9260_nand_spare_data_size;
				nand_regs->nand_spare_size = asm9260_nand_spare_data_size;
			}
			else if (column == mtd->writesize)
			{
				nand_regs->nand_control = ((((nand_regs->nand_control)) | (DATA_SIZE_CUSTOM << NAND_CTRL_CUSTOM_SIZE_EN)) & (~(ECC_EN << NAND_CTRL_ECC_EN))) 
										& (~(SPARE_EN << NAND_CTRL_SPARE_EN));
				nand_regs->nand_data_size = mtd->oobsize;
			}
 
			/*2、选择chip，配置NAND地址*/
			asm9260_nand_make_addr_lp(mtd, page_addr, column, NandAddr);
			nand_regs->nand_addr0_l = addr[0];
			nand_regs->nand_addr0_h = addr[1];

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
			if (column == 0)
			{
				flag = 1;

				/*DMA配置，DMA有两种模式，其中SFR managed模式在单个DMA包情况下使用*/
				nand_regs->nand_dma_addr = asm9260_nand_dma_write_buf_phy;
				nand_regs->nand_dma_cnt  = mtd->writesize + asm9260_nand_spare_data_size;
				break;
			}
#endif

			/*3、发起命令*/
			nand_regs->nand_command = (PROGRAM_PAGE_2 << NAND_CMD_CMD1)
								    | (PROGRAM_PAGE_1 << NAND_CMD_CMD0)
								    | (ADDR_SEL_0 << NAND_CMD_ADDR_SEL)
								    | (INPUT_SEL_BIU << NAND_CMD_INPUT_SEL)
								    | (SEQ12);

			break;
		case NAND_CMD_STATUS:
			
			asm9260_nand_controller_config(mtd);
			nand_regs->nand_control = ((((nand_regs->nand_control) & (~(SPARE_EN << NAND_CTRL_SPARE_EN))) & (~(ECC_EN<< NAND_CTRL_ECC_EN))) 
									| (DATA_SIZE_CUSTOM << NAND_CTRL_CUSTOM_SIZE_EN));
			nand_regs->nand_data_size = 1;
			nand_regs->nand_command = (READ_STATUS<<NAND_CMD_CMD0) |
							          (ADDR_SEL_0<<NAND_CMD_ADDR_SEL) |
							          (INPUT_SEL_BIU<<NAND_CMD_INPUT_SEL) |
							          (SEQ1);

			read_cache_byte_cnt = 0;
			break;

		case NAND_CMD_ERASE1:

			asm9260_nand_make_addr_lp(mtd, page_addr, column, NandAddr);
			nand_regs->nand_addr0_l = addr[0];
			nand_regs->nand_addr0_h = addr[1];

			asm9260_nand_controller_config(mtd);
			nand_regs->nand_control = (nand_regs->nand_control) & ((~(ECC_EN << NAND_CTRL_ECC_EN)) & (~(SPARE_EN << NAND_CTRL_SPARE_EN)));

			nand_regs->nand_command = (ERASE_BLOCK_2<<NAND_CMD_CMD1)
									| (ERASE_BLOCK_1<<NAND_CMD_CMD0)
									| (ADDR_SEL_0<<NAND_CMD_ADDR_SEL)
									| (INPUT_SEL_BIU<<NAND_CMD_INPUT_SEL)
									| (SEQ14);
			break;

		default:
			printk("don't support this command : 0x%x!\n", command);
	}

	if((command == NAND_CMD_RESET) || (command == NAND_CMD_READID) || (command == NAND_CMD_STATUS) || (command == NAND_CMD_ERASE1))
	{
		ret = !asm9260_nand_dev_ready(0);
		if (ret)
			DBG("wait for device ready timeout, ret = 0x%x.\n", ret);
	}
	else if ((command == NAND_CMD_READ0))
	{
#ifdef	CONFIG_MTD_NAND_ASAP9260_DMA
	
		if (column == 0)
		{
			ret = !asm9260_nand_dev_ready(0);
			if (ret)
				DBG("wait for device ready timeout, ret = 0x%x.\n", ret);

			ret = !asm9260_nand_dma_ready();
			if (ret)
				DBG("wait for dma ready timeout, ret = 0x%x.\n", ret);
		}
		else
		{
			ret = !asm9260_nand_controller_ready();
			if (ret)
				DBG("wait for device ready timeout, ret = 0x%x.\n", ret);
		}
#else
		ret = !asm9260_nand_controller_ready();
		if (ret)
			DBG("wait for device ready timeout, ret = 0x%x.\n", ret);
#endif
	}
	return ;
}

static u_int8_t asm9260_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t this_byte;

	if ((read_cache_byte_cnt <= 0) || (read_cache_byte_cnt > 4))
	{
		*read_val = ioread32(&nand_regs->nand_fifo_data);
		read_cache_byte_cnt = 4;
	}

	this_byte = read_cache[sizeof(read_cache) - read_cache_byte_cnt];
	read_cache_byte_cnt--;

	return this_byte;
}

static uint16_t asm9260_nand_read_word(struct mtd_info *mtd)
{
	uint16_t this_word = 0;
	uint16_t *val_tmp = (uint16_t *)read_cache;
		
	if ((read_cache_byte_cnt <= 0) || (read_cache_byte_cnt > 4))
	{
		*read_val = ioread32(&nand_regs->nand_fifo_data);
		read_cache_byte_cnt = 4;
	}

	if (read_cache_byte_cnt == 4)
		this_word = val_tmp[0];
	else if (read_cache_byte_cnt == 2)
		this_word = val_tmp[1];

	read_cache_byte_cnt -= 2;

	return this_word;
}

static void asm9260_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	uint32_t i;
	uint32_t *tmpbuf = (u32 *)buf;

	if (len & 0x3)
	{
		printk("Unsupported length\n");
		return;
	}

	for (i = 0; i < (len>>2); i++)
	{
		tmpbuf[i] = (nand_regs->nand_fifo_data);
	}
}


static void asm9260_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	u32 i;
	u32 *tmpbuf = (u32 *)buf;

	if (len & 0x3)
	{
		printk("Unsupported length\n");
		return;
	}

	for (i = 0; i < (len >> 2); i++)
	{
		nand_regs->nand_fifo_data = tmpbuf[i];
	}

}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
/**
 * nand_verify_buf - [DEFAULT] Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 * Default verify function for 8bit buswith
 */
static int asm9260_nand_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i;

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	memcpy(asm9260_nand_verify_buffer, asm9260_nand_dma_read_buf_virt, mtd->writesize + mtd->oobsize);
#else
	asm9260_nand_read_buf(mtd, asm9260_nand_verify_buffer, mtd->writesize + asm9260_nand_spare_data_size);
#endif

	for (i = 0; i < len; i++)
	{
		if (buf[i] != asm9260_nand_verify_buffer[i])
		{
			printk("nand verify buffer error!!  val i: 0x%x\n", i);
			return -EFAULT;
		}
	}
	return 0;
}
#endif

static int asm9260_nand_write_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, const uint8_t *buf,
		int oob_required)
{
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	memcpy(asm9260_nand_dma_write_buf_virt, buf, mtd->writesize);  // copy data to buf
	memcpy(asm9260_nand_dma_write_buf_virt + mtd->writesize, chip->oob_poi, mtd->oobsize);  // copy data to oobbuf                                                
#else
	uint8_t *temp_ptr;
	temp_ptr = (uint8_t *)buf;
	chip->write_buf(mtd, temp_ptr, mtd->writesize);

	temp_ptr = chip->oob_poi;
	chip->write_buf(mtd, temp_ptr, asm9260_nand_spare_data_size);
#endif
	return 0;
}

static int asm9260_nand_read_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, uint8_t *buf,
		int oob_required, int page)
{
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	memcpy(buf, asm9260_nand_dma_read_buf_virt, mtd->writesize);  // copy data to buf
	memcpy(chip->oob_poi, asm9260_nand_dma_read_buf_virt + mtd->writesize, mtd->oobsize);  // copy data to oobbuf                                                
#else
	uint8_t *temp_ptr;
	temp_ptr = buf;
	chip->read_buf(mtd, temp_ptr, mtd->writesize);

	temp_ptr = chip->oob_poi;
	memset(temp_ptr, 0xff, mtd->oobsize);
	chip->read_buf(mtd, temp_ptr, asm9260_nand_spare_data_size);
#endif

	return 0;
}

static int asm9260_nand_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	int status = 0;
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	int ret = 0;
#endif

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	chip->dev_ready(mtd);

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	ret = !asm9260_nand_dma_ready();
	if (ret != 0)
		printk("ASM9260 dma not ready\n");
	
	ret = asm9260_nand_dma_error();
	if (ret != 0)
		printk("ASM9260 dma error\n");
#endif
	chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	status = (int)chip->read_byte(mtd);

	return status;
}


static void asm9260_nand_init_chip(struct nand_chip *nand_chip)
{
	nand_chip->select_chip = asm9260_select_chip;
	nand_chip->cmd_ctrl    = asm9260_cmd_ctrl;
	nand_chip->IO_ADDR_R   = (void  __iomem	*)(&nand_regs->nand_fifo_data);
	nand_chip->IO_ADDR_W   = (void  __iomem	*)(&nand_regs->nand_fifo_data);
	nand_chip->cmdfunc     = asm9260_nand_command_lp;
	nand_chip->read_byte   = asm9260_nand_read_byte;
	nand_chip->read_word   = asm9260_nand_read_word;
	nand_chip->read_buf    = asm9260_nand_read_buf;
	nand_chip->write_buf   = asm9260_nand_write_buf;
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	nand_chip->verify_buf  = asm9260_nand_verify_buf;
#endif
	nand_chip->dev_ready   = asm9260_nand_dev_ready;
	nand_chip->chip_delay  = 100;
	nand_chip->waitfunc    = asm9260_nand_wait;	

#ifdef CONFIG_MTD_NAND_ASM9260_HWECC
	nand_chip->ecc.mode = NAND_ECC_HW;
#else 
	nand_chip->ecc.mode = NAND_ECC_NONE;
#endif
	
	nand_chip->ecc.write_page = asm9260_nand_write_page_hwecc;
	nand_chip->ecc.read_page  = asm9260_nand_read_page_hwecc;
	nand_chip->ecc.calculate = NULL;
	nand_chip->ecc.correct   = NULL;
	nand_chip->ecc.hwctl     = NULL;
	
}

int asm9260_ecc_cap_select(int nand_page_size, int nand_oob_size)
{
	int ecc_bytes = 0;
	int i;

	for (i=(ARRAY_SIZE(ecc_info_table) - 1); i>=0; i--)
	{
		if ((nand_oob_size - ecc_info_table[i].ecc_bytes_per_sector * (nand_page_size >> 9)) > (28 + 2))
		{
			asm9260_nand_ecc_correction_ability = ecc_info_table[i].ecc_cap;
			asm9260_nand_acceptable_err_level = ecc_info_table[i].ecc_threshold;
			ecc_bytes = ecc_info_table[i].ecc_bytes_per_sector * (nand_page_size >> 9);
			break;
		}
	}

	return ecc_bytes;
}

static int asm9260_nand_probe(struct platform_device *dev)
{
	struct platform_device *pdev = dev;
	//struct resource *mem_res, *irq_res;
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;
#ifdef CONFIG_MTD_PARTITIONS
	struct asm9260_nand_data *asm9260_default_mtd_part;
#endif
	int res = 0;

#if 0
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		printk(KERN_ERR "asm9260_nand: can't get I/O resource mem\n");
		return -ENXIO;
	}

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_res) {
		printk(KERN_ERR "asm9260_nand: can't get irq resource\n");
		return -ENXIO;;
	}
	
	nand_regs = ioremap(mem_res->start, mem_res->end - mem_res->start + 1);
	if (nand_regs == NULL) {
		printk(KERN_ERR "asm9260_nand: ioremap failed\n");
		return -EIO;
	}
#endif	

	//asm9260_default_mtd_part = pdev->dev.platform_data;
	nand_regs = (struct asm9260_nand_regs *)(NAND_BASE_ADDR);

	asm9260_nand = kzalloc(sizeof(struct nand_chip), GFP_KERNEL);
	if (!asm9260_nand) {
		printk(KERN_ERR "asm9260_nand: failed to allocate nand_chip storage\n");
		return -ENOMEM;
	}

	asm9260_mtd = kzalloc(sizeof(struct mtd_info), GFP_KERNEL);
	if (!asm9260_mtd) {
		printk(KERN_ERR "asm9260_nand: failed to allocate mtd_info storage\n");
		goto err_mtd_info_alloc;
	}

	/* initialise the hardware */
	res = asm9260_nand_inithw(0);
	if (res != 0)
	{
		printk("init failed,res = 0x%x\n",res);
		return -EIO;
	}

	asm9260_nand_init_chip(asm9260_nand);

	asm9260_mtd->priv = asm9260_nand;
	asm9260_mtd->owner = THIS_MODULE;

	/* first scan to find the device and get the page size */
	if (nand_scan_ident(asm9260_mtd, 1, NULL)) {
		res = -ENXIO;
		goto err_scan_ident;
	}

	if (asm9260_nand->ecc.mode == NAND_ECC_HW) {
		/* ECC is calculated for the whole page (1 step) */
		asm9260_nand->ecc.size = asm9260_mtd->writesize;

		/* set ECC page size and oob layout */
		switch (asm9260_mtd->writesize) {
			case 2048:
				asm9260_nand->ecc.bytes  = asm9260_ecc_cap_select(2048, asm9260_mtd->oobsize);
				asm9260_nand->ecc.layout = &asm9260_nand_oob_64;	
				asm9260_nand->ecc.strength = 4;
				break;
				
			case 4096:
				asm9260_nand->ecc.bytes  = asm9260_ecc_cap_select(4096, asm9260_mtd->oobsize);
				
				if (asm9260_mtd->oobsize == 128) { 
					asm9260_nand->ecc.layout = &asm9260_nand_oob_128;
					asm9260_nand->ecc.strength = 6;
				} else if (asm9260_mtd->oobsize == 218) {
					asm9260_nand->ecc.layout = &asm9260_nand_oob_218;
					asm9260_nand->ecc.strength = 14;
				} else if (asm9260_mtd->oobsize == 224) {
					asm9260_nand->ecc.layout = &asm9260_nand_oob_224;
					asm9260_nand->ecc.strength = 14;
				} else
					printk("!!! NAND Warning !!  Unsupported Oob size [%d] !!!!\n",asm9260_mtd->oobsize);

				break;
				
			case 8192:
				asm9260_nand->ecc.bytes = asm9260_ecc_cap_select(8192, asm9260_mtd->oobsize);

				if (asm9260_mtd->oobsize == 256) {
					asm9260_nand->ecc.layout = &asm9260_nand_oob_256;
					asm9260_nand->ecc.strength = 8;
				} else if (asm9260_mtd->oobsize == 436) {
					asm9260_nand->ecc.layout = &asm9260_nand_oob_436;
					asm9260_nand->ecc.strength = 14;
				} else if (asm9260_mtd->oobsize == 448) {
					asm9260_nand->ecc.layout = &asm9260_nand_oob_448;
					asm9260_nand->ecc.strength = 16;
				} else
					printk("!!! NAND Warning !!  Unsupported Oob size [%d] !!!!\n",asm9260_mtd->oobsize);
				
				break;

			default:
				printk("!!! NAND Warning !!  Unsupported Page size [%d] !!!!\n",asm9260_mtd->writesize);
				break;
		}
	}
	else
		printk("CONFIG_MTD_NAND_ASM9260_HWECC must be chosen in menuconfig!");
	
	asm9260_nand_spare_data_size = asm9260_mtd->oobsize - asm9260_nand->ecc.bytes;
	DBG("asm9260_nand_spare_data_size : 0x%x. \n", asm9260_nand_spare_data_size);

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	asm9260_nand_verify_buffer = kzalloc(asm9260_mtd->writesize + asm9260_mtd->oobsize, GFP_KERNEL);
	if (!asm9260_nand_verify_buffer) {
		printk(KERN_ERR "asm9260_nand_verify_buffer: failed to allocate mtd_info storage\n");
		goto err_verify_buffer_alloc;
	}
#endif

#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	asm9260_nand_dma_buf_virt = dma_alloc_coherent(NULL, (asm9260_mtd->writesize + asm9260_mtd->oobsize) * 2, 
													&asm9260_nand_dma_buf_phy, GFP_KERNEL);
	if (!asm9260_nand_dma_buf_virt)
	{
		printk(KERN_ERR "asm9260_nand: failed to allocate dma buffer storage\n");
		goto err_dma_buffer_alloc;
	}
	
	asm9260_nand_dma_read_buf_virt = asm9260_nand_dma_buf_virt;
	asm9260_nand_dma_read_buf_phy  = asm9260_nand_dma_buf_phy;
	asm9260_nand_dma_write_buf_virt = asm9260_nand_dma_buf_virt + (asm9260_mtd->writesize + asm9260_mtd->oobsize);
	asm9260_nand_dma_write_buf_phy  = asm9260_nand_dma_buf_phy + (asm9260_mtd->writesize + asm9260_mtd->oobsize);
#endif

	/* second phase scan */
	if (nand_scan_tail(asm9260_mtd)) {
		res = -ENXIO;
		goto err_scan_tail;
	}

	DBG("mtd->writesize 0x%x\n", asm9260_mtd->writesize);
	DBG("mtd->erasesize 0x%x\n", asm9260_mtd->erasesize);
	DBG("mtd->oobsize 0x%x\n", asm9260_mtd->oobsize);
	DBG("chip->chipsize 0x%x\n", (uint32_t)asm9260_nand->chipsize);
	DBG("chip->page_shift 0x%x\n", asm9260_nand->page_shift);
	DBG("chip->phys_erase_shift 0x%x\n", asm9260_nand->phys_erase_shift);
	DBG("chip->pagemask 0x%x\n", asm9260_nand->pagemask);
	DBG("chip->badblockpos 0x%x\n", asm9260_nand->badblockpos);
	DBG("chip->bbt_erase_shift 0x%x\n", asm9260_nand->bbt_erase_shift);
	DBG("chip->buffers 0x%x\n", (uint32_t)asm9260_nand->buffers);
	DBG("chip->chip_shift 0x%x\n", asm9260_nand->chip_shift);
	DBG("chip->options 0x%x\n", asm9260_nand->options);

	printk("%s:%i", __func__, __LINE__);
	asm9260_mtd->name = "NAND";
#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
	num_partitions = parse_mtd_partitions(asm9260_mtd, part_probes, &partitions, 0);
#endif
	if (num_partitions <= 0)
	{
		num_partitions = asm9260_default_mtd_part->mtd_part_num;
		partitions = asm9260_default_mtd_part->asm9260_mtd_part;
	}

	res = add_mtd_partitions(asm9260_mtd, partitions, num_partitions);
#else
	res = mtd_device_register(asm9260_mtd, partitions, num_partitions);
#endif

	return res;


err_scan_tail:
#ifdef CONFIG_MTD_NAND_ASAP9260_DMA
	dma_free_coherent(NULL, (asm9260_mtd->writesize + asm9260_mtd->oobsize) * 2, asm9260_nand_dma_buf_virt, asm9260_nand_dma_buf_phy);
err_dma_buffer_alloc:
#endif
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	kfree(asm9260_nand_verify_buffer);
err_verify_buffer_alloc:
#endif
err_scan_ident:
	kfree(asm9260_mtd);
err_mtd_info_alloc:
	kfree(asm9260_nand);
	
	return res;
}


static int asm9260_nand_remove(struct platform_device *dev)
{
	nand_release(asm9260_mtd);
	kfree(asm9260_nand);
	kfree(asm9260_mtd);

	return 0;
}


static int asm9260_nand_suspend(struct platform_device *dev , pm_message_t state)
{
	return 0;
}

static int asm9260_nand_resume(struct platform_device *dev )
{
	return 0;
}

static struct platform_driver asm9260_nand_driver = {
	.probe		= asm9260_nand_probe,
	.remove		= asm9260_nand_remove,
	.suspend	= asm9260_nand_suspend,
	.resume		= asm9260_nand_resume,
	.driver		= {
		.name	= "asm9260-nand",
		.owner	= THIS_MODULE,
	},
};


static int __init asm9260_nand_init(void)
{
	printk("ASM9260 NAND Driver, (c) 2012 Alpscale.\n");
	
	return platform_driver_register(&asm9260_nand_driver);
}

static void __exit asm9260_nand_exit(void)
{
	platform_driver_unregister(&asm9260_nand_driver);
}

module_init(asm9260_nand_init);
module_exit(asm9260_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chen Dongdong <chendd@alpscale.cn>");
MODULE_DESCRIPTION("ASM9260 MTD NAND driver");


