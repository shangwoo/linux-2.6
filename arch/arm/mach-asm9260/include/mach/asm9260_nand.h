#ifndef __ASM_ARCH_ASM9260_NAND_H
#define __ASM_ARCH_ASM9260_NAND_H

#include <linux/device.h>
#include <linux/platform_device.h>

struct asm9260_nand_data {
	int mtd_part_num;
	struct mtd_partition *asm9260_mtd_part;
};


#endif

