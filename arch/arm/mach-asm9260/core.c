/***********************************************
 *  linux/arch/arm/mach-asm9260/core.c
 *  Copyright (C) 2011-2014 Alpscale
 *
 */

#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

/* we map the io address to virtual address here*/
static struct map_desc asm9260_io_desc[] __initdata = {
	{	/* IO space	*/
		.virtual	= (unsigned long)0xf0000000,
		.pfn		= __phys_to_pfn(0x80000000),
		.length		= 0x00800000,
		.type		= MT_DEVICE
	},
	{	/*LCD IO space	*/
		.virtual	= (unsigned long)0xf0a00000,
		.pfn		= __phys_to_pfn(0x80800000),
		.length		= 0x00009000,
		.type		= MT_DEVICE
	},
	{	/*GPIO IO space	*/
		.virtual	= (unsigned long)0xf0800000,
		.pfn		= __phys_to_pfn(0x50000000),
		.length		= 0x00100000,
		.type		= MT_DEVICE
	},
	{	/* SRAM space Cacheable  */
		.virtual	= (unsigned long)0xd0000000,
		.pfn		= __phys_to_pfn(0x40000000),
		.length		= 0x00100000,
#ifdef CONFIG_SRAM_MEM_CACHED
		.type		= MT_MEMORY
#else
		.type		= MT_DEVICE
#endif
	},
};


static void __init asm9260_map_io(void)
{
	/* we remap our io to high address 0xe0000000 */
	iotable_init(asm9260_io_desc, ARRAY_SIZE(asm9260_io_desc));
}

static void __init asm9260_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const char * const asm9260_dt_board_compat[] __initconst = {
	"alpscale,asm9260",
	NULL
};

DT_MACHINE_START(ASM9260, "Alpscale ASM9260 (Device Tree Support)")
	.map_io		= asm9260_map_io,    /*MAP_IO*/
	.init_machine	= asm9260_init,
	.dt_compat	= asm9260_dt_board_compat,
MACHINE_END
