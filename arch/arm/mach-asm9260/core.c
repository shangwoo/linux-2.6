/*
 * Copyright (C) 2014 Oleksij Rempel <linux@rempel-privat.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/of_platform.h>
#include <asm/mach/arch.h>

static const char * const asm9260_dt_board_compat[] __initconst = {
	"alphascale,asm9260",
	NULL
};

DT_MACHINE_START(ASM9260, "Alphascale ASM9260 (Device Tree Support)")
	.dt_compat	= asm9260_dt_board_compat,
MACHINE_END
