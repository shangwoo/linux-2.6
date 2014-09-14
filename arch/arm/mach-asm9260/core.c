/*
 * Copyright (C) 2014 Oleksij Rempel <linux@rempel-privat.de>
 *  Co-author: Du Huanpeng <u74147@gmail.com>
 * map_desc based on:
 *  linux/arch/arm/mach-asm9260/core.c
 *  Copyright (C) 2011-2014 Alphascale
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
