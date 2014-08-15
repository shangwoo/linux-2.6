/* linux/include/asm/fb.h
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 *
 * Inspired by pxafb.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 *  Changelog:
 *	07-Sep-2004	RTP	Created file
 *	03-Nov-2004	BJD	Updated and minor cleanups
 *	03-Aug-2005     RTP     Renamed to fb.h
*/

#ifndef __ASM_ARM_FB_H
#define __ASM_ARM_FB_H

struct asm9260fb_val {
	__u32	defval;
	__u32	min;
	__u32	max;
};

struct asm9260fb_hw {
	unsigned long	lcdcon1;
	unsigned long	lcdcon2;
	unsigned long	lcdcon3;
	unsigned long	lcdcon4;
};

struct asm9260fb_mach_info {
	unsigned char	fixed_syncs;	/* do not update sync/border */

	/* Screen size */
	int		width;
	int		height;

	/* Screen info */
	struct asm9260fb_val xres;
	struct asm9260fb_val yres;
	struct asm9260fb_val bpp;

	/* lcd configuration registers */
	struct asm9260fb_hw  regs;

	/* GPIOs */

	unsigned long	gpcup;
	unsigned long	gpcup_mask;
	unsigned long	gpccon;
	unsigned long	gpccon_mask;
	unsigned long	gpdup;
	unsigned long	gpdup_mask;
	unsigned long	gpdcon;
	unsigned long	gpdcon_mask;

	/* lpc3600 control register */
	unsigned long	lpcsel;
};

void __init set_asm9260fb_info(struct asm9260fb_mach_info *hard_asm9260fb_info);

#endif /* __ASM_ARM_FB_H */
