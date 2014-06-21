/*
 *  asm9260-pcm.h --
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  ALSA PCM interface for the  asm9260 CPU
 */

#ifndef _ASM9260_PCM_H
#define _ASM9260_PCM_H

#define ST_RUNNING		(1<<0)
#define ST_OPENED		(1<<1)

struct asm9260_pcm_dma_params {
	char  *client;	/* stream identifier */
	int channel;				/* Channel ID */
	dma_addr_t dma_addr;
	int dma_size;			/* Size of the DMA transfer */
};

/* platform data */
extern struct snd_soc_platform asm9260_soc_platform;

#endif
