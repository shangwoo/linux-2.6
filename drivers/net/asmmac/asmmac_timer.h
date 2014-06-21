/*******************************************************************************
  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: AlphaScale
*******************************************************************************/
#ifndef __ASMMAC_TIMER_H__
#define __ASMMAC_TIMER_H__

struct asmmac_timer {
	void (*timer_start) (unsigned int new_freq);
	void (*timer_stop) (void);
	unsigned int freq;
	unsigned int enable;
};

/* Open the HW timer device and return 0 in case of success */
int asmmac_open_ext_timer(struct net_device *dev, struct asmmac_timer *tm);
/* Stop the timer and release it */
int asmmac_close_ext_timer(void);
/* Function used for scheduling task within the asmmac */
void asmmac_schedule(struct net_device *dev);

#if defined(CONFIG_ASMMAC_TMU_TIMER)
extern int tmu2_register_user(void *fnt, void *data);
extern void tmu2_unregister_user(void);
#endif

#endif /* __ASMMAC_TIMER_H__ */
