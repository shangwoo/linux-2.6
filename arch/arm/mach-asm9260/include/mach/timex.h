
#ifndef __TIMEX_H__
#define __TIMEX_H__

#include <linux/kernel.h>
#include <linux/types.h>

#define CLOCK_TICK_RATE		1000000
void as3310_timer2_init(ushort counter);
void as3310_timer2_release(void);

#endif // __TIMEX_H__

