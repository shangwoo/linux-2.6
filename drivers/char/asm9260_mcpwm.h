/*******************************************************************************
* Copyright (C) 2014 AlphaScale												   											   
* file:asm9260_mcpwm.h                                                                                                                                                                                                                                   
*******************************************************************************/
#ifndef __ASM9260_MCPWM_H__
#define __ASM9260_MCPWM_H__

/******************************************************************* 
 * config mcpwm register 
 * chan: mcpwm channel 
 * lim: register HW_MCPWM_LIM num 
 * mat: register HW_MCPWM_MAT num
 * dt: register HW_MCPWM_DT num
 */
struct mcpwm_config
{
   int chan;                     /* mcpwm channel num */
   unsigned long lim[3];         /* lim register val of 3 channels */
   unsigned long mat[3];         /* mat register val of 3 channels*/
   unsigned long dt[3];          /* dead time of capture channels */   
};

/********************************************************************* 
 * mcpwm register default value
 */
#define MCPWM_CON_DEFAULT_VAL       0x00010101
#define MCPWM_INT_DAFAULT_VAL       0x0000ffff
#define MCPWM_INTF_DAFAULT_VAL      0x0000ffff
#define MCPWM_CNTCON_DAFAULT_VAL    0xffffffff  
#define MCPWM_CAPCON_DAFAULT_VAL    0xffffffff

/******************************************************************* 
 * mcpwm enable 
 */
#define MCPWM_CLOCK_ENABLE          0x20000000

/******************************************************************* 
 * mcpwm channel mode 
 */
#define MCPWM_MODE_TIMER_CHAN0      0x20000000
#define MCPWM_MODE_TIMER_CHAN1      0x40000000
#define MCPWM_MODE_TIMER_CHAN2      0x80000000
#define MCPWM_MODE_COUNTER_CHAN0    0x20000001  
#define MCPWM_MODE_COUNTER_CHAN1    0x40000100  
#define MCPWM_MODE_COUNTER_CHAN2    0x80010000  

/******************************************************************* 
 * mcpwm channel num 
 */
#define MCPWM_CHANNEL0              0x1
#define MCPWM_CHANNEL1              0x2
#define MCPWM_CHANNEL2              0x4

/******************************************************************* 
 * mcpwm running mode 
 */
#define MCPWM_CHAN0_RUN_EDGE        0x00000001
#define MCPWM_CHAN1_RUN_EDGE        0x00000100
#define MCPWM_CHAN2_RUN_EDGE        0x00010000
#define MCPWM_ALL_RUN_EDGE          0x00010101
#define MCPWM_CHAN0_RUN_DTEN_EDGE   (0x00000001|(1<<3))
#define MCPWM_CHAN1_RUN_DTEN_EDGE   (0x00000100|(1<<11))
#define MCPWM_CHAN2_RUN_DTEN_EDGE   (0x00010000|(1<<19))
#define MCPWM_ALL_RUN_DTEN_EDGE     (0x00010101|(1<<3)|(1<<11)|(1<<19))

/******************************************************************* 
 * mcpwm capture mode 
 */
#define MCPWM_CAP0_DOWN             ((1<<1)|0x001c0000)
#define MCPWM_CAP1_DOWN             ((1<<9)|0x001c0000)
#define MCPWM_CAP2_DOWN             ((1<<17)|0x001c0000)
#define MCPWM_CAP0_WITH_DT_DOWN     ((1<<1)|0x00fc0000)
#define MCPWM_CAP1_WITH_DT_DOWN     ((1<<9)|0x00fc0000)
#define MCPWM_CAP2_WITH_DT_DOWN     ((1<<17)|0x00fc0000)

/******************************************************************* 
 * mcpwm capture channel interrupt enable 
 */
#define MCPWM_CAP0_INTEN             0x00000004
#define MCPWM_CAP1_INTEN             0x00000040
#define MCPWM_CAP2_INTEN             0x00000400

/******************************************************************* 
 * mcpwm interrupt type
 */
#define MCPWM_CHAN0_INT             0x1
#define MCPWM_CHAN1_INT             0x2
#define MCPWM_CHAN2_INT             0x4
#define MCPWM_INTERRUPT_NONE        0

/******************************************************************* 
 * mcpwm interrupt event 
 */
#define MCPWM_EVENT_ON              1
#define MCPWM_EVENT_NONE            0

/******************************************************************* 
 * mcpwm capture channel interrupt flag 
 */
#define MCPWM_CAP0_INT_FLAG          0x4
#define MCPWM_CAP1_INT_FLAG          0x40
#define MCPWM_CAP2_INT_FLAG          0x400

/******************************************************************* 
 * mcpwm working mode 
 */
#define MCPWM_MODE_OUTPUT           1
#define MCPWM_MODE_COUNTER          2
#define MCPWM_MODE_CAPTURE          3
#define MCPWM_MODE_CAPTURE_WITH_DT  4

#endif  /*__ASM9260_MCPWM_H__*/
