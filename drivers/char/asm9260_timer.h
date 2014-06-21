/*******************************************************************************
* Copyright (C) 2014 AlphaScale												   											   
* file:asm9260_timer.h                                                                                                                                                                                                                                   
*******************************************************************************/
#ifndef __ASM9260_TIMER_H__
#define __ASM9260_TIMER_H__

/************************************************************************* 
 * config timer register 
 * chan: timer channel
 * mrval: register HW_TIMER1_MR value
 * emr: register HW_TIMER1_EMR value
 * countval: when work in counter mode, set counter value
 */
struct timer_config
{
        int chan;                         /* timer1 channel num */
	unsigned long mrval[4];           /* mr register val of 4 channels*/
	unsigned long thval[4];           /* th register val of 4 channels*/
	unsigned long emr;                /* emr mode */
	unsigned long countval[4];        /* count val of 4 channels,used in counter mode */
};

/******************************************************************* 
 * timer1 register 
 */
struct timer_reg
{
   unsigned long timer_pr;
   unsigned long timer_mcr;
   unsigned long timer_tcr;
   unsigned long timer_mr[4];
   unsigned long timer_pwmth[4];
   unsigned long timer_ir;
   unsigned long timer_pwmc;
   unsigned long timer_ctcr;
   unsigned long timer_emr;
   unsigned long timer_ccr;
};

/******************************************************************* 
 * timer1 enable 
 */
#define TIMER1_CLOCK_ENABLE          0x00000020

/******************************************************************* 
 * timer1 channel num 
 */
#define TIMER1_CHANNEL0              0x1
#define TIMER1_CHANNEL1              0x2
#define TIMER1_CHANNEL2              0x4
#define TIMER1_CHANNEL3              0x8

/******************************************************************* 
 * timer1 working mode 
 */
#define TIMER1_MODE_PWM              1
#define TIMER1_MODE_SINGLE_PWM       2
#define TIMER1_MODE_EMR              3
#define TIMER1_MODE_MATCH_INTERRUPT  4  
#define TIMER1_MODE_COUNTER          5  
#define TIMER1_MODE_CAPTURE          6  

/******************************************************************* 
 * timer1 interrupt type
 */
#define TIMER1_CHAN0_INT             0x1
#define TIMER1_CHAN1_INT             0x2
#define TIMER1_CHAN2_INT             0x4
#define TIMER1_CHAN3_INT             0x8
#define TIMER1_CAPTURE_INT           0x10
#define TIMER1_INTERRUPT_NONE        0

/******************************************************************* 
 * timer1 interrupt flag  
 */
#define TIMER1_CHAN0_INT_FLAG        0x1
#define TIMER1_CHAN1_INT_FLAG        0x2
#define TIMER1_CHAN2_INT_FLAG        0x4
#define TIMER1_CHAN3_INT_FLAG        0x8
#define TIMER1_CAPTURE_INT_FLAG      0x10

/******************************************************************* 
 * timer1 emr mode 
 */
#define TIMER1_EMR_NONE        0x00000000                 
#define TIMER1_EMR_HIGH        0x00000020     
#define TIMER1_EMR_LOW         0x00000010    
#define TIMER1_EMR_TOGGLE      0x00000030

/******************************************************************* 
 * timer1 emr mode register configuration
 */
#define TIMER1_EMR_CHAN0_NONE        0x00000000                 
#define TIMER1_EMR_CHAN0_HIGH        0x00000020     
#define TIMER1_EMR_CHAN0_LOW         0x00000010    
#define TIMER1_EMR_CHAN0_TOGGLE      0x00000030
#define TIMER1_EMR_CHAN1_NONE        0x00000000     
#define TIMER1_EMR_CHAN1_HIGH        0x00000080     
#define TIMER1_EMR_CHAN1_LOW         0x00000040    
#define TIMER1_EMR_CHAN1_TOGGLE      0x000000c0
#define TIMER1_EMR_CHAN2_NONE        0x00000000     
#define TIMER1_EMR_CHAN2_HIGH        0x00000200     
#define TIMER1_EMR_CHAN2_LOW         0x00000100    
#define TIMER1_EMR_CHAN2_TOGGLE      0x00000300
#define TIMER1_EMR_CHAN3_NONE        0x00000000     
#define TIMER1_EMR_CHAN3_HIGH        0x00000800     
#define TIMER1_EMR_CHAN3_LOW         0x00000400    
#define TIMER1_EMR_CHAN3_TOGGLE      0x00000c00
   

/******************************************************************* 
 * timer1 mode
 */
#define TIMER1_COUNTER_CHAN0_UP      0x00000002
#define TIMER1_COUNTER_CHAN1_UP      0x00000008
#define TIMER1_COUNTER_CHAN2_UP      0x00000020
#define TIMER1_COUNTER_CHAN3_UP      0x00000080
#define TIMER1_CAPTURE_UP            0x00000005

/******************************************************************* 
 * timer1 channel interrupt configuration 
 */
#define TIMER1_CHANNEL0_INT_SET      0x00000001
#define TIMER1_CHANNEL1_INT_SET      0x00000008
#define TIMER1_CHANNEL2_INT_SET      0x00000040
#define TIMER1_CHANNEL3_INT_SET      0x00000200

/******************************************************************* 
 * timer1 interrupt event 
 */
#define TIMER1_EVENT_ON              1
#define TIMER1_EVENT_NONE            0

/******************************************************************* 
 * timer1 channel pwm output 
 */
#define TIMER1_CHANNEL0_PWM          0x00000001
#define TIMER1_CHANNEL1_PWM          0x00000002
#define TIMER1_CHANNEL2_PWM          0x00000004
#define TIMER1_CHANNEL3_PWM          0x00000008
#define TIMER1_CHANNEL0_SINGLE_PWM   0x00000011
#define TIMER1_CHANNEL1_SINGLE_PWM   0x00000022
#define TIMER1_CHANNEL2_SINGLE_PWM   0x00000044
#define TIMER1_CHANNEL3_SINGLE_PWM   0x00000088

/******************************************************************* 
 * timer1 channel reset and start
 */
#define TIMER1_CHANNEL0_RESET        (1<<4)
#define TIMER1_CHANNEL1_RESET        (1<<5)
#define TIMER1_CHANNEL2_RESET        (1<<6)
#define TIMER1_CHANNEL3_RESET        (1<<7)
#define TIMER1_CHANNEL0_START        (1<<0)
#define TIMER1_CHANNEL1_START        (1<<1)
#define TIMER1_CHANNEL2_START        (1<<2)
#define TIMER1_CHANNEL3_START        (1<<3)

#endif  /*__ASM9260_TIMER_H__*/
