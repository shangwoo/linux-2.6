/*

PWM HEADER file
 
Change log: 

------------------- Version 1.0  ----------------------
zhangbo 2007-10-29
    -- Create file

*/

#ifndef _PWM_H_
#define _PWM_H_

#define INACTIVE_STATE_HIGE 3
#define INACTIVE_STATE_LOW  2
#define ACTIVE_STATE_HIGE  3
#define ACTIVE_STATE_LOW   2

#define PWM_BACKLIGHT   3
#define BL_UP           1
#define BL_DOWN         0

void percent_demo(int a,int b);
int change_bl_level(int ud);
int get_bl_level(void);
char calcu_freq(int p,int h);
int active_pwm(int num,int period,int high);
/************period & high should be the number of ns  **************/
void as3310_pwm_init(int);



#endif // _PWM_H_
 
