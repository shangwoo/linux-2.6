/*
 *  linux/drivers/mmc/i2c.h - 
 *
 *  Copyright (C) 2012 Alpscale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AS9260_I2C_H
#define AS9260_I2C_H

typedef enum {
	I2C_Speed_Standard =0,
	I2C_Speed_Fast		
}i2c_speed_mode;

struct as9260_i2c_hw_data{
    int class;  //the I2C controller(i2c adapter supported probe class)
    i2c_speed_mode speedmode; //0,standard,100kb/s. 1,fast,400kb/s.
};

#endif
