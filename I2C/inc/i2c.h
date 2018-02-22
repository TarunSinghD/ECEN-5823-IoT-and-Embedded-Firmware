/*
 * i2c.h
 *
 *  Created on: Oct 7, 2017
 *      Author: tarun
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_
#include "em_i2c.h"
#include "main.h"

#define I2C_PORT 	gpioPortC

#define SDA_PORT				I2C_PORT
#define SCL_PORT				I2C_PORT
#define SDA_PIN					11
#define SCL_PIN					10
#define SI7021_PORT				gpioPortD
#define SI7021_PIN				9
#define SI7021_ADDR				0x40
#define I2C_READ				0x01
#define I2C_WRITE				0x00
#define CMD_NO_HOLD_MASTER_MODE	0xe3
#define CMD_HOLD_MASTER_MODE	0xf3
void i2c_init(void);
void i2c_enable(void);
float read_temperature();
void load_power_enable(void);
void load_power_disable(void);
void delay_80ms(void);



#endif /* INC_I2C_H_ */
