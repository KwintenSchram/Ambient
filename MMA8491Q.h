/*
 * MMA8491Q.h
 *
 *  Created on: 19 Mar 2017
 *      Author: Fifth
 */
#include "stm32l1xx_hal.h"
#ifndef MMA8491Q_H_
#define MMA8491Q_H_
#define	STATE					(uint8_t)0x01
#define	GPIO_ADDRESS			(uint8_t)0x20
#define	GPIO_READ				(uint8_t)0x00
#define	GPIO_WRITE_PORT0		(uint8_t)0x02
#define	 MMA8491Q_I2C_ADDRESS				(uint8_t)0x55

	float x_;
	float y_;
	float z_;
	uint16_t cx;
	uint16_t cy;
	uint16_t cz;
	uint16_t state;


I2C_HandleTypeDef *hi2cLib;
 uint8_t READ_REGISTER_MMA8491Q(uint8_t buf[],uint8_t reg,uint8_t length);
 uint8_t WRITE_REGISTER_MMA8491Q(uint8_t pData[],uint8_t length);
 void setI2CInterface_MMA8491Q(I2C_HandleTypeDef *hi2c);
 void read_MMA8491Q();
 void enable_MMA8491Q();
 void disable_MMA8491Q();
 float value_MMA8491Q(uint16_t val);
 void readRegister_MMA8491Q(uint8_t reg);


#endif /* MMA8491Q_H_ */
