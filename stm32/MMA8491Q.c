/*
 * MMA8491Q.c
 *
 *  Created on: 19 Mar 2017
 *      Author: Fifth
 */


#include "MMA8491Q.h"
void setI2CInterface_MMA8491Q(I2C_HandleTypeDef *hi2c)
{
	hi2cLib=hi2c;
}
uint8_t WRITE_REGISTER_MMA8491Q(uint8_t pData[],uint8_t length)
{
	uint8_t status=HAL_I2C_Master_Transmit(hi2cLib, MMA8491Q_I2C_ADDRESS<<1, pData,length, HAL_MAX_DELAY);
	return status;
}
uint8_t READ_REGISTER_MMA8491Q(uint8_t buf[],uint8_t reg,uint8_t length)
{
	uint8_t status = HAL_I2C_Mem_Read(hi2cLib, MMA8491Q_I2C_ADDRESS<<1, reg, I2C_MEMADD_SIZE_8BIT, buf, length, HAL_MAX_DELAY);
	return status;
}
void read_MMA8491Q()
{
	readRegister_MMA8491Q(STATE);
}

void enable_MMA8491Q()
{
	//setPinLow/high
}

void disable_MMA8491Q()
{
	//setPinLowHigh
}

float value_MMA8491Q(uint16_t val)
{
	if (val < (uint16_t)0x2000)
		return val/1024;   // read value is between 0 and 8191 (0g 8g)
	else
		return ((0x1FFF & val) - 0x2000)/1024;
}

void readRegister_MMA8491Q(uint8_t reg)
{

	enable_MMA8491Q();
	HAL_Delay(5);
	uint8_t buf[7];
	READ_REGISTER_MMA8491Q(buf,reg,6);

	cx = buf[0];
	cx <<= 8;
	cx |= buf[1];
	cx >>= 2;
	cy = buf[2];
	cy <<= 8;
	cy |= buf[3];
	cy >>= 2;
	cz = buf[4];
	cz <<= 8;
	cz |= buf[5];
	cz >>= 2;
	x_ = value_MMA8491Q(cx);
	y_ = value_MMA8491Q(cy);
	z_ = value_MMA8491Q(cz);
	disable_MMA8491Q();
}
