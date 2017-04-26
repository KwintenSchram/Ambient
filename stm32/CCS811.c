/*
 * CCS811.c
 *
 *  Created on: 17 Mar 2017
 *      Author: Fifth
 */

#include "CCS811.h"
void setI2CInterface_CCS811(I2C_HandleTypeDef *hi2c)
{
	hi2cLib=hi2c;
}
uint8_t WRITE_REGISTER_CCS811(uint8_t pData[],uint8_t length)
{
	uint8_t status=HAL_I2C_Master_Transmit(hi2cLib, CCS811_ADDR<<1, pData,length, HAL_MAX_DELAY);
	return status;
}
uint8_t READ_REGISTER_CCS811(uint8_t buf[],uint8_t reg,uint8_t length)
{
	uint8_t status = HAL_I2C_Mem_Read(hi2cLib, CCS811_ADDR<<1, reg, I2C_MEMADD_SIZE_8BIT, buf, length, HAL_MAX_DELAY);
	return status;
}

uint8_t getData_CCS811(uint32_t *data)
{
	uint8_t buf[4];
	uint8_t status =READ_REGISTER_CCS811(buf,CSS811_ALG_RESULT_DATA,4);
	uint32_t requiredData= buf[0]<<24 | buf[1]<<16|buf[2]<<8|buf[3];
	*data=requiredData;
	return status;
}

void parseResult_CCS811(uint32_t *data,uint16_t *CO2,uint16_t *tVOC)
{
	  *CO2 = *data>>16;
	  *tVOC = *data;
}

void setMode_CCS811(uint8_t mode)
{
  if (mode > 4) mode = 4; //Error correction
  uint8_t setting[1];
  READ_REGISTER_CCS811(setting,CSS811_MEAS_MODE,1);
  setting[0] &= ~(0b00000111 << 4); //Clear DRIVE_MODE bits
  setting[0] |= (mode << 4); //Mask in mode
  uint8_t writeData[2]={CSS811_MEAS_MODE, setting[0]};
  WRITE_REGISTER_CCS811(writeData,2);
}

uint8_t configure_CCS811(uint8_t mode)
{
	uint8_t state=0;
	uint8_t hwID[1];
	READ_REGISTER_CCS811(hwID,CSS811_HW_ID,1);

	if (hwID[0] == 0x81)
	{
		uint8_t ready[1];
		READ_REGISTER_CCS811(ready,CSS811_STATUS,1);
		if(ready[0] & 1 << 4)
		{
			uint8_t data[1];
			data[0]=CSS811_APP_START;
			WRITE_REGISTER_CCS811(data,1);
			setMode_CCS811(mode);
			state=1;
		}
		else
		{
			state=0;
			getError_CCS811();
		}
	}
	else
	{
		state=0;
		getError_CCS811();
	}
	return state;
}
uint8_t getError_CCS811()
{
	uint8_t error[1];
	READ_REGISTER_CCS811(error,CSS811_ERROR_ID,1);

	if (error[0] & 1 << 5)
		return HEATER_SUPPLY;
	else if (error[0] & 1 << 4)
		return HEATER_FAULT;
	else if (error[0] & 1 << 3)
		return MAX_RESISTANCE;
	else if (error[0] & 1 << 2)
		return MEASMODE_INVALID;
	else if (error[0] & 1 << 1)
		return READ_REG_INVALID;
	else if (error[0] & 1 << 0)
		return WRITE_REG_INVALID;
	else
		return NO_ERROR;
}

uint8_t dataAvailable_CCS811()
{
	uint8_t value[1];
	READ_REGISTER_CCS811(value,CSS811_STATUS,1);

	if(value[0] & 1 << 3)
		return (1);
	else
		return 0;
}
uint8_t checkForError_CCS811()
{
	uint8_t value[1];
	READ_REGISTER_CCS811(value,CSS811_STATUS,1);
	return (value[0] & 1 << 0);
}
uint16_t getBaseline_CCS811()
{
	uint8_t baselineElements[2];
	uint16_t  baseline;
	READ_REGISTER_CCS811(baselineElements,CSS811_BASELINE,2);
	baseline= baselineElements[0]<<8|baselineElements[1];
	return baseline;
}

uint8_t appValid_CCS811()
{
	uint8_t value[1];
	READ_REGISTER_CCS811(value,CSS811_STATUS,1);
	return (value[0] & 1 << 4);
}

void enableInterrupts_CCS811()
{
	uint8_t setting[1];
	READ_REGISTER_CCS811(setting,CSS811_MEAS_MODE,1);
	setting[0] |= (1 << 3); //Set INTERRUPT bit
	uint8_t writeData[2]={CSS811_MEAS_MODE, setting[0]};
	WRITE_REGISTER_CCS811(writeData,2);
}

void disableInterrupts_CCS811()
{
	uint8_t setting[1];
	READ_REGISTER_CCS811(setting,CSS811_MEAS_MODE,1);
	setting[0] &= ~(1 << 3); //Clear INTERRUPT bit
	uint8_t writeData[2]={CSS811_MEAS_MODE, setting[0]};
	WRITE_REGISTER_CCS811(writeData,2);
}
