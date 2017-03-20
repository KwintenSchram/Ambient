/*
 * CCS811.h
 *
 *  Created on: 17 Mar 2017
 *      Author: Fifth
 */
#include "stm32l1xx_hal.h"

#ifndef CCS811_H_
#define CCS811_H_


#define CCS811_ADDR 0x5B
#define CSS811_STATUS 0x00
#define CSS811_MEAS_MODE 0x01
#define CSS811_ALG_RESULT_DATA 0x02
#define CSS811_RAW_DATA 0x03
#define CSS811_ENV_DATA 0x05
#define CSS811_NTC 0x06
#define CSS811_THRESHOLDS 0x10
#define CSS811_BASELINE 0x11
#define CSS811_HW_ID 0x20
#define CSS811_HW_VERSION 0x21
#define CSS811_FW_BOOT_VERSION 0x23
#define CSS811_FW_APP_VERSION 0x24
#define CSS811_ERROR_ID 0xE0
#define CSS811_APP_START 0xF4
#define CSS811_SW_RESET 0xFF
#define WRITE_REG_INVALID 0
#define READ_REG_INVALID 1
#define MEASMODE_INVALID 2
#define MAX_RESISTANCE 3
#define HEATER_FAULT 4
#define HEATER_SUPPLY 5
#define NO_ERROR 5

I2C_HandleTypeDef *hi2cLib;

void setI2CInterface_CCS811(I2C_HandleTypeDef *hi2c);
uint8_t WRITE_REGISTER_CCS811(uint8_t pData[],uint8_t length);
uint8_t READ_REGISTER_CCS811(uint8_t buf[],uint8_t reg,uint8_t length);
void parseResult_CCS811(uint32_t *data,uint16_t *CO2,uint16_t *tVOC);
uint8_t getData_CCS811(uint32_t *data);
void setMode_CCS811(uint8_t mode);
uint8_t configure_CCS811(uint8_t mode);
uint8_t getError_CCS811();
uint8_t dataAvailable_CCS811();
uint16_t getBaseline_CCS811();
uint8_t appValid_CCS811();
void enableInterrupts_CCS811();
void disableInterrupts_CCS811();

#endif /* CCS811_H_ */
