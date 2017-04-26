/*
 * MAG3110.c
 *
 *  Created on: 19 Mar 2017
 *      Author: Fifth
 */


#include "MAG3110.h"
void setI2CInterface_MAG3110(I2C_HandleTypeDef *hi2c)
{
	hi2cLib=hi2c;
}
uint8_t WRITE_REGISTER_MAG3110(uint8_t pData[],uint8_t length)
{
	uint8_t status=HAL_I2C_Master_Transmit(hi2cLib, MAG3110_I2C_ADDRESS<<1, pData,length, HAL_MAX_DELAY);
	return status;
}
uint8_t READ_REGISTER_MAG3110(uint8_t buf[],uint8_t reg,uint8_t length)
{
	uint8_t status = HAL_I2C_Mem_Read(hi2cLib, MAG3110_I2C_ADDRESS<<1, reg, I2C_MEMADD_SIZE_8BIT, buf, length, HAL_MAX_DELAY);
	return status;
}

uint8_t configure_MAG3110()
{
	uint8_t state=0;
	uint8_t hwID[1];
	READ_REGISTER_MAG3110(hwID,MAG3110_WHO_AM_I,1);

	if (hwID[0] == MAG3110_WHO_AM_I_RSP)
	{
		reset_MAG3110();
		state=1;
	}
	else
	{
		state=0;
	}
	return state;
}

void reset_MAG3110()
{
	enterStandby_MAG3110();
	uint8_t data1[2]={MAG3110_CTRL_REG1, 0x00};
	uint8_t data2[2]={MAG3110_CTRL_REG2, 0x80};
	WRITE_REGISTER_MAG3110(data1,2);//Set everything to 0
	WRITE_REGISTER_MAG3110(data2,2);//Enable Auto Mag Reset, non-raw mode


	calibrationMode = 0;
	activeMode = 0;
	rawMode = 0;
	calibrated = 0;

	setOffset_MAG3110(MAG3110_X_AXIS, 0);
	setOffset_MAG3110(MAG3110_Y_AXIS, 0);
	setOffset_MAG3110(MAG3110_Z_AXIS, 0);
}

void setOffset_MAG3110(uint8_t axis, uint8_t offset)
{
	offset = offset << 1;

	uint8_t msbAddress = axis + 8;
	uint8_t lsbAddress = msbAddress + 1;
	uint8_t data1[2]={msbAddress, (uint8_t)((offset >> 8) & 0xFF)};
	WRITE_REGISTER_MAG3110(data1,2);

	//delay(15);
	uint8_t data2[2]={lsbAddress, (uint8_t) offset & 0xFF};
	WRITE_REGISTER_MAG3110(data2,2);
}
void enterStandby_MAG3110()
{
	activeMode = 0;
	uint8_t hwID[1];
	READ_REGISTER_MAG3110(hwID,MAG3110_CTRL_REG1,1);
	//Clear bits 0 and 1 to enter low power standby mode
	uint8_t data2[2]={MAG3110_CTRL_REG1, (hwID[0] & ~(0x3))};
	WRITE_REGISTER_MAG3110(data2,2);
}


uint8_t dataReady_MAG3110()
{
uint8_t data[1];
	READ_REGISTER_MAG3110(data,MAG3110_DR_STATUS,1);
	return ((data[0] & 0x8) >> 3);
}


void readMag_MAG3110(uint8_t* x, uint8_t* y, uint8_t* z)
{
	//Read each axis
	*x = readAxis_MAG3110(MAG3110_OUT_X_MSB);
	*y = readAxis_MAG3110(MAG3110_OUT_Y_MSB);
	*z = readAxis_MAG3110(MAG3110_OUT_Z_MSB);
}
uint8_t readAxis_MAG3110(uint8_t axis)
{
	uint8_t lsbAddress, msbAddress;
	uint8_t lsb[1] , msb[1] ;

	msbAddress = axis;
	lsbAddress = axis+1;

	READ_REGISTER_MAG3110(msb,msbAddress,1);
	//delay//needs at least 1.3us free time between start and stop
	READ_REGISTER_MAG3110(lsb,lsbAddress,1);

	uint8_t out = (lsb[0] | (msb[0] << 8)); //concatenate the MSB and LSB
	return out;
}

void readMicroTeslas_MAG3110(float* x, float* y, float* z)
{
	//Read each axis and scale to Teslas
	*x = (float) readAxis_MAG3110(MAG3110_OUT_X_MSB) * 0.1f;
	*y = (float) readAxis_MAG3110(MAG3110_OUT_Y_MSB) * 0.1f;
	*z = (float) readAxis_MAG3110(MAG3110_OUT_Z_MSB) * 0.1f;

}

//Note: Must be calibrated to use readHeading!!!
float readHeading_MAG3110()
{

	uint8_t x, y, z;
	readMag_MAG3110(&x, &y, &z);

	float xf = (float) x * 1.0f;
	float yf = (float) y * 1.0f;

	return atan2(-yf*y_scale, xf*x_scale) * 57.2958;
}


void setDR_OS_MAG3110(uint8_t DROS)
{
	uint8_t wasActive = activeMode;

	if(activeMode)
		enterStandby_MAG3110(); //Must be in standby to modify CTRL_REG1

	//If we attempt to write to CTRL_REG1 right after going into standby
	//It might fail to modify the other bits

	HAL_Delay(100);

	 //Get the current control register
	uint8_t current[1];
	READ_REGISTER_MAG3110(current,MAG3110_CTRL_REG1,1);
	current[0]=current[0] & 0x07;//And chop off the 5 MSB
	uint8_t write[2]={MAG3110_CTRL_REG1, (current[1] | DROS)};
	WRITE_REGISTER_MAG3110(write,2); //Write back the register with new DR_OS set

	//delay(100);
	HAL_Delay(100);

	//Start sampling again if we were before
	if(wasActive)
		exitStandby_MAG3110();
}

void exitStandby_MAG3110()
{
	activeMode = 1;
	uint8_t current[1] ;
	READ_REGISTER_MAG3110(current,MAG3110_CTRL_REG1,1);
	uint8_t data[2]={MAG3110_CTRL_REG1, (current[0] | MAG3110_ACTIVE_MODE)};
	WRITE_REGISTER_MAG3110(data,2);

}

void triggerMeasurement_MAG3110()
{

	uint8_t current[1];
	READ_REGISTER_MAG3110(current,MAG3110_CTRL_REG1,1);
	uint8_t data[2]={MAG3110_CTRL_REG1, (current[0] |  0x02)};
	WRITE_REGISTER_MAG3110(data,2);
}

//Note that AUTO_MRST_EN will always read back as 0
//Therefore we must explicitly set this bit every time we modify CTRL_REG2
void toggleRawData_MAG3110(uint8_t raw)
{
	if(raw) //Turn on raw (non-user corrected) mode
	{
		rawMode = 1;
		uint8_t data[2]={MAG3110_CTRL_REG2, MAG3110_AUTO_MRST_EN | (0x01 << 5)};
	WRITE_REGISTER_MAG3110(data,2);

	}
	else //Turn off raw mode
	{
		rawMode = 0;
			uint8_t data[2]={MAG3110_CTRL_REG2, MAG3110_AUTO_MRST_EN & ~(0x01 << 5)};
	WRITE_REGISTER_MAG3110(data,2);
	}
}

//If you look at the datasheet, the offset registers are kind of strange
//The offset is stored in the most significant 15 bits.
//Bit 0 of the LSB register is always 0 for some reason...
//So we have to left shift the values by 1
//Ask me how confused I was...


//See above
int readOffset_MAG3110(uint8_t axis)
{
	return (readAxis_MAG3110(axis+8)) >> 1;
}

void start_MAG3110()
{
	exitStandby_MAG3110();
}

uint8_t isActive_MAG3110()
{
	return activeMode;
}

uint8_t isRaw_MAG3110()
{
	return rawMode;
}

uint8_t isCalibrated_MAG3110()
{
	return calibrated;
}

uint8_t isCalibrating_MAG3110()
{
	return calibrationMode;
}

uint8_t getSysMode_MAG3110()
{
	uint8_t current[1];
	READ_REGISTER_MAG3110(current,MAG3110_SYSMOD,1);
	return current[0];
}

void enterCalMode_MAG3110()
{
	calibrationMode = 1;
	//Starting values for calibration
	x_min = 32767;
	x_max = 0x8000;

	y_min = 32767;
	y_max = 0x8000;

	//Read raw readings for calibration
	toggleRawData_MAG3110(1);

	calibrated = 0;

	//Set to active mode, highest DROS for continous readings
	setDR_OS_MAG3110(MAG3110_DR_OS_80_16);
	if(!activeMode)
		start_MAG3110();
}

void calibrate_MAG3110()
{
	uint8_t x, y, z;
	readMag_MAG3110(&x, &y, &z);

	uint8_t changed = 0; //Keep track of if a min/max is updated
	if(x < x_min)
    {
      x_min = x;
      changed = 1;
    }
    if(x > x_max)
    {
      x_max = x;
      changed = 1;
    }
    if(y < y_min)
    {
      y_min = y;
      changed = 1;
    }
    if(y > y_max)
    {
      y_max = y;
      changed = 1;
    }
/*
	if(changed)
     timeLastChange = millis(); //Reset timeout counter

    if(millis() > 5000 && millis() - timeLastChange > 5000) //If the timeout has been reached, exit calibration
	 exitCalMode_MAG3110();
	 */
}

void exitCalMode_MAG3110()
{
	//Calculate offsets
	x_offset = (x_min + x_max)/2;

	y_offset = (y_min + y_max)/2;

	x_scale = 1.0/(x_max - x_min);
	y_scale = 1.0/(y_max - y_min);

	setOffset_MAG3110(MAG3110_X_AXIS, x_offset);
	//Set the offsets
	setOffset_MAG3110(MAG3110_Y_AXIS, y_offset);

	//Use the offsets (set to normal mode)
	toggleRawData_MAG3110(0);

	calibrationMode = 0;
	calibrated = 1;

	//Enter standby and wait
	//enterStandby_MAG3110();
}
/*
 */
