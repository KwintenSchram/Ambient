# STM32LIB
Libraries for STM32 to control following devices:
* High-Precision Pressure Sensor (MPL3115A2)
* Low-power 3D Magnetometer (MAG3110)
* 3-Axis, Digital Accelerometer (MMA8491Q)
* CO2 and TVOC Air Quality Sensor (CCS811)

Ported from following sources:
https://www.sparkfun.com/products/14181
https://github.com/larsch/rpi-mems-sensor
https://github.com/sparkfun/SparkFun_MAG3110_Breakout_Board_Arduino_Library
https://github.com/arduino-org/arduino-library-lucky-shield/blob/master/src/lib/MMA8491Q.h

Notes:
- Accelerometer hasn't been tested. A pin will need to be enabled and disabled before reading can be succcesfull.
- Magnetometer calibration hasn't been tested.
- Pressure Sensor only includes code to read out the pressure in pascal. It includes two configuration modes:
* Reading the sensor regardless of whether the sensor has new data
* Receiving an interrupt signal when new data is available
