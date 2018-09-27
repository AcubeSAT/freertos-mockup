#ifndef _MPU9250_LIB_H_
#define _MPU9250_LIB_H_

#include "TWI.h"
#include "delay.h"
#include "MPU9250_Definitions.h"

//Scaling constants for the accelerometer and gyroscope
#define AFS_2G  0x00
#define AFS_4G  0x08
#define AFS_8G  0x10
#define AFS_16G 0x18

#define GFS_250DPS  0x00
#define GFS_500DPS  0x08
#define GFS_1000DPS 0x10
#define GFS_2000DPS 0x18

#define AD0 1 //Select between two address possible

#if AD0
#define MPU9250_ADDR 0x68
#else
#define MPU9250_ADDR 0x69
#endif

#define AK8963_ADDR  0x0C
#define AK8963_14BIT 0x00
#define AK8963_16BIT 0x10

#define AK8963_POWERDOWN 0x00
#define AK8963_SINGLEMEAS 0x01
#define AK8963_CONT8HZ 0x02
#define AK8963_CONT100HZ 0x06
#define AK8963_EXTTRIG 0x04
#define AK8963_SELFTEST 0x08
#define AK8963_FUSEROM 0x0F

/*******************************************************************************************************
 * Make the necessary initializations for the device and the I2C interface for the MCU.
 *******************************************************************************************************/
extern void MPU9250Init(uint8_t afs, uint8_t gfs);

/*******************************************************************************************************
 * It is a good practice to call this function after the initialization, before any data reading happens
 * to ensure calibrated data is read. If the device is installed in a permanent location, then the
 * calibration parameters only need to be calculated and saved to the appropriate register once.
 *******************************************************************************************************/
extern void MPU9250Calibration(float *gyroCalib);

/*******************************************************************************************************
 * Check the condition of the module, according to the factory set values.
 *******************************************************************************************************/
extern void MPU9250SelfTest(void);

/*******************************************************************************************************
 * Read the accelerometer data from the register and save them in the array provided as a pointer
 * The first element is the value for the x-axis, the second element is the y-axis value and the third
 * element of the array is the value of acceleration for the z-axis.
 *******************************************************************************************************/
extern void MPU9250ReadAccelDataRaw(int16_t *acceleration);

/*******************************************************************************************************
 * Read the gysroscope data from the register and save them in the array provided as a pointer
 * The first element is the value for the x-axis, the second element is the y-axis value and the third
 * element of the array is the value of angular velocity for the z-axis.
 *******************************************************************************************************/
extern void MPU9250ReadGyroDataRaw(int16_t *angular);

/*******************************************************************************************************
 * Read the temperature from the integrated thermometer and return it.
 *******************************************************************************************************/
extern int16_t MPU9250ReadTempDataRaw(void);

/*******************************************************************************************************
 * Get the correctly scaled X,Y,Z accelerometer data. Be carefull as the return is a pointer to double.
 *******************************************************************************************************/
extern void MPU9250GetAcceleration(double *acc);

/*******************************************************************************************************
 * Get the correctly scaled X,Y,Z gyroscope data. Be carefull as the return is a pointer to double.
 *******************************************************************************************************/
extern void MPU9250GetAngularVel(double *angVel);
extern void MPU9250_GetCalibAccelGyro(double* Accel, double* Gyro, float* gyrCal);
extern void MPU9250_SetFullScaleGyroRange(uint8_t range);

extern void AK8963Init(uint8_t resolution, uint8_t mode, float *adjVals);
extern void AK8963GetMagnRaw(int16_t *magnField);
extern uint8_t AK8963GetID(void);
extern void AK8963GetMagnuT(float *mField, float *adjVals);


#endif //_MPU9250_LIB_H_
