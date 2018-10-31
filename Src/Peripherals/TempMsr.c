/*
 * TempMsr.c - temperature measure routine
 *
 *  Created on: 17 Οκτ 2018
 *  Author: Grigoris Pavlakis
 */

#include "Peripherals/TempMsr.h"

volatile int32_t sensorData = 0;
volatile int32_t temp = 0;

int32_t getTemp()
{
	if (!LL_ADC_IsCalibrationOnGoing(ADC1))  //is the ADC still calibrating?
	{
		LL_ADC_REG_StartConversionSWStart(ADC1);  //start the damn conversion!
		sensorData = LL_ADC_REG_ReadConversionData12(ADC1);
		temp = (int32_t)( ( (10 * V25 - 8 * sensorData) / (AVGSLOPE * 10) ) + 25 + BIAS);
		//manufacturer's formula for determining the temperature in Celsius, adjusted for unit
		//compliance and for minimizing errors due to possible floating-point operations
		return temp;
	}
	return -300;   //error code - calibration is still ongoing
}
