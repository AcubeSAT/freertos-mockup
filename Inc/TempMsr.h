/*
 * TempMsr.h - temp. sensor configuration and function declarations
 *
 *  Created on: 17 Οκτ 2018
 *  Author: Grigoris Pavlakis
 */

#ifndef INC_TEMPMSR_H_
#define INC_TEMPMSR_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_bus.h"

#define AVGSLOPE 4.3
//average slope of T-V chart according to datasheet pg 79
//(min is 4 mV/C, max 4.6, default (4.3): typical)

#define V25 1430
//voltage of temperature sensor at 25C according to datasheet pg 79 (in mV)
//(min is 1340, max is 1520, default(1430): typical)

#define BIAS 20
//according to the manual (pg 235), due to mfg processes
//there is an offset in the V(T) plot different
//to every chip (up to +-45oC) that needs to be found.
//(default is 0 so it MUST be calculated before any meaningful measurements
//are made)

int32_t getTemp();  //returns current MCU temp. in Celsius
void ADC_TempMsr_Init();  //initializes the ADC, this needs to be run in prvHardwareSetup()
void ADC_Config();

#endif /* INC_TEMPMSR_H_ */
 
