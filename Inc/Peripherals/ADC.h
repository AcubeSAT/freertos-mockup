/*
 * ADC.h - Headers and helper elements
 * for the initialization routines in ADC.c
 *
 *      Created on: 31 Oct 2018
 *      Author: Grigoris Pavlakis <grigpavl@ece.auth.gr>
 */

#ifndef INC_PERIPHERALS_ADC_H_
#define INC_PERIPHERALS_ADC_H_

enum ADC_UsageMode {TEMPMSR};
//the usage mode for which the ADC should be initialized
//(extend it with anything else)

#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_bus.h"

void ADC_Init(ADC_TypeDef* ADC, enum ADC_UsageMode mode);

#endif /* INC_PERIPHERALS_ADC_H_ */
