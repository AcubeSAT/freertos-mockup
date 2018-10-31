/*
 * ADC.h - Headers and helper elements
 * for the initialization routines in ADC.c
 *
 *      Created on: 31 Oct 2018
 *      Author: Grigoris Pavlakis <grigpavl@ece.auth.gr>
 */

#ifndef INC_PERIPHERALS_ADC_H_
#define INC_PERIPHERALS_ADC_H_

enum ADC_UsageMode {TEMPMSR, BATLVL}; //the mode for which the ADC should be initialized

void ADC_Init(ADC_UsageMode mode)

#endif /* INC_PERIPHERALS_ADC_H_ */
