/*
 * ADC.c - Low-level multi-mode ADC initialization routine
 *
 *      Created on: 31 Oct 2018
 *      Author: Grigoris Pavlakis <grigpavl@ece.auth.gr>
 */

#include "Peripherals/ADC.h"

void ADC_Init(ADC_TypeDef* ADC, enum ADC_UsageMode mode)
{
	/*Common settings for both ADCs*/

	if (ADC == ADC1) //find which ADC do we want to initialize and enable its clock
	{
	    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	}
	else
	{
	    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
	}

	LL_ADC_REG_SetTriggerSource(ADC, LL_ADC_REG_TRIG_SOFTWARE);   //set the chosen ADC to software triggering mode
    LL_ADC_REG_SetContinuousMode(ADC, LL_ADC_REG_CONV_SINGLE);    //enable continuous conversion
    LL_ADC_REG_SetSequencerLength(ADC, LL_ADC_REG_SEQ_SCAN_DISABLE);  //disable sequencer scanning

    /*Usage-specific initialization settings*/
	switch(mode)
	{
	    case TEMPMSR:
	    	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC), LL_ADC_PATH_INTERNAL_TEMPSENSOR);  //connect to temp. sensor
	    	uint32_t wait_loop_index = 80;  //CPU cycles which correspond to ~10 us, suggested stabilization time for temp.sensor stabilization

	    	while(wait_loop_index != 0)
	    	{
	    	    wait_loop_index--;
	    	}

	        LL_ADC_REG_SetSequencerRanks(ADC, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);  //set sequencer rank to 1
	        LL_ADC_SetChannelSamplingTime(ADC, LL_ADC_CHANNEL_16, LL_ADC_SAMPLINGTIME_239CYCLES_5);   //set sampling time for temp. sensing
	    	break;
		    //if you want to use the ADC for your own purposes make sure to add it to the enum
	    	//ADC_UsageMode in ADC.h and build a case for its initialization code (in low level please, to keep performance high)
	    	//

	    //case <YOUR-CASE-HERE>:
	    //	break;
	 }

	LL_ADC_Enable(ADC);   //enable the ADC
	LL_ADC_StartCalibration(ADC);   //start automatic calibration

}


