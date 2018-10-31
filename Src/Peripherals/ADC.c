/*
 * ADC.c - Low-level multi-mode ADC initialization routine
 *
 *      Created on: 31 Oct 2018
 *      Author: Grigoris Pavlakis <grigpavl@ece.auth.gr>
 */

void ADC_TempMsr_Init()
{
	ADC_Config();  //setup the basic stuff (clocks, channels etc)

	LL_ADC_Enable(ADC1);   //enable the ADC

	LL_ADC_StartCalibration(ADC1);   //start automatic calibration of the ADC

}

void ADC_Config()
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);  //enable the clock for ADC1

    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
    uint32_t wait_loop_index = 80;  //CPU cycles which correspond to ~10 us, suggested stabilization time for temp.sensor stabilization

    while(wait_loop_index != 0)
    {
    	wait_loop_index--;
    }

    /* Set ADC group regular trigger source */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

    /* Set ADC group regular continuous mode */
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

    /* Set ADC group regular sequencer length and scan direction */
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);

    /* Set ADC group regular sequence: channel on the selected sequence rank. */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);

    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_16, LL_ADC_SAMPLINGTIME_239CYCLES_5);
}


