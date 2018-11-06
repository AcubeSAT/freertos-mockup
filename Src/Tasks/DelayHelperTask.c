#include "Tasks/DelayHelperTask.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_tim.h"

static TIM_HandleTypeDef tim; // Timer handle

/**
 * Set up HAL_Delay() for pre-FreeRTOS initialisation
 */
void vSetupDelayHelper() {
	// Set interrupt priority and enable TIMER2 interrupt in NVIC
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// Clock the TIMER2 peripheral
	__HAL_RCC_TIM2_CLK_ENABLE();

	tim.Instance = TIM2;
	tim.Init.Period = 72;
	tim.Init.Prescaler = 1000;
	tim.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&tim);
	HAL_TIM_Base_Start_IT(&tim);
}

void vDisableDelayHelper() {
	HAL_TIM_Base_Stop_IT(&tim);
	HAL_TIM_Base_DeInit(&tim);

	__HAL_RCC_TIM2_CLK_DISABLE();
}

void TIM2_IRQHandler(void) {
	/* Clear the pending bit in NVIC and TIMER2 */
	HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
	LL_TIM_ClearFlag_UPDATE(TIM2);

	/* Increment the counter used to measure execution time */
	HAL_IncTick();
}
