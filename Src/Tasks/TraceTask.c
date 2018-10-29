#include "Tasks/TraceTask.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_tim.h"

volatile unsigned long ulHighFrequencyTimerTicks;

/**
 * Set up FreeRTOS tracing
 */
void vSetupTrace() {
	// Set interrupt priority and enable TIMER1 interrupt in NVIC
	HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

	// Clock the TIMER1 peripheral
	__HAL_RCC_TIM1_CLK_ENABLE();

	TIM_HandleTypeDef tim; // Timer handle
	tim.Instance = TIM1;
	tim.Init.Period = 1800;
	tim.Init.Prescaler = 1;
	tim.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&tim);
	HAL_TIM_Base_Start_IT(&tim);

	ulHighFrequencyTimerTicks = 0;
}

void TIM1_UP_IRQHandler(void) {
	/* Clear the pending bit in NVIC and TIMER1 */
	HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
	LL_TIM_ClearFlag_UPDATE(TIM1);

	/* Increment the counter used to measure execution time */
	ulHighFrequencyTimerTicks++;
}
