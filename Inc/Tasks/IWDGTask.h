/*
 * IWDGTask.h
 *
 *  Created on: Nov 18, 2018
 *      Author: athanasios
 */

#ifndef INC_TASKS_IWDGTASK_H_
#define INC_TASKS_IWDGTASK_H_
#include "stm32f1xx_hal_iwdg.h"
#define TIMx_CLK_ENABLE()                 __HAL_RCC_TIM4_CLK_ENABLE()
#define TIMx_CLK_DISABLE()                __HAL_RCC_TIM4_CLK_DISABLE()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                         TIM4_IRQn

void vRefreshIWDGTask(void * pvParameters);
void vSetupIWDG();

#endif /* INC_TASKS_IWDGTASK_H_ */
