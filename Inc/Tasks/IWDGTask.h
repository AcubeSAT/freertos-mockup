/*
 * IWDGTask.h
 *
 *  Created on: Nov 18, 2018
 *      Author: athanasios
 */

#ifndef INC_TASKS_IWDGTASK_H_
#define INC_TASKS_IWDGTASK_H_
#include "stm32f1xx_hal_iwdg.h"

void vRefreshIWDGTask(void * pvParameters);
void vSetupIWDG();

#endif /* INC_TASKS_IWDGTASK_H_ */
