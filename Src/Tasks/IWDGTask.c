#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal_iwdg.h"
#include "Tasks/IWDGTask.h"

IWDG_HandleTypeDef IwdgHandle;

/**
 * A task that periodically refreshes the independent watchdog
 */
void vRefreshIWDGTask(void * pvParameters) {
	TickType_t xLastWakeTime;
	IwdgHandle.Instance = IWDG;
	const TickType_t xFrequency = 80;

	xLastWakeTime = (uint32_t) 0;

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		HAL_IWDG_Refresh(&IwdgHandle);
	}
}

void vSetupIWDG() {
	IwdgHandle.Instance = IWDG;
	IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32;
	IwdgHandle.Init.Reload = 4095;
	HAL_IWDG_Init(&IwdgHandle);
}
