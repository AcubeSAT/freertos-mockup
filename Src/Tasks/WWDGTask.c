#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Tasks/WWDGTask.h"

WWDG_HandleTypeDef hwwdg;
TaskHandle_t xRefreshWWDGTask;
/**
 * A task that periodically refreshes the window watchdog
 */
void vRefreshWWDGTask(void * pvParameters) {
	TickType_t xLastWakeTime;
	hwwdg.Instance = WWDG;
	const TickType_t xFrequency = 80;

	xLastWakeTime = (uint32_t) 0;

	while (1) {
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		HAL_WWDG_Refresh(&hwwdg);
	}
}

void vSetupWWDG() {
	//PCLK1/4096 = 24MHz/4096 = 5859.375 hz
	//5859.375/WWDG_PRESCALER_8= 732.421875 hz
	// 124 - 63 = 65 65/732.421875 = 88.7ms max time
	// 80 - 63 = 17 17/732.421875 = 22ms min time
	hwwdg.Instance = WWDG;
	hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
	hwwdg.Init.Window = 80;
	hwwdg.Init.Counter = 124;
	hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
	HAL_WWDG_Init(&hwwdg);

	__HAL_RCC_WWDG_CLK_ENABLE();
}
