#include "Tasks/BlinkyTask.h"
#include "Tasks/CheckTask.h"
#include "Tasks/NRF24Task.h"
#include "Tasks/SensorTask.h"
#include "Tasks/TraceTask.h"
#include "Tasks/UARTTask.h"
#include "Tasks/WWDGTask.h"
#include "stm32f1xx.h"
#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_hal.h"

static const int brightnessLevel = 500; //global variable that represents the maximum brightness in which tasks remain suspended
TaskHandle_t xReceiveSuspendTask;
BaseType_t areTasksSuspended = 0;

void vReceiveSuspendTask(void *pvParameters) {
	while (1) {
		if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) {

			vTaskSuspend(xBlinkyTask);
			vTaskSuspend(xCheckTask1);
			vTaskSuspend(xCheckTask2);
#if SAT_Enable_Sensors
			vTaskSuspend(xMPU9250Task);
#endif
			vTaskSuspend(xUARTTask);
			vTaskSuspend(xRefreshWWDGTask);

#if SAT_Enable_NRF24
			vTaskSuspend(xTransmitTask);
			vTaskSuspend(xReceiveTask);
#endif
			areTasksSuspended = 1;

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT);
			LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);
		}
	}
}

void vResumeSuspendedTask(void *pvParameters) {
	while (1) {
		if (xSensorData.brightness > brightnessLevel && areTasksSuspended) {
			NVIC_SystemReset();
			areTasksSuspended = 0;
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

