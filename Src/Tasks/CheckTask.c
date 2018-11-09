#include "Tasks/UARTTask.h"
#include "uart.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Tasks/CheckTask.h"

TaskHandle_t xCheckHandle1 = NULL;
TaskHandle_t xCheckHandle2 = NULL;
StaticTask_t xCheckTaskBuffer;
StackType_t xCheckTaskStack[ CHECK_TASK_STACK_SIZE ];

/**
 * A task that periodically prints a heartbeat message via UART
 */
void vCheckTask(void *pvParameters) {
	uint8_t value = (uint8_t) pvParameters;

	for (;;) {
		osQueueUARTMessage("%d SystemGood %d \r\n", value, xTaskGetTickCount());
		//taskYIELD();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void vSetupCheck() {
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO;
	GPIO.Pin = GPIO_PIN_13; // Indicator LED pin
	GPIO.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO);

	// Blink C13 LED in a cool way
	for (int a = 0; a < 20; a++) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(a);
	}
}
