#include "Tasks/UARTTask.h"
#include "uart.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * A task that periodically prints a heartbeat message via UART
 */

// Preliminary version of flash reading function
uint32_t ulFlashRead(uint32_t ulAddress)
{
	return *(uint32_t *)ulAddress;
}

void vCheckTask(void *pvParameters) {
	uint8_t value = (uint8_t) pvParameters;
	uint8_t *payl_addr = (uint8_t *)0x20002c7f;

	for (;;) {
		osQueueUARTMessage("%d SystemGood %d \r\n", value, xTaskGetTickCount());
		osQueueUARTMessage("Var payload_length value: %d\r\n", *payl_addr);
		osQueueUARTMessage("Value read using function: %ld\r\n", ulFlashRead((uint32_t)0x20002c7f));
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
		Manual_Delay(a);
	}
}
