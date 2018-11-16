#include "Tasks/UARTTask.h"
#include "uart.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * A task that periodically prints a heartbeat message via UART
 */

// Preliminary version of flash reading function
uint16_t ulFlashRead(uint32_t ulAddress) {
	return *(uint16_t *)ulAddress;
}

void vCheckTask(void *pvParameters) {
	uint32_t value = (uint32_t) pvParameters;
	char cTempArray[12] = {"\0"};
#define MEM_ADDR 0x0800a148

	for (;;) {
		osQueueUARTMessage("Addr: 0x%08X, Value: %c\r\n", (uint32_t)(MEM_ADDR),
				(char)ulFlashRead((uint32_t)(MEM_ADDR)));

		/*for (size_t i = 0; i < 38; i++) {
			cTempArray[i] = (char)ulFlashRead((uint32_t)(MEM_ADDR + i));
			osQueueUARTMessage("Addr: 0x%08X, Value: %c\r\n", (uint32_t)(MEM_ADDR + i), cTempArray[i]);
		}
		osQueueUARTMessage("%u SystemGood %d \r\n", value, xTaskGetTickCount());
		osQueueUARTMessage("Value of var from memory addr 0x%08X: %s\r\n", MEM_ADDR, cTempArray);
		*/
		//taskYIELD();
		vTaskDelay(pdMS_TO_TICKS(3000));
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
