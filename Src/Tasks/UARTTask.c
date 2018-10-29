#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "uart.h"
#include "stdarg.h"

#include "Tasks/UARTTask.h"
QueueHandle_t xUARTQueue;

#define LL_UART_DMA_CHAN_TX __LL_DMA_GET_CHANNEL(UART_DMA_CHAN_TX)
#define LL_DMA_IsActive

DMA_HandleTypeDef dma;

/**
 * A task that prints strings via UART through DMA
 */
void vUARTTask(void *pvParameters) {
	UARTMessage_t message;

	while (1) {
		if (xQueueReceive(xUARTQueue, &message, portMAX_DELAY)) { // Receive a message from the queue
			LL_USART_EnableDMAReq_TX(UART_PORT); // Enable DMA in the USART registers
			LL_DMA_SetDataLength(DMA1, LL_UART_DMA_CHAN_TX, strlen(message)); // Set amount of copied bits for DMA
			LL_DMA_ConfigAddresses(DMA1, LL_UART_DMA_CHAN_TX, message,
					&(UART_PORT->DR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
			LL_DMA_EnableChannel(DMA1, LL_UART_DMA_CHAN_TX); // Enable the DMA transaction

			// Note: The following polling method doesn't make sense, since it wastes the processor speed.
			// An interrupt should be used instead.
			while (__HAL_DMA_GET_FLAG(&dma, __HAL_DMA_GET_TC_FLAG_INDEX(&dma))
					== RESET) {
			} // Wait until the transaction is complete
			__HAL_DMA_CLEAR_FLAG(&dma, __HAL_DMA_GET_TC_FLAG_INDEX(&dma)); // Clear the Transaction Complete flag so it can be used later

			LL_DMA_DisableChannel(DMA1, LL_UART_DMA_CHAN_TX); // Disable the DMA channel (this is necessary, so it can be reused later)
			LL_USART_DisableDMAReq_TX(UART_PORT); // Disable DMA in the USART registers
			vPortFree(message); // Free up the memory held by the message string
		}
	}
}

/**
 * Queue a UART message so that it can be printed later
 */
void osQueueUARTMessage(const char * format, ...) {
	// TODO: Less copying around bits

	va_list arg;
	char buffer[128];

	va_start(arg, format);
	vsnprintf(buffer, 128, format, arg);
	va_end(arg);

	//configASSERT(strlen(message) < 127);

	UARTMessage_t pcUARTMessage = pvPortMalloc(strlen(buffer) + 1);

	if (pcUARTMessage == NULL) {
		UART_SendStr("ERROR! Not enough memory to store UART string\r\n");
	} else {
		strcpy(pcUARTMessage, buffer);

		// TODO: Show a warning if the queue is full (e.g. replace the last
		// message in the queue)
		if (xQueueSend(xUARTQueue, (void*) (&pcUARTMessage),
				(TickType_t) 0) == pdFAIL) {
			// Make sure to deallocate the failed message
			vPortFree(pcUARTMessage);
		}
	}
}

void vSetupUART() {
	UART_Init(115200); //Initialize the UART with the set baud rate
	UART_SendStr("CubeSAT hardware initialization...\r\n");

	// DMA (Direct Memory Access) initialisation
	__HAL_RCC_DMA1_CLK_ENABLE();
	dma.Instance = UART_DMA_CHAN_TX; // DMA channel for UART
	dma.Init.Direction = DMA_MEMORY_TO_PERIPH; // Transfer data from memory to peripheral
	dma.Init.PeriphInc = DMA_PINC_DISABLE; // Disable incrementing a pointer
	dma.Init.MemInc = DMA_MINC_ENABLE; // Disable incrementing a pointer
	dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // Transfer each byte
	dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    // Transfer each byte
	dma.Init.Mode = DMA_NORMAL;
	dma.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&dma);
	//	__HAL_LINKDMA(huart,hdmatx,dma);
}
