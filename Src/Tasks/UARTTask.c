#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "uart.h"
#include "stdarg.h"

#include "Tasks/UARTTask.h"

#define LL_UART_DMA_CHAN_TX __LL_DMA_GET_CHANNEL(UART_DMA_CHAN_TX)
#define LL_DMA_IsActive

DMA_HandleTypeDef dma;

QueueHandle_t xUARTQueue;
TaskHandle_t xUARTTaskHandle;

/**
 * A task that prints strings via UART through DMA
 */
void vUARTTask(void *pvParameters) {
	UARTMessage_t message = NULL;

	while (1) {
		if (xQueueReceive(xUARTQueue, &message, portMAX_DELAY)) { // Receive a message from the queue
			LL_USART_EnableDMAReq_TX(UART_PORT); // Enable DMA in the USART registers
			LL_DMA_SetDataLength(DMA1, LL_UART_DMA_CHAN_TX, strlen(message)); // Set amount of copied bits for DMA
			LL_DMA_ConfigAddresses(DMA1, LL_UART_DMA_CHAN_TX, (uint32_t)message,
					(uint32_t)&(UART_PORT->DR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
			LL_DMA_EnableChannel(DMA1, LL_UART_DMA_CHAN_TX); // Enable the DMA transaction

			xTaskNotifyWait(0x00, 0x00, NULL, portMAX_DELAY);
			vPortFree(message); // Free up the memory held by the message string
			message = NULL;
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

	LL_DMA_EnableIT_TC(DMA1, LL_UART_DMA_CHAN_TX);

	NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 12, 0));
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void DMA_UART_TX_ISR(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Notify the UART1 TX task that transaction has ended
	vTaskNotifyGiveFromISR(xUARTTaskHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
