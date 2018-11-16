#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_gpio.h"
#include "string.h"

#include "MockupConfig.h"
#include "Tasks/NRF24Task.h"
#include "Tasks/BlinkyTask.h"
#include "Tasks/SensorTask.h"
#include "Tasks/UARTTask.h"
#include "Tasks/GPSTask.h"
#include "uart.h"
#include "flashOps.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "nrf24.h"

#if SAT_Enable_NRF24
volatile uint8_t stopRX = 0; //Logic variable to indicate the stopping of the RX
uint8_t nRF24_payload[32] = {"\0"}; //Buffer to store a payload of maximum width

TaskHandle_t xReceiveTask;
SemaphoreHandle_t xnRF24Semaphore;


void vSetupNRF24() {
	// NRF24 Interrupt pin initialization
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

	/***** NRF24 Interrupt pin init *****/
	// Initialize the interrupt pin
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE2);

	/**/
	LL_EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_FLOATING);

	/* EXTI interrupt service init */
	NVIC_SetPriority(EXTI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 15));
	NVIC_EnableIRQ(EXTI2_IRQn);

	nRF24_GPIO_Init(); //Start the pins used by the NRF24
	nRF24_Init(); //Initialize the nRF24L01 to its default state

	nRF24_CE_L(); //RX/TX disabled

	//A small check for debugging
	UART_SendStr("nRF24L01+ check: ");
	if (!nRF24_Check()) {
		UART_SendStr("FAIL\r\n");
		NVIC_SystemReset();
		while (1)
			;
	} else {
		UART_SendStr("OK\r\n");
	}

	// This is simple transmitter with Enhanced ShockBurst (to one logic address):
	//   - TX address: 'BaseS'
	//   - Payload: 32 bytes
	//   - RF channel: 99 (2499MHz)
	//   - data rate: 2Mbps
	//   - CRC scheme: 2 byte
	nRF24_SetRFChannel(99); //Set RF channel
	nRF24_SetDataRate(nRF24_DR_2Mbps); //Set data rate
	nRF24_SetCRCScheme(nRF24_CRC_2byte); //Set CRC scheme
	nRF24_SetAddrWidth(5); //Set address width, its common for all pipes (RX and TX)

	//Configure TX PIPE
	static const uint8_t nRF24_ADDR_Tx[] = { 'B', 'a', 's', 'e', 'S' }; //Address of the receiving end
	static const uint8_t nRF24_ADDR_Rx[] = { 'C', 'u', 'b', 'e', 'S' }; //Address of the current module
	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR_Tx); //Program TX address
	nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR_Tx); //Program address for pipe#0, must be same as TX (for Auto-ACK)

	//Configure RX PIPE
	nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR_Rx); //Program address for pipe
	nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 32); // Auto-ACK: enabled, payload length: 32 bytes

	nRF24_SetTXPower(nRF24_TXPWR_0dBm);	//Set TX power (maximum)
	nRF24_SetAutoRetr(nRF24_ARD_2500us, 10); //Configure auto retransmit: 10 retransmissions with pause of 2500s in between
	nRF24_EnableAA(nRF24_PIPE0); //Enable Auto-ACK for pipe#0 (for ACK packets)

	nRF24_SetOperationalMode(nRF24_MODE_TX); //Set operational mode (PTX == transmitter)
	nRF24_ClearIRQFlags(); //Clear any pending IRQ flags
	nRF24_SetPowerMode(nRF24_PWR_UP); //Wake the transceiver
}

void vTransmitTask(void *pvParameters) {
	nRF24_SetOperationalMode(nRF24_MODE_TX); //Set operational mode (PTX == transmitter)
	sprintf((char *) nRF24_payload, "%s", "S0");
	nRF24_TransmitPacket(nRF24_payload, 32);

	while (1) {
		if (xEventGroupWaitBits(xDataEventGroup,
		DATA_EVENT_GROUP_BH1750_Msk | DATA_EVENT_GROUP_MPU9250_Msk,
		pdTRUE, pdTRUE, portMAX_DELAY)) {
			if (xSemaphoreTake(xnRF24Semaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
				UART_SendStr("FATAL Error: nRF24Transmit timeout");
			} else {
				nRF24_SetOperationalMode(nRF24_MODE_TX); //Set operational mode (PTX == transmitter)
				nRF24_ClearIRQFlags(); //Clear any pending IRQ flags
				GPIOC->BSRR = 1 << 13;

				sprintf((char *) nRF24_payload, "B%.2f", xSensorData.brightness);
				nRF24_TransmitPacket(nRF24_payload, 32);

				/*memset((uint8_t *)nRF24_payload, '\0', 32); //Fill all the array space with zeros
				 sprintf((char *)nRF24_payload, "B%.2f %.2f %.2f %.2f", q0, q1, q2, q3);
				 nRF24_TransmitPacket(nRF24_payload, 32);*/

				sprintf((char *) nRF24_payload, "X%ld %ld",
						(int32_t) (xSensorData.acc[0] * 100000.0),
						(int32_t) (xSensorData.gyr[0] * 100000.0));
				nRF24_TransmitPacket(nRF24_payload, 32);

				sprintf((char *) nRF24_payload, "Y%ld %ld",
						(int32_t) (xSensorData.acc[1] * 100000.0),
						(int32_t) (xSensorData.gyr[1] * 100000.0));
				nRF24_TransmitPacket(nRF24_payload, 32);

				sprintf((char *) nRF24_payload, "Z%ld %ld",
						(int32_t) (xSensorData.acc[2] * 100000.0),
						(int32_t) (xSensorData.gyr[2] * 100000.0));
				nRF24_TransmitPacket(nRF24_payload, 32);

				GPIOC->BRR = 1 << 13;

				// Prepare the sensor for receiving the data while sleeping
				nRF24_SetOperationalMode(nRF24_MODE_RX); //Set operational mode (PRX == receiver)
				nRF24_CE_H();

				nRF24_FlushTX();
				xSemaphoreGive(xnRF24Semaphore);
			}
		}
	}
}

void vReceiveTask(void *pvParameters) {
	uint8_t payload_length; //Length of received payload
	volatile uint8_t *payL_addr = &payload_length;

	//vFlashWrite((uint32_t)0x20002c7f, (uint64_t) 5);

	while (1) {
		if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) {
			if (xSemaphoreTake(xnRF24Semaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
				UART_SendStr("FATAL Error: nRF24Receive timeout");
			} else {
				char* tokenCh = NULL; //Save the tokenized string

				//Set operational mode (PRX == receiver)
				nRF24_SetOperationalMode(nRF24_MODE_RX);
				nRF24_CE_H();

				if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
					nRF24_ReadPayload(nRF24_payload, &payload_length); //Get the payload from the transceiver
					nRF24_ClearIRQFlags(); //Clear all pending IRQ flags

					tokenCh = strtok((char*) nRF24_payload, ":");
					if (strstr(tokenCh, "L1")) {
						// TODO: Provide true functionality on the reception, like sending a message
						tokenCh = strtok(NULL, ":"); // Tokenize the string
						if (strstr(tokenCh, "1")) {
							osQueueUARTMessage("->>  Received +1 command\r\n");
							vBlinkyFadeIn();
						} else if (strstr(tokenCh, "0")) {
							osQueueUARTMessage("->>  Received  0 command\r\n");
							vBlinkyFadeOut();
						} else {
							osQueueUARTMessage("->>  Received content: %s", nRF24_payload);
						}
					} else if (strstr(tokenCh, "GPS")) {
						tokenCh = strtok(NULL, ":"); // Tokenize the string
						if (strstr(tokenCh, "Data")) {
							osQueueUARTMessage("->>  Received GPS command\r\n");
#if SAT_Enable_GPS
							xTaskNotifyGive(xGPSTaskHandle);
#endif
						}
					} else if (strstr(tokenCh, "Patch")) {
						tokenCh = strtok(NULL, ":"); // Tokenize the string
						osQueueUARTMessage("->>  Received value change command\r\n");

						uint16_t addID[2] = {418, 420};
						uint16_t data[2] = {0x3334, 0x3639};

						vFlashWrite((uint32_t)(0x0800a000), addID, data, (size_t)2);
						osQueueUARTMessage("->>  Addr: 0x%08X, Value: %c\r\n",
								(char)ulFlashRead((uint32_t)(0x0800a000 + addID[0])));
					}
				}
				nRF24_FlushRX();
				xSemaphoreGive(xnRF24Semaphore);
			}
		}
	}
}

void vTaskInfoTransmitTask(void *pvParameters) {
	TaskStatus_t status[12];
	UBaseType_t NumTasks;
	while (1) {
		if (xSemaphoreTake(xnRF24Semaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
			UART_SendStr("FATAL Error: nRF24Transmit timeout");
		} else {
			nRF24_SetOperationalMode(nRF24_MODE_TX); //Set operational mode (PTX == transmitter)
			NumTasks = uxTaskGetSystemState(status, uxTaskGetNumberOfTasks(), NULL);
			for (int i = 0; i < NumTasks; i++) {
				if (i == 0) {
					sprintf((char *) nRF24_payload, "%s", "{T");
					nRF24_TransmitPacket(nRF24_payload, 32);
				}
				sprintf((char *) nRF24_payload, "%d%lu%s", status[i].eCurrentState,
						status[i].ulRunTimeCounter, status[i].pcTaskName); //
				nRF24_TransmitPacket(nRF24_payload, 32);
			}
			sprintf((char *) nRF24_payload, "%s", "}T");
			nRF24_TransmitPacket(nRF24_payload, 32);

			// Prepare the sensor for receiving the data while sleeping
			nRF24_SetOperationalMode(nRF24_MODE_RX); //Set operational mode (PRX == receiver)
			nRF24_CE_H();

			nRF24_FlushTX();
			xSemaphoreGive(xnRF24Semaphore);
			vTaskDelay(pdMS_TO_TICKS(5000));
		}
	}
}

void NRF24_RX_ISR(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	vTaskNotifyGiveFromISR(xReceiveTask, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif
