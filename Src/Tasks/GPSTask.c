#include "Tasks/GPSTask.h"

#define DMA_RX_BUFFER_SIZE          550
#define COMMAND_REQUEST_SIZE		12
#define GPS_MESSAGE_QUEUE_SIZE		15

// Private function prototypes
void prvGPSDMAMessageTX(char *pcTxMessage);
void prvGPSDMAMessageRX(void);

// Private variables
char cDMA_RX_Buffer[DMA_RX_BUFFER_SIZE] = {"\0"};
char cDMA_TX_Buffer[COMMAND_REQUEST_SIZE + 1];

// Task handle
TaskHandle_t xGPSMsgRXTask;
QueueHandle_t xGPSQueue;
GPSData_t xGPSData;

LL_DMA_InitTypeDef dma_usart_rx;  // Define the DMA structure for RX
LL_DMA_InitTypeDef dma_usart_tx;  // Define the DMA structure for TX

/************************ Sample NMEA cSentences *************************
 * $GPGLL,4916.45,N,12311.12,W,225444,A,*1D                             *
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A *
 * $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47    *
 ************************************************************************/

/*********************************************
 * USART1 DMA TX Channel --> DMA1, Channel 4 *
 * USART1 DMA RX Channel --> DMA1, Channel 5 *
 * *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-* *
 * USART3 DMA TX Channel --> DMA1, Channel 2 *
 * USART3 DMA RX Channel --> DMA1, Channel 3 *
 *********************************************/

void vSetupGPS(void) {
	/********************** GPS USART Init **********************/
	__HAL_RCC_USART3_CLK_ENABLE();  // Enable USART clock

	GPIO_InitTypeDef GPIO_UGPS;

	GPIO_UGPS.Pin = GPIO_PIN_10;
	GPIO_UGPS.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_UGPS.Mode = GPIO_MODE_AF_PP; // TX as AF with Push-Pull
	HAL_GPIO_Init(GPIOB, &GPIO_UGPS);

	GPIO_UGPS.Pin = GPIO_PIN_11;
	GPIO_UGPS.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_UGPS.Mode = GPIO_MODE_INPUT; // RX as in without pull-up
	HAL_GPIO_Init(GPIOB, &GPIO_UGPS);

	UART_HandleTypeDef UART_GPS;

	UART_GPS.Instance = UART_PORT_GPS;
	UART_GPS.Init.BaudRate = UART_BAUDRATE_GPS;
	UART_GPS.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART_GPS.Init.Mode = UART_MODE_TX_RX;
	UART_GPS.Init.WordLength = UART_WORDLENGTH_8B;
	UART_GPS.Init.Parity = UART_PARITY_NONE;
	UART_GPS.Init.StopBits = UART_STOPBITS_1;
	HAL_UART_Init(&UART_GPS);

	__HAL_UART_ENABLE(&UART_GPS);
	LL_USART_EnableDMAReq_RX(UART_PORT_GPS);
	LL_USART_EnableDMAReq_TX(UART_PORT_GPS);
	LL_USART_EnableIT_IDLE(UART_PORT_GPS);

	// Set the priority to configMAX_SYSCALL_INTERRUPT_PRIORITY + 1
	NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 11 + 1, 0));
	NVIC_EnableIRQ(USART3_IRQn);
	/********************** GPS USART Init **********************/

	/********************** GPS USART RX DMA **********************/
	LL_DMA_StructInit(&dma_usart_rx);
	dma_usart_rx.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	dma_usart_rx.MemoryOrM2MDstAddress = (uint32_t)cDMA_RX_Buffer;
	dma_usart_rx.NbData = DMA_RX_BUFFER_SIZE;
	dma_usart_rx.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	dma_usart_rx.PeriphOrM2MSrcAddress = (uint32_t)&(UART_PORT_GPS->DR);
	dma_usart_rx.Mode = LL_DMA_MODE_NORMAL;
	LL_DMA_Init(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_RX_GPS, &dma_usart_rx);

	LL_DMA_EnableChannel(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_RX_GPS);
	/********************** GPS USART RX DMA **********************/

	/********************** GPS USART TX DMA **********************/
	LL_DMA_StructInit(&dma_usart_tx);
	dma_usart_tx.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	dma_usart_tx.MemoryOrM2MDstAddress = (uint32_t)&(UART_PORT_GPS->DR);
	dma_usart_tx.NbData = COMMAND_REQUEST_SIZE;
	dma_usart_tx.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	dma_usart_tx.PeriphOrM2MSrcAddress = (uint32_t)&cDMA_TX_Buffer;
	dma_usart_tx.Mode = LL_DMA_MODE_NORMAL;
	LL_DMA_Init(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_TX_GPS, &dma_usart_tx);

	LL_DMA_EnableIT_TC(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_TX_GPS);

	/* **************************************************************************** *
	 * TODO: Modify the priorities (always above 11), to comply with all other...   *
	 * ...tasks and interrupts in the final code.									*
	 ********************************************************************************/
	NVIC_SetPriority(DMA1_Channel2_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 13, 0));
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/********************** GPS USART TX DMA **********************/

	/* ******************************** Create the GPS queue ****************************** *
	 * In this GPS queue the objects are the pointers to the allocated message space		*
	 * so there is no need to be a large number, since the messages are of known number.	*
	 * Choosing a small queue size also frees some memory.									*
	 * A misconception for the current implementation is that the message is passed as a	*
	 * whole to the queue.																	*
	 * ************************************************************************************ */
	xGPSQueue = xQueueCreate(GPS_MESSAGE_QUEUE_SIZE, sizeof(GPSMessage_t *));
}

void vGPSTask(void *pvParameters) {
	GPSMessage_t xSentence = NULL;
	char *pcTokSstr = NULL;

	osQueueUARTMessage("GPS task started!.....\r\n");

	while(1) {
		if(xQueueReceive(xGPSQueue, &xSentence, portMAX_DELAY)) {
			pcTokSstr = strtok(xSentence, "\r\n");
			while(pcTokSstr) {
				switch(cGetGPSData(pcTokSstr)) {
					case MINMEA_SENTENCE_GGA: {
						osQueueUARTMessage("*************** GGA ***************\r\n");
						osQueueUARTMessage("Lat: %d\r\n", xGPSData.Latitude.value);
						osQueueUARTMessage("Lon: %d\r\n", xGPSData.Longitude.value);
						osQueueUARTMessage("HDOP: %d\r\n", xGPSData.HDOP.value);
						osQueueUARTMessage("Time: %d:%d:%d.%d\r\n", xGPSData.Time.hours,
								xGPSData.Time.minutes, xGPSData.Time.seconds, xGPSData.Time.microseconds);
						osQueueUARTMessage("Fix quality: %d\r\n", xGPSData.fix_quality);
						osQueueUARTMessage("***********************************\r\n\r\n");
					}
					break;

					case MINMEA_SENTENCE_GLL: {
						osQueueUARTMessage("*************** GLL ***************\r\n");
						osQueueUARTMessage("Lat: %d\r\n", xGPSData.Latitude.value);
						osQueueUARTMessage("Lon: %d\r\n", xGPSData.Longitude.value);
						osQueueUARTMessage("Time: %d:%d:%d.%d\r\n", xGPSData.Time.hours,
								xGPSData.Time.minutes, xGPSData.Time.seconds, xGPSData.Time.microseconds);
						osQueueUARTMessage("***********************************\r\n\r\n");
					}
					break;

					case MINMEA_SENTENCE_GSA: {
						osQueueUARTMessage("*************** GSA ***************\r\n");
						osQueueUARTMessage("Fix type: %d\r\n", xGPSData.fix_type);
						osQueueUARTMessage("Fix mode: %c\r\n", xGPSData.fix_mode);
						osQueueUARTMessage("HDOP: %d\r\n", xGPSData.HDOP.value);
						osQueueUARTMessage("VDOP: %d:%d:%d.%d\r\n", xGPSData.VDOP.value);
						osQueueUARTMessage("PDOP: %d\r\n", xGPSData.PDOP.value);
						osQueueUARTMessage("***********************************\r\n\r\n");
					}
					break;

					/*case MINMEA_SENTENCE_GSV: {
						osQueueUARTMessage("*************** GSV ***************\r\n");
						osQueueUARTMessage("Total satellites: %d\r\n", xGPSData.total_sats);
						osQueueUARTMessage("Total messages: %d\r\n", xGPSData.total_msgs);
						osQueueUARTMessage("Message number: %d\r\n", xGPSData.msg_nr);

						for(size_t counter = 0; counter < 4; counter++) {
							osQueueUARTMessage("\r\nSatellite %d:\r\n", counter);
							osQueueUARTMessage("SNR: %d\r\n", xGPSData.sats[counter].snr);
							osQueueUARTMessage("Number: %d\r\n", xGPSData.sats[counter].nr);
							osQueueUARTMessage("Azimuth: %d\r\n", xGPSData.sats[counter].azimuth);
							osQueueUARTMessage("Elevation: %d\r\n", xGPSData.sats[counter].elevation);
						}
						osQueueUARTMessage("***********************************\r\n\r\n");
					}
					break;*/

					case MINMEA_SENTENCE_RMC: {
						osQueueUARTMessage("*************** RMC ***************\r\n");
						osQueueUARTMessage("Lat: %d\r\n", xGPSData.Latitude.value);
						osQueueUARTMessage("Lon: %d\r\n", xGPSData.Longitude.value);
						osQueueUARTMessage("Date: %d/%d/%d\r\n", xGPSData.Date.day,
								xGPSData.Date.month, xGPSData.Date.year);
						osQueueUARTMessage("Time: %d:%d:%d.%d\r\n", xGPSData.Time.hours,
								xGPSData.Time.minutes, xGPSData.Time.seconds, xGPSData.Time.microseconds);
						osQueueUARTMessage("Speed: %d\r\n", xGPSData.Speed.value);
						osQueueUARTMessage("Course: %d\r\n", xGPSData.Course.value);
						osQueueUARTMessage("***********************************\r\n");
					}
					break;

					case MINMEA_SENTENCE_VTG: {
						osQueueUARTMessage("*************** VTG ***************\r\n");
						osQueueUARTMessage("Speed (knots): %d\r\n", xGPSData.Speed_knots.value);
						osQueueUARTMessage("Speed (km/h): %d\r\n", xGPSData.Speed_kph.value);
						osQueueUARTMessage("Magnetic track (deg): %d\r\n", xGPSData.Mag_Track_Deg.value);
						osQueueUARTMessage("True track (deg): %d\r\n", xGPSData.True_Track_Deg.value);
						osQueueUARTMessage("***********************************\r\n");
					}
					break;

					case MINMEA_SENTENCE_ZDA: {
						osQueueUARTMessage("*************** ZDA ***************\r\n");
						osQueueUARTMessage("Date: %d/%d/%d\r\n", xGPSData.Date.day,
								xGPSData.Date.month, xGPSData.Date.year);
						osQueueUARTMessage("Time: %d:%d:%d.%d\r\n", xGPSData.Time.hours,
								xGPSData.Time.minutes, xGPSData.Time.seconds, xGPSData.Time.microseconds);
						osQueueUARTMessage("***********************************\r\n");
					}
					break;

					case MINMEA_INVALID:
						osQueueUARTMessage("Invalid NMEA sentence provided....\r\n");
						osQueueUARTMessage("Sentence: %s\r\n", pcTokSstr);
						break;

					default:
						osQueueUARTMessage("NMEA sentence error....\r\n");
						break;
				}
				pcTokSstr = strtok(NULL, "\r\n");  // Get the other strings, if any
			}
			vPortFree(xSentence);  // Free up the pointer memory (Very important for the program!)
			xSentence = NULL;  // Also reset the pointer to avoid any problems
		}
	}
}

void vGPSMessageRXTask(void *pvParameters) {
	while(1) {
		if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) {
			prvGPSDMAMessageRX();
		}
	}
}

void vRequestGPSData(int8_t cNmeaCommand) {
	char pcCommandString[COMMAND_REQUEST_SIZE + 1] = {"\0"};
	switch(cNmeaCommand) {
		case MINMEA_SENTENCE_GGA:
			sprintf(pcCommandString, "$CC%sQ,%s\r\n", "GP", "GGA");
			break;

		case MINMEA_SENTENCE_GLL:
			sprintf(pcCommandString, "$CC%sQ,%s\r\n", "GP", "GLL");
			break;

		case MINMEA_SENTENCE_GSA:
			sprintf(pcCommandString, "$CC%sQ,%s\r\n", "GP", "GSA");
			break;

		case MINMEA_SENTENCE_GSV:
			sprintf(pcCommandString, "$CC%sQ,%s\r\n", "GP", "GSV");
			break;

		case MINMEA_SENTENCE_RMC:
			sprintf(pcCommandString, "$CC%sQ,%s\r\n", "GP", "RMC");
			break;

		case MINMEA_SENTENCE_VTG:
			sprintf(pcCommandString, "$CC%sQ,%s\r\n", "GP", "VTG");
			break;

		case MINMEA_SENTENCE_ZDA:
			sprintf(pcCommandString, "$CC%sQ,%s\r\n", "GP", "ZDA");
			break;
	}
	prvGPSDMAMessageTX(pcCommandString);
}

int8_t cGetGPSData(char *cSentence) {
	int8_t cIDCode = minmea_sentence_id(cSentence, false);
	switch(cIDCode) {
		case MINMEA_SENTENCE_GGA: {
			struct minmea_sentence_gga frame;
			if(minmea_parse_gga(&frame, cSentence)) {
				xGPSData.Latitude = frame.latitude;
				xGPSData.Longitude = frame.longitude;
				xGPSData.Time = frame.time;
				xGPSData.fix_quality = frame.fix_quality;
				xGPSData.HDOP = frame.hdop;
			}
		}
		break;

		case MINMEA_SENTENCE_GLL: {
			struct minmea_sentence_gll frame;
			if(minmea_parse_gll(&frame, cSentence)) {
				xGPSData.Latitude = frame.latitude;
				xGPSData.Longitude = frame.longitude;
				xGPSData.Time = frame.time;
			}
		}
		break;

		case MINMEA_SENTENCE_GSA: {
			struct minmea_sentence_gsa frame;
			if(minmea_parse_gsa(&frame, cSentence)) {
				xGPSData.fix_type = frame.fix_type;
				xGPSData.fix_mode = frame.mode;
				xGPSData.PDOP = frame.pdop;
				xGPSData.HDOP = frame.hdop;
				xGPSData.VDOP = frame.vdop;
			}
		}
		break;

		case MINMEA_SENTENCE_GSV: {
			struct minmea_sentence_gsv frame;
			if(minmea_parse_gsv(&frame, cSentence)) {
				xGPSData.total_msgs = frame.total_msgs;
				xGPSData.msg_nr = frame.msg_nr;
				xGPSData.total_sats = frame.total_sats;
				xGPSData.sats[0] = frame.sats[0];
				xGPSData.sats[1] = frame.sats[1];
				xGPSData.sats[2] = frame.sats[2];
				xGPSData.sats[3] = frame.sats[3];
			}
		}
		break;

		case MINMEA_SENTENCE_RMC: {
			struct minmea_sentence_rmc frame;
			if(minmea_parse_rmc(&frame, cSentence)) {
				xGPSData.Latitude = frame.latitude;
				xGPSData.Longitude = frame.longitude;
				xGPSData.Date = frame.date;
				xGPSData.Time = frame.time;
				xGPSData.Speed = frame.speed;
				xGPSData.Course = frame.course;
			}
		}
		break;

		case MINMEA_SENTENCE_VTG: {
			struct minmea_sentence_vtg frame;
			if(minmea_parse_vtg(&frame, cSentence)) {
				xGPSData.Speed_knots = frame.speed_knots;
				xGPSData.Speed_kph = frame.speed_kph;
				xGPSData.Mag_Track_Deg = frame.magnetic_track_degrees;
				xGPSData.True_Track_Deg = frame.true_track_degrees;
			}
		}
		break;

		case MINMEA_SENTENCE_ZDA: {
			struct minmea_sentence_zda frame;
			if(minmea_parse_zda(&frame, cSentence)) {
				xGPSData.Date = frame.date;
				xGPSData.Time = frame.time;
			}
		}
		break;

		default:
			cIDCode = -10;
			break;
	}
	return cIDCode;
}

void prvGPSDMAMessageTX(char *pcTxMessage) {
	LL_DMA_SetDataLength(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_TX_GPS, strlen(pcTxMessage));
	LL_DMA_ConfigAddresses(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_TX_GPS,
			(uint32_t)pcTxMessage, (uint32_t)&(UART_PORT_GPS->DR),
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
	LL_DMA_EnableChannel(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_TX_GPS);
}

void prvGPSDMAMessageRX(void) {
	size_t xLen = 0;
	size_t xBufDataLen = LL_DMA_GetDataLength(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_RX_GPS);
	GPSMessage_t pcRxMessage = NULL;

	if(xBufDataLen == DMA_RX_BUFFER_SIZE)
		xLen = DMA_RX_BUFFER_SIZE;
	else
		xLen = DMA_RX_BUFFER_SIZE - xBufDataLen;

	pcRxMessage = (char *)pvPortMalloc(xLen + 1);

	if (pcRxMessage == NULL)
	{
		UART_SendStr("ERROR! Not enough memory to store GPS string\r\n");
	}
	else
	{
		strcpy(pcRxMessage, cDMA_RX_Buffer);
		pcRxMessage[xLen] = '\0';  // Append a null terminator
		if (xQueueSend(xGPSQueue, (void* ) (&pcRxMessage), (TickType_t ) 0) == pdFAIL)
		{
			// Make sure to deallocate the failed message
			vPortFree(pcRxMessage);
		}
	}

	// Reset the flags in the DMA to prepare for the next transaction
	LL_DMA_ClearFlag_GI3(LL_UART_DMA_HANDLE);
	LL_DMA_ClearFlag_HT3(LL_UART_DMA_HANDLE);
	LL_DMA_ClearFlag_TC3(LL_UART_DMA_HANDLE);
	LL_DMA_ClearFlag_TE3(LL_UART_DMA_HANDLE);

	LL_DMA_DisableChannel(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_RX_GPS);
	LL_DMA_SetDataLength(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_RX_GPS,
			(uint32_t)DMA_RX_BUFFER_SIZE);
	LL_DMA_EnableChannel(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_RX_GPS);
}

void DMA_GPS_TX_ISR(void) {
	if(LL_DMA_IsActiveFlag_TC2(LL_UART_DMA_HANDLE)) {
		LL_DMA_ClearFlag_TC2(LL_UART_DMA_HANDLE);
		LL_DMA_DisableChannel(LL_UART_DMA_HANDLE, LL_UART_DMA_CHAN_TX_GPS);
	}
}

void DMA_GPS_RX_ISR(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Send a notification to FREERTOS for the task to take priority
	vTaskNotifyGiveFromISR(xGPSMsgRXTask, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
