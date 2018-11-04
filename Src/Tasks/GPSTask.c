#include "Tasks/GPSTask.h"

#if SAT_Enable_GPS

#define DMA_RX_BUFFER_SIZE          550
#define COMMAND_REQUEST_SIZE		12

// Private function prototypes
void prvGPSDMAMessageTX(char *pcTxMessage);
void prvGPSDMAMessageRX(void);

// Private variables
char cDMA_RX_Buffer[DMA_RX_BUFFER_SIZE] = {"\0"};
char cDMA_TX_Buffer[COMMAND_REQUEST_SIZE + 1];

// Task handle
TaskHandle_t xGPSMsgRXTask;
TaskHandle_t xGPSTaskHandle;
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
}

void vGPSTask(void *pvParameters) {
	char cRF24Msg[32] = {"\0"};

	osQueueUARTMessage("GPS task started!.....\r\n");

	while(1) {
		if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) {
			if (xSemaphoreTake(xnRF24Semaphore, pdMS_TO_TICKS(250))) {
				nRF24_SetOperationalMode(nRF24_MODE_TX);
				nRF24_ClearIRQFlags();

				sprintf(cRF24Msg, "Time: %d:%d:%d", xGPSData.Time.hours,
						xGPSData.Time.minutes, xGPSData.Time.seconds);
				nRF24_TransmitPacket((uint8_t *)cRF24Msg, 32);

				sprintf(cRF24Msg, "Date: %d/%d/%d", xGPSData.Date.day,
						xGPSData.Date.month, xGPSData.Date.year);
				nRF24_TransmitPacket((uint8_t *)cRF24Msg, 32);

				sprintf(cRF24Msg, "Lat: %.4f", xGPSData.Latitude);
				nRF24_TransmitPacket((uint8_t *)cRF24Msg, 32);

				sprintf(cRF24Msg, "Lon: %.4f", xGPSData.Longitude);
				nRF24_TransmitPacket((uint8_t *)cRF24Msg, 32);

				sprintf(cRF24Msg, "Speed: %.2f km/h", xGPSData.Speed);
				nRF24_TransmitPacket((uint8_t *)cRF24Msg, 32);

				nRF24_SetOperationalMode(nRF24_MODE_RX);
				nRF24_CE_H();

				nRF24_FlushTX();
				xSemaphoreGive(xnRF24Semaphore);
			}
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
				xGPSData.Latitude = minmea_tocoord(&frame.latitude);
				xGPSData.Longitude = minmea_tocoord(&frame.longitude);
				xGPSData.Time = frame.time;
				xGPSData.fix_quality = frame.fix_quality;
				xGPSData.HDOP = minmea_tofloat(&frame.hdop);
				osQueueUARTMessage("*************** GGA ***************\r\n");
				osQueueUARTMessage("Lat: %f\r\n", xGPSData.Latitude);
				osQueueUARTMessage("Lon: %f\r\n", xGPSData.Longitude);
				osQueueUARTMessage("HDOP: %f\r\n", xGPSData.HDOP);
				osQueueUARTMessage("Time: %d:%d:%d.%d\r\n", xGPSData.Time.hours,
						xGPSData.Time.minutes, xGPSData.Time.seconds, xGPSData.Time.microseconds);
				osQueueUARTMessage("Fix quality: %d\r\n", xGPSData.fix_quality);
				osQueueUARTMessage("***********************************\r\n\r\n");

			}
		}
		break;

		case MINMEA_SENTENCE_GLL: {
			struct minmea_sentence_gll frame;
			if(minmea_parse_gll(&frame, cSentence)) {
				xGPSData.Latitude = minmea_tocoord(&frame.latitude);
				xGPSData.Longitude = minmea_tocoord(&frame.longitude);
				xGPSData.Time = frame.time;
				osQueueUARTMessage("*************** GLL ***************\r\n");
				osQueueUARTMessage("Lat: %f\r\n", xGPSData.Latitude);
				osQueueUARTMessage("Lon: %f\r\n", xGPSData.Longitude);
				osQueueUARTMessage("Time: %d:%d:%d.%d\r\n", xGPSData.Time.hours,
						xGPSData.Time.minutes, xGPSData.Time.seconds, xGPSData.Time.microseconds);
				osQueueUARTMessage("***********************************\r\n\r\n");

			}
		}
		break;

		case MINMEA_SENTENCE_GSA: {
			struct minmea_sentence_gsa frame;
			if(minmea_parse_gsa(&frame, cSentence)) {
				xGPSData.fix_type = frame.fix_type;
				xGPSData.fix_mode = frame.mode;
				xGPSData.PDOP = minmea_tofloat(&frame.pdop);
				xGPSData.HDOP = minmea_tofloat(&frame.hdop);
				xGPSData.VDOP = minmea_tofloat(&frame.vdop);
				osQueueUARTMessage("*************** GSA ***************\r\n");
				osQueueUARTMessage("Fix type: %d\r\n", xGPSData.fix_type);
				osQueueUARTMessage("Fix mode: %c\r\n", xGPSData.fix_mode);
				osQueueUARTMessage("HDOP: %f\r\n", xGPSData.HDOP);
				osQueueUARTMessage("VDOP: %f\r\n", xGPSData.VDOP);
				osQueueUARTMessage("PDOP: %f\r\n", xGPSData.PDOP);
				osQueueUARTMessage("***********************************\r\n\r\n");

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
				xGPSData.Latitude = minmea_tocoord(&frame.latitude);
				xGPSData.Longitude = minmea_tocoord(&frame.longitude);
				xGPSData.Date = frame.date;
				xGPSData.Time = frame.time;
				xGPSData.Speed = minmea_tofloat(&frame.speed);
				xGPSData.Course = minmea_tofloat(&frame.course);
				osQueueUARTMessage("*************** RMC ***************\r\n");
				osQueueUARTMessage("Lat: %f\r\n", xGPSData.Latitude);
				osQueueUARTMessage("Lon: %f\r\n", xGPSData.Longitude);
				osQueueUARTMessage("Date: %d/%d/%d\r\n", xGPSData.Date.day,
						xGPSData.Date.month, xGPSData.Date.year);
				osQueueUARTMessage("Time: %d:%d:%d.%d\r\n", xGPSData.Time.hours,
						xGPSData.Time.minutes, xGPSData.Time.seconds, xGPSData.Time.microseconds);
				osQueueUARTMessage("Speed: %f\r\n", xGPSData.Speed);
				osQueueUARTMessage("Course: %f\r\n", xGPSData.Course);
				osQueueUARTMessage("***********************************\r\n\r\n");

			}
		}
		break;

		case MINMEA_SENTENCE_VTG: {
			struct minmea_sentence_vtg frame;
			if(minmea_parse_vtg(&frame, cSentence)) {
				xGPSData.Speed_knots = minmea_tofloat(&frame.speed_knots);
				xGPSData.Speed_kph = minmea_tofloat(&frame.speed_kph);
				xGPSData.Mag_Track_Deg = minmea_tofloat(&frame.magnetic_track_degrees);
				xGPSData.True_Track_Deg = minmea_tofloat(&frame.true_track_degrees);
				osQueueUARTMessage("*************** VTG ***************\r\n");
				osQueueUARTMessage("Speed (knots): %f\r\n", xGPSData.Speed_knots);
				osQueueUARTMessage("Speed (km/h): %f\r\n", xGPSData.Speed_kph);
				osQueueUARTMessage("Magnetic track (deg): %f\r\n", xGPSData.Mag_Track_Deg);
				osQueueUARTMessage("True track (deg): %f\r\n", xGPSData.True_Track_Deg);
				osQueueUARTMessage("***********************************\r\n");

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
	char *pcTokSstr = NULL;

	if(xBufDataLen == DMA_RX_BUFFER_SIZE)
		xLen = DMA_RX_BUFFER_SIZE;
	else
		xLen = DMA_RX_BUFFER_SIZE - xBufDataLen;
	cDMA_RX_Buffer[xLen] = '\0';  // Append a null terminator

	pcTokSstr = strtok(cDMA_RX_Buffer, "\r\n");
	while(pcTokSstr) {
		switch(cGetGPSData(pcTokSstr)) {
			case MINMEA_INVALID:
				osQueueUARTMessage("Invalid NMEA sentence provided....\r\n");
				osQueueUARTMessage("Sentence: %s\r\n", pcTokSstr);
				break;
			case -10:
				osQueueUARTMessage("NMEA sentence error....\r\n");
				break;
		}
		pcTokSstr = strtok(NULL, "\r\n");  // Get the other strings, if any
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
#endif

