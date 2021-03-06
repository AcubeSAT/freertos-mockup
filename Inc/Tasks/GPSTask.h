#ifndef INC_TASKS_GPSTASK_H_
#define INC_TASKS_GPSTASK_H_

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#include "../minmea.h"

#include "Peripherals/uart.h"
#include "Tasks/UARTTask.h"
#include "Tasks/NRF24Task.h"
#include "MockupConfig.h"
#include "nrf24.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define LL_UART_DMA_HANDLE					DMA1
#define LL_UART_DMA_CHAN_RX_GPS				LL_DMA_CHANNEL_3
#define LL_UART_DMA_CHAN_TX_GPS				LL_DMA_CHANNEL_2
#define UART_BAUDRATE_GPS					(uint32_t)9600
#define UART_PORT_GPS						USART3

#define GPS_DEBUGGING_MSGS					1

#if SAT_Enable_GPS

typedef struct
{
	float Latitude;
	float Longitude;
	float Speed;
	float Speed_knots;
	float Speed_kph;
	float Course;
	float Mag_Track_Deg;
	float True_Track_Deg;
	float PDOP;
	float HDOP;
	float VDOP;
	struct minmea_date Date;
	struct minmea_time Time;

	/***** Info from GSV command *****/
	struct minmea_sat_info sats[4];
	int total_msgs;
	int msg_nr;
	int total_sats;
	/*********************************/

	int fix_quality;
	int fix_type;
	char fix_mode;
} GPSData_t;

typedef char * GPSMessage_t;
extern GPSData_t xGPSData;
extern TaskHandle_t xGPSMsgRXTask;
extern TaskHandle_t xGPSTaskHandle;

extern void vSetupGPS(void);  // Initial setup of the USART, DMA and the message queue
extern void vGPSTask(void *pvParameters);  // Main GPS task
extern void vGPSMessageRXTask(void *pvParameters);
extern void vRequestGPSData(int8_t cNmeaCommand);  // Provide any of the MINMEA_SENTENCE_XXX as the argument
extern void vGPSDMAMessageTX(char *pcTxMessage);
extern int8_t cGetGPSData(char *cSentence);

extern void DMA_GPS_TX_ISR(void);
extern void DMA_GPS_RX_ISR(void);
#endif

#endif /* INC_TASKS_GPSTASK_H_ */
