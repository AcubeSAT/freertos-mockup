#include "Tasks/BlinkyTask.h"
#include "Tasks/CheckTask.h"
#include "Tasks/NRF24Task.h"
#include "Tasks/SensorTask.h"
#include "Tasks/TraceTask.h"
#include "Tasks/UARTTask.h"
#include "Tasks/WWDGTask.h"
#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

static int brightnessLevel = 100; //global variable that represents the maximum brightness in which tasks remain suspended
TaskHandle_t xReceiveSuspendTask;

void vReceiveSuspendTask(void *pvParameters) {
	while (1) {
		if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY)) {

			//uint8_t payload_length; //Length of received payload (Do I use this variable?)

			char* tokenCh = NULL; //Save the tokenized string
			char* checkString = "LetThereBeLight"; //string that needs to be received to resume

			/*
			 * Some code to read the notification
			 */

			//TODO: (Should I?) Use a loop to run through a list of each task's TaskHandle_t

			//Suspension of each task except the brightness sensor, this task, vResumeTask
			if (strcmp(tokenCh,checkString)==0){
				vTaskSuspend(xBlinkyTask);
				vTaskSuspend(xCheckTask);
			#if SAT_Enable_Sensors
				vTaskSuspend(xMPU9250Task);
			#endif
				vTaskSuspend(xUARTTask);
				vTaskSuspend(xRefreshWWDGTask);

			#if SAT_Enable_NRF24
				vTaskSuspend(xTransmitTask);
				vTaskSuspend(xReceiveTask);
			#endif
			}
		}
	}
}

void vResumeSuspendedTask(void *pvParameters) {
	while (1) {
		if (xSensorData.brightness>brightnessLevel){
					vTaskResume(xBlinkyTask);
					vTaskResume(xCheckTask);
				#if SAT_Enable_Sensors
					vTaskResume(xMPU9250Task);
				#endif
					vTaskResume(xUARTTask);
					vTaskResume(xRefreshWWDGTask);

				#if SAT_Enable_NRF24
					vTaskResume(xTransmitTask);
					vTaskResume(xReceiveTask);
				#endif
		}
	}
}

