#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "MPU9250.h"
#include "BH1750.h"
#include "uart.h"
#include "event_groups.h"

#include "Tasks/SensorTask.h"
#include "Tasks/UARTTask.h"

SensorData_t xSensorData;
SemaphoreHandle_t xI2CSemaphore;
StaticSemaphore_t xI2CMutexBuffer;

EventGroupHandle_t xDataEventGroupHandle;
StaticEventGroup_t xDataCreatedEventGroup;

TaskHandle_t xBH1750Handle = NULL;
TaskHandle_t xMPU9250Handle = NULL;

StaticTask_t xBH1750TaskBuffer;
StaticTask_t xMPU9250TaskBuffer;

#if SAT_Enable_Sensors
StackType_t xBH1750TaskStack[BH1750_TASK_STACK_SIZE];
StackType_t xMPU9250TaskStack[MPU9250_TASK_STACK_SIZE];
#endif

#if SAT_Enable_Sensors
float gyrCal[3]; //Save the calibration values

void vBH1750Task(void *pvParameters) {
	// Store the last wake time so that we can delay later
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while (1) {
		if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
			UART_SendStr("FATAL Error: I2C timeout");
			vTaskSuspend(NULL); // Stop this task
		} else {
			xSensorData.brightness = BH1750_GetBrightnessCont();
			xSemaphoreGive(xI2CSemaphore);
			xEventGroupSetBits(xDataEventGroupHandle, DATA_EVENT_GROUP_BH1750_Msk);

			if (SAT_Serial_Debug)
				osQueueUARTMessage("bri %f\r\n", xSensorData.brightness);
		}

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAT_Acq_Period_ms));
	}
}

void vMPU9250Task(void *pvParameters) {
	// Store the last wake time so that we can delay later
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while (1) {
		if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
			UART_SendStr("FATAL Error: I2C timeout");
			vTaskSuspend(NULL); // Stop this task
		} else {
			MPU9250_GetCalibAccelGyro(xSensorData.acc, xSensorData.gyr, gyrCal);
			AK8963GetMagnuT(xSensorData.magn, xSensorData.magn_adj);
			xSemaphoreGive(xI2CSemaphore);

			xEventGroupSetBits(xDataEventGroupHandle, DATA_EVENT_GROUP_MPU9250_Msk);

			if (SAT_Serial_Debug) {
				osQueueUARTMessage(
						"acc dump %.2f %.2f %.2f %.2f %.2f %.2f\r\nmag dump %.2f %.2f %.2f\r\n",
						100 * xSensorData.acc[0], 100 * xSensorData.acc[1],
						100 * xSensorData.acc[2], 100 * xSensorData.gyr[0],
						100 * xSensorData.gyr[1], 100 * xSensorData.gyr[2],
						xSensorData.magn[0], xSensorData.magn[1],
						xSensorData.magn[2]);
			}
		}

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAT_Acq_Period_ms));
	}
}

void vSetupSensors() {
	TWIInit(); //Initialize I2C

	MPU9250Calibration(gyrCal); //Get the gyroscope calibration values
	MPU9250Init(AFS_2G, GFS_500DPS); //Initialize the MPU9250
	MPU9250_SetFullScaleGyroRange(GFS_2000DPS); //Set the gyroscope scale to full scale

	AK8963Init(AK8963_16BIT, AK8963_CONT100HZ, xSensorData.magn_adj);

	BH1750_Init(BH1750_CONTHRES); //I2C is already initialized above
}
#endif
