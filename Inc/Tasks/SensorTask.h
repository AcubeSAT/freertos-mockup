#ifndef INC_TASKS_SENSORTASK_H_
#define INC_TASKS_SENSORTASK_H_

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
extern TaskHandle_t xMPU9250Task;

/**
 * Sensor data holder
 */
typedef struct {
	double brightness;
	double acc[3];  // Readings from the accelerometer
	double gyr[3];  // Readings from the gyroscope
	float magn[3];  // Readings of the magnetometer
	float magn_adj[3];  // Magnetometer adjustment values
} SensorData_t;

extern SensorData_t xSensorData;

/**
 * Sensor event groups
 */
EventGroupHandle_t xDataEventGroup; // Event group for reception of data from sensors
#define DATA_EVENT_GROUP_BH1750_Pos   0
#define DATA_EVENT_GROUP_MPU9250_Pos  1

#define DATA_EVENT_GROUP_BH1750_Msk  (1 << 0)
#define DATA_EVENT_GROUP_MPU9250_Msk (1 << 1)

/**
 * Sensor I2C semaphore
 */
SemaphoreHandle_t xI2CSemaphore;

/**
 * Sensor tasks & functions
 */
#if SAT_Enable_Sensors
void vBH1750Task(void *pvParameters);
void vMPU9250Task(void *pvParameters);
void vSetupSensors();
#endif

#endif /* INC_TASKS_SENSORTASK_H_ */
