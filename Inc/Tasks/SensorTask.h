#ifndef INC_TASKS_SENSORTASK_H_
#define INC_TASKS_SENSORTASK_H_

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

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
extern EventGroupHandle_t xDataEventGroupHandle; // Event group for reception of data from sensors
/* Declare a variable to hold the data associated with the created
  event group. */
extern StaticEventGroup_t xDataCreatedEventGroup;

#define DATA_EVENT_GROUP_BH1750_Pos   0
#define DATA_EVENT_GROUP_MPU9250_Pos  1

#define DATA_EVENT_GROUP_BH1750_Msk  (1 << 0)
#define DATA_EVENT_GROUP_MPU9250_Msk (1 << 1)

/**
 * Sensor I2C semaphore
 */
extern SemaphoreHandle_t xI2CSemaphore;
extern StaticSemaphore_t xI2CMutexBuffer;
/**
 * Sensor tasks & functions
 */
#if SAT_Enable_Sensors
void vBH1750Task(void *pvParameters);
void vMPU9250Task(void *pvParameters);
void vSetupSensors();

extern TaskHandle_t xBH1750Handle;
extern TaskHandle_t xMPU9250Handle;

#define BH1750_TASK_STACK_SIZE 400
#define MPU9250_TASK_STACK_SIZE 400

extern StaticTask_t xBH1750TaskBuffer;
extern StaticTask_t xMPU9250TaskBuffer;

extern StackType_t xBH1750TaskStack[BH1750_TASK_STACK_SIZE];
extern StackType_t xMPU9250TaskStack[MPU9250_TASK_STACK_SIZE];
#endif

#endif /* INC_TASKS_SENSORTASK_H_ */
