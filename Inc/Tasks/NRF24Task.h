#ifndef INC_TASKS_NRF24TASK_H_
#define INC_TASKS_NRF24TASK_H_

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if SAT_Enable_NRF24
extern TaskHandle_t xReceiveTask;
extern TaskHandle_t xTransmitHandle;
extern TaskHandle_t xTaskInfoTransmitHandle;
extern SemaphoreHandle_t xnRF24Semaphore;
extern StaticSemaphore_t xnRF24MutexBuffer;

void vSetupNRF24();
void vTransmitTask(void *pvParameters);
void vReceiveTask(void *pvParameters);
void vTaskInfoTransmitTask(void *pvParameters);
void NRF24_RX_ISR();


#define TRANSMIT_TASK_STACK_SIZE 500
#define RECEIVE_TASK_STACK_SIZE 500
#define TASKINFOTRANSMIT_TASK_STACK_SIZE 400

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
extern StaticTask_t xTransmitTaskBuffer;
extern StaticTask_t xReceiveTaskBuffer;
extern StaticTask_t xTaskInfoTransmitTaskBuffer;

/* Structure that will hold the TCB of the task being created. */

extern StackType_t xTransmitTaskStack[ TRANSMIT_TASK_STACK_SIZE ];
extern StackType_t xReceiveTaskStack[RECEIVE_TASK_STACK_SIZE];
extern StackType_t xTaskInfoTransmitTaskStack[TASKINFOTRANSMIT_TASK_STACK_SIZE];

#endif

#endif /* INC_TASKS_NRF24TASK_H_ */
