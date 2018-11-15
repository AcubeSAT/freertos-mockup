#ifndef INC_TASKS_NRF24TASK_H_
#define INC_TASKS_NRF24TASK_H_

#include <stdlib.h>
#include "stm32f1xx_ll_rcc.h"

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if SAT_Enable_NRF24
extern TaskHandle_t xReceiveTask;
extern SemaphoreHandle_t xnRF24Semaphore;

void vSetupNRF24();
void vTransmitTask(void *pvParameters);
void vReceiveTask(void *pvParameters);
void vTaskInfoTransmitTask(void *pvParameters);
void NRF24_RX_ISR();
#endif

#endif /* INC_TASKS_NRF24TASK_H_ */
