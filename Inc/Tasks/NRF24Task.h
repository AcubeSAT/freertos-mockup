#ifndef INC_TASKS_NRF24TASK_H_
#define INC_TASKS_NRF24TASK_H_

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#if SAT_Enable_NRF24
extern TaskHandle_t xReceiveTask;
extern TaskHandle_t xTransmitTask;

void vSetupNRF24();
void vTransmitTask(void *pvParameters);
void vReceiveTask(void *pvParameters);
void NRF24_RX_ISR();
#endif

#endif /* INC_TASKS_NRF24TASK_H_ */
