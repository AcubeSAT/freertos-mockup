#ifndef INC_TASKS_UARTTASK_H_
#define INC_TASKS_UARTTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// TODO: Use dynamic allocation for message strings
typedef char * UARTMessage_t;
extern QueueHandle_t xUARTQueue;
extern TaskHandle_t xUARTTask;

void vSetupUART();
void osQueueUARTMessage(const char * format, ...);
void vUARTTask(void *pvParameters);

#endif /* INC_TASKS_UARTTASK_H_ */
