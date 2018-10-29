#ifndef INC_TASKS_UARTTASK_H_
#define INC_TASKS_UARTTASK_H_

#include "FreeRTOS.h"
#include "queue.h"

// TODO: Use dynamic allocation for message strings
typedef char * UARTMessage_t;
extern QueueHandle_t xUARTQueue;

void vSetupUART();
void osQueueUARTMessage(const char * format, ...);
void vUARTTask(void *pvParameters);

#endif /* INC_TASKS_UARTTASK_H_ */
