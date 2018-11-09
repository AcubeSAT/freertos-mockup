#ifndef INC_TASKS_UARTTASK_H_
#define INC_TASKS_UARTTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
// TODO: Use dynamic allocation for message strings
typedef char * UARTMessage_t;
extern QueueHandle_t xUARTQueue;
extern StaticQueue_t xStaticUARTQueue;
extern TaskHandle_t xUARTHandle;

void vSetupUART();
void osQueueUARTMessage(const char * format, ...);
void vUARTTask(void *pvParameters);
#define UART_TASK_STACK_SIZE 300

/* Structure that will hold the TCB of the task being created. */
extern StaticTask_t xUARTTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
extern StackType_t xUARTTaskStack[UART_TASK_STACK_SIZE];

#endif /* INC_TASKS_UARTTASK_H_ */
