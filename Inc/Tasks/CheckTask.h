
#ifndef INC_TASKS_CHECKTASK_H_
#define INC_TASKS_CHECKTASK_H_

void vCheckTask(void *pvParameters);
void vSetupCheck();

extern TaskHandle_t xCheckHandle1;
extern TaskHandle_t xCheckHandle2;
#define CHECK_TASK_STACK_SIZE 200

/* Structure that will hold the TCB of the task being created. */
extern StaticTask_t xCheckTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
extern StackType_t xCheckTaskStack[ CHECK_TASK_STACK_SIZE ];

#endif /* INC_TASKS_CHECKTASK_H_ */
