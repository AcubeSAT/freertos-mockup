#ifndef INC_TASKS_BLINKYTASK_H_
#define INC_TASKS_BLINKYTASK_H_

void vBlinkyTask(void *pvParameters);
void vSetupBlinky();

void vBlinkyFadeIn();
void vBlinkyFadeOut();

#define BLINKY_TASK_STACK_SIZE 200

extern TaskHandle_t xBlinkyHandle;
/* Structure that will hold the TCB of the task being created. */
extern StaticTask_t xBlinkyTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
extern StackType_t xBlinkyTaskStack[BLINKY_TASK_STACK_SIZE];
#endif /* INC_TASKS_BLINKYTASK_H_ */
