#ifndef INC_TASKS_WWDGTASK_H_
#define INC_TASKS_WWDGTASK_H_

void vRefreshWWDGTask(void * pvParameters);
void vSetupWWDG();

extern TaskHandle_t xWWDGHandle;

#define REFRESHWWDG_TASK_STACK_SIZE 200

/* Structure that will hold the TCB of the task being created. */
extern StaticTask_t xRefreshWWDGTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
extern StackType_t xRefreshWWDGTaskStack[REFRESHWWDG_TASK_STACK_SIZE];
#endif /* INC_TASKS_WWDGTASK_H_ */
