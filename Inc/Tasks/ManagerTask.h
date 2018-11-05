#ifndef INC_TASKS_MANAGERTASK_H_
#define INC_TASKS_MANAGERTASK_H_

void vReceiveSuspendTask(void *pvParameters);
void vResumeSuspendedTask(void *pvParameters);

extern TaskHandle_t xReceiveSuspendTask;

#endif /*INC_TASKS_MANAGERTASK_H_*/
