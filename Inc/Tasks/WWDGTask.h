#ifndef INC_TASKS_WWDGTASK_H_
#define INC_TASKS_WWDGTASK_H_

void vRefreshWWDGTask(void * pvParameters);
void vSetupWWDG();
extern TaskHandle_t xRefreshWWDGTask;


#endif /* INC_TASKS_WWDGTASK_H_ */
