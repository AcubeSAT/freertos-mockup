
#ifndef INC_TASKS_CHECKTASK_H_
#define INC_TASKS_CHECKTASK_H_

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"


void vCheckTask(void *pvParameters);
void vSetupCheck();
extern TaskHandle_t xCheckTask1;
extern TaskHandle_t xCheckTask2;

#endif /* INC_TASKS_CHECKTASK_H_ */
