#ifndef INC_TASKS_BLINKYTASK_H_
#define INC_TASKS_BLINKYTASK_H_

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"


void vBlinkyTask(void *pvParameters);
void vSetupBlinky();

void vBlinkyFadeIn();
void vBlinkyFadeOut();

extern TaskHandle_t xBlinkyTask;


#endif /* INC_TASKS_BLINKYTASK_H_ */
