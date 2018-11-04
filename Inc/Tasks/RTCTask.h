#ifndef INC_TASKS_RTCTASK_H_
#define INC_TASKS_RTCTASK_H_


void vSetUpRTC();	//RTC clock configuration
void vRTCTask(void *pvParameters);

void vRTCInit();		//RTC initialization


#endif /* INC_TASKS_RTCTASK_H_ */
