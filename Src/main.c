#include <stm32f1xx.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "uart.h"
#include "main.h"

#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_tim.h"

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "Tasks/BlinkyTask.h"
#include "Tasks/CheckTask.h"
#include "Tasks/DelayHelperTask.h"
#include "Tasks/NRF24Task.h"
#include "Tasks/SensorTask.h"
#include "Tasks/TraceTask.h"
#include "Tasks/UARTTask.h"
#include "Tasks/WWDGTask.h"

void prvSetupHardware();


int main(void){
	prvSetupHardware();
	UART_SendStr("CubeSAT booting up...\r\n");
	xI2CSemaphore = xSemaphoreCreateMutexStatic(&xI2CMutexBuffer);
	//xI2CSemaphore = xSemaphoreCreateMutex();
	xDataEventGroupHandle = xEventGroupCreateStatic(&xDataCreatedEventGroup);
	//xDataEventGroup = xEventGroupCreate();

	xCheckHandle1 = xTaskCreateStatic(vCheckTask, "Check", CHECK_TASK_STACK_SIZE, (void*) 1, 1, xCheckTaskStack, &xCheckTaskBuffer );
	xCheckHandle2 = xTaskCreateStatic(vCheckTask, "Check", CHECK_TASK_STACK_SIZE, (void*) 2, 8, xCheckTaskStack, &xCheckTaskBuffer );

	//xTaskCreate(vCheckTask, "Check", 200, (void*) 1, 1, NULL);
	//xTaskCreate(vCheckTask, "Check", 200, (void*) 2, 8, NULL);
#if SAT_Enable_Sensors
	xMPU9250Handle = xTaskCreateStatic(vMPU9250Task, "MPU9250", MPU9250_TASK_STACK_SIZE,NULL, 4,xMPU9250TaskStack, &xMPU9250TaskBuffer);
	//xTaskCreate(vMPU9250Task, "MPU9250", 400, NULL, 4, NULL);
	xBH1750Handle = xTaskCreateStatic(vBH1750Task, "BH1750", BH1750_TASK_STACK_SIZE,NULL, 4,xBH1750TaskStack, &xBH1750TaskBuffer);
	//xTaskCreate(vBH1750Task, "BH1750", 400, NULL, 4, NULL);
#endif

	xUARTHandle = xTaskCreateStatic(vUARTTask, "UART", UART_TASK_STACK_SIZE,NULL, 3,xUARTTaskStack, &xUARTTaskBuffer);
	//xTaskCreate(vUARTTask, "UART", 300, NULL, 3, NULL);
	xWWDGHandle = xTaskCreateStatic(vRefreshWWDGTask, "RefreshWWDG", REFRESHWWDG_TASK_STACK_SIZE,NULL, 6,xRefreshWWDGTaskStack, &xRefreshWWDGTaskBuffer);
	//xTaskCreate(vRefreshWWDGTask, "RefreshWWDG", 100, NULL, 6, NULL);
	xBlinkyHandle = xTaskCreateStatic(vBlinkyTask, "Blinky", BLINKY_TASK_STACK_SIZE,NULL, 3,xBlinkyTaskStack, &xBlinkyTaskBuffer);
	//xTaskCreate(vBlinkyTask, "Blinking", 200, NULL, 3, NULL);

#if SAT_Enable_NRF24
	xTransmitHandle = xTaskCreateStatic(vTransmitTask, "Transmit", TRANSMIT_TASK_STACK_SIZE,NULL, 2,xTransmitTaskStack, &xTransmitTaskBuffer);
//	xTaskCreate(vTransmitTask, "NRF_TX", 500, NULL, 2, NULL);
	xReceiveTask = xTaskCreateStatic(vReceiveTask, "Receive", RECEIVE_TASK_STACK_SIZE,NULL, 2,xReceiveTaskStack, &xReceiveTaskBuffer);

//	xTaskCreate(vReceiveTask, "NRF_RX", 500, NULL, 2, &xReceiveTask);

	xTaskInfoTransmitHandle = xTaskCreateStatic(vTaskInfoTransmitTask, "TaskInfoTransmit", TASKINFOTRANSMIT_TASK_STACK_SIZE,NULL, 1,xTaskInfoTransmitTaskStack, &xTaskInfoTransmitTaskBuffer);

 //	xTaskCreate(vTaskInfoTransmitTask, "NRF_TX_TaskInfo", 400, NULL, 1, NULL);

	xnRF24Semaphore = xSemaphoreCreateMutexStatic(&xnRF24MutexBuffer);
 //	xnRF24Semaphore = xSemaphoreCreateMutex();
#endif

	//xUARTQueue = xQueueCreate(45, sizeof(UARTMessage_t *));
 	xUARTQueue = xQueueCreateStatic(45, sizeof(UARTMessage_t *), 45*sizeof(UARTMessage_t *),&xStaticUARTQueue);

	osQueueUARTMessage("Hello world %d from FreeRTOS\r\n", xTaskGetTickCount());
	osQueueUARTMessage("Compiled at " __DATE__ " " __TIME__ "\r\n");
	vSetupWWDG();
	vDisableDelayHelper(); // Don't waste time on HAL_Delay
	vTaskStartScheduler();
}

void prvClkConfig() {
	/* Set FLASH latency */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

	/* Enable HSE oscillator */
	LL_RCC_HSE_Enable();
	while (LL_RCC_HSE_IsReady() != 1) {
	};

	/* Main PLL configuration and activation */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
	LL_RCC_PLL_Enable();
	while (LL_RCC_PLL_IsReady() != 1) {
	};

	/* Sysclk activation on the main PLL */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
	};

	/* Set APB1 & APB2 prescaler*/
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	/* Set systick to 1ms in using frequency set to 72MHz */
	LL_Init1msTick(72000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(72000000);
}

void prvSetupHardware() {
	// Initialize & configure the processor clock
	prvClkConfig();

	// Init interrupts necessary for FreeRTOS
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	// Make sure HAL_Delay() and HAL_GetTicks() are usable
	vSetupDelayHelper();

	// Initialise some clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// Initialise all other tasks
	vSetupUART();
#if SAT_Enable_NRF24
	vSetupNRF24();
#endif
#if SAT_Enable_Sensors
	vSetupSensors();
#endif
	vSetupBlinky();
	vSetupCheck();

	if (SAT_Enable_FreeRTOS_Trace) {
		vSetupTrace();
	}
}

