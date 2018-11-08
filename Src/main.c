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
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_bus.h"

#include "MockupConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "Tasks/BlinkyTask.h"
#include "Tasks/CheckTask.h"
#include "Tasks/NRF24Task.h"
#include "Tasks/SensorTask.h"
#include "Tasks/TraceTask.h"
#include "Tasks/UARTTask.h"
#include "Tasks/WWDGTask.h"
#include "Tasks/RTCTask.h"

void prvSetupHardware();

int main(void) {
	prvSetupHardware();
	UART_SendStr("CubeSAT booting up...\r\n");

	xI2CSemaphore = xSemaphoreCreateMutex();
	xDataEventGroup = xEventGroupCreate();

	xTaskCreate(vCheckTask, "Check", 250, (void*) 1, 1, NULL);
	xTaskCreate(vCheckTask, "Check", 250, (void*) 2, 8, NULL);
#if SAT_Enable_Sensors
	xTaskCreate(vMPU9250Task, "MPU9250", 400, NULL, 4, NULL);
	xTaskCreate(vBH1750Task, "BH1750", 400, NULL, 4, NULL);
#endif

	xTaskCreate(vUARTTask, "UART", 300, NULL, 3, NULL);
	xTaskCreate(vRefreshWWDGTask, "RefreshWWDG", 200, NULL, 6, NULL);
	xTaskCreate(vBlinkyTask, "Blinking", 200, NULL, 3, NULL);
	xTaskCreate(vRTCTask, "RTC", 200, NULL, 3, NULL);

#if SAT_Enable_NRF24
	xTaskCreate(vTransmitTask, "NRF_TX", 500, NULL, 1, NULL);
	xTaskCreate(vReceiveTask, "NRF_RX", 400, NULL, 1, &xReceiveTask);
#endif

	xUARTQueue = xQueueCreate(45, sizeof(UARTMessage_t *));

	osQueueUARTMessage("Hello world %d from FreeRTOS\r\n", xTaskGetTickCount());
	vSetupWWDG();
	vTaskStartScheduler();
}

void prvClkConfig() {
	/* Set FLASH latency */
	/* How many clock cycles should SYSCLK to access flash memory */
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

	/* Enable LSE oscillator */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableBkUpAccess();
	LL_RCC_ForceBackupDomainReset();
	LL_RCC_ReleaseBackupDomainReset();
	LL_RCC_LSE_Enable();
	while (LL_RCC_LSE_IsReady() != 1) {
	};

}

void prvSetupHardware() {
	// Initialize & configure the processor clock
	prvClkConfig();

	// Init interrupts necessary for FreeRTOS
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	//	Delay_Init(); // Don't initialise the delay, prvClkConfig()
	// takes care of that for us already

	// Initialise some clocks
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;

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

	vSetUpRTC();
	vRTCInit();

	if (SAT_Enable_FreeRTOS_Trace) {
		vSetupTrace();
	}
}

