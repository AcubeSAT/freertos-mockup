#include <stm32f1xx.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "uart.h"
#include "main.h"
#include "nrf24.h"
#include "MPU6050.h"
#include "BH1750.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_hal_wwdg.h"

#define SAT_Acq_Period_ms 			50 // The acquisition period of the satellite

#define SAT_Serial_Debug 			0 // Whether to send debug data serially (for every data reception)
#define SAT_Enable_NRF24 			1 // Whether to enable NRF24L01+
#define SAT_Enable_Sensors 			1 // Whether to enable I2C sensors
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

float gyrCal[3]; //Save the calibration values

volatile uint8_t stopRX = 0; //Logic variable to indicate the stopping of the RX
uint8_t nRF24_payload[32]; //Buffer to store a payload of maximum width
WWDG_HandleTypeDef hwwdg;

LL_EXTI_InitTypeDef LL_EXTI_InitStructure;

struct SensorData_t {
	double brightness;
	float acc[6];
} xSensorData;

// TODO: Use dynamic allocation for message strings
typedef char * UARTMessage_t;
SemaphoreHandle_t xI2CSemaphore;
QueueHandle_t xUARTQueue;
EventGroupHandle_t xDataEventGroup; // Event group for reception of data from sensors
TaskHandle_t xReceiveTask;

// LED blinking flags
uint8_t blinkingEnabled = 1;
uint8_t blinkingFadingOut = 0;
uint8_t blinkingFadingIn = 0;

#define DATA_EVENT_GROUP_BH1750_Pos   0
#define DATA_EVENT_GROUP_MPU6050_Pos  1

#define DATA_EVENT_GROUP_BH1750_Msk  (1 << 0)
#define DATA_EVENT_GROUP_MPU6050_Msk (1 << 1)

void prvSetupHardware();

volatile unsigned long ulHighFrequencyTimerTicks;
WWDG_HandleTypeDef hwwdg;


void vRefreshWWDG( void * pvParameters )
 {
 TickType_t xLastWakeTime;
 hwwdg.Instance = WWDG;
 const TickType_t xFrequency = 80;

     xLastWakeTime = (uint32_t) 0;

     while(1)
     {
         vTaskDelayUntil( &xLastWakeTime, xFrequency );
         HAL_WWDG_Refresh(&hwwdg);
     }
 }
void osQueueUARTMessage(const char * format, ...)
{
	// TODO: Less copying around bits

	va_list arg;
	char buffer[128];

	va_start(arg, format);
	vsnprintf(buffer, 128, format, arg);
	va_end(arg);

	//configASSERT(strlen(message) < 127);

	UARTMessage_t pcUARTMessage = pvPortMalloc(strlen(buffer) + 1);

	if (pcUARTMessage == NULL) {
		UART_SendStr("ERROR! Not enough memory to store UART string\r\n");
	} else {
		strcpy(pcUARTMessage, buffer);

		// TODO: Show a warning if the queue is full (e.g. replace the last
		// message in the queue)
		if (xQueueSend(xUARTQueue, (void* ) (&pcUARTMessage),
				(TickType_t ) 0) == pdFAIL) {
			// Make sure to deallocate the failed message
			vPortFree(pcUARTMessage);
		}
	}
}

static void vBlinkyTask(void *pvParameters) {
	const float frequency = 0.0007;

	const float blinkStep = 1.10;
	//"exponential" rate of change in led intensity

	uint8_t blinkingActive = blinkingEnabled;

	uint32_t ticksDelta = 0;

	double value1 = 0, value2 = 0;

	while (1) {
		float ticks = xTaskGetTickCount() - ticksDelta;

		// Only spend time blinking when blinking is enabled
		if (blinkingFadingOut) {
			value1 = 1023 - (1023 - value1) / blinkStep;
			value2 = 1023 - (1023 - value2) / blinkStep;

			if (value1 >= 1020 && value2 >= 1020) {
				// Fadeout complete
				blinkingFadingOut = 0;

				value1 = value2 = 1023;

				// Set LEDS to Hi-Z
//				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT);
//				LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);
			}
		} else if (blinkingFadingIn) {
//			LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
//			LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);

			// Fade up
			value1 = 1023 - (1024 - value1) * 1.04;
			value2 = 1023 - (1024 - value2) * 1.04;

			if (value1 <= 707 && value2 <= 707) {
				// Fadein complete
				blinkingFadingIn = 0;
				ticksDelta = xTaskGetTickCount();
				value1 = value2 = 707;
			}
		} else if (blinkingEnabled) {
			value1 = 1023 * pow(sin(frequency * ticks) / 2.0 + 0.5, 0.5);
			value2 = 1023 * pow(sin(frequency * ticks * 1.1) / 2.0 + 0.5, 0.5);
		}

		TIM4->CCR3 = (int) value1;
		TIM4->CCR4 = (int) value2;

		vTaskDelay(pdMS_TO_TICKS(5));
	}
}
static void vUARTTask(void *pvParameters) {
	UARTMessage_t message;

	while (1) {
		if (xQueueReceive(xUARTQueue, &message, portMAX_DELAY)) { // Receive a message from the queue
			LL_USART_EnableDMAReq_TX(UART_PORT); // Enable DMA in the USART registers
			LL_DMA_SetDataLength(DMA1, LL_UART_DMA_CHAN_TX, strlen(message)); // Set amount of copied bits for DMA
			LL_DMA_ConfigAddresses(DMA1, LL_UART_DMA_CHAN_TX, message, &(UART_PORT->DR), LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // Send message from memory to the USART Data Register
			LL_DMA_EnableChannel(DMA1, LL_UART_DMA_CHAN_TX); // Enable the DMA transaction

			// Note: The following polling method doesn't make sense, since it wastes the processor speed.
			// An interrupt should be used instead.
			while (__HAL_DMA_GET_FLAG(&dma, __HAL_DMA_GET_TC_FLAG_INDEX(&dma)) == RESET) {} // Wait until the transaction is complete
			__HAL_DMA_CLEAR_FLAG(&dma, __HAL_DMA_GET_TC_FLAG_INDEX(&dma)); // Clear the Transaction Complete flag so it can be used later

			LL_DMA_DisableChannel(DMA1, LL_UART_DMA_CHAN_TX); // Disable the DMA channel (this is necessary, so it can be reused later)
			LL_USART_DisableDMAReq_TX(UART_PORT); // Disable DMA in the USART registers
			vPortFree(message); // Free up the memory held by the message string
		}
	}
}

#if SAT_Enable_Sensors
static void vBH1750Task(void *pvParameters)
{
	// Store the last wake time so that we can delay later
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1) {
		if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
			UART_SendStr("FATAL Error: I2C timeout");
			vTaskSuspend(NULL); // Stop this task
		} else {
			xSensorData.brightness = BH1750_GetBrightnessCont();
			xSemaphoreGive(xI2CSemaphore);
			xEventGroupSetBits(xDataEventGroup, DATA_EVENT_GROUP_BH1750_Msk);

			if (SAT_Serial_Debug) osQueueUARTMessage("bri %f\r\n", xSensorData.brightness);
		}

		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( SAT_Acq_Period_ms ) );
	}
}

static void vMPU6050Task(void *pvParameters)
{
	// Store the last wake time so that we can delay later
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while(1) {
		if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
			UART_SendStr("FATAL Error: I2C timeout");
			vTaskSuspend(NULL); // Stop this task
		} else {
			MPU6050_GetCalibAccelGyro(xSensorData.acc, gyrCal);
			xSemaphoreGive(xI2CSemaphore);

			xEventGroupSetBits(xDataEventGroup, DATA_EVENT_GROUP_MPU6050_Msk);

			if (SAT_Serial_Debug) {
				osQueueUARTMessage("acc dump %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
						100 * xSensorData.acc[0],
						100 * xSensorData.acc[1],
						100 * xSensorData.acc[2],
						100 * xSensorData.acc[3],
						100 * xSensorData.acc[4],
						100 * xSensorData.acc[5]);
			}
		}

		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( SAT_Acq_Period_ms ) );
	}
}
#endif

static void vCheckTask(void *pvParameters) {
	uint8_t value = (uint8_t) pvParameters;

	for (;;) {
		osQueueUARTMessage("%d SystemGood %d \r\n", value, xTaskGetTickCount());
		//taskYIELD();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

#if SAT_Enable_NRF24
static void vTransmitTask(void *pvParameters)
{
	  memset((uint8_t *)nRF24_payload, '\0', 32); //Fill all the array space with zeros
          sprintf((char *)nRF24_payload, "%s", "S0");
          nRF24_TransmitPacket(nRF24_payload, 32);

	while (1) {

		if (xEventGroupWaitBits(
						xDataEventGroup,
						DATA_EVENT_GROUP_BH1750_Msk | DATA_EVENT_GROUP_MPU6050_Msk,
						pdTRUE,
						pdTRUE,
						portMAX_DELAY)) {
			nRF24_SetOperationalMode(nRF24_MODE_TX); //Set operational mode (PTX == transmitter)
			nRF24_ClearIRQFlags(); //Clear any pending IRQ flags
			GPIOC->BSRR = 1 << 13;
			memset((uint8_t *)nRF24_payload, '\0', 32); //Fill all the array space with zeros
			sprintf((char *)nRF24_payload, "B%.2f", xSensorData.brightness);
			nRF24_TransmitPacket(nRF24_payload, 32);

			/*memset((uint8_t *)nRF24_payload, '\0', 32); //Fill all the array space with zeros
			 sprintf((char *)nRF24_payload, "B%.2f %.2f %.2f %.2f", q0, q1, q2, q3);
			 nRF24_TransmitPacket(nRF24_payload, 32);*/

			memset((uint8_t *)nRF24_payload, '\0', 32);//Fill all the array space with zeros
			sprintf((char *)nRF24_payload, "X%ld %ld", (int32_t)(xSensorData.acc[0]*100000.0), (int32_t)(xSensorData.acc[3]*100000.0));
			nRF24_TransmitPacket(nRF24_payload, 32);

			memset((uint8_t *)nRF24_payload, '\0', 32);//Fill all the array space with zeros
			sprintf((char *)nRF24_payload, "Y%ld %ld", (int32_t)(xSensorData.acc[1]*100000.0), (int32_t)(xSensorData.acc[4]*100000.0));
			nRF24_TransmitPacket(nRF24_payload, 32);

			memset((uint8_t *)nRF24_payload, '\0', 32);//Fill all the array space with zeros
			sprintf((char *)nRF24_payload, "Z%ld %ld", (int32_t)(xSensorData.acc[2]*100000.0), (int32_t)(xSensorData.acc[5]*100000.0));
			nRF24_TransmitPacket(nRF24_payload, 32);

			GPIOC->BRR = 1 << 13;

			// Prepare the sensor for receiving the data while sleeping
			nRF24_SetOperationalMode(nRF24_MODE_RX); //Set operational mode (PRX == receiver)
			nRF24_CE_H();
		}
	}
}

static void vReceiveNRFTask(void *pvParameters)
{
	while (1)
	{
		if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY))
		{
			uint8_t payload_length; //Length of received payload
			char* tokenCh = NULL; //Save the tokenized string

			nRF24_SetOperationalMode(nRF24_MODE_RX); //Set operational mode (PRX == receiver)
			nRF24_CE_H();
			if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
			{
				nRF24_ReadPayload(nRF24_payload, &payload_length); //Get the payload from the transceiver
				nRF24_ClearIRQFlags(); //Clear all pending IRQ flags

				tokenCh = strtok((char*)nRF24_payload, ":");
				if(strstr(tokenCh, "L1"))
				{
					tokenCh = strtok (NULL, ":");
					if(strstr(tokenCh, "1"))
					{
						osQueueUARTMessage("->>  Received +1 command\r\n");
						blinkingEnabled = 1;
						blinkingFadingIn = 1;
					}
					else if(strstr(tokenCh, "0"))
					{
						osQueueUARTMessage("->>  Received  0 command\r\n");
						blinkingEnabled = 0;
						blinkingFadingOut = 1;
					}
					else
					{
						osQueueUARTMessage("->>  Received content: %s", nRF24_payload);
					}
				}
			}
		}
	}
}
#endif

static  void WWDGInit(void)
{
  //PCLK1/4096 = 24MHz/4096 = 5859.375 hz
  //5859.375/WWDG_PRESCALER_8= 732.421875 hz
  // 124 - 63 = 65 65/732.421875 = 88.7ms max time
  // 80 - 63 = 17 17/732.421875 = 22ms min time
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 80;
  hwwdg.Init.Counter = 124;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  HAL_WWDG_Init(&hwwdg);

  __HAL_RCC_WWDG_CLK_ENABLE();
}

int main(void)
{

	prvSetupHardware();
	UART_SendStr("CubeSAT booting up...\r\n");
	//vCheckTask(0);

	xI2CSemaphore = xSemaphoreCreateMutex();
	xDataEventGroup = xEventGroupCreate();

	xTaskCreate(vCheckTask, "Check", 250, (void*) 1, 2, NULL);
	xTaskCreate(vCheckTask, "Check", 250, (void*) 2, 2, NULL);
	if (SAT_Enable_Sensors) {
		xTaskCreate(vMPU6050Task, "MPU6050", 400, NULL, 4, NULL);
		xTaskCreate(vBH1750Task, "BH1750", 400, NULL, 4, NULL);
	}

	xTaskCreate(vUARTTask, "UART", 300, NULL, 3, NULL);
    xTaskCreate(vRefreshWWDG, "RefreshWWDG", 200, NULL, 6, NULL);
    xTaskCreate(vBlinkyTask, "Blinking", 100, NULL, 3, NULL);

    if (SAT_Enable_NRF24)
    {
        xTaskCreate(vTransmitTask, "Transmit", 600, NULL, 1, NULL);
        xTaskCreate(vReceiveNRFTask, "NRF_RX", 600, NULL, 1, NULL);
    }

	xUARTQueue = xQueueCreate(45, sizeof(UARTMessage_t *));

	osQueueUARTMessage("Hello world %d from FreeRTOS\r\n", xTaskGetTickCount());
	StartupEffect();
	WWDGInit();
	vTaskStartScheduler();
}

void prvClkConfig() {
	/* Set FLASH latency */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

	/* Enable HSE oscillator */
//	LL_RCC_HSE_EnableBypass();
	LL_RCC_HSE_Enable();
	while (LL_RCC_HSE_IsReady() != 1) {};

	/* Main PLL configuration and activation */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
	LL_RCC_PLL_Enable();
	while (LL_RCC_PLL_IsReady() != 1) {};

	/* Sysclk activation on the main PLL */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {};

	/* Set APB1 & APB2 prescaler*/
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	/* Set systick to 1ms in using frequency set to 72MHz */
	LL_Init1msTick(72000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(72000000);
}

void prvSetupHardware() 
{
    LL_EXTI_InitTypeDef EXTI_InitStruct;
    
	// Initialize & configure the processor clock
	prvClkConfig();

	// Init interrupts necessary for FreeRTOS
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    
    // NRF24 Interrupt pin initialization
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    
    // Initialize the interrupt pin
    LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE2);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_FLOATING);

    /* EXTI interrupt init*/
    NVIC_SetPriority(EXTI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 15));
    NVIC_EnableIRQ(EXTI2_IRQn);

    __HAL_RCC_GPIOA_CLK_ENABLE();

	//LED Pins Init
	__HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock of GPIO-A
	__HAL_RCC_GPIOB_CLK_ENABLE(); // Enable clock of GPIO-B
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable clock of GPIO-C
	GPIO_InitTypeDef GPIO;
	GPIO.Pin = GPIO_PIN_8 | GPIO_PIN_9; // Bright LED pins
	GPIO.Mode = GPIO_MODE_AF_PP;
	GPIO.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO);
	GPIO.Pin = GPIO_PIN_13; // Indicator LED pin
	GPIO.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO);
	__HAL_AFIO_REMAP_TIM3_ENABLE(); // Enable remap for Timer 3 pins


	// LED Timer Initialization (for PWM)
	__HAL_RCC_TIM4_CLK_ENABLE();
	TIM_HandleTypeDef tim; // Timer handle
	TIM_OC_InitTypeDef oC; // Output Channel handle
	tim.Instance = TIM4;
	tim.Init.Period = 1024;
	tim.Init.Prescaler = 64;
	tim.Init.ClockDivision = 0;
	tim.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_PWM_Init(&tim);
	oC.OCMode = TIM_OCMODE_PWM1;
	oC.OCPolarity = TIM_OCPOLARITY_HIGH;
	oC.Pulse = 900;
	HAL_TIM_PWM_ConfigChannel(&tim, &oC, TIM_CHANNEL_3);
	oC.Pulse = 200;
	HAL_TIM_PWM_ConfigChannel(&tim, &oC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&tim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&tim, TIM_CHANNEL_4);

//	Delay_Init(); // Don't initialise the delay, prvClkConfig()
				  // takes care of that for us already
	UART_Init(115200); //Initialize the UART with the set baud rate
	UART_SendStr("CubeSAT hardware initialization...\r\n");

	// DMA (Direct Memory Access) initialisation
	__HAL_RCC_DMA1_CLK_ENABLE();
	dma.Instance = UART_DMA_CHAN_TX; // DMA channel for UART
	dma.Init.Direction = DMA_MEMORY_TO_PERIPH; // Transfer data from memory to peripheral
	dma.Init.PeriphInc = DMA_PINC_DISABLE; // Disable incrementing a pointer
	dma.Init.MemInc = DMA_MINC_ENABLE; // Disable incrementing a pointer
	dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // Transfer each byte
	dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    // Transfer each byte
	dma.Init.Mode = DMA_NORMAL;
	dma.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&dma);
//	__HAL_LINKDMA(huart,hdmatx,dma);

	if (SAT_Enable_Sensors) {
		MPU6050_I2C_Init(); //Initialize I2C

		MPU6050_Initialize(); //Initialize the MPU6050
		MPU6050_GyroCalib(gyrCal); //Get the gyroscope calibration values
		MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000); //Set the gyroscope scale to full scale
		MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2); //Set the accelerometer scale

		BH1750_Init(BH1750_CONTHRES); //I2C is already initialized above
	}

	if (SAT_Enable_NRF24) {

		nRF24_GPIO_Init(); //Start the pins used by the NRF24
		nRF24_Init(); //Initialize the nRF24L01 to its default state

		nRF24_CE_L(); //RX/TX disabled

		//A small check for debugging
		UART_SendStr("nRF24L01+ check: ");
		if (!nRF24_Check()) {
			UART_SendStr("FAIL\r\n");
			NVIC_SystemReset();
			while (1)
				;
		} else {
			UART_SendStr("OK\r\n");
		}

		// This is simple transmitter with Enhanced ShockBurst (to one logic address):
		//   - TX address: 'ESB'
		//   - payload: 10 bytes
		//   - RF channel: 40 (2440MHz)
		//   - data rate: 2Mbps
		//   - CRC scheme: 2 byte
		nRF24_SetRFChannel(99); //Set RF channel
		nRF24_SetDataRate(nRF24_DR_2Mbps); //Set data rate
		nRF24_SetCRCScheme(nRF24_CRC_2byte); //Set CRC scheme
		nRF24_SetAddrWidth(5); //Set address width, its common for all pipes (RX and TX)

		//Configure TX PIPE
		static const uint8_t nRF24_ADDR_Tx[] = { 'B', 'a', 's', 'e', 'S' }; //Address of the receiving end
		static const uint8_t nRF24_ADDR_Rx[] = { 'C', 'u', 'b', 'e', 'S' }; //Address of the current module
		nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR_Tx); //Program TX address
		nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR_Tx); //Program address for pipe#0, must be same as TX (for Auto-ACK)

		//Configure RX PIPE
		nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR_Rx); //Program address for pipe
		nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 32); // Auto-ACK: enabled, payload length: 32 bytes

		nRF24_SetTXPower(nRF24_TXPWR_0dBm);	//Set TX power (maximum)
		nRF24_SetAutoRetr(nRF24_ARD_2500us, 10); //Configure auto retransmit: 10 retransmissions with pause of 2500s in between
		nRF24_EnableAA(nRF24_PIPE0); //Enable Auto-ACK for pipe#0 (for ACK packets)

		nRF24_SetOperationalMode(nRF24_MODE_TX); //Set operational mode (PTX == transmitter)
		nRF24_ClearIRQFlags(); //Clear any pending IRQ flags
		nRF24_SetPowerMode(nRF24_PWR_UP); //Wake the transceiver
	}

	if (SAT_Enable_FreeRTOS_Trace) {
		// Set interrupt priority and enable TIMER1 interrupt in NVIC
		HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

		// Clock the TIMER1 peripheral
		__HAL_RCC_TIM1_CLK_ENABLE();

		TIM_HandleTypeDef tim; // Timer handle
		tim.Instance = TIM1;
		tim.Init.Period = 1800;
		tim.Init.Prescaler = 1;
		tim.Init.CounterMode = TIM_COUNTERMODE_UP;
		HAL_TIM_Base_Init(&tim);
		HAL_TIM_Base_Start_IT(&tim);

		ulHighFrequencyTimerTicks = 0;
	}
}

void TIM1_UP_IRQHandler(void)
{
    /* Clear the pending bit in NVIC and TIMER1 */
	HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
	LL_TIM_ClearFlag_UPDATE(TIM1);

    /* Increment the counter used to measure execution time */
    ulHighFrequencyTimerTicks++;
}

void NRF24_RX_ISR(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	vTaskNotifyGiveFromISR(xReceiveTask, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Manual_Delay( uint32_t ms ) {
	if (ms == 0) return;

    uint32_t l = 72000/3 * ms;
    asm volatile( "0:" "SUBS %[count], 1;" "BNE 0b;" :[count]"+r"(l) );
}

void StartupEffect(){
    int a;
 	for( a = 0; a < 20; a++){
 		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
 		Manual_Delay(a);
 	}
}

