#include <stm32f10x.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "uart.h"
#include "delay.h"
#include "nrf24.h"
#include "MPU6050.h"
#include "BH1750.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

float gyrCal[3]; //Save the calibration values

volatile uint8_t stopRX = 0; //Logic variable to indicate the stopping of the RX

// TODO: Use dynamic allocation for message strings
typedef char * UARTMessage_t;
SemaphoreHandle_t xI2CSemaphore;
QueueHandle_t xUARTQueue;

void prvSetupHardware();

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
		UART_SendStr("ERROR! Not enough memory to store UART string");
	} else {
		strcpy(pcUARTMessage, buffer);
		
		// TODO: Show a warning if the queue is full (e.g. replace the last
		// message in the queue)
		if (xQueueSend(xUARTQueue, (void*) (&pcUARTMessage), (TickType_t) 0) == pdFAIL) {
			// Make sure to deallocate the failed message
			vPortFree(pcUARTMessage);
		}
	}
}


static void vBlinkyTask(void *pvParameters)
{
	const float frequency = 0.0007;
	
  while(1) {
		float ticks = xTaskGetTickCount();
		
		//double value1 = 1023 * pow(sin(frequency * ticks)/2.0 + 0.5, 0.5);
		//double value2 = 1023 * pow(sin(frequency * ticks + 2)/2.0 + 0.5, 0.5);
		
		double value1 = 1023 * pow(sin(frequency * ticks)/2.0 + 0.5, 1.5);
		double value2 = 1023 * pow(sin(frequency * ticks * 1.1)/2.0 + 0.5, 1.5);
		
		TIM4->CCR3 = (int) value1;
		TIM4->CCR4 = (int) value2;
		
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
static void vUARTTask(void *pvParameters)
{
	UARTMessage_t message;
	
	while (1) {
		if (xQueueReceive(xUARTQueue, &message, portMAX_DELAY)) {
			UART_SendStr(message);
			vPortFree(message);
		}
	}
}
static void vBH1750Task(void *pvParameters)
{
	// Store the last wake time so that we can delay later
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	while(1) {
		double bright;
		if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
			UART_SendStr("FATAL Error: I2C timeout");
			vTaskSuspend(NULL); // Stop this task
		}	else {
			bright = BH1750_GetBrightnessCont();
			xSemaphoreGive(xI2CSemaphore);
			osQueueUARTMessage("I got a value from the brightness sensor! So awesome! %f\r\n", 10 * bright);
		}
		
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
	}
}

static void vMPU6050Task(void *pvParameters)
{
	// Store the last wake time so that we can delay later
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	float acgrData[6];
	
	while(1) {
		if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(250)) == pdFALSE) {
			UART_SendStr("FATAL Error: I2C timeout");
			vTaskSuspend(NULL); // Stop this task
		} else {
			MPU6050_GetCalibAccelGyro(acgrData, gyrCal); 
			xSemaphoreGive(xI2CSemaphore);

			osQueueUARTMessage("I got a value from the acceleration sensor! So awesome! %f %f %f\r\n",
				100 * acgrData[0],
				100 * acgrData[1],
				100 * acgrData[2]);
		}
		
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
	}
}


static void vCheckTask( void *pvParameters )
{
	uint8_t value = (uint8_t) pvParameters;
	
    for( ;; )
	{
		UART_SendStr("SystemGood ");
		UART_SendInt(value);
		UART_SendStr("\r\n");
		taskYIELD();
	}
}

int main(void)
{

	prvSetupHardware();
	UART_Init(115200);
	UART_SendStr("CubeSAT booting up...\r\n");
	//vCheckTask(0);
	
	xI2CSemaphore = xSemaphoreCreateMutex();
	
	//xTaskCreate( vCheckTask, "Check", 100, (void*)1, 2, NULL );
	//xTaskCreate( vCheckTask, "Check", 100, (void*)2, 2, NULL );
	xTaskCreate(vBH1750Task, "BH1750", 500, NULL, 4, NULL);
	xTaskCreate(vMPU6050Task, "MPU6050", 500, NULL, 4, NULL);
	xTaskCreate(vUARTTask, "UART", 200, NULL, 3, NULL);
	xTaskCreate(vBlinkyTask, "LEDFade", 100, NULL, 2, NULL);
	
	xUARTQueue = xQueueCreate( 15, sizeof( UARTMessage_t * ) );
	
	osQueueUARTMessage("Hello world %d from FreeRTOS\r\n", 17);
	
	vTaskStartScheduler();
}

void prvSetupHardware()
{
	// Init interrupts necessary for FreeRTOS
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
	
	uint8_t payload_length; //Length of received payload
	char* tokenCh = NULL; //Save the tokenized string
	
	//LED Pins Init
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN; //Enabling the clock of C pins.
	GPIOB->CRH |= GPIO_CRH_MODE8|GPIO_CRH_MODE9; //Resetting the bits of the register besides the last 4.
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);	
	
	
	//Timer Initialization
	//TODO make the timer for 50ms
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Enable Timer 3 clock
	//RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM3->PSC = 18000; //Set prescaler to 18000 (ticks)
	//TIM4->PSC = 18000;
	TIM3->ARR = 150; //Set auto reload to aprrox. 75 (ms)
	//TIM4->ARR = 250; //Around 500ms
	
	// LED Timer Initialization
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseStructure.TIM_Period = 1024;
  TIM_TimeBaseStructure.TIM_Prescaler = 64;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 900;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1000;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	// Enable remapping
	AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP;
	AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_FULLREMAP;
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
	

//	NVIC_EnableIRQ(TIM3_IRQn); //Enable Timer 3 interrupt
//	TIM3->DIER = TIM_DIER_UIE; //Enable Timer 3 interrupt
	
	Delay_Init();
	UART_Init(115200); //Initialize the UART with the set baud rate
		
	MPU6050_I2C_Init(); //Initialize I2C
	MPU6050_Initialize(); //Initialize the MPU6050
	MPU6050_GyroCalib(gyrCal); //Get the gyroscope calibration values
	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000); //Set the gyroscope scale to full scale
	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2); //Set the accelerometer scale
	
	BH1750_Init(BH1750_CONTHRES); //I2C is already initialized above
			/*
	nRF24_GPIO_Init(); //Start the pins used by the NRF24
	nRF24_Init(); //Initialize the nRF24L01 to its default state
	
	nRF24_CE_L(); //RX/TX disabled

	//A small check for debugging
	UART_SendStr("nRF24L01+ check: ");
	if (!nRF24_Check()) 
	{
		UART_SendStr("FAIL\r\n");
		while (1);
	}
	else
	{UART_SendStr("OK\r\n");}
	
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
	*/
}



