#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "Tasks/UARTTask.h"
#include "stm32f1xx_ll_tim.h"
#include "Tasks/BlinkyTask.h"

// LED blinking flags
uint8_t blinkingEnabled = 1;
uint8_t blinkingFadingOut = 0;
uint8_t blinkingFadingIn = 0;

TaskHandle_t xBlinkyHandle = NULL;

StaticTask_t xBlinkyTaskBuffer;
StackType_t xBlinkyTaskStack[BLINKY_TASK_STACK_SIZE];

/**
 * A task that makes LEDs blink
 */
void vBlinkyTask(void *pvParameters) {
	const float frequency = 0.0007;

	const float blinkStep = 1.10;
	//"exponential" rate of change in led intensity

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

		LL_TIM_OC_SetCompareCH3(TIM4, (uint32_t) value1);
		LL_TIM_OC_SetCompareCH4(TIM4, (uint32_t) value2);

		vTaskDelay(pdMS_TO_TICKS(4));
	}
}

void vSetupBlinky() {
	//LED Pins Init
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable clock of GPIO-C

	GPIO_InitTypeDef GPIO;
	GPIO.Pin = GPIO_PIN_8 | GPIO_PIN_9; // Bright LED pins
	GPIO.Mode = GPIO_MODE_AF_PP;
	GPIO.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO);

	// LED Timer Initialization (for PWM)
	__HAL_RCC_TIM4_CLK_ENABLE();
	TIM_HandleTypeDef tim; // Timer handle
	TIM_OC_InitTypeDef oC; // Output Channel handle
	tim.Instance = TIM4;
	tim.Init.Period = 1024;
	tim.Init.Prescaler = 24;
	tim.Init.ClockDivision = 0;
	tim.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&tim);
	oC.OCMode = TIM_OCMODE_PWM1;
	oC.OCPolarity = TIM_OCPOLARITY_HIGH;
	oC.Pulse = 900;
	HAL_TIM_PWM_ConfigChannel(&tim, &oC, TIM_CHANNEL_3);
	oC.Pulse = 200;
	HAL_TIM_PWM_ConfigChannel(&tim, &oC, TIM_CHANNEL_4);

	// Workaround for an issue with PWM requiring a reset
	// of the microcontroller, since this set doesn't take
	// place on the HAL functions
	TIM4->CCMR2 &= ~TIM_CCMR2_CC3S & ~TIM_CCMR2_CC4S;

	HAL_TIM_PWM_Start(&tim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&tim, TIM_CHANNEL_4);
}

/**
 * Fade in (enable) the LEDs
 */
void vBlinkyFadeIn() {
	blinkingEnabled = 1;
	blinkingFadingIn = 1;
}

/**
 * Fade out (disable) the LEDs
 */
void vBlinkyFadeOut() {
	blinkingEnabled = 0;
	blinkingFadingOut = 1;
}
