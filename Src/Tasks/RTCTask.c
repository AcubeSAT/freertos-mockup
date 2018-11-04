#include "Tasks/RTCTask.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc_ex.h"
#include "stm32f1xx_hal_rtc.h"
#include "Tasks/UARTTask.h"


//#include "stm32f1xx_ll_rcc.h"
//#include "stm32f1xx_ll_pwr.h"
//#include "stm32f1xx_ll_rtc.h"


/*
 * A task that create a calendar and send time each second
 */


RTC_HandleTypeDef hrtc;	//contains the configuration information for RTC
RTC_TimeTypeDef sTime;	//time info
RTC_DateTypeDef DateToUpdate;	//date info

void vRTCTask(void *pvParameters)
{

	vSetUpRTC();
	vRTCInit();
	while(1)
	{

		HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);	//get time
		HAL_RTC_GetDate(&hrtc,&DateToUpdate,RTC_FORMAT_BIN);	//get date
		osQueueUARTMessage("Time:  %d:%d:%d",sTime.Hours,sTime.Minutes,sTime.Seconds);
		HAL_Delay(1000);

	}
}

//RTC clock configuration
void vSetUpRTC()
{

	RCC_PeriphCLKInitTypeDef PeriphClkInit;


	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;

    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);	//Initializes the RCC extended peripherals clocks according to the specified parameters in the RCC_PeriphCLKInitTypeDef


	/*
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);	//enable power interface

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_BKP);	//enable backup interface

	LL_PWR_EnableBkUpAccess();	//enable backup access

	LL_RCC_ForceBackupDomainReset();	//Reset all the backup domain

	LL_RCC_ReleaseBackupDomainReset();	//mode that any reset except the LL_RCC_ForceBackupDomainReset() is not valid

	LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);	//set RTC clock

	LL_RCC_EnableRTC();	//Enable RTC
	 */

}

//RTC initialization
void vRTCInit()
{
	
	/**Initialize RTC Only 
    */
  hrtc.Instance = RTC;	//Register base address
  hrtc.Init.AsynchPrediv = 32767;	//Specifies the RTC Asynchronous Predivider value = 32767 +1 = 32768
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;	//Specifies which signal will be routed to the RTC Tamper pin
  HAL_RTC_Init(&hrtc);	//Initializes the RTC peripheral

	/**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  /* Random Date */

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0;		
  DateToUpdate.Year = 0;	//year 2000
  HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);


	/*
	LL_RTC_InitTypeDef RTC_InitStruct;
	LL_RTC_TimeTypeDef RTC_TimeStruct;


	//LL_PWR_EnableBkUpAccess();


	//LL_RCC_EnableRTC();

	RTC_InitStruct.AsynchPrescaler = 32767;	//32767 + 1 = 32768
	LL_RTC_Init(RTC, &RTC_InitStruct);	//it calls actually  LL_RTC_SetAsynchPrescaler() that writes(only available in init mode) the prescaler

	//LL_RTC_SetAsynchPrescaler(RTC, 32767);	// why CubeMx? you can set RTC registers only in RTC init mode

	RTC_TimeStruct.Hours = 0;
	RTC_TimeStruct.Minutes = 0;
	RTC_TimeStruct.Seconds = 0;
	LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);
	*/
}

