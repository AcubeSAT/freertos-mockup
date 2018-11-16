#ifndef INC_FLASHOPS_H_
#define INC_FLASHOPS_H_

#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

#include "FreeRTOS.h"
#include "task.h"

#define FLASH_DEBUG_CHECK 0

static inline uint16_t ulFlashRead(uint32_t ulAddress) {
	return *(uint16_t *)ulAddress;
}


extern void vFlashWrite(uint32_t ulPageAddress, const uint16_t *puAddressID, const uint16_t *puData,
		size_t xDataLength);

#endif /* INC_FLASHOPS_H_ */
