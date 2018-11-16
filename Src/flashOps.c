#include "flashOps.h"

/* ************************************************************************************ *
 * @brief Write data to FLASH, after erasing first, one page at a time
 *
 * @note Be careful with the addresses because they have to be aligned,
 * 		 otherwise there is a high chance of hardware fault error.
 *
 * @param ulPageAddress: FLASH page base address as provided in the manual of the MCU
 * @param puAddressID: An array containing all the offsets where data will be stored
 * 					   Also note that the values have to be in ascending order.
 * @param puData: Array containing the data values to be written
 * 				  This array is indexed in accordance to the ID array,
 * 				  which means index relationship 1-1.
 * @param xDataLength: The length of the address offset array or the data array.
 * @retcval none
 * ************************************************************************************ */
void vFlashWrite(uint32_t ulPageAddress, const uint16_t *puAddressID, const uint16_t *puData,
		size_t xDataLength) {
	FLASH_EraseInitTypeDef eraseStruct;
	uint32_t pageError = 0;
	uint16_t puRawPageData[1024];

	taskENTER_CRITICAL();  // Enter in a critical section

	// Read the flash content before deleting and writing
	for (size_t i = 0; i < 1024; i++) {
		puRawPageData[i] = ulFlashRead(ulPageAddress + i);
	}

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPTVERR | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

	eraseStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseStruct.NbPages = (uint32_t)1;  // For the moment operate on just one page at each call
	eraseStruct.PageAddress = ulPageAddress;

	HAL_FLASHEx_Erase(&eraseStruct, &pageError);  // Erase the page to prepare for writing

	for (size_t i = 0, j = 0; i < 1024; i += 2) {
		if (i == puAddressID[j]) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ulPageAddress + i,
					(uint64_t)puData[j++]);
		} else {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ulPageAddress + i, puRawPageData[i]);
		}

		if (j == xDataLength) {
			j = 0;
		}
	}

#if FLASH_DEBUG_CHECK
	// Error checking for debugging purposes
	volatile uint16_t errCount = 0;
	volatile size_t errCountID[10];
	volatile uint16_t errValue[10];

	for (size_t i = 0, j = 0; i < 1024; i++) {
		if ( !(puRawPageData[i] == ulFlashRead(ulPageAddress + i)) ) {
			errCount++;
			errCountID[j] = i;
			errValue[j++] = ulFlashRead(ulPageAddress + i);
		}
	}
#endif

	HAL_FLASH_Lock();
	taskEXIT_CRITICAL();  // Exit the critical section, since the desired operation has finished

	NVIC_SystemReset();  // Reset the system for now. It will be fixed in later versions
}
