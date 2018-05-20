#ifndef TMP102_H_
#define TMP102_H_

#include <stm32f10x.h>

extern void tmp102Read(char* pBuffer1, uint8_t SensorAdd, uint16_t NumByteToRead1);

#endif
