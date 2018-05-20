/*
 * BH1750.h
 *
 * A library to interface with the BH1750FVI light sensor
 *
 * Could use some more features, but works nicely with the TWI library
 *
 * Make sure to use your preferred delay() function in the place of HAL_Delay()
 */

#ifndef BH1750_H_
#define BH1750_H_

#include "stm32f10x.h"
#include "delay.h"
#include "TWI.h"

//Device address
#define BH1750_ADDR 0x23 //Address of the BH sensor

//Instruction set as given in the data sheet
#define BH1750_POWERDOWN  0x00
#define BH1750_POWERON    0x01
#define BH1750_RESET      0x07
#define BH1750_CONTHRES   0x10
#define BH1750_CONTHRES2  0x11
#define BH1750_CONTLRES   0x13
#define BH1750_ONEHRES    0x20
#define BH1750_ONEHRES2   0x21
#define BH1750_ONELRES    0x23
#define BH1750_CHNGMTIMEH 0x40
#define BH1750_CHNGMTIMEL 0x60

//Time changing values affecting only high resolution mode
#define BH1750_MTREG1 0x1F //1.85 lx/count for HRES1 and 0.93 lx/count for HRES2
#define BH1750_MTREG2 0x45 //0.83 lx/count for HRES1 and 0.42 lx/count for HRES2
#define BH1750_MTREG3 0xFE //0.23 lx/count for HRES1 and 0.11 lx/count for HRES2

extern void BH1750_Init(uint8_t resMode);
extern void BH1750_SetSensitivity(uint8_t sens);
extern double BH1750_GetBrightnessCont(void);
extern double BH1750_GetBrightnessSingle(void);



#endif /* BH1750_H_ */
