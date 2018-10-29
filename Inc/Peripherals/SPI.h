#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED

#include <stm32f1xx.h>

#define MSB_FIRST 0 //Send the MSB first
#define LSB_FIRST 1 //Send the LSB first
#define SPI_INT_ENABLE 0 //Enable the SPI interrupt

//SPI clock frequency selection
#define SCK_FOSC_2 0x00 //Select a clock divider by 2
#define SCK_FOSC_4 0x01 //Select a clock divider by 4
#define SCK_FOSC_8 0x10 //Select a clock divider by 8
#define SCK_FOSC_16 0x18 //Select a clock divider by 16
#define SCK_FOSC_32 0x20 //Select a clock divider by 32
#define SCK_FOSC_64 0x28 //Select a clock divider by 64
#define SCK_FOSC_128 0x30 //Select a clock divider by 128

extern void SPI_MasterInit(uint8_t bit_order, uint8_t clk_freq); //Initialize SPI as a master
extern uint8_t SPI_Transfer(uint8_t data); //Use it to both transmission and reception

#endif // SPI_H_INCLUDED
