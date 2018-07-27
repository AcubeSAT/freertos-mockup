#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
#include <stm32f1xx.h>
#include <stm32f1xx_hal_gpio.h>
#include "SPI.h"
//#include <stm32f10x_spi.h>


// SPI port peripheral
//#define nRF24_SPI_PORT             SPI2

// nRF24 GPIO peripherals
#define nRF24_GPIO_PERIPHERALS     (RCC_APB2ENR_IOPAEN)

// CE (chip enable) pin (PB11)
#define nRF24_CE_PORT              GPIOA
#define nRF24_CE_PIN               GPIO_PIN_3
#define nRF24_CE_L()               HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN, GPIO_PIN_RESET)
#define nRF24_CE_H()               HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN, GPIO_PIN_SET)

// CSN (chip select negative) pin (PB12)
#define nRF24_CSN_PORT             GPIOA
#define nRF24_CSN_PIN              GPIO_PIN_4
#define nRF24_CSN_L()              HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN, GPIO_PIN_RESET)
#define nRF24_CSN_H()              HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN, GPIO_PIN_SET)


// IRQ pin (PB10)
#define nRF24_IRQ_PORT             GPIOA
#define nRF24_IRQ_PIN              GPIO_PIN_2


// Function prototypes
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
