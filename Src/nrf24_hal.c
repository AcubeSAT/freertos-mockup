#include "nrf24_hal.h"


// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately
void nRF24_GPIO_Init(void) 
{
  GPIO_InitTypeDef PORT;

  // Enable the nRF24L01 GPIO peripherals
	RCC->APB2ENR |= nRF24_GPIO_PERIPHERALS;
	
	//Configure nRF24 IRQ pin
	PORT.Mode	= GPIO_MODE_OUTPUT_PP;
	PORT.Speed	= GPIO_SPEED_FREQ_LOW;
	PORT.Pin	= nRF24_IRQ_PIN;
	HAL_GPIO_Init(nRF24_IRQ_PORT, &PORT);
	
  /*// Configure CSN pin
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_Pin = nRF24_CSN_PIN;
	GPIO_Init(nRF24_CSN_PORT, &PORT);
	nRF24_CSN_H();*/
	SPI_MasterInit(MSB_FIRST, SCK_FOSC_8);

	// Configure CE pin
	PORT.Pin = nRF24_CE_PIN;
	HAL_GPIO_Init(nRF24_CE_PORT, &PORT);
	nRF24_CE_L();
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
/*uint8_t nRF24_LL_RW(uint8_t data) {
	 // Wait until TX buffer is empty
	while (SPI_I2S_GetFlagStatus(nRF24_SPI_PORT, SPI_I2S_FLAG_TXE) == RESET);
	// Send byte to SPI (TXE cleared)
	SPI_I2S_SendData(nRF24_SPI_PORT, data);
	// Wait while receive buffer is empty
	while (SPI_I2S_GetFlagStatus(nRF24_SPI_PORT, SPI_I2S_FLAG_RXNE) == RESET);

	// Return received byte
	return (uint8_t)SPI_I2S_ReceiveData(nRF24_SPI_PORT);
}*/

uint8_t nRF24_LL_RW(uint8_t data)
{
	SPI1->DR = data; // Load data into the buffer
	while(!(SPI1->SR & SPI_SR_TXE)); //Wait until SPI is not busy
	while(!(SPI1->SR & SPI_SR_RXNE));
	while((SPI1->SR & SPI_SR_BSY));
	return SPI1->DR; // Return received data
}
