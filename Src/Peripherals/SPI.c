#include "SPI.h"

void SPI_MasterInit(uint8_t bit_order, uint8_t clk_freq)
{
	uint8_t clear;
	
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN|RCC_APB2ENR_IOPAEN;
	
	SPI1->CR1 = (SPI1->CR1 & 0xC7)|clk_freq; //Set SCK frequency to clk_freq
	SPI1->CR1 |= SPI_CR1_SSM|SPI_CR1_MSTR; //Software slave management and master mode
	SPI1->CR1 |= SPI_CR1_SSI; //Set the slave select pin to high
	SPI1->CR1 |= SPI_CR1_SPE; //Enable SPI1
	
	//Set pins to output AF PushPull
	GPIOA->CRL &= ~(GPIO_CRL_CNF5_0|GPIO_CRL_CNF6_0|GPIO_CRL_CNF7_0);
	GPIOA->CRL |= GPIO_CRL_CNF5_1|GPIO_CRL_CNF6_1|GPIO_CRL_CNF7_1;
	
	GPIOA->CRL &= ~(GPIO_CRL_CNF4_0|GPIO_CRL_CNF4_1); //Set the SS pin (PA4) to output PP
	
	GPIOA->CRL |= GPIO_CRL_MODE4|GPIO_CRL_MODE5|GPIO_CRL_MODE6|GPIO_CRL_MODE7; //Set high speed mode
	
	//Clear the registers by reading them
	clear = SPI1->DR;
	clear = SPI1->SR;
	
	GPIOA->ODR |= GPIO_ODR_ODR4; //Disable slave
}

uint8_t SPI_Transfer(uint8_t data)
{
	SPI1->DR = data; // Load data into the buffer
	while(!(SPI1->SR & SPI_SR_TXE)); //Wait until SPI is not busy
	while(!(SPI1->SR & SPI_SR_RXNE));
	while((SPI1->SR & SPI_SR_BSY));
	return SPI1->DR; // Return received data
}
