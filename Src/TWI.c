#include "TWI.h"

/***************************************************
 * Important!!!
 * Generally you should read SR1 first and then SR2
 ***************************************************/

void TWIInit(void)
{
	//Enable the bus clocks
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	/*****************************************************************************************
	 * A very important notice!!!!
	 * Always define the CNF first, to tell the MCU what is the output/input state of the PIN
	 * And THEN define the speed of the pin (if it is output).
	 * If the proccess is not done with this order, then the pins will not work properly
	 *****************************************************************************************/

	//SDA is pin 6 and SDL is pin 7
	GPIOB->CRL |= GPIO_CRL_CNF6|GPIO_CRL_CNF7; //Set to AF output OD
	GPIOB->CRL |= GPIO_CRL_MODE6|GPIO_CRL_MODE7; //Set to high speed output

	I2C1->CR1 |= I2C_CR1_PE; //Enable I2C1

	I2C1->CR1 |= I2C_CR1_PE; //Enable I2C1
	I2C1->CR2 = (I2C1->CR2 & 0xFFC0)|0x24; //Set the peripheral speed to 36MHz
	I2C1->CCR |= I2C_CCR_FS; //Set the high speed mode
	I2C1->CCR = (I2C1->CCR & 0xF000)|0x3C; //Set the speed of CCR
	I2C1->TRISE = 0xC; //Set max trise for high speed
}

void TWIWrite(uint8_t u8data)
{
	I2C1->DR = u8data; //Write data to the register
	while(!(I2C1->SR1 & I2C_SR1_BTF) || !(I2C1->SR2 & I2C_SR2_BUSY)); //Wait until the byte transfer is complete
}

void TWIStart(void)
{
	I2C1->CR1 |= I2C_CR1_START; //Generate a start sequence
	while(!(I2C1->SR1 &I2C_SR1_SB) || !(I2C1->SR2 & I2C_SR2_BUSY)); //Wait until a start sequence is generated
}

void TWIStop(void)
{
	I2C1->CR1 |= I2C_CR1_STOP; //Generate a stop bit
}

uint8_t TWIReadACK(void)
{
	I2C1->CR1 |= I2C_CR1_ACK; //Enable acknowledge
	while(!(I2C1->SR1 & I2C_SR1_RXNE) || !(I2C1->SR2 & I2C_SR2_BUSY)); //Wait until data is received
	return I2C1->DR; //Read the data from the register
}

uint8_t TWIReadNACK(void)
{
	I2C1->CR1 &= ~I2C_CR1_ACK; //Disable acknowledge
	while(!(I2C1->SR1 & I2C_SR1_RXNE) || !(I2C1->SR2 & I2C_SR2_BUSY)); //Wait until data is received
	return I2C1->DR; //Read the data from the register
}

//You need to call that function after generating a start sequence to address the device
void TWISendAddr(uint8_t addr, uint8_t tr_dir)
{
	if(tr_dir) //Set to direction as transmitter, which means we are sending a command
	{
		I2C1->DR = ((addr << 1)&0xFE); //Set the address
		uint16_t sr1_mask = I2C_SR1_ADDR|I2C_SR1_TXE;
		uint16_t sr2_mask = I2C_SR2_BUSY|I2C_SR2_TRA|I2C_SR2_MSL;
		while(!(I2C1->SR1 & sr1_mask) || !(I2C1->SR2 & sr2_mask));
	}
	else //Direction receiver, or we want to receive data
	{
		I2C1->DR = ((addr << 1)|0x01);
		while(!(I2C1->SR1 & I2C_SR1_ADDR) || !(I2C1->SR2 & (I2C_SR2_BUSY|I2C_SR2_MSL)));
	}
}

uint8_t TWIReadByte(uint8_t dev_addr, uint8_t registe)
{
	uint8_t byte_read = 0; //Save the read byte

	TWIStart(); //Send a start condition to initialize communication
	TWISendAddr(dev_addr, 1); //Send the write command to the sensor memory
	TWIWrite(registe); //Send the register you want

	//TWI Restart condition
	TWIStart();

	TWISendAddr(dev_addr, 0); //Send the read command to the sensor
        // TODO: Do we need to send the stop bit earlier?
	TWIStop(); //Signal the microcontroller to stop I2C communication after the next byte is sent
	byte_read = TWIReadNACK(); //Read one byte

	return byte_read;
}

void TWIReadBytes(uint8_t dev_addr, uint8_t registe, uint8_t *byte_read, uint8_t byte_count)
{
	TWIStart(); //Send a start condition to initialize communication
	TWISendAddr(dev_addr, 1); //Send the write command to the sensor memory
	TWIWrite(registe); //Send the register you want

	//TWI Restart condition
	TWIStart();

	TWISendAddr(dev_addr, 0); //Send the read command to the sensor
	for(uint8_t i = 0; i < byte_count; i++) //and read the returned bytes
	{
		if (i == byte_count - 1) {
			TWIStop(); //Signal the microcontroller to stop I2C communication after the next byte is sent
			byte_read[i] = TWIReadNACK(); //When the last byte is to be received, don't acknowledge to the slave
		} else {byte_read[i] = TWIReadACK();} //If the bytes that are expected are more than one, send an acknowledgment to the slave
	}
}

void TWIWriteByte(uint8_t dev_addr, uint8_t command_register, uint8_t command)
{
	TWIStart(); //Send a start condition to initialize communication
	TWISendAddr(dev_addr, 1); //Send the write command to the sensor memory
	TWIWrite(command_register); //Send the register address for the command to be sent
	TWIWrite(command); //The wanted command is sent
	TWIStop(); //Stop the I2C communication
}