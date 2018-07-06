#include "TMP102.h"

//Read a tmp102 sensor on a given temp_number or channel
void tmp102Read(char* pBuffer1, uint8_t SensorAdd, uint16_t NumByteToRead1)
{
	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	/* Send sensor address */
	I2C_Send7bitAddress(I2C1, SensorAdd, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Make sure pointer register is set to read from temp register */
	I2C_SendData(I2C1, 0x00);
	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/* Send START condition a second time */
	I2C_GenerateSTART(I2C1, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	/* Send Sensor address for read */
	I2C_Send7bitAddress(I2C1, SensorAdd, I2C_Direction_Receiver);
	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	// Now read the 2 bytes in
	/* While there is data to be read */
	while(NumByteToRead1)
	{
	if(NumByteToRead1 == 1)
	{
	/* Disable Acknowledgement */
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	/* Send STOP Condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
	}
	/* Test on EV7 and clear it */
	if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
	/* Read a byte */
	*pBuffer1 = I2C_ReceiveData(I2C1);
	/* Point to the next location where the byte read will be saved */
	pBuffer1++;
	/* Decrement the read bytes counter */
	NumByteToRead1--;
	}
	}
	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}