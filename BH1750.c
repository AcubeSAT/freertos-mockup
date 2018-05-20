#include "BH1750.h" //Include the definitions of functions and many other necessary things

void BH1750_WriteCommand(uint8_t comm)
{
	TWIStart();
	TWISendAddr(BH1750_ADDR, 1);
	TWIWrite(comm);
	TWIStop();
}

uint16_t BH1750_ReadWord(void)
{
	uint16_t rcvData = 0;
	TWIStart(); //Generate a start condition
	TWISendAddr(BH1750_ADDR, 0); //Send the address and set the mode to receiver
	
	rcvData = TWIReadACK() << 8; //Get the high byte first
	
	//I doubt it, further investigation is needed
	//TWIStop(); //Send a stop condition in the next bit
	
	rcvData |= TWIReadNACK(); //And then the low byte and fuse them together
	TWIStop(); //Send a stop condition in the next bit
	
	return rcvData;
}

void BH1750_Init(uint8_t resMode)
{
	//TWIInit(); //Initialize the I2C interface
	
	BH1750_WriteCommand(BH1750_POWERON);
	BH1750_WriteCommand(BH1750_RESET);
	BH1750_WriteCommand(resMode);
}

double BH1750_GetBrightnessSingle(void)
{
	uint16_t level = 0;
	
	BH1750_WriteCommand(BH1750_ONELRES);
	Delay_ms(20); //Wait for data acquisition

	//Receive data value
	level = BH1750_ReadWord(); //Read the measured value of brightness
	level /= 1.2; //Convert bits to lux

	return level;
}

double BH1750_GetBrightnessCont(void) 
{
	uint16_t level = 0;

	//Receive data value
	level = BH1750_ReadWord(); //Read the measured value of brightness
	level /= 1.2; //Convert bits to lux

	return level;
}

/*
void BH1750_SetSensitivity(uint8_t sens)
{
    BH1750_WriteCommand(BH1750_CHNGMTIMEH|((sens >> 5) & 0x07)); //Select and send the conditioned high bits
    BH1750_WriteCommand(BH1750_CHNGMTIMEL|(sens & 0x1F)); //Select and send the low bits of the byte
    
    bh1750.MTREG = sens; //Set the current MTREG value
}*/
