#include "MPU9250.h"

void MPU9250Init(uint8_t afs, uint8_t gfs)
{
	uint8_t regVal = 0;

	//Delay_Init();
	//TWIInit2(I2C2_INIT); //Initialize I2C2 interface

	TWIWriteByte(MPU9250_ADDR, PWR_MGMT_1, 0x00); //Clear the sleep mode bit and enable all the sensors
	Delay_ms(100); //Wait for the registers to reset
	TWIWriteByte(MPU9250_ADDR, PWR_MGMT_1, 0x01);
	Delay_ms(200);

	//Disable FSYNC pin and set the thermometer and gyro bandwidth to 41 and 42 Hz respectively
	//Minimum delay for the the setting is 5.9ms, so update can not be higher than 1/0.0059=170Hz
	//Setting the DLPF_CFG[2:0] = 011, sets the sample rate to 1kHz for both
	TWIWriteByte(MPU9250_ADDR, CONFIG, 0x03);

	//Use a 200Hz sample rate (divider 4)
	TWIWriteByte(MPU9250_ADDR, SMPLRT_DIV, 0x04);

	regVal = TWIReadByte(MPU9250_ADDR, GYRO_CONFIG);
	regVal &= ~(0x02); //Clear Fchoice bits[1:0]
	regVal &= ~(0x18); //Clear AFS[4:3]
	regVal |= gfs; //Set the gyro to fullscale
	TWIWriteByte(MPU9250_ADDR, GYRO_CONFIG, regVal);

	regVal = TWIReadByte(MPU9250_ADDR, ACCEL_CONFIG);
	regVal &= ~(0x18); //Clear AFS bits
	regVal |= afs; //Assign the provided scale to the register
	TWIWriteByte(MPU9250_ADDR, ACCEL_CONFIG, regVal);

	regVal = TWIReadByte(MPU9250_ADDR, ACCEL_CONFIG2);
	regVal &= ~(0x0F); //Clear fchoice_b and DLPF config bits
	regVal |= 0x03; //Set the low pass filter bandwidth
	TWIWriteByte(MPU9250_ADDR, ACCEL_CONFIG2, regVal);

	TWIWriteByte(MPU9250_ADDR, INT_PIN_CFG, 0x22);
	TWIWriteByte(MPU9250_ADDR, INT_ENABLE, 0x01);

	Delay_ms(100); //Let some time to set things up
}

void MPU9250Calibration(float *gyroCalib)
{

	uint16_t gyrosensitivity  = 131;   //Gyro scale 131 LSB/degrees/sec
  //uint16_t accelsensitivity = 16384; //Accelerometer scale 16384 LSB/g

	/*
	uint8_t data[12]; //Data array to hold accelerometer and gyro x, y, z, data
  uint16_t packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  //Reset device by writing 1 to bit 7 (Reset bit)
  TWIWriteByte(MPU9250_ADDR, PWR_MGMT_1, 0x80);
  Delay_ms(100);

  TWIWriteByte(MPU9250_ADDR, PWR_MGMT_1, 0x01);
  TWIWriteByte(MPU9250_ADDR, PWR_MGMT_2, 0x00);
  Delay_ms(200);

  //Configure device for bias calculation
  TWIWriteByte(MPU9250_ADDR, INT_ENABLE, 0x00); //Disable all interrupts
  TWIWriteByte(MPU9250_ADDR, FIFO_EN, 0x00); //Disable FIFO
  TWIWriteByte(MPU9250_ADDR, PWR_MGMT_1, 0x00); //Turn on internal clock source
  TWIWriteByte(MPU9250_ADDR, I2C_MST_CTRL, 0x00); //Disable I2C master
  TWIWriteByte(MPU9250_ADDR, USER_CTRL, 0x00); //Disable FIFO and I2C master modes
  TWIWriteByte(MPU9250_ADDR, USER_CTRL, 0x0C); //Reset FIFO and DMP
  Delay_ms(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  TWIWriteByte(MPU9250_ADDR, CONFIG, 0x01); //Set low-pass filter to 188 Hz
  TWIWriteByte(MPU9250_ADDR, SMPLRT_DIV, 0x00); //Set sample rate to 1 kHz
  TWIWriteByte(MPU9250_ADDR, GYRO_CONFIG, 0x00); //Set gyro full-scale to 250 degrees per second, maximum sensitivity
  TWIWriteByte(MPU9250_ADDR, ACCEL_CONFIG, 0x00); //Set accelerometer full-scale to 2 g, maximum sensitivity

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  TWIWriteByte(MPU9250_ADDR, USER_CTRL, 0x40); //Enable FIFO
  TWIWriteByte(MPU9250_ADDR, FIFO_EN, 0x78); //Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  Delay_ms(40); //Accumulate 40 samples in 40 milliseconds = 480 bytes

  //At end of sample accumulation, turn off FIFO sensor read
  TWIWriteByte(MPU9250_ADDR, FIFO_EN, 0x04); //Disable gyro and accelerometer sensors for FIFO
  TWIReadBytes(MPU9250_ADDR, FIFO_COUNTH, data, 2); //Read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1]; //Create the value from the received bits
  packet_count = fifo_count/12; //How many sets of full gyro and accelerometer data for averaging

  for (uint16_t i = 0; i < packet_count; i++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};

    TWIReadBytes(MPU9250_ADDR, FIFO_R_W, data, 12); //Read data for averaging

		//Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    //Sum individual signed 16-bit biases to get accumulated signed 32-bit biases.
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  //Calculate the average
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  //Construct the gyro biases for pushing to the hardware gyro bias registers, which are reset to zero upon device startup.
  //Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
  data[0] = ((-gyro_bias[0]/4  >> 8) & 0xFF);

  //Biases are additive, so change sign on calculated average gyro biases
  data[1] = ((-gyro_bias[0]/4)       & 0xFF);
  data[2] = ((-gyro_bias[1]/4  >> 8) & 0xFF);
  data[3] = ((-gyro_bias[1]/4)       & 0xFF);
  data[4] = ((-gyro_bias[2]/4  >> 8) & 0xFF);
  data[5] = ((-gyro_bias[2]/4)       & 0xFF);

  //Push gyro biases to hardware registers
  TWIWriteByte(MPU9250_ADDR, XG_OFFSET_H, data[0]);
  TWIWriteByte(MPU9250_ADDR, XG_OFFSET_L, data[1]);
  TWIWriteByte(MPU9250_ADDR, YG_OFFSET_H, data[2]);
  TWIWriteByte(MPU9250_ADDR, YG_OFFSET_L, data[3]);
  TWIWriteByte(MPU9250_ADDR, ZG_OFFSET_H, data[4]);
  TWIWriteByte(MPU9250_ADDR, ZG_OFFSET_L, data[5]);

  //Output scaled gyro biases for debugging or further use
  gyroCalib[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroCalib[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroCalib[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
	*/

	int16_t data[3]; //Data array to hold accelerometer and gyro x, y, z, data
	uint8_t gyrOff[6];
	int32_t gyro_bias[3]  = {0, 0, 0};
	uint16_t numSamp = 50;

	uint8_t temp[6];

  //Reset device by writing 1 to bit 7 (Reset bit)
  TWIWriteByte(MPU9250_ADDR, PWR_MGMT_1, 0x80);
  Delay_ms(100);

  TWIWriteByte(MPU9250_ADDR, PWR_MGMT_1, 0x01);
  TWIWriteByte(MPU9250_ADDR, PWR_MGMT_2, 0x00);
  Delay_ms(200);

  //Configure device for bias calculation
  TWIWriteByte(MPU9250_ADDR, INT_ENABLE, 0x00); //Disable all interrupts
  TWIWriteByte(MPU9250_ADDR, FIFO_EN, 0x00); //Disable FIFO
  TWIWriteByte(MPU9250_ADDR, PWR_MGMT_1, 0x01); //Turn on internal clock source
  TWIWriteByte(MPU9250_ADDR, I2C_MST_CTRL, 0x00); //Disable I2C master
  TWIWriteByte(MPU9250_ADDR, USER_CTRL, 0x00); //Disable FIFO and I2C master modes
  TWIWriteByte(MPU9250_ADDR, USER_CTRL, 0x0C); //Reset FIFO and DMP
  Delay_ms(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  TWIWriteByte(MPU9250_ADDR, CONFIG, 0x04); //Set low-pass filter to 20Hz
  TWIWriteByte(MPU9250_ADDR, SMPLRT_DIV, 0x00); //Set sample rate to 1 kHz
  TWIWriteByte(MPU9250_ADDR, GYRO_CONFIG, 0x00); //Set gyro full-scale to 250 degrees per second, maximum sensitivity
	/*
	// Configure FIFO to capture accelerometer and gyro data for bias calculation
  TWIWriteByte(MPU9250_ADDR, USER_CTRL, 0x40); //Enable FIFO
  TWIWriteByte(MPU9250_ADDR, FIFO_EN, 0x70); //Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  Delay_ms(6); //Accumulate 40 samples in 40 milliseconds = 480 bytes

  //At end of sample accumulation, turn off FIFO sensor read
  TWIWriteByte(MPU9250_ADDR, FIFO_EN, 0x04); //Disable gyro and accelerometer sensors for FIFO
  TWIReadBytes(MPU9250_ADDR, FIFO_COUNTH, temp, 2); //Read FIFO sample count
	fifoSize = temp[0] << 8|temp[1];
	gyroCalib[0] = fifoSize;
	uint16_t times = 0;

	int16_t packets = fifoSize/6;
	gyroCalib[1] = fifoSize;
	uint8_t temp2[2];

	for(uint16_t i = 0; i < packets; i++)
	{
		TWIReadBytes(MPU9250_ADDR, FIFO_R_W, temp, 6);
		//gyroCalib[0] = temp[5];

		gyro_bias[0] += (int32_t)(temp[0] << 8|temp[1]);
		gyro_bias[1] += (int32_t)(temp[2] << 8|temp[3]);
		gyro_bias[2] += (int32_t)(temp[4] << 8|temp[5]);
	}
  gyro_bias[0]  /= (int32_t) packets;
  gyro_bias[1]  /= (int32_t) packets;
  gyro_bias[2]  /= (int32_t) packets;
	*/

	for(int i = 0; i < numSamp; i++)
	{
		MPU9250ReadGyroDataRaw(data);
		gyro_bias[0] += (int32_t)data[0];
		gyro_bias[1] += (int32_t)data[1];
		gyro_bias[2] += (int32_t)data[2];
		Delay_ms(10);
	}
	gyro_bias[0] /= -4.0*numSamp;
	gyro_bias[1] /= -4.0*numSamp;
	gyro_bias[2] /= -4.0*numSamp;

	gyrOff[0] = (uint8_t)(gyro_bias[0] >> 8) &0xFF;
	gyrOff[1] = (uint8_t)(gyro_bias[0]) &0xFF;
	gyrOff[2] = (uint8_t)(gyro_bias[1] >> 8) &0xFF;
	gyrOff[3] = (uint8_t)(gyro_bias[1]) &0xFF;
	gyrOff[4] = (uint8_t)(gyro_bias[2] >> 8) &0xFF;
	gyrOff[5] = (uint8_t)(gyro_bias[2]) &0xFF;

	TWIWriteByte(MPU9250_ADDR, XG_OFFSET_H, gyrOff[0]);
  TWIWriteByte(MPU9250_ADDR, XG_OFFSET_L, gyrOff[1]);
  TWIWriteByte(MPU9250_ADDR, YG_OFFSET_H, gyrOff[2]);
  TWIWriteByte(MPU9250_ADDR, YG_OFFSET_L, gyrOff[3]);
  TWIWriteByte(MPU9250_ADDR, ZG_OFFSET_H, gyrOff[4]);
  TWIWriteByte(MPU9250_ADDR, ZG_OFFSET_L, gyrOff[5]);

	gyroCalib[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroCalib[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroCalib[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
}

void MPU9250ReadAccelDataRaw(int16_t *acceleration)
{
	uint8_t data[6]; //Save X,Y,Z acceleration data

	TWIReadBytes(MPU9250_ADDR, ACCEL_XOUT_H, data, 6);

	acceleration[0] = ((int16_t)data[0] << 8)|data[1];
	acceleration[1] = ((int16_t)data[2] << 8)|data[3];
	acceleration[2] = ((int16_t)data[4] << 8)|data[5];
}

void MPU9250ReadGyroDataRaw(int16_t *angular)
{
	uint8_t data[6]; //Save X,Y,Z angular velocity data

	TWIReadBytes(MPU9250_ADDR, GYRO_XOUT_H, data, 6);

	angular[0] = ((int16_t)data[0] << 8)|data[1];
	angular[1] = ((int16_t)data[2] << 8)|data[3];
	angular[2] = ((int16_t)data[4] << 8)|data[5];
}

int16_t MPU9250ReadTempDataRaw(void)
{
	uint8_t data[2]; //Save the received temperature bytes

	TWIReadBytes(MPU9250_ADDR, TEMP_OUT_H, data, 2);

	return ((int16_t)data[0] << 8)|data[1];
}

void MPU9250GetAcceleration(double *acc)
{
	int16_t data[3]; //Save X,Y,Z acceleration data
	uint8_t scale = 0; //Save the current scale, as read from the register
	double multFactor = 0.0; //Save the conversion factor for the acelerometer

	scale = (TWIReadByte(MPU9250_ADDR, ACCEL_CONFIG) & (0x03 << 3)) >> 3; //Get the current reading scale
	MPU9250ReadAccelDataRaw(data); //Get the raw accelerometer data

	//Select the conversion factor according to scale setting
	switch(scale)
	{
		case 0:
			multFactor = 1.0/16384.0;
			break;
		case 1:
			multFactor = 1.0/8192.0;
			break;
		case 2:
			multFactor = 1.0/4096.0;
			break;
		case 3:
			multFactor = 1.0/2048.0;
			break;
	}
	acc[0] = (double)data[0]*multFactor;
	//acc[0] = scale;
	acc[1] = (double)data[1]*multFactor;
	acc[2] = (double)data[2]*multFactor;
}

void MPU9250GetAngularVel(double *angVel)
{
	int16_t data[3]; //Save raw X,Y,Z angular velocity data
	uint8_t scale = 0; //Save the current scale, as read from the register
	double multFactor = 0.0; //Save the conversion factor for the gyroscope

	scale = (TWIReadByte(MPU9250_ADDR, GYRO_CONFIG) & (0x03 << 3)) >> 3; //Get the current reading scale
	MPU9250ReadGyroDataRaw(data); //Get the raw accelerometer data

	//Select the conversion factor according to scale setting
	switch(scale)
	{
		case 0:
			multFactor = 1.0/131.0;
			break;
		case 1:
			multFactor = 1.0/65.5;
			break;
		case 2:
			multFactor = 1.0/32.8;
			break;
		case 3:
			multFactor = 1.0/16.4;
			break;
	}
	angVel[0] = (double)data[0]*multFactor;
	angVel[1] = (double)data[1]*multFactor;
	angVel[2] = (double)data[2]*multFactor;
}

void MPU9250_GetCalibAccelGyro(double* Accel, double* Gyro, float* gyrCal)
{
	MPU9250GetAcceleration(Accel);  // Get the acceleration scaled
	MPU9250GetAngularVel(Gyro);  // Get the angular velocity calibrated to scale

	// Use the calibration provided
	Gyro[0] = (Gyro[0] + (double)gyrCal[0])*0.0174533; //Return in rad/s
	Gyro[1] = (Gyro[1] + (double)gyrCal[1])*0.0174533; //Return in rad/s
	Gyro[2] = (Gyro[2] + (double)gyrCal[2])*0.0174533; //Return in rad/s
}

void MPU9250_SetFullScaleGyroRange(uint8_t range)
{
	uint8_t regVal = 0;
	regVal = TWIReadByte(MPU9250_ADDR, GYRO_CONFIG);
	regVal &= ~(0x18); //Clear AFS[4:3]
	regVal |= range; //Set the gyro to fullscale
	TWIWriteByte(MPU9250_ADDR, GYRO_CONFIG, regVal);
}

void AK8963Init(uint8_t resolution, uint8_t mode, float *adjVals)
{
	uint8_t tempAdj[3];

	TWIWriteByte(MPU9250_ADDR, USER_CTRL, 0x00); // Connect auxiliary I2C lines with main ones
	Delay_ms(10);

	TWIWriteByte(AK8963_ADDR, AK8963_CNTL, AK8963_POWERDOWN); //Power down the magnetometer
	Delay_ms(10);
	TWIWriteByte(AK8963_ADDR, AK8963_CNTL, 0x0F); //Enter in Fuse-ROM mode to read calibration values
	Delay_ms(10);

	TWIReadBytes(AK8963_ADDR, AK8963_ASAX, tempAdj, 3);
	adjVals[0] = ((tempAdj[0] - 128)*0.5)/128.0 + 1.0;
	adjVals[1] = ((tempAdj[1] - 128)*0.5)/128.0 + 1.0;
	adjVals[2] = ((tempAdj[2] - 128)*0.5)/128.0 + 1.0;
	TWIWriteByte(AK8963_ADDR, AK8963_CNTL, AK8963_POWERDOWN);
	Delay_ms(10);

	TWIWriteByte(AK8963_ADDR, AK8963_CNTL, resolution|mode);
	Delay_ms(10);
}

uint8_t AK8963GetID(void)
{
	return TWIReadByte(AK8963_ADDR, AK8963_WIA);
}

void AK8963GetMagnRaw(int16_t *magnField)
{
	uint8_t data[7]; //Read X,Y,Z data and ST2 register value

	if(TWIReadByte(AK8963_ADDR, AK8963_ST1) & 0x01)
	{
		TWIReadBytes(AK8963_ADDR, AK8963_HXL, data, 7);
		if(!(data[6] & 0x08)) //Place where the HOFL bit is ST2 is
		{
			//Bytes are returned in little-endian
			magnField[0] = (int16_t)(((int16_t)data[1] << 8)|data[0]);
			magnField[1] = (int16_t)(((int16_t)data[3] << 8)|data[2]);
			magnField[2] = (int16_t)(((int16_t)data[5] << 8)|data[4]);
		}
	}
}

//Pass the adjustment values as returned from initialization
void AK8963GetMagnuT(float *mField, float *adjVals)
{
	int16_t data[3]; //Read X,Y,Z data and ST2 register value
	uint8_t scale = 0;
	float scaleFactor = 0.0;

	scale = (TWIReadByte(AK8963_ADDR, AK8963_CNTL) & (1 << 4)) >> 4;
	AK8963GetMagnRaw(data); //Get the raw magnetic values

	switch(scale)
	{
		case 0:
			scaleFactor = 4912.0/8190.0;
		  break;
		case 1:
			scaleFactor = 4912.0/32760.0;
			break;
	}

	//We divide by 4912 because this is the maximum value of uT provided at maximum 32768, as described in datasheet
	mField[0] = (((float)data[0]*adjVals[0])*scaleFactor);
	mField[1] = (((float)data[1]*adjVals[1])*scaleFactor);
	mField[2] = (((float)data[2]*adjVals[2])*scaleFactor);
}
