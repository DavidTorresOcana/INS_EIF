
#include "I2CLib.h"
#include "Motion_Sensors.h"
#include "HMC5883.h"


//#define GAIN 			1090
#define GAIN 			0.00121951219512195121951219512195f

void Delay(uint32_t delay);


void HMC5883_Init(void)
{
	  
//	I2C_WrData(HMC5883_ADDR, (uint8_t )ConfigRegA, SampleAveraging_8 | DataOutputRate_75HZ | NormalOperation);
	I2C_WrData(HMC5883_ADDR, (uint8_t )ConfigRegA, 0x18);
	I2C_WrData(HMC5883_ADDR, (uint8_t )ConfigRegB,0x60 );		//+/- 2gauss
	
	Delay(100);
	I2C_WrData(HMC5883_ADDR, (uint8_t) ModeRegister, (uint8_t)ContinuousConversion);
}


 Mag_Data HMC5883_Read(void)
{
	 Mag_Data data;
	uint8_t buff[6];

	uint8_t buffer[1]={DataOutputXMSB};//{IDRegisterA};//DataOutputXMSB};
	I2C_WrBuf(HMC5883_ADDR, buffer, 1);
	I2C_RdBuf(HMC5883_ADDR, buff, sizeof(buff));

	data.x =(float)big_endian(&buff[0])*GAIN;
 	data.z =(float)big_endian(&buff[2])*GAIN;
	data.y =(float)big_endian(&buff[4])*GAIN;

/*
	data.x =(float)big_endian(&buff[0])/1090;
 	data.z =(float)big_endian(&buff[2])/1090;
	data.y =(float)big_endian(&buff[4])/1090;
*/

	return data;
}





//---------------------------------------------------COMPENSACION
//   void measure(float roll, float pitch)
//   {
//     byte  hmc5883ready;  // JI - 12/11/11
//     float magX;
//     float magY;
//     float tmp;

//     sendByteI2C(compassAddress, 0x09);                  // JI - 12/11/11 - Request status register
//     hmc5883ready = 0x01 & readByteI2C(compassAddress);  // JI - 12/11/11 - Mask ready status

//     if (hmc5883ready == 0x01)                           // JI - 12/11/11 - Read only if data ready
//     {
//       sendByteI2C(compassAddress, 0x03);
//       Wire.requestFrom(compassAddress, 6);

//       #if defined(SPARKFUN_9DOF)
//         // JI - 11/24/11 - SparkFun DOF on v2p1 Shield Configuration
//         // JI - 11/24/11 - 5883L X axis points aft
//         // JI - 11/24/11 - 5883L Sensor Orientation 3
//         measuredMagX = -((Wire.read() << 8) | Wire.read()) * magCalibration[XAXIS];  // JI - 12/01/11 - magCalibration[XAXIS], was magCalibration[YAXIS]
//         measuredMagZ = -((Wire.read() << 8) | Wire.read()) * magCalibration[ZAXIS];
//         measuredMagY =  ((Wire.read() << 8) | Wire.read()) * magCalibration[YAXIS];  // JI - 12/01/11 - magCalibration[YAXIS], was magCalibration[XAXIS]
//       #elif defined(SPARKFUN_5883L_BOB)
//         // JI - 11/24/11 - Sparkfun 5883L Breakout Board Upside Down on v2p0 shield
//         // JI - 11/24/11 - 5883L is upside down, X axis points forward
//         // JI - 11/24/11 - 5883L Sensor Orientation 5
//         measuredMagX =  ((Wire.read() << 8) | Wire.read()) * magCalibration[XAXIS];  // JI - 12/01/11 - magCalibration[XAXIS], was magCalibration[YAXIS]
//         measuredMagZ =  ((Wire.read() << 8) | Wire.read()) * magCalibration[ZAXIS];
//         measuredMagY =  ((Wire.read() << 8) | Wire.read()) * magCalibration[YAXIS];  // JI - 12/01/11 - magCalibration[YAXIS], was magCalibration[XAXIS]
//       #else
//         !! Define 5883L Orientation !!
//       #endif

//       Wire.endTransmission();

//       updateRegisterI2C(compassAddress, 0x02, 0x01);  // JI - 12/11/11 - Perform single conversion

//       cosRoll =  cos(roll);
//       sinRoll =  sin(roll);
//       cosPitch = cos(pitch);
//       sinPitch = sin(pitch);

//       magX = ((float)measuredMagX * magScale[XAXIS] + magOffset[XAXIS]) * cosPitch + \
//              ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * sinRoll * sinPitch + \
//              ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * cosRoll * sinPitch;

//       magY = ((float)measuredMagY * magScale[YAXIS] + magOffset[YAXIS]) * cosRoll - \
//              ((float)measuredMagZ * magScale[ZAXIS] + magOffset[ZAXIS]) * sinRoll;

//       tmp  = sqrt(magX * magX + magY * magY);

//       hdgX = magX / tmp;
//       hdgY = -magY / tmp;
//     }
//   }


