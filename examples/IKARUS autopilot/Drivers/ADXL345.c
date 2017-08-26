#include "I2CLib.h"

#include "Motion_Sensors.h"
#include "ADXL345.h"

#include "Systick.h"

#define ADXL345_GAIN		0.004
extern float accel_offset_x;
extern float accel_offset_y;
extern float accel_offset_z;


void ADXL345_Init(void)
{
//		I2C_RdData(ADXL345_ADDR_LOW, 0x00, buffer_out, 1);
	
		I2C_WrData(ADXL345_ADDR_LOW, BW_RATE,0x0A);//50Hz //0x0D);//200Hz
	
		I2C_WrData(ADXL345_ADDR_LOW, DATA_FORMAT, 0x0b);//0x0B);//full range //0x00);// +-2g
		I2C_WrData(ADXL345_ADDR_LOW, POWER_CTL, 0x08);
	
		I2C_WrData(ADXL345_ADDR_LOW, OFSX, 0);
		I2C_WrData(ADXL345_ADDR_LOW, OFSY, 0);

		I2C_WrData(ADXL345_ADDR_LOW, OFSZ, 0);

		Delay(100);
	
}

 Accel_Data ADXL345_Read(void)
{	
	uint8_t _buff[6];
	 Accel_Data data;

	I2C_RdData(ADXL345_ADDR_LOW, DATAX0, _buff, 6); //read the acceleration data from the ADXL345


  // each axis reading comes in 10 bit resolution, ie 2 bytes. Least Significat Byte first!!
  data.x = (int16_t)((((int16_t)_buff[1]) << 8) | _buff[0])*ADXL345_GAIN;		// 4mg/LSB
  data.y = (int16_t)((((int16_t)_buff[3]) << 8) | _buff[2])*ADXL345_GAIN;
  data.z = (int16_t)((((int16_t)_buff[5]) << 8) | _buff[4])*ADXL345_GAIN;
	
	data.x-=accel_offset_x;
	data.y-=accel_offset_y;
	data.z-=accel_offset_z;
	

	
	return data;
}


