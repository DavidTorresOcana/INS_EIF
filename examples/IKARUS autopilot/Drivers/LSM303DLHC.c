#include "I2CLib.h"
#include "Motion_Sensors.h"
#include "LSM303DLHC.h"
#include "Systick.h"
#include "math.h"


extern float accel_offset_x;
extern float accel_offset_y;
extern float accel_offset_z;

#define LSM303DLHC_ACEL_GAIN 0.001f*0.06666666666666666666666666666667f
#define LSM303DLHC_MAG_GAIN 1.0f/430

void LSM303DLHC_Init(void)
{
	
	
	I2C_WrData(LSM303_ACC_HIGH, CTRL_REG1_A,0x27);  // 0x27 = normal power mode, all accel axes on

//  if ((fs==8)||(fs==4))
//    LSM303_write((0x00 | (fs-fs/2-1)<<4), CTRL_REG4_A);  // set full-scale
//  else
 	I2C_WrData(LSM303_ACC_HIGH, CTRL_REG4_A,0x80);	//2g
	
	
	I2C_WrData(LSM303_MAG_LOW, CRA_REG_M,0x14);  // 0x14 = mag 30Hz output rate
	I2C_WrData(LSM303_MAG_LOW, CRB_REG_M,0x80);  // 0x80 = mag 4 gauss
	I2C_WrData(LSM303_MAG_LOW, MR_REG_M,0x00); // 0x00 = continouous conversion mode
		
	Delay(300);
}

 Accel_Data LSM303DLHC_Acel_Read(void)
{	
	uint8_t _buff[6];
	 Accel_Data data;

	I2C_RdData(LSM303_ACC_HIGH, OUT_X_L_A|0x80, _buff, 6); //read the acceleration data


  data.x = (int16_t)((((int16_t)_buff[1]) << 8) | _buff[0])*LSM303DLHC_ACEL_GAIN;		// 1mg/LSB
  data.y = (int16_t)((((int16_t)_buff[3]) << 8) | _buff[2])*LSM303DLHC_ACEL_GAIN;
  data.z = (int16_t)((((int16_t)_buff[5]) << 8) | _buff[4])*LSM303DLHC_ACEL_GAIN;
	

 	
	data.x-=accel_offset_x;
	data.y-=accel_offset_y;
	data.z-=accel_offset_z;
	

	
	return data;
}


Mag_Data LSM303DLHC_Mag_Read(void)
{	
	uint8_t _buff[6];
	Mag_Data data;

	I2C_RdData(LSM303_MAG_LOW, OUT_X_H_M|0x80, _buff, 6); 

	data.x = (int16_t)((((int16_t)_buff[0]) << 8) | _buff[1])*LSM303DLHC_MAG_GAIN;		
  data.y = (int16_t)((((int16_t)_buff[2]) << 8) | _buff[3])*LSM303DLHC_MAG_GAIN;
  data.z = (int16_t)((((int16_t)_buff[4]) << 8) | _buff[5])*LSM303DLHC_MAG_GAIN;

//   data.x = (int16_t)((((int16_t)_buff[1]) << 8) | _buff[0])*LSM303DLHC_MAG_GAIN;		
//   data.y = (int16_t)((((int16_t)_buff[3]) << 8) | _buff[2])*LSM303DLHC_MAG_GAIN;
//   data.z = (int16_t)((((int16_t)_buff[5]) << 8) | _buff[4])*LSM303DLHC_MAG_GAIN;
	

	
	return data;
}
