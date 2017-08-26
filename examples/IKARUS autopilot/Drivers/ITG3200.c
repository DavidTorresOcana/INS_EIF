#include "I2CLib.h"
#include "Motion_Sensors.h"
#include "ITG3200.h"
#include "Systick.h"
#include "math.h"

extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;

void ITG3200_Init(void)
{
	//I2C_WrData(ITG3200_ADDR_LOW, SMPLRT_DIV, NOSRDIVIDER);

	
	I2C_WrData(ITG3200_ADDR_LOW, SMPLRT_DIV, 0);
	I2C_WrData(ITG3200_ADDR_LOW, DLPF_FS, 0x1C);
	I2C_WrData(ITG3200_ADDR_LOW, INT_CFG, 0x90);
	I2C_WrData(ITG3200_ADDR_LOW, PWR_MGM, 0x01);
	
	//Angel Original
	//I2C_WrData(ITG3200_ADDR_LOW, DLPF_FS, RANGE2000<<3 | BW256_SR8);
	//////
	
	
	

	Delay(100);

	
}

float ITG3200_ReadTemp(void)
{
  uint8_t _buff[2];
	float _Temp;
	I2C_RdData(ITG3200_ADDR_LOW, TEMP_OUT, _buff, 2); //read the acceleration data from the ADXL345

  _Temp = 35 + ((_buff[0] << 8 | _buff[1]) + 13200) / 280.0;    // F=C*9/5+32
	return _Temp;
}

 Gyro_Data ITG3200_Read(void){
	 Gyro_Data _gyro;
  uint8_t _buff[6];
	I2C_RdData(ITG3200_ADDR_LOW, GYRO_XOUT, _buff, 6); //read the acceleration data from the ADXL345

  _gyro.x = (int16_t)(_buff[0] << 8 | _buff[1])/14.375;
  _gyro.y = (int16_t)(_buff[2] << 8 | _buff[3])/14.375;
  _gyro.z = (int16_t)(_buff[4] << 8 | _buff[5])/14.375;
	
	_gyro.x -= gyro_offset_x;
	_gyro.y -= gyro_offset_y;
	_gyro.z -= gyro_offset_z;
	

// if(fabs(_gyro.x)<0.1){
// 	_gyro.x=0;
// }	 
// if(fabs(_gyro.y)<0.1){
// 	_gyro.y=0;
// }
// if(fabs(_gyro.z)<0.1){
// 	_gyro.z=0;
// }

		return _gyro;
}





