
#include "I2CLib.h"
#include "Motion_Sensors.h"
#include "L3GD20.h"
#include "Systick.h"
#include "math.h"

extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;


void L3GD20_Init(void){
	
	//Reset
	I2C_WrData(L3GD20_ADDR_HIGH, L3G_CTRL_REG5,0x80);
	Delay(300);
	//Pen=Xen=Yen=Zen=1
	I2C_WrData(L3GD20_ADDR_HIGH, L3G_CTRL_REG1, 0x0F);

	
	Delay(300);
	//Full Scale
		I2C_WrData(L3GD20_ADDR_HIGH, L3G_CTRL_REG4, 0xb0); //0x30
	
	Delay(300);
	
	//Filtro HPF
	I2C_WrData(L3GD20_ADDR_HIGH, L3G_CTRL_REG2, 0x30);//0x20); //0x30
	//HPF EN
	I2C_WrData(L3GD20_ADDR_HIGH, L3G_CTRL_REG5,0x10);

	
}

 Gyro_Data L3GD20_Read(void){
	 Gyro_Data _gyro;
	 uint8_t _buff[6];
	 	
	 I2C_RdData(L3GD20_ADDR_HIGH , L3G_OUT_X_L| 0x80, _buff, 6);
	
  _gyro.x = (int16_t)(_buff[1] << 8 | _buff[0])*0.07;
  _gyro.y = (int16_t)(_buff[3] << 8 | _buff[2])*0.07;
  _gyro.z = (int16_t)(_buff[5] << 8 | _buff[4])*0.07;
	
	_gyro.x -= gyro_offset_x;
	_gyro.y -= gyro_offset_y;
	_gyro.z -= gyro_offset_z;
	
	return _gyro;
}



// float L3G4200D_ReadTemp(void)
// {
//   uint8_t _buff[2];
// 	float _Temp;
// 	I2C_RdData(ITG3200_ADDR_LOW, TEMP_OUT, _buff, 2); //read the acceleration data from the ADXL345

//   _Temp = 35 + ((_buff[0] << 8 | _buff[1]) + 13200) / 280.0;    // F=C*9/5+32
// 	return _Temp;
// }


