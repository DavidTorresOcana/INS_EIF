#include "I2CLib.h"
#include "Motion_Sensors.h"
#include "MPU6050.h"
#include "Systick.h"

// MPU6000 4g range => g = 4096
//#define GRAVITY 16384 
//#define GYRO_SENS 131

// MPU6050 sensibility (theorical 0.0076 => 1/131LSB/º/s)
//#define Gyro_Gain 0.0076
// MPU6000 sensibility  (theorical 0.0152 => 1/65.6LSB/deg/s at 500deg/s) (theorical 0.0305 => 1/32.8LSB/deg/s at 1000deg/s) ( 0.0609 => 1/16.4LSB/deg/s at 2000deg/s)
//#define Gyro_Gain 0.0609

#define GRAVITY 8192 
#define GRAVITY_INV   0.00006103515625f//0.0001220703125f
//#define GRAVITY 4096
#define GYRO_SENS 16.4f
#define GYRO_SENS_INV 0.06097560975609756097560975609756f
extern float accel_offset_x;
extern float accel_offset_y;
extern float accel_offset_z;

extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;

void MPU6050_Init(void)
{
	
	
    // Chip reset
		I2C_WrData(MPU6050_ADDR, MPUREG_PWR_MGMT_1, BIT_H_RESET);
    Delay(100);
    // Wake up device and select GyroZ clock (better performance)
    I2C_WrData(MPU6050_ADDR, MPUREG_PWR_MGMT_1, 0); //MPU_CLK_SEL_PLLGYROZ);
    Delay(1);
    
	// SAMPLE RATE
    //MPU6000_SPI_write(MPUREG_SMPLRT_DIV,0x04);     // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz     
//    I2C_WrData(MPU6050_ADDR, MPUREG_SMPLRT_DIV,19);     // Sample rate = 50Hz    Fsample= 1Khz/(19+1) = 50Hz     
	    I2C_WrData(MPU6050_ADDR, MPUREG_SMPLRT_DIV,31);     // Sample rate = 50Hz    Fsample= 1Khz/(19+1) = 50Hz     
    Delay(10);
    // FS & DLPF   FS=2000º/s, DLPF = 20Hz (low pass filter)
    I2C_WrData(MPU6050_ADDR, MPUREG_CONFIG, BITS_DLPF_CFG_256HZ_NOLPF2);  
    Delay(10);
    I2C_WrData(MPU6050_ADDR, MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);  // Gyro scale 2000º/s
    Delay(10);
		I2C_WrData(MPU6050_ADDR, MPUREG_ACCEL_CONFIG,0x00);//2G       // Accel scale 4g (4096LSB/g)
	
    Delay(10);
    // Wake up device and select GyroZ clock (better performance)
 //   I2C_WrData(MPU6050_ADDR, MPUREG_PWR_MGMT_1, 0); //MPU_CLK_SEL_PLLGYROZ);
		Delay(100);
}


 Accel_Data MPU6050_Accel_Read(void){
	  	 
	 
	 uint8_t _buff[6];
  Accel_Data data;
	
	 I2C_RdData(MPU6050_ADDR, MPUREG_ACCEL_XOUT_H, _buff, 6); //read the acceleration data from the ADXL345

// 	data.x	= (float)big_endian(&_buff[0])/GRAVITY;
// 	data.y = (float)big_endian(&_buff[2])/GRAVITY;
// 	data.z = (float)big_endian(&_buff[4])/GRAVITY;
	 
	data.x	= (float)big_endian(&_buff[0])*GRAVITY_INV;
	data.y = (float)big_endian(&_buff[2])*GRAVITY_INV;
	data.z = (float)big_endian(&_buff[4])*GRAVITY_INV;
	 

	
	data.x-=accel_offset_x;
	data.y-=accel_offset_y;
	data.z-=accel_offset_z;
	
	 return data;
	 
 }

  Gyro_Data MPU6050_Gyro_Read(void){
	  	 
	 
	uint8_t _buff[6];
	Gyro_Data data;
	
	I2C_RdData(MPU6050_ADDR, MPUREG_GYRO_XOUT_H, _buff, 6);

// 	data.x	= (float)big_endian(&_buff[0])/GYRO_SENS;
// 	data.y = (float)big_endian(&_buff[2])/GYRO_SENS;
// 	data.z = (float)big_endian(&_buff[4])/GYRO_SENS;
		
	data.x	= (float)big_endian(&_buff[0])*GYRO_SENS_INV;
	data.y = (float)big_endian(&_buff[2])*GYRO_SENS_INV;
	data.z = (float)big_endian(&_buff[4])*GYRO_SENS_INV;
	
		
	data.x -= gyro_offset_x;
	data.y -= gyro_offset_y;
	data.z -= gyro_offset_z;
		
	
	return data;
	 
 }
 
 
// Read gyros and accel sensors on MPU6000
void MPU6050_Read( Accel_Data* ad, Gyro_Data* gd)
{
	uint8_t sensores[14];
	


	//i2c_read_buff(BUS, MPU6050, 0x3B, (char*)sensores, sizeof(sensores));
	I2C_RdData(MPU6050_ADDR,  0x3B, sensores, sizeof(sensores));
	
	ad->x	= ((float)big_endian(&sensores[0]))*GRAVITY_INV;
	ad->y = ((float)big_endian(&sensores[2]))*GRAVITY_INV;
	ad->z = ((float)big_endian(&sensores[4]))*GRAVITY_INV;

	//datos.temp = ((float)big_endian(&sensores[6])+12412)/340.0f;
	
	gd->x = ((float)big_endian(&sensores[8]))*GYRO_SENS_INV;
	gd->y = ((float)big_endian(&sensores[10]))*GYRO_SENS_INV;
	gd->z = ((float)big_endian(&sensores[12]))*GYRO_SENS_INV;
	
	ad->x-=accel_offset_x;
	ad->y-=accel_offset_y;
	ad->z-=accel_offset_z;
	
	gd->x -= gyro_offset_x;
	gd->y -= gyro_offset_y;
	gd->z -= gyro_offset_z;
}



// // Read gyros and accel sensors on MPU6000
// struct MPU6050_Data MPU6050_Read(void)
// {
// 	uint8_t sensores[14];
// 	struct MPU6050_Data datos;
// 	


// 	//i2c_read_buff(BUS, MPU6050, 0x3B, (char*)sensores, sizeof(sensores));
// 	I2C_RdData(MPU6050,  0x3B, sensores, sizeof(sensores));
// 	
// 	datos.acc_x = (float)big_endian(&sensores[0])/GRAVITY;
// 	datos.acc_y = (float)big_endian(&sensores[2])/GRAVITY;
// 	datos.acc_z = (float)big_endian(&sensores[4])/GRAVITY;

// 	datos.temp = ((float)big_endian(&sensores[6])+12412)/340.0f;
// 	
// 	datos.gyro_x = (float)big_endian(&sensores[8])/ GYRO_SENS;
// 	datos.gyro_y = (float)big_endian(&sensores[10])/ GYRO_SENS;
// 	datos.gyro_z = (float)big_endian(&sensores[12])/ GYRO_SENS;
// 		
// 	return datos;	
// }
