#include <math.h>
//#include "stm32f4xx_conf.h"
#include "stm32f10x.h"
#include "Systick.h"

#include "I2CLib.h"

#include "Motion_Sensors.h"

//#define HMC5883

//#define ADXL345

//#define L3G4200D
//#define ITG3200
//#define L3GD20
//#define LSM303DLHC

#define MPU6050

//Includes
#ifdef MPU6050
	#include "MPU6050.h"
#else

	#ifdef ADXL345
		#include "ADXL345.h"
	#elif defined(LSM303DLHC)
		#include "LSM303DLHC.h"
	#else
		#error "No se ha definido un acelerometro"
	#endif


	#ifdef L3G4200D
			#include "L3G4200D.h"
	#elif defined (ITG3200)
			#include "ITG3200.h"
	#elif defined (L3GD20)
			#include "L3GD20.h"
	#else
		#error "No se ha definido un giroscopo"
	#endif

#endif

#ifdef HMC5883
	#include "HMC5883.h"
#elif defined(LSM303DLHC)
		//#include "LSM303DLHC.h"
#else
//	#error "No se ha definido un compas digital"
#endif


float accel_offset_x=0;
float accel_offset_y=0;
float accel_offset_z=0;
float gyro_offset_x=0;
float gyro_offset_y=0;
float gyro_offset_z=0;

void Init_Tim2(void);

//COMUNES
void Motion_Sensors_Init(void){
		
		I2C_LowLevel_Init();
	
		Accel_Init();
		
//		Accel_Calibrate();	//La calibracion se toma de la config
	
		Gyro_Init();	
	
		Gyro_Calibrate();
	
		Init_Tim2();
}


uint16_t Motion_Sensors_Read( Mag_Data* md, Accel_Data* ad, Gyro_Data* gd){
	uint16_t time;	
	
//	*md=Mag_Read();
	
// 	*ad=Accel_Read();
// 	
// 	time=TIM2->CNT;
// 	TIM2->CNT = 0;
// 	
// 	*gd=Gyro_Read();
	
	MPU6050_Read(ad,gd);
 	
	time=TIM2->CNT;
	TIM2->CNT = 0;
	
	return time;
}

void Init_Tim2(void){

	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

  
/* -----------------------------------------------------------------------
    TIM14 Configuration: Output Compare Timing Mode:
  */
//  TIM_TimeBaseStructure.TIM_Prescaler = 83;
//	aux=SystemCoreClock/1000000;
//	aux=(aux/4)-1;	//168Mhz
	//aux=(aux/2)-1;	//48Mhz
	//Ticks de 0.5uSec
	TIM_TimeBaseStructure.TIM_Prescaler = 31;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	  /* TIM14 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

short big_endian(uint8_t *buff)
{
	short tmp;
	tmp = (buff[0])<<8 |buff[1];
	return tmp;
}

//----------------------------------------------Acelerometro



void Accel_Init(void)
{
	#ifdef ADXL345
		 ADXL345_Init();
	#elif defined (MPU6050)
		MPU6050_Init();
	#elif defined(LSM303DLHC)
		LSM303DLHC_Init();
	#endif	
	
}

Accel_Data Accel_Read(void)
{
	#ifdef ADXL345
		return ADXL345_Read();
	#elif defined (MPU6050)
		return MPU6050_Accel_Read();
	#elif defined(LSM303DLHC)	
		return LSM303DLHC_Acel_Read();
	#endif
	
}

void Accel_Set_Calibration(float x, float y, float z){
	accel_offset_x=x;
	accel_offset_y=y;
	accel_offset_z=z;
}

void Accel_Calibrate(void)
{
	Accel_Data acel;
	int i;
	float x=0;
	float y=0;
	float z=0;
	
	accel_offset_x=0;
	accel_offset_y=0;
	accel_offset_z=0;
	
  	for(i=0;i<100;i++)    // We take some readings...
    {
	    acel=Accel_Read();
	    x+=acel.x;
			y+=acel.y;
			z+=acel.z;
			Delay(10);
    }
    
		x/=100;
		y/=100;
		z/=100;	
  	z-=1.0f; // valor medido 1g - teorico 1g => valor medido para 0g (offset)
		accel_offset_x=x;
		accel_offset_y=y;
		accel_offset_z=z;
	
}

//----------------------------------------------Giroscopo


void Gyro_Init(void)
{
	#ifdef L3G4200D
			 L3G4200D_Init();
	#elif defined (ITG3200)
			ITG3200_Init();
	#elif defined (L3GD20)
			L3GD20_Init();
	#endif
	
}

Gyro_Data Gyro_Read(void)
{
	#ifdef L3G4200D
			return L3G4200D_Read();
	#elif defined (ITG3200)
			return ITG3200_Read();
	#elif defined (MPU6050)
			return MPU6050_Gyro_Read();
	#elif defined (L3GD20)
			return	L3GD20_Read();
	#endif
}

void Gyro_Calibrate(void)
{

	Gyro_Data gyro;
	int i;
	
		float x=0;
		float y=0;
		float z=0;
	
		gyro_offset_x=0;
		gyro_offset_y=0;
		gyro_offset_z=0;
		
		gyro=Gyro_Read();
	
	  	for(i=0;i<200;i++)    // We take some readings...
	    {
		    gyro=Gyro_Read();
		    x+=gyro.x;
			y+=gyro.y;
			z+=gyro.z;
				
				Delay(5);
	    }
    
			x/=200;
			y/=200;
			z/=200;
		
	  	gyro_offset_x=x;
		gyro_offset_y=y;
		gyro_offset_z=z;

}


//----------------------------------------------Magnetometro
#define _RAD2DEG 57.295779513082320876798154814105f
void Mag_Init(void)
{
	#ifdef HMC5883
		 HMC5883_Init();
	#elif defined(LSM303DLHC)
	//	LSM303DLHC_Init();	
	#endif	
	
}
/*
 Mag_Data Mag_Read(void)
{
	#ifdef HMC5883
		return HMC5883_Read();
	#elif defined(LSM303DLHC)	
		return LSM303DLHC_Mag_Read();
	#endif
}

float getHeading(void){
	
	 Mag_Data data;

	data=Mag_Read();
		
	return computeHeading(&data);
}

float computeHeading( Mag_Data* data){
	float heading;
	
//	heading=atan2(data->y,data->x)*_RAD2DEG ;
		heading=atan2(data->z,data->x)*_RAD2DEG ;
	
	return heading;
}
*/
