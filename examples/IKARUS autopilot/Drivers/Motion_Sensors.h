
#ifndef MOTSEN_H
#define MOTSEN_H

#include "misc.h"

#ifndef PI
	#define PI 3.14159265359f
#endif

//Estructuras de datos
typedef  struct 
{
		float x, y, z;	//g
}Accel_Data;

typedef struct 
{
	float x, y,z;			//º/s
}Gyro_Data;

typedef struct 
{
	float x,y,z;
}Mag_Data;





//Funciones comunes
void Motion_Sensors_Init(void);
uint16_t Motion_Sensors_Read( Mag_Data* md, Accel_Data* ad, Gyro_Data* gd);
short big_endian(uint8_t *buff);

//ACELEROMETRO
void Accel_Init(void);
 Accel_Data Accel_Read(void);
void Accel_Calibrate(void);
void Accel_Set_Calibration(float x, float y, float z);
//GIROSCOPO
void Gyro_Init(void);
 Gyro_Data Gyro_Read(void);
void Gyro_Calibrate(void);


//MAGNETRONOMO
void Mag_Init(void);
Mag_Data Mag_Read(void);
float getHeading(void);
float computeHeading( Mag_Data* data);




#endif
