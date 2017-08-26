#include "I2CLib.h"
#include "MS5611.h"
#include "Math.h"

void Delay(uint32_t nTime);

uint16_t C1, C2, C3, C4, C5, C6;
uint32_t D1, D2;

uint16_t MS5611_Read_PROM(uint8_t cmd)
{
	uint8_t buff[2];
	uint16_t res=0;
	I2C_WrBuf(MS5611, &cmd, 1);
	I2C_RdBuf(MS5611, buff, 2);
	res=(buff[0]<<8)|buff[1];
	return res;
}

uint32_t MS5611_Read_ADC(uint8_t cmd)
{
	uint8_t buff[3];
	uint32_t res = 0;
	uint8_t cmd2 = MS5611_CMD_ADC_READ;

	I2C_WrBuf(MS5611, &cmd, 1);
	Delay(20);
	I2C_WrBuf(MS5611, &cmd2, 1);
	I2C_RdBuf(MS5611, buff, 3);
	res=(buff[0]<<16)|(buff[1]<<8)|buff[2];
	return res;
}


void MS6511_Init(void)
{
	uint8_t cmd;

	cmd = MS5611_CMD_RESET; // reset
	I2C_WrBuf(MS5611, &cmd, 1);

    Delay(10);

	C1 = MS5611_Read_PROM( MS5611_CMD_PROM_READ_C1);
	C2 = MS5611_Read_PROM( MS5611_CMD_PROM_READ_C2);
	C3 = MS5611_Read_PROM( MS5611_CMD_PROM_READ_C3);
	C4 = MS5611_Read_PROM( MS5611_CMD_PROM_READ_C4);
	C5 = MS5611_Read_PROM( MS5611_CMD_PROM_READ_C5);
	C6 = MS5611_Read_PROM( MS5611_CMD_PROM_READ_C6);
}


int32_t MS6511_Read(void)
{
	int32_t dT, TEMP;
	int64_t OFF, SENS;
	int32_t P;

	int32_t T2;
	int64_t OFF2, SENS2;

	D1 = MS5611_Read_ADC (MS5611_CMD_D1_OSR_4096);
	D2 = MS5611_Read_ADC (MS5611_CMD_D2_OSR_4096);

//	C1 = 40127;
//	C2 = 36924;
//	C3 = 23317;
//	C4 = 23282;
//	C5 = 33464;
//	C6 = 28312;
//	D1 = 9085466;
//	D2 = 8569150;

	dT = D2 - C5*(1<<8);
	TEMP = 2000 + dT * C6 / (1<<23);
	OFF = (int64_t)C2 * (1<<16) + (C4*dT)/(1<<7);
	SENS = C1 * (1<<15) + (C3*dT)/(1<<8);

/*
	if(TEMP<2000)
	{
		T2 = (int64_t)dT * dT / (1<<31);
		OFF2 = 5 * (TEMP - 2000)*(TEMP - 2000) / 2;
		SENS2 = OFF2 / 2;
		if(TEMP<-1500)
		{
			OFF2 = OFF2 + 7 * (TEMP + 1500)*(TEMP + 1500); 
			SENS2 = SENS2 + 11 * (TEMP + 1500)*(TEMP + 1500)/2;
		}	
	}
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	*/
	
	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	P= (D1 * SENS / (1<<21) - OFF) /(1<<15);
	return P;
}

