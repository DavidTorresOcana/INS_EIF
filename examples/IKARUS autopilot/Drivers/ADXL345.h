
#define ADXL345_ADDR_HIGH 	0x1D
#define ADXL345_ADDR_LOW 	0x53

#define POWER_CTL 		0x2D	//Power Control Register
#define DATA_FORMAT   0x31
#define BW_RATE				0x2C
#define OFSX					0x1E
#define OFSY					0x1F
#define OFSZ					0x20


#define DATAX0 				0x32	//X-Axis Data 0
#define DATAX1 				0x33	//X-Axis Data 1
#define DATAY0 				0x34	//Y-Axis Data 0
#define DATAY1 				0x35	//Y-Axis Data 1
#define DATAZ0 				0x36	//Z-Axis Data 0
#define DATAZ1 				0x37	//Z-Axis Data 1



void ADXL345_Init(void);
 Accel_Data ADXL345_Read(void);
void Accel_Calibrate(void);

