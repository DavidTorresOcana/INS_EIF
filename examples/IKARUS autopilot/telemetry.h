
//Defifinicion de los frame ID
//COMANDOS CON DATOS
#define SET_FULL_IMU_CFG 			0x00	//Configuracion general de la imu
#define SET_IMU_CFG 			0x01	//Configuracion general de la imu
#define SET_EKF_CONFIG  0x02	//Configuracion Kalman 
#define SET_FI_CONFIG  0x03	//Configuracion Free IMU
#define SET_DCM_CONFIG  0x04	//Configuracion DCM

//COMANDOS SIN DATOS
#define RESET_IMU								0x08
#define WRITE_IMU_FLASH					0x09	
#define READ_IMU_FLASH					0x0A
#define LOAD_FACTORY_IMU_FLASH	0x0B	
#define RELOAD_IMU_RAM					0x0C	
#define READ_IMU_RAM						0x0D	
#define CALIBRATE_IMU_FLASH 		0x0E
#define CALIBRATE_IMU_RAM 			0x0F
#define ZERO_GYROS 							0x10

//INFORMACION DE SALIDA
#define TELEMETRY_INFO 	0x20

#define DATA_REQ 0x30




#pragma push
#pragma pack(1) 

//Estructura para los datos de telemetria
typedef struct {
	
	float compass_heading;
	float acel_x;
	float acel_y;
	float acel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float fi_pitch;
	float fi_roll;
	float fi_yaw;
	float kf_pitch;
	float kf_roll;
	float kf_yaw;
	float dcm_pitch;
	float dcm_roll;
	float dcm_yaw;
	float period;
	
	//Hora RTC
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	

	
} Telemetry_info;

#pragma pop

void telmetry_Init(void);
void update_telemetry_info(void);
void send_data_packet(uint8_t * ptr, int size, char frame_id);
void send_telemetry(void);
void build_data_packet(char *ptr, uint16_t tele_length,char frame_id, char * buff, uint16_t * buff_legnth);
void telemetry_rx(char data);

