
#define FI 0
#define EKF 1
#define DCM 2

typedef struct{
	float accel_offset_x;
	float accel_offset_y;
	float accel_offset_z;
	uint8_t pitch_sel;
	uint8_t roll_sel;
	FreeImu_Config  free_imu_cfg;
	EKF_Config ekf_config;
	DCM_Config  dcm_cfg;
	uint32_t check;
}IkarusIMUConfig;

typedef struct{
	uint32_t serialNumber;
	
} Version;

void Ikarus_IMU_Init(void);
void Ikarus_IMU_Request(uint8_t req);
void Ikarus_IMU_Command(uint8_t cmd);
void Ikarus_IMU_Calibrate(uint32_t flash_en);

void loadFactorySettings(IkarusIMUConfig * cfg_ptr);
void writeConfigToFlash(IkarusIMUConfig * cfg_ptr);
void loadConfigFromFlash(IkarusIMUConfig * cfg_ptr);

void load_IMU_Config_From_Byte_Array(uint8_t buff[]);
void load_EFK_Config_From_Byte_Array(uint8_t buff[]);
void load_Free_IMU_Config_From_Byte_Array(uint8_t buff[]);
void load_DCM_Config_From_Byte_Array(uint8_t buff[]);



#define CONFIG_ADDR 0x0801FC00

#define CHECK_VALUE 0x12345678

#define REQ_IMU_CFG_RAM 0x00
#define REQ_IMU_CFG_FLASH 0x01
#define REQ_EKF_CONFIG  0x02	//Configuracion Kalman 
#define REQ_FI_CONFIG  0x03	//Configuracion Free IMU
#define REQ_DCM_CONFIG  0x04	//Configuracion DCM

