#include "Motion_Sensors.h"
#include "UM6_states.h"
#include "FreeImu.h"
#include "DCM.h"
#include "FLASH.h"
#include "telemetry.h"
#include "led_driver.h"
#include "Ikarus_IMU.h"

IkarusIMUConfig imu_config;

uint8_t imuCalibrateFlag;
uint8_t imuCalibrateFlashFlag;
uint8_t zeroGyrosFlag;


extern float accel_offset_x;
extern float accel_offset_y;
extern float accel_offset_z;


void Ikarus_IMU_Init(void){
	//Limpia banderas
	imuCalibrateFlag=0;
	imuCalibrateFlashFlag=0;
	zeroGyrosFlag=0;
	//Carga las configuraciones
	loadConfigFromFlash(&imu_config);
	Accel_Set_Calibration(imu_config.accel_offset_x, imu_config.accel_offset_y, imu_config.accel_offset_z);	
	//Inicializa los algoritmos
	EKF_Init(&imu_config.ekf_config);
	FreeImu_Init(&imu_config.free_imu_cfg);
	DCM_Init(&imu_config.dcm_cfg);
	
}

void copy_buff(uint8_t * dst, uint8_t * src, int len){
	int i;
	for(i=0;i<len;i++){
		dst[i]=src[i];
	}
}

void Ikarus_IMU_Request(uint8_t req){
	uint8_t temp_buff[256];
	Led_Toggle_Start(1);
	switch(req){
		case REQ_IMU_CFG_RAM:
			temp_buff[0]=REQ_IMU_CFG_RAM;
			copy_buff(&temp_buff[1],(uint8_t *)&imu_config,sizeof(imu_config));
			 send_data_packet(temp_buff, sizeof(imu_config)+1, DATA_REQ);
			break;
		case REQ_IMU_CFG_FLASH:
			temp_buff[0]=REQ_IMU_CFG_FLASH;
			copy_buff(&temp_buff[1],(uint8_t *)CONFIG_ADDR,sizeof(imu_config));
			 send_data_packet(temp_buff, sizeof(imu_config)+1, DATA_REQ);
			break;

		case REQ_EKF_CONFIG:
			break;
		case REQ_FI_CONFIG:
			break;
		case REQ_DCM_CONFIG:
			break;
	}
}

void Ikarus_IMU_Command(uint8_t cmd){
	Led_Toggle_Start(1);
	switch(cmd)
		{
		case RESET_IMU:
			NVIC_SystemReset();
			break;
		
		case WRITE_IMU_FLASH:
			writeConfigToFlash(&imu_config);
			break;
		
		case RELOAD_IMU_RAM:
				Ikarus_IMU_Init();
			break;
		
		case LOAD_FACTORY_IMU_FLASH:
			loadFactorySettings(&imu_config);
			writeConfigToFlash(&imu_config);
			NVIC_SystemReset();
			break;
		
		case CALIBRATE_IMU_FLASH:
			imuCalibrateFlag=1;
			imuCalibrateFlashFlag=1;
			break;
		
		case CALIBRATE_IMU_RAM:
			imuCalibrateFlag=1;
			imuCalibrateFlashFlag=0;
			break;
		
		case ZERO_GYROS:
			zeroGyrosFlag=1;
			break;
		
		case READ_IMU_FLASH:
		case READ_IMU_RAM: 
			break;
	}
}


void Ikarus_IMU_Calibrate(uint32_t flash_en){
			
			Led_Toggle_Cont();
			//Pone a cero los gyros
			Gyro_Calibrate();
			//Calibra el offset de los acelerometros
			Accel_Calibrate();
		
			//Actualiza la configuración en ram
			imu_config.accel_offset_x=accel_offset_x;
			imu_config.accel_offset_y=accel_offset_y;
			imu_config.accel_offset_z=accel_offset_z;

			if(flash_en){ //Si se habilita la escritura en flash
				IkarusIMUConfig cfg_temp;	
				//Lee la configuración de la flash, para luego actualizarle la calibración
				loadConfigFromFlash(&cfg_temp);
				cfg_temp.accel_offset_x=accel_offset_x;
				cfg_temp.accel_offset_y=accel_offset_y;
				cfg_temp.accel_offset_z=accel_offset_z;
				//Se escrib en la flash
				writeConfigToFlash(&cfg_temp);
			}
			
			//Inicializa los algoritmos
			EKF_Init(&imu_config.ekf_config);
			FreeImu_Init(&imu_config.free_imu_cfg);
			DCM_Init(&imu_config.dcm_cfg);
			Led_Toggle_Stop();
}


void loadConfigFromFlash(IkarusIMUConfig * cfg_ptr){
	IkarusIMUConfig * flash_ptr;
	flash_ptr=(IkarusIMUConfig *) CONFIG_ADDR;
	*cfg_ptr=*flash_ptr;
	if (cfg_ptr->check!=CHECK_VALUE){
		loadFactorySettings(cfg_ptr);
	}
}


void writeConfigToFlash(IkarusIMUConfig * cfg_ptr){
		uint32_t * prt;

		prt=(uint32_t *) cfg_ptr;
	
		Flash_Clear(CONFIG_ADDR); 
		Flash_Write_Buff(CONFIG_ADDR, prt,sizeof(IkarusIMUConfig));
	
 
}

void loadFactorySettings(IkarusIMUConfig * cfg_ptr){

		cfg_ptr->accel_offset_x=0;
		cfg_ptr->accel_offset_y=0;
		cfg_ptr->accel_offset_z=0;
		
		cfg_ptr->pitch_sel=FI;	
		cfg_ptr->roll_sel=FI;	

		cfg_ptr->free_imu_cfg.kp=0.5f;
		cfg_ptr->free_imu_cfg.ki=0.1f;
	
		cfg_ptr->ekf_config.process_var=0.1;
		cfg_ptr->ekf_config.accel_var=2.0;
		cfg_ptr->ekf_config.accel_ref_x=0.0;
		cfg_ptr->ekf_config.accel_ref_y=0.0;
		cfg_ptr->ekf_config.accel_ref_z=-1.0;
		cfg_ptr->ekf_config.mag_var=2.0;
		cfg_ptr->ekf_config.mag_ref_x=0.0;
		cfg_ptr->ekf_config.mag_ref_y=0.0;
		cfg_ptr->ekf_config.mag_ref_z=-1.0;
	

		
		cfg_ptr->dcm_cfg.kp=0.8f;
		cfg_ptr->dcm_cfg.ki=0.00002f;
	
		cfg_ptr->check=CHECK_VALUE;
		
		writeConfigToFlash(cfg_ptr);
		
}




void load_IMU_Config_From_Byte_Array(uint8_t buff[]){
	

	imu_config.accel_offset_x=*((float *) &buff[0]);
	imu_config.accel_offset_y=*((float *) &buff[4]);
	imu_config.accel_offset_z=*((float *) &buff[8]);
	imu_config.pitch_sel=buff[12];
	imu_config.roll_sel=buff[13];
	
	Accel_Set_Calibration(imu_config.accel_offset_x, imu_config.accel_offset_y, imu_config.accel_offset_z);	
	

}

void load_DCM_Config_From_Byte_Array(uint8_t buff[]){

	imu_config.dcm_cfg.kp=*((float *) &buff[0]);
	imu_config.dcm_cfg.ki=*((float *) &buff[4]);
	DCM_Init(&imu_config.dcm_cfg);

}

void load_Free_IMU_Config_From_Byte_Array(uint8_t buff[]){

	imu_config.free_imu_cfg.kp=*((float *) &buff[0]);
	imu_config.free_imu_cfg.ki=*((float *) &buff[4]);
	FreeImu_Init(&imu_config.free_imu_cfg);

}

void load_EFK_Config_From_Byte_Array(uint8_t buff[]){
	imu_config.ekf_config.process_var=*((float *) &buff[0]);
	imu_config.ekf_config.accel_var=*((float *) &buff[4]);
	imu_config.ekf_config.accel_ref_x=*((float *) &buff[8]);
	imu_config.ekf_config.accel_ref_y=*((float *) &buff[12]);
	imu_config.ekf_config.accel_ref_z=*((float *) &buff[16]);
	imu_config.ekf_config.mag_var=*((float *) &buff[20]);
	imu_config.ekf_config.mag_ref_x=*((float *) &buff[24]);
	imu_config.ekf_config.mag_ref_y=*((float *) &buff[28]);
	imu_config.ekf_config.mag_ref_z=*((float *) &buff[32]);
	EKF_Init(&imu_config.ekf_config);

}

