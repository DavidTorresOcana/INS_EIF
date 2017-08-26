#include "stm32f10x.h"
#include "Systick.h"

#include "Motion_Sensors.h"
#include "UM6_states.h"
#include "FreeImu.h"
#include "DCM.h"

#include "ParserNMEA.h"
#include "telemetry.h"
#include "FLASH.h"

#include "Ikarus_IMU.h"

#include "led_driver.h"

#include "EIF_n_dof.h"
#include "EIF_init.h"


#ifdef GPS
	extern GPSInfo gpsinfo;
#endif

 Accel_Data acel_data;
 Gyro_Data gyro_data;
 Mag_Data mag_data;
 uint16_t elapsedTime;
 float heading;
 float ypr_fi[3];
 float ypr_kf[3];
 float ypr_dcm[3];
 
 extern AHRS_state_data gStateData;
 

extern IkarusIMUConfig imu_config;

void processSensorsEFK(void);
real_T *fltcpy( float const *src_array, size_t len);
 
void GPIO_Configuration(void);
void RCC_Configuration(void);
void PWM_Init(void);
void setPWMoutput(void);

uint8_t readButton(void);


extern uint8_t imuCalibrateFlag;
extern uint8_t imuCalibrateFlashFlag;
extern uint8_t zeroGyrosFlag;

//EIF  variables
float ypr_eif[2];	
real_T eul[3] ={0,0,0};
int32_T number_sensors = 2;
char_T sensors[8] ={ 'g', 'y', 'r', 'o' , 'a', 'c', 'c', 'e' };

real_T X[7],X_out[7];
real_T P[49],P_out[49];
real_T Q[36];
real_T sigmas[2];

real_T AT;// ?????????
real_T measures[6];


//real_T gyro_stack=0, acell_stack=0,acell_stack1=0,sigma_gyro_test,sigma_acell_test;
// ////// MAIN////////////////
int main(void){
	
 int i=0,kk=0;
				   
	RCC_Configuration();
	 
	Led_Init();
	
	PWM_Init();
	Init_SysTick();

#ifndef GPS	
	telmetry_Init();
#else
	USART2_Init();
#endif
		
	
	Led_Toggle_Cont();
	Delay(1000);
	
	Motion_Sensors_Init();	
	elapsedTime=Motion_Sensors_Read(&mag_data, &acel_data, &gyro_data);
	Led_Toggle_Stop();

	Ikarus_IMU_Init();


	// EIF_init
	// eul_0[3] ={0,0,0};
	//number_sensors = 2;
	//sensors =['gyro';'acce'];
	
	EIF_init( eul, X, P, Q,sigmas);

	while(1)
	{		
		//kk++;
											 
		Delay(1);
		// processSensorsEFK();

		// Execute measurements
		elapsedTime=Motion_Sensors_Read(&mag_data, &acel_data, &gyro_data);
		
		measures[0] =  gyro_data.x*(PI/180.0);
		measures[1] =  -gyro_data.y*(PI/180.0);
		measures[2] = -gyro_data.z*(PI/180.0);

		measures[3] =  -acel_data.x;
		measures[4] = acel_data.y;
		measures[5] = acel_data.z; 

		// Characterisation of signals
		//gyro_stack= gyro_stack+pow(measures[1],2);
		//sigma_gyro_test = sqrt( gyro_stack/((float) kk) ) ; 
		//acell_stack1= acell_stack1+measures[4];
		//acell_stack= acell_stack+pow(measures[4]-(acell_stack1/((float) kk)),2);
		//sigma_acell_test = sqrt( acell_stack/((float) kk) ) ;
		 
		// EIF 
		AT = 0.0000005f*elapsedTime; // typically 0.011
		EIF_n_dof( (const real_T*) X,(const real_T*) P,(const real_T*)Q,(const real_T*) sigmas,(const real_T*) measures , (const real_T) AT,  eul,X_out, P_out);	   // AT has to be defined correctly!!!!
		
		//X=fltcpy( (const real_T*) X_out, (size_t) 7);
		memcpy(X,X_out,7*sizeof(real_T));
		memcpy(P,P_out,49*sizeof(real_T));		 


		//ypr_eif[0] = (float) measures[0]*(180.0/PI);	  // This is for data collection
		//ypr_eif[1] = (float) measures[5]*45.0;			   
		ypr_eif[0] = eul[1]*(180.0/PI);	 // in degs
		ypr_eif[1] = eul[0]*(180.0/PI);	 // in degs
		// end EIF

		setPWMoutput();
		
				
		if(imuCalibrateFlag==1){
			Ikarus_IMU_Calibrate(imuCalibrateFlashFlag);
			imuCalibrateFlag=0;
			
		}
		if(zeroGyrosFlag==1){
			Led_ON();
			Gyro_Calibrate();
			zeroGyrosFlag=0;
			Led_OFF();
		}
		
		if(readButton()==1 && imuCalibrateFlag==0){
			imuCalibrateFlag=1;
		}
#ifndef GPS		
		if(i>=10){
			i=0;
			send_telemetry();
		}else{
			i++;
		}
#else  
		if(gpsinfo.pos_valid){
			Led_ON();
		}else{
			Led_OFF();
		}
		
#endif
	}
}

// Copy float arrays
real_T * fltcpy( const real_T *src_array, size_t len){
	real_T *p=malloc(len*sizeof(real_T));

	memcpy(p,src_array,len*sizeof(real_T));

	return p;
}
void processSensorsEFK(void)
{
		elapsedTime=Motion_Sensors_Read(&mag_data, &acel_data, &gyro_data);
		
		//Kalman
		 // Copy new gyro data to state_data structure
		gStateData.gyro_x =gyro_data.x;
		gStateData.gyro_y =-gyro_data.y;
		gStateData.gyro_z =gyro_data.z;
	 

		gStateData.accel_x =acel_data.x;
		gStateData.accel_y =-acel_data.y;
		gStateData.accel_z =-acel_data.z;
	


		EKF_EstimateStates();
		 
		ypr_kf[0]=gStateData.psi;
		ypr_kf[1]=gStateData.theta;
		ypr_kf[2]=gStateData.phi;
		
		//Free_imu
		getYawPitchRoll(ypr_fi);
		
		//DCM
		Calculate_DCM();
}

#define V_AMPL 512


void setPWMoutput(void){
	
		float pitch, roll;
		
// 		switch (imu_config.pitch_sel){
// 			case FI:
// 				pitch = ypr_fi[1];
// 			break;
// 			case EKF:
// 				pitch = ypr_kf[1];
// 			break;
// 			case DCM:
// 				pitch = ypr_dcm[1];
// 			break;			
// 		}
// 	
// 		
// 		switch (imu_config.roll_sel){
// 			case FI:
// 				roll = ypr_fi[2];
// 			break;
// 			case EKF:
// 				roll = ypr_kf[2];
// 			break;
// 			case DCM:
// 				roll = ypr_dcm[2];
// 			break;			
// 		}

		//pitch = ypr_dcm[1];
		//roll = ypr_dcm[2];	
		
		//EIF angles
		pitch = ypr_eif[0];
		roll = ypr_eif[1];

		if(pitch >90.0f)
			pitch = 90.0f;
		else if(pitch <-90.0f)
			pitch = -90.0f;
		
		if(roll > 90.0f)
			roll = 90.0f;
		else if (roll < -90.0f)
			roll = -90.0f;
		
		TIM_SetCompare2(TIM1, 512+(int)(V_AMPL*pitch/90.f));
		TIM_SetCompare3(TIM1, 512+(int)(V_AMPL*roll/90.f));

}
		

void assert_failed(uint8_t * p, uint32_t u){
	
}

 /* set up clock sources */ 
void RCC_Configuration(void) { 
	/* reset */ 
	RCC_DeInit(); 
	/* activate HSI (Internal High Speed oscillator) */ 
	RCC_HSICmd(ENABLE); 
	while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET); 
	/* configure prescalers */
	RCC_HCLKConfig(RCC_SYSCLK_Div1);	// HCLK = 64 MHz, AHB
	RCC_PCLK1Config(RCC_HCLK_Div2); // APB1 = 32 MHz 
	RCC_PCLK2Config(RCC_HCLK_Div1); // APB2 = 64 MHz 
	/* set up FLASH */ 
	FLASH_SetLatency(FLASH_Latency_2); 
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); 
	/* set up PLL */ 
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16); // 64 MHz 
	RCC_PLLCmd(ENABLE); 
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); 
	/* set up SYSCLK */ 
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //SYSCLK = PLLCLK => 64 MHz 
	while (RCC_GetSYSCLKSource() != 0x08); 

 } 



//PA11
uint8_t readButton(void){
	uint8_t bitstatus;
	  if ((GPIOA->IDR & GPIO_Pin_11) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}


