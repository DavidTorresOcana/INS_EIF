
#include "stm32f10x.h"

#include "telemetry.h"

#include "Motion_Sensors.h"

#include "UM6_states.h"
#include "FreeImu.h"
#include "DCM.h"
#include "Ikarus_IMU.h"

//Estructura con la telemetria

Telemetry_info tele_data;

//Estructuras externas

extern  Accel_Data acel_data;
extern Gyro_Data gyro_data;
extern Mag_Data mag_data;
extern float heading;
extern float ypr_fi[];
extern float ypr_kf[];
extern float ypr_dcm[];
extern uint16_t elapsedTime;
extern AHRS_state_data gStateData;

//Buffer de transmision
char buff_tx[256];
uint16_t tx_length;
uint16_t tx_idx;

//buffer de rececpcion
#define TEL_BUFF_SIZE 520
char buff_rx[TEL_BUFF_SIZE];

uint16_t rx_length;
uint16_t tel_idx;
unsigned char tel_crc;

typedef enum {TEL_IDLE=0, TEL_LEN_MSB, TEL_LEN_LSB, TEL_DATA, TEL_CRC} Tel_States ;
Tel_States tel_state;

void USART2_Init(void);

void telmetry_Init(void){

	tel_state=TEL_IDLE;
	tel_idx=0;
	rx_length=0;
	tx_length=0;
	tx_idx=0;
	USART2_Init();
}


void telmetry_rx_parse(void){

	

	switch(buff_rx[0]){
		
		case SET_IMU_CFG:
			load_IMU_Config_From_Byte_Array( (uint8_t*) &buff_rx[1]);		
			break;
		case SET_EKF_CONFIG:
			load_EFK_Config_From_Byte_Array( (uint8_t*) &buff_rx[1]);		
			break;
		case SET_FI_CONFIG:
			load_Free_IMU_Config_From_Byte_Array( (uint8_t*) &buff_rx[1]);
			break;
		case SET_DCM_CONFIG:
			load_DCM_Config_From_Byte_Array( (uint8_t*) &buff_rx[1]);
			break;
		case DATA_REQ:
			Ikarus_IMU_Request(buff_rx[1]);
			break;
		case RESET_IMU:
		case WRITE_IMU_FLASH:
		case READ_IMU_FLASH:
		case LOAD_FACTORY_IMU_FLASH:
		case RELOAD_IMU_RAM:
		case READ_IMU_RAM: 
    case CALIBRATE_IMU_FLASH:
		case CALIBRATE_IMU_RAM:
		case ZERO_GYROS:
			Ikarus_IMU_Command(buff_rx[0]);
		break;
		

	}


}

void telemetry_rx(char data){
	switch(tel_state){
		case TEL_IDLE:
			if(data==0x7e){
				tel_idx=0;
				rx_length=0;
				tel_state=TEL_LEN_MSB;
				tel_crc=0;
			}
			break;
		case TEL_LEN_MSB:
			rx_length = data <<8;
			tel_state=TEL_LEN_LSB;
			break;		
		case TEL_LEN_LSB:
			rx_length |= data;
			rx_length--;	//quitamos el crc
			
			if(rx_length>512){
				tel_state=TEL_IDLE;
			}else{
				tel_state=TEL_DATA;
			}
			break;		
		case TEL_DATA:
			buff_rx[tel_idx]=data;
			tel_crc+=data;
			tel_idx++;
			if(tel_idx == rx_length){	//El CRC se recoge en el siguiente estado
				tel_state=TEL_CRC;
			}
			break;		
		case TEL_CRC:
			tel_crc=0xff-tel_crc;
			if(tel_crc==data){
				telmetry_rx_parse();
			}
			tel_state=TEL_IDLE;
			break;				
	}
}




void update_telemetry_info(void){
	//Hora RTC
//	RTC_TimeTypeDef time;
	


	tele_data.compass_heading=heading;
	

	
	tele_data.acel_x=acel_data.x;
	tele_data.acel_y=acel_data.y;
	tele_data.acel_z=acel_data.z;

	tele_data.gyro_x=gyro_data.x; 
	tele_data.gyro_y=gyro_data.y; 
	tele_data.gyro_z=gyro_data.z; 
	
// 	tele_data.mag_x=mag_data.x; 
// 	tele_data.mag_y=mag_data.y; 
// 	tele_data.mag_z=mag_data.z; 	
	
	tele_data.fi_pitch=ypr_fi[1];
	tele_data.fi_roll=ypr_fi[2];
	tele_data.fi_yaw=ypr_fi[0];
	
 	tele_data.kf_pitch=ypr_kf[1]; 
	tele_data.kf_roll=ypr_kf[2]; 
	tele_data.kf_yaw=ypr_kf[0];
	
	tele_data.dcm_pitch=-ypr_dcm[1]; 
	tele_data.dcm_roll=ypr_dcm[2]; 
	tele_data.dcm_yaw=ypr_dcm[0];
	tele_data.period=1/(elapsedTime*0.0000005f);

	//Hora RTC
/*	RTC_GetTime(RTC_Format_BIN, &time);
	
	tele_data.hour=time.RTC_Hours;
	tele_data.minute=time.RTC_Minutes;
	tele_data.second=time.RTC_Seconds;
*/	


}

void send_data_packet(uint8_t * ptr, int size, char frame_id){
	
	build_data_packet((char *) ptr , size, frame_id, buff_tx, &tx_length);
	
	//LANZAR TRANSMISION
	tx_idx=0;
	USART_SendData(USART2,buff_tx[0]);
}

void send_telemetry(void){
	update_telemetry_info();
	build_data_packet((char *) &tele_data , sizeof(Telemetry_info), TELEMETRY_INFO, buff_tx, &tx_length);
	
	//LANZAR TRANSMISION
	tx_idx=0;
	USART_SendData(USART2,buff_tx[0]);
}

void build_data_packet(char *ptr, uint16_t tele_length,char frame_id, char * buff, uint16_t * buff_legnth){

		uint16_t packet_length;

	unsigned char crc=0;
	uint16_t i;

	packet_length=tele_length+2; //Long=id+paquete+crc
	
	buff[0]=0x7E;
	buff[1]=packet_length>>8;
	buff[2]=packet_length%256;
	buff[3]=	frame_id; //Frame id
	crc=frame_id;
	for(i=0;i<tele_length;i++){
		buff[i+4]=*ptr++;
		crc+=buff[i+4];
	}
	crc=0xff-crc;
	buff[packet_length+2]=crc; //Ultima posicion del buffer
	
	*buff_legnth=packet_length+3;	//Tamaño total del paquete: datos a transmitir
	
}

#ifndef GPS

void USART2_Init(){
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	
	  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	 
	
	
  /* Configure USARTy Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  
  /* Configure USARTy Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USARTy */
  USART_Init(USART2, &USART_InitStructure);
  
  /* Enable USARTy Receive and Transmit interrupts */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	
  /* Enable the USARTy */
  USART_Cmd(USART2, ENABLE);
	
	
	  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void USART2_IRQHandler(void)
{
 if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {

//		telemetry_rx(USART_ReceiveData(USART2));
		telemetry_rx((uint16_t)(USART2->DR & (uint16_t)0x01FF));
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
   }
  
  if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
  {   
		tx_idx++;
		if(tx_idx>=tx_length){
			tx_idx=0;
		}else{
			//USART_SendData(USART2,buff_tx[tx_idx]);	
			USART2->DR = (buff_tx[tx_idx] & (uint16_t)0x01FF);
		}
		USART_ClearITPendingBit(USART2, USART_IT_TC);


  
  }
}

#endif;
