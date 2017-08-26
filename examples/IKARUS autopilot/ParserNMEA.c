#include "ParserNMEA.h"
#include "stm32f10x.h"
#include "led_driver.h"

enum Estados
	{
	WaitS, WaitG, WaitP, WaitCmd0, RxG0, RxG1, IsGGA, RxR0, RxM1, IsRMC,
	GGA_hora, GGA_lat, GGA_N, GGA_lon, GGA_W, GGA_fix, GGA_nsats, GGA_hdop, GGA_alt,
	RMC_hora, RMC_valid, RMC_lat, RMC_N, RMC_lon, RMC_W, RMC_velo, RMC_rumbo,
	GGA_WaitX, GGA_CRC, RMC_WaitX, RMC_CRC, AUX_CRC};

struct GGA gga;
struct RMC rmc;

float gps_f;
char gps_char;
char gps_sign;

unsigned char gps_tmp;
unsigned char gps_crc;

unsigned char recv_crc;
unsigned char calc_crc;
float gps_comma;

char NMEA_estado=WaitS;
//struct FiltroSimpleF fsAltitud, fsVelocidad;

GPSInfo gpsinfo;
char CheckedNMEA;
char ReceivedUART;


void GPS_Calculate() 
{
//	float relAlt;
//
//	gpsinfo.alt_filter=filtroSimpleF(&fsAltitud,gpsinfo.alt);
//				
//	relAlt=getRelAltitude();	
//	if(relAlt>gpsinfo.altitudeMAX)
//			gpsinfo.altitudeMAX=relAlt;
//
//	gpsinfo.knots_filtered=filtroSimpleF(&fsVelocidad,gpsinfo.knots);
//	
//	if(gpsinfo.knots_filtered>gpsinfo.knotsMAX)
//		gpsinfo.knotsMAX=gpsinfo.knots_filtered;		
}

			
float gps_adjust(float v)
{
	int deg=v/100;
	return deg+(v-deg*100)/60;
}


void nmea_fsm(char uart_data)
{
	static char skip=0;
	unsigned char i=0;
	
	gpsinfo.conected=1;
	gpsinfo.uart_timeout=0;
	
	if(uart_data=='$')
	{
		NMEA_estado=WaitG;
		gps_crc='$';		// El $ no se cuenta. '$' ^ '$' => 0
		calc_crc=0;
		recv_crc=0xff;
	}
	else if(uart_data=='*')			// comienza chksum
	{
		if(NMEA_estado==GGA_WaitX)
			NMEA_estado=GGA_CRC;
		else if(NMEA_estado==RMC_WaitX)
			NMEA_estado=RMC_CRC;
		else
			NMEA_estado=AUX_CRC;
		calc_crc=gps_crc;
		recv_crc=0;
		skip=0;
	}
	else if((uart_data==13||uart_data==10)&&skip==0)
	{
		skip=1;
		if(calc_crc==recv_crc)
		{
			gpsinfo.nmea_ok=1;
			gpsinfo.nmea_timeout=0;
			
			if(NMEA_estado==GGA_CRC)
			{
				if(gga.fix)
				{		
					gpsinfo.lon=gga.lon;
					gpsinfo.lat=gga.lat;
					gpsinfo.alt=gga.alt;			// Metrico (m)

					gpsinfo.pos_valid=1;
				}
				else
					gpsinfo.pos_valid=0;

				gpsinfo.hora=gga.hora;
				gpsinfo.fix=gga.fix;
				gpsinfo.numsats=gga.nsats;
				gpsinfo.hdop=gga.hdop;

				gpsinfo.GGA_received++;			
			}
			else if(NMEA_estado==RMC_CRC)
			{
				if(rmc.active=='A')
				{					
					gpsinfo.lon=rmc.lon;
					gpsinfo.lat=rmc.lat;
					gpsinfo.knots=rmc.knots;			
					gpsinfo.rumbo=rmc.rumbo;			
					
					if(gpsinfo.rumbo>180){
					gpsinfo.rumbo=180 - gpsinfo.rumbo;
					}
				
					gpsinfo.pos_valid=1;
				}
				else
					gpsinfo.pos_valid=0;

				gpsinfo.hora=rmc.hora;
				gpsinfo.act=rmc.active;

				gpsinfo.RMC_received++;
			}
			// Fix hora....
			for(i=0;i<3&&gpsinfo.hora>235959;i++)
				gpsinfo.hora/=10;
								
		}
		else
		{
			gpsinfo.bad_crc++;
			gpsinfo.nmea_ok=0;
		}
		NMEA_estado=WaitS;
	}
	else if(uart_data==',')
	{
		switch(NMEA_estado)
		{
			case IsGGA:
				NMEA_estado=GGA_hora;
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				break;

			case IsRMC:
				NMEA_estado=RMC_hora;
				gps_f=0;
				gps_comma=0;
				gps_sign=0;	
				break;
				
			case GGA_hora:
				gga.hora=gps_f;
				gps_f=0;
				gps_comma=0;
				NMEA_estado=GGA_lat;
				break;

			case GGA_lat:
				NMEA_estado=GGA_N;
				break;
			case GGA_N:
				if(gps_char=='S'||gps_char=='s')
					gps_f=-gps_f;

				gga.lat=gps_adjust(gps_f);
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				NMEA_estado=GGA_lon;
				break;

			case GGA_lon:
				NMEA_estado=GGA_W;
				break;

			case GGA_W:
				if(gps_char=='W'||gps_char=='w')
					gps_f=-gps_f;
				gga.lon=gps_adjust(gps_f);
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				NMEA_estado=GGA_fix;
				break;
		
			case GGA_fix:
				gga.fix=gps_char-'0';
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				NMEA_estado=GGA_nsats;
				break;

			case GGA_nsats:
				gga.nsats=(int)gps_f;
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				NMEA_estado=GGA_hdop;
				break;

			case GGA_hdop:
				gga.hdop=gps_f;
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				NMEA_estado=GGA_alt;
				break;
			
			case GGA_alt:
				gga.alt=gps_f;
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				NMEA_estado=GGA_WaitX;
				break;

			case RMC_hora:
				rmc.hora=gps_f;
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				NMEA_estado=RMC_valid;
				break;

			case RMC_valid:
				rmc.active=gps_char;
				gps_f=0;
				gps_comma=0;
				gps_sign=0;
				NMEA_estado=RMC_lat;
				break;


			case RMC_lat:
				NMEA_estado=RMC_N;
				break;

			case RMC_N:
				if(gps_char=='S'||gps_char=='s')
					gps_f=-gps_f;

				rmc.lat=gps_adjust(gps_f);
				gps_f=0;
				gps_comma=0;
				NMEA_estado=RMC_lon;
				break;

			case RMC_lon:
				NMEA_estado=RMC_W;
				break;

			case RMC_W:
				if(gps_char=='W'||gps_char=='w')
					gps_f=-gps_f;
				rmc.lon=gps_adjust(gps_f);
				gps_f=0;
				gps_comma=0;
				NMEA_estado=RMC_velo;
				break;

			case RMC_velo:
				rmc.knots=gps_f;
				gps_f=0;
				gps_comma=0;
				NMEA_estado=RMC_rumbo;
				break;

			case RMC_rumbo:
				rmc.rumbo=gps_f;
				gps_f=0;
				gps_comma=0;
				NMEA_estado=RMC_WaitX;
				break;

			default:
				break;
		}
	}
	else 
	{
		switch(NMEA_estado)
		{
			case WaitG:	
				if(uart_data=='G')
					NMEA_estado=WaitP;
				else
					NMEA_estado=WaitS;
				break;

			case WaitP:	
				if(uart_data=='P')
					NMEA_estado=WaitCmd0;
				else
					NMEA_estado=WaitS;
				break;
				
			case WaitCmd0:	
				if(uart_data=='G')
					NMEA_estado=RxG0;
				else if(uart_data=='R')
					NMEA_estado=RxR0;
				else
					NMEA_estado=WaitS;
				break;

			case RxG0:	
				if(uart_data=='G')
					NMEA_estado=RxG1;
				else
					NMEA_estado=WaitS;
				break;

			case RxR0:	
				if(uart_data=='M')
				{
					NMEA_estado=RxM1;
				}
				else
					NMEA_estado=WaitS;
				break;

			case RxG1:	
				if(uart_data=='A')
				{
					NMEA_estado=IsGGA;
				}
				else
					NMEA_estado=WaitS;
				break;

			case RxM1:	
				if(uart_data=='C')
				{
					NMEA_estado=IsRMC;
				}
				else
					NMEA_estado=WaitS;
				break;

			case IsRMC:
				break;

			case IsGGA:	
				break;

			case GGA_fix:
			case GGA_N:
			case GGA_W:
			case RMC_valid:
			case RMC_N:
			case RMC_W:
					gps_char=uart_data;
					break;
			case GGA_hora:
			case GGA_lat:
			case GGA_lon:
			case GGA_nsats:
			case GGA_hdop:
			case GGA_alt:
			case RMC_hora:
			case RMC_lat:
			case RMC_lon:
			case RMC_velo:
			case RMC_rumbo:
				if(uart_data=='-')
					gps_sign=1;
				else if(uart_data=='.')
					gps_comma=0.1f;
				else if(gps_comma==0)
				{
					if(gps_sign)
						gps_f=10*gps_f-(uart_data-'0');
					else
						gps_f=10*gps_f+uart_data-'0';
				}
				else
				{
					if(gps_sign)
						gps_f=gps_f-gps_comma*(uart_data-'0');
					else
						gps_f=gps_f+gps_comma*(uart_data-'0');
					gps_comma=gps_comma/10;
				}
				break;

			case GGA_CRC:
			case RMC_CRC:
			case AUX_CRC:
				recv_crc=recv_crc<<4;
				if(uart_data>='A'&&uart_data<='F')
					recv_crc|=uart_data - 'A' + 0x0A;
				else if(uart_data>='a'&&uart_data<='f')
					recv_crc|=uart_data - 'a' + 0x0A;
				if(uart_data>='0'&&uart_data<='9')
					recv_crc|=uart_data - '0';
				break;

			case GGA_WaitX:
			case RMC_WaitX:
			default:
				break;
		}
	}
	gps_crc^=uart_data;
}


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
	
	
	USART_InitStructure.USART_BaudRate = 9600;//38400;
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

		nmea_fsm((uint8_t)(USART2->DR & (uint16_t)0x00FF));

		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
   }
  
  if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
  {   

		USART_ClearITPendingBit(USART2, USART_IT_TC);


  
  }
}
