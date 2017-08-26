#include "stm32f10x.h"
#include "led_driver.h"

void Init_Tim3(void);
uint8_t led_status;
uint8_t led_toggle_en;
int16_t led_time;
 //LED EN PA12
void Led_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	Led_OFF();
	led_toggle_en=0;
	led_time=0;
	
	Init_Tim3();
	
}

void Init_Tim3(void){

	  NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 64000-1;	//1miliSeg
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 100;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); 

	/* TIM7 enable counter */
  TIM_Cmd(TIM3, ENABLE);


	
	  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void Led_ON(void){
	GPIOA->BSRR = GPIO_Pin_12;
	led_status=1;
}

void Led_OFF(void){
	GPIOA->BRR = GPIO_Pin_12;
	led_status=0;
}


void Led_Toggle(void){
	if(led_status==0){
		Led_ON();
	}else{
		Led_OFF();
	}
}


void Led_Toggle_Start(int16_t time){
	led_toggle_en=1;
	led_time=10*time;
}
void Led_Toggle_Stop(void){
	led_toggle_en=0;
	Led_OFF();
}

void Led_Toggle_Cont(void){
	Led_Toggle_Start(CONT_TOOGLE);
}

void TIM3_IRQHandler(void)
{

 if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)
  {
		if(led_toggle_en==1){
				Led_Toggle();
			if(led_time!=CONT_TOOGLE){
				if(led_time==0){
					led_toggle_en=0;
					Led_OFF();
				}else{
					led_time--;
				}
			}
		}

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
 
  }
}


