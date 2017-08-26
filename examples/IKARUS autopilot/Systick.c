#include "stm32f10x.h"
//#include "stm32f4xx_conf.h"


static uint32_t TimingDelay;
uint32_t timer=0;
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{

  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
	timer++;
}

uint8_t Init_SysTick(void)
{
	return (SysTick_Config(64000000/1000));//SystemCoreClock / 1000
}
