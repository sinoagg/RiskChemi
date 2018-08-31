#include "delay.h"

uint8_t fac_us=72;               //主频72M

void Delay_Init(uint8_t SYSCLK)  //延时us初始化函数
{
  #if SYSTEM_SUPPORT_OS 
    u32 reload;
  #endif
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  fac_us=SYSCLK; 
    
  #if SYSTEM_SUPPORT_OS 
    reload=SYSCLK; 
    reload*=1000000/delay_ostickspersec; 
    fac_ms=1000/delay_ostickspersec; 
    SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;
    SysTick->LOAD=reload; 
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; 
  #else
  #endif
}

void Delay_us(uint32_t nus)         //延时us
{
    uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD; 
    ticks=nus*fac_us; 
    told=SysTick->VAL; 
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break; 
        }
    }
}
