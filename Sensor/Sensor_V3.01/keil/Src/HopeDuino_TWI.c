#include "stm32f1xx_hal.h"
#include "delay.h"
#include "stm32f1xx_hal_gpio.h"
#include "HopeDuino_TWI.h"

void SetTCLK(void)	//时钟线置位
{
  //GPIO_WriteBit(CMT2119_CLK_PORT, CMT2119_CLK_PIN, SET);
	HAL_GPIO_WritePin(CMT2119_CLK_PORT,CMT2119_CLK_PIN,GPIO_PIN_SET);
	
}

void ClrTCLK(void)
{
  //GPIO_WriteBit(CMT2119_CLK_PORT, CMT2119_CLK_PIN, RESET);
	HAL_GPIO_WritePin(CMT2119_CLK_PORT,CMT2119_CLK_PIN,GPIO_PIN_RESET);
}

void InputTDAT(void)
{
  //GPIO_Init(CMT2119_DATA_PORT, CMT2119_DATA_PIN, GPIO_Mode_In_PU_No_IT);
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	HAL_GPIO_WritePin(CMT2119_DATA_PORT, CMT2119_DATA_PIN, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = CMT2119_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(CMT2119_DATA_PORT, &GPIO_InitStruct);
}

void OutputTDAT(void)
{
  //GPIO_Init(CMT2119_DATA_PORT, CMT2119_DATA_PIN, GPIO_Mode_Out_PP_Low_Fast);
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(CMT2119_DATA_PORT, CMT2119_DATA_PIN, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = CMT2119_DATA_PIN;
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(CMT2119_DATA_PORT,&GPIO_InitStruct);
}

void SetTDAT(void)	
{
  //GPIO_WriteBit(CMT2119_DATA_PORT, CMT2119_DATA_PIN, SET);
	HAL_GPIO_WritePin(CMT2119_DATA_PORT, CMT2119_DATA_PIN, GPIO_PIN_SET);
}

void ClrTDAT(void)
{
  //GPIO_WriteBit(CMT2119_DATA_PORT, CMT2119_DATA_PIN, RESET);
	HAL_GPIO_WritePin(CMT2119_DATA_PORT, CMT2119_DATA_PIN, GPIO_PIN_RESET);
}

uint8_t TDAT_H(void)	
{
  uint8_t stat;
  //stat = (uint8_t)GPIO_ReadInputDataBit(CMT2119_DATA_PORT, CMT2119_DATA_PIN);
	if (HAL_GPIO_ReadPin(CMT2119_DATA_PORT,GPIO_PIN_13)==0u)
		stat=0;
	else if(HAL_GPIO_ReadPin(CMT2119_DATA_PORT,GPIO_PIN_13)==1u)
		stat=1;
  return stat; 
}
uint8_t TDAT_L(void)
{
  uint8_t stat;
  //stat = (uint8_t)GPIO_ReadInputDataBit(CMT2119_DATA_PORT, CMT2119_DATA_PIN);
	if (HAL_GPIO_ReadPin(CMT2119_DATA_PORT,GPIO_PIN_13)==0u)
		stat=0;
	else if(HAL_GPIO_ReadPin(CMT2119_DATA_PORT,GPIO_PIN_13)==1u)
		stat=1;
  return !stat; 
}


/**********************************************************
**Name: 	vTWIInit
**Func: 	Init Port & config
**Note: 	
**********************************************************/
void vTWIInit(void)
{
 // GPIO_Init(CMT2119_DATA_PORT, CMT2119_DATA_PIN, GPIO_Mode_Out_PP_Low_Fast);
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(CMT2119_DATA_PORT, CMT2119_DATA_PIN|CMT2119_CLK_PIN, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = CMT2119_DATA_PIN|CMT2119_CLK_PIN;
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(CMT2119_DATA_PORT,&GPIO_InitStruct);
 // GPIO_Init(CMT2119_CLK_PORT, CMT2119_CLK_PIN, GPIO_Mode_Out_PP_Low_Fast);
	
  SetTDAT(); 
  OutputTDAT();	
}

/**********************************************************
**Name: 	vTWIReset
**Func: 	TWI Reset 
**Note: 	
**********************************************************/
void vTWIReset(void)
{
  uint8_t bitcnt; 
  
  SetTCLK();			//CLK = 1;
  OutputTDAT();
  ClrTDAT();			//DAT = 0;
 	
  for(bitcnt=32; bitcnt!=0; bitcnt--)
  {
    SetTCLK();
    Delay_us(TWI_SPEED);
    ClrTCLK();
    Delay_us(TWI_SPEED);
  }
  SetTCLK();			//CLK = 1;
  ClrTDAT();			//DAT = 0;
  vTWIWrite(0x8D, 0x00);        //0x8D00 
}


/**********************************************************
**Name: 	vTWIWriteByte
**Func: 	TWI send one byte
**Note: 	
**********************************************************/
void vTWIWriteByte(uint8_t dat)
{
 	uint8_t bitcnt;	
 	
 	SetTCLK();			//CLK = 1;
 	
	OutputTDAT();
 	ClrTDAT();			//DAT = 0;
 		
 	for(bitcnt=8; bitcnt!=0; bitcnt--)
        {
          SetTCLK();
          if(dat&0x80) SetTDAT();
          else ClrTDAT();
          Delay_us(TWI_SPEED);
          ClrTCLK();
          Delay_us(TWI_SPEED);
          dat <<= 1;
        }
 	
 	SetTCLK();			//CLK = 1;
 	ClrTDAT();			//DAT = 0;
}

/**********************************************************
**Name: 	bTWIReadByte
**Func: 	TWI read one byte
**Note: 	
**********************************************************/
uint8_t bTWIReadByte(void)
{
  uint8_t RdPara = 0;
  uint8_t bitcnt;
  
  InputTDAT(); 
  SetTCLK();			//CLK = 1; 
   
  for(bitcnt=8; bitcnt!=0; bitcnt--)
  {
    SetTCLK();
    Delay_us(TWI_SPEED);
    RdPara <<= 1;
    ClrTCLK();
    Delay_us(TWI_SPEED);
    if(TDAT_H()) RdPara |= 0x01;
    else RdPara |= 0x00; 
  }
  SetTCLK();	
  OutputTDAT();
  ClrTDAT();
  return(RdPara);
}

/**********************************************************
**Name: 	vTWIWrite
**Func: 	TWI send one word
**Note: 	
**********************************************************/
void vTWIWrite(uint8_t adr, uint8_t dat)
{
  adr |= 0x80;			//Bit14=0 Write   
  adr &= 0xBF;				

  vTWIWriteByte(adr);
  vTWIWriteByte(dat);
}

/**********************************************************
**Name: 	bTWIRead
**Func: 	TWI read one word
**Note: 	
**********************************************************/
uint8_t bTWIRead(uint8_t adr)
{
  adr |= 0xC0;				//Bit14=1 Read	
  vTWIWriteByte(adr);			//地址
  return(bTWIReadByte());
}







