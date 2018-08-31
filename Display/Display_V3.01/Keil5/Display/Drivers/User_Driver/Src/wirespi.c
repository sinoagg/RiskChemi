#include "wirespi.h"



// CSB--------------------------------------------------
void SetCSB(void)       //CSB置高
{
	HAL_GPIO_WritePin(GPIOB, CSB_Pin, GPIO_PIN_SET);
}
void ClrCSB(void)       //CSB置低
{
  HAL_GPIO_WritePin(GPIOB, CSB_Pin, GPIO_PIN_RESET);
}
// FCSB--------------------------------------------------
void SetFCSB(void)      //FCSB拉高
{
  HAL_GPIO_WritePin(GPIOB, FCSB_Pin, GPIO_PIN_SET);
}
void ClrFCSB(void)      //FCSB置低
{
  HAL_GPIO_WritePin(GPIOB, FCSB_Pin, GPIO_PIN_RESET);
}
// CLK--------------------------------------------------	
void SetSDCK(void)      //CLK拉高
{
  HAL_GPIO_WritePin(GPIOB, SCL_Pin, GPIO_PIN_SET);
}
void ClrSDCK(void)      //CLK置低
{
  HAL_GPIO_WritePin(GPIOB, SCL_Pin, GPIO_PIN_RESET);
}
// DIO--------------------------------------------------
void SetSDIO(void)      //SDA拉高
{
  HAL_GPIO_WritePin(GPIOB, SDA_Pin, GPIO_PIN_SET);
}
void ClrSDIO(void)      //SDA置低
{
  HAL_GPIO_WritePin(GPIOB, SDA_Pin, GPIO_PIN_RESET);
}
// DIO MODE CHANGE--------------------------------------------------
void InputSDIO(void)    //SDA配置为输入模式
{
  GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void OutputSDIO(void)   //SDA配置为输出模式
{
  GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
	
uint8_t SDIO_H(void)   //读取SDA引脚电平
{
  unsigned char stat;
  stat =  HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin);
  return stat; 
}
uint8_t SDIO_L(void)   //读取SDA引脚电平并取反
{
  unsigned char stat;
  stat =  HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin);
  return !stat; 
}

void vSpi3Init(void)    //初始化无线接收模块的输出配置引脚
{	
  HAL_GPIO_WritePin(GPIOB, FCSB_Pin|CSB_Pin|SDA_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
}
/**********************************************************
**Name: 	bSpi3WriteByte
**Func: 	SPI-3 write one byte
**Input:
**Output:  
**********************************************************/
void vSpi3WriteByte(uint8_t dat) 
{

  uint8_t bitcnt;	
  SetFCSB();				
  OutputSDIO();		
  SetSDIO();				
  ClrSDCK();				
  ClrCSB();
  Delay_us(SPI3_SPEED);
  for(bitcnt=8; bitcnt>0; bitcnt--)
  {
		ClrSDCK();	
		Delay_us(SPI3_SPEED);
		if(dat&0x80)
			SetSDIO();
		else
			ClrSDIO();
		SetSDCK();
		dat <<= 1; 		
		Delay_us(SPI3_SPEED);
  }
  ClrSDCK();		
  SetSDIO();
}

/**********************************************************
**Name: 	bSpi3ReadByte
**Func: 	SPI-3 read one byte
**Input:
**Output:  
**********************************************************/
uint8_t bSpi3ReadByte(void)
{
  uint8_t RdPara = 0;
  uint8_t bitcnt;

  ClrCSB(); 
  InputSDIO();			
  Delay_us(SPI3_SPEED);
  for(bitcnt=8; bitcnt!=0; bitcnt--)
  {
    ClrSDCK();
    RdPara <<= 1;
    Delay_us(SPI3_SPEED);
    SetSDCK();
    Delay_us(SPI3_SPEED);
    if(SDIO_H())
      RdPara |= 0x01;
    else
      RdPara |= 0x00;
  } 
  ClrSDCK();
  OutputSDIO();
  SetSDIO();
  SetCSB();			
  return(RdPara);	
}
/**********************************************************
**Name:	 	vSpi3Write
**Func: 	SPI Write One word
**Input: 	Write word
**Output:	none
**********************************************************/
void vSpi3Write(uint16_t dat)
{
  vSpi3WriteByte((uint8_t)(dat>>8)&0x7F);
  vSpi3WriteByte((uint8_t)dat);
  SetCSB();
}

/**********************************************************
**Name:	 	bSpi3Read
**Func: 	SPI-3 Read One byte
**Input: 	readout addresss
**Output:	readout byte
**********************************************************/
uint8_t bSpi3Read(uint8_t addr)
{
  vSpi3WriteByte(addr|0x80);
  return(bSpi3ReadByte());
}

/**********************************************************
**Name:	 	vSpi3WriteFIFO
**Func: 	SPI-3 send one byte to FIFO
**Input: 	one byte buffer
**Output:	none
**********************************************************/
void vSpi3WriteFIFO(uint8_t dat)
{
  uint8_t bitcnt;	

  SetCSB();	
  OutputSDIO();	
  ClrSDCK();
  ClrFCSB();			
  for(bitcnt=8; bitcnt!=0; bitcnt--)
  {
    ClrSDCK();
    
    if(dat&0x80)
      SetSDIO();		
    else
      ClrSDIO();
    Delay_us(SPI3_SPEED);
    SetSDCK();
    Delay_us(SPI3_SPEED);
    dat <<= 1;
  }
  ClrSDCK();	
  Delay_us(SPI3_SPEED);		//Time-Critical
  Delay_us(SPI3_SPEED);		//Time-Critical
  SetFCSB();
  SetSDIO();
  Delay_us(SPI3_SPEED);		//Time-Critical
  Delay_us(SPI3_SPEED);		//Time-Critical
}

/**********************************************************
**Name:	 	bSpi3ReadFIFO
**Func: 	SPI-3 read one byte to FIFO
**Input: 	none
**Output:	one byte buffer
**********************************************************/
uint8_t bSpi3ReadFIFO(void)
{
  uint8_t RdPara=0;
  uint8_t bitcnt;	
  
  SetCSB();
  InputSDIO();
  ClrSDCK();
  Delay_us(10);
  ClrFCSB();
  Delay_us(50);
  /*if(SDIO_H())
      RdPara |= 0x01;		//NRZ MSB
    else
      RdPara |= 0x00;		//NRZ MSB
  */
  for(bitcnt=0; bitcnt<8; bitcnt++)
  {
    RdPara <<= 1;
    SetSDCK();
    Delay_us(SPI3_SPEED);
    if(SDIO_H())
      RdPara |= 0x01;		//NRZ MSB
    else
      RdPara |= 0x00;		//NRZ MSB
    ClrSDCK();
    Delay_us(SPI3_SPEED);
    
  }
 	
  ClrSDCK();
  Delay_us(SPI3_SPEED);		//Time-Critical
  Delay_us(SPI3_SPEED);		//Time-Critical
  SetFCSB();
  Delay_us(10);
  //OutputSDIO();
  //SetSDIO();
  Delay_us(SPI3_SPEED);		//Time-Critical
  Delay_us(SPI3_SPEED);		//Time-Critical
  return(RdPara);
}

/**********************************************************
**Name:	 	vSpi3BurstWriteFIFO
**Func: 	burst wirte N byte to FIFO
**Input: 	array length & head pointer
**Output:	none
**********************************************************/
void vSpi3BurstWriteFIFO(uint8_t ptr[], uint8_t length)
{
  uint8_t i;
  if(length!=0x00)
  {
    for(i=0;i<length;i++)
      vSpi3WriteFIFO(ptr[i]);
  }
  return;
}

/**********************************************************
**Name:	 	vSpiBurstRead
**Func: 	burst wirte N byte to FIFO
**Input: 	array length  & head pointer
**Output:	none
**********************************************************/
void vSpi3BurstReadFIFO(uint8_t ptr[], uint8_t length)
{
  uint8_t i;
  if(length!=0)
  {
    for(i=0;i<length;i++)
      ptr[i] = bSpi3ReadFIFO();
  }	
  return;
}

