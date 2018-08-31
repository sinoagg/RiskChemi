#ifndef __WIRESPI_H
#define __WIRESPI_H	 

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "delay.h"

/* Define---------------------------------------------------------------------*/
#define	SPI3_SPEED	50

/*Private Function----------------------------------------------------------------------*/
void SetCSB(void);       //CSB置高
void ClrCSB(void);       //CSB置低
void SetFCSB(void);      //FCSB拉高
void ClrFCSB(void);      //FCSB置低
void SetSDCK(void);      //CLK拉高
void ClrSDCK(void);      //CLK置低	 
void SetSDIO(void);      //SDA拉高	 
void ClrSDIO(void);      //SDA置低	 
void InputSDIO(void);    //SDA配置为输入模式	 
void OutputSDIO(void);   //SDA配置为输出模式	 
uint8_t SDIO_H(void);    //读取SDA引脚电平	 
uint8_t SDIO_L(void);    //读取SDA引脚电平并取反
void vSpi3WriteByte(uint8_t dat);   
uint8_t bSpi3ReadByte(void);

/*Export Function	------------------------------------------------------------*/
void vSpi3Init(void);                  //初始化无线接收模块的输出配置引脚
void vSpi3Write(uint16_t dat);         //SPI写数据
uint8_t bSpi3Read(uint8_t addr);       //SPI读数据
void vSpi3WriteFIFO(uint8_t dat);      //SPI写FIFO
uint8_t bSpi3ReadFIFO(void);           //SPI读FIFO
void vSpi3BurstWriteFIFO(uint8_t ptr[], uint8_t length);
void vSpi3BurstReadFIFO(uint8_t ptr[], uint8_t length);

#endif

