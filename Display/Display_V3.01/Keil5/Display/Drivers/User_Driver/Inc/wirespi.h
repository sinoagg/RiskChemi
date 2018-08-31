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
void SetCSB(void);       //CSB�ø�
void ClrCSB(void);       //CSB�õ�
void SetFCSB(void);      //FCSB����
void ClrFCSB(void);      //FCSB�õ�
void SetSDCK(void);      //CLK����
void ClrSDCK(void);      //CLK�õ�	 
void SetSDIO(void);      //SDA����	 
void ClrSDIO(void);      //SDA�õ�	 
void InputSDIO(void);    //SDA����Ϊ����ģʽ	 
void OutputSDIO(void);   //SDA����Ϊ���ģʽ	 
uint8_t SDIO_H(void);    //��ȡSDA���ŵ�ƽ	 
uint8_t SDIO_L(void);    //��ȡSDA���ŵ�ƽ��ȡ��
void vSpi3WriteByte(uint8_t dat);   
uint8_t bSpi3ReadByte(void);

/*Export Function	------------------------------------------------------------*/
void vSpi3Init(void);                  //��ʼ�����߽���ģ��������������
void vSpi3Write(uint16_t dat);         //SPIд����
uint8_t bSpi3Read(uint8_t addr);       //SPI������
void vSpi3WriteFIFO(uint8_t dat);      //SPIдFIFO
uint8_t bSpi3ReadFIFO(void);           //SPI��FIFO
void vSpi3BurstWriteFIFO(uint8_t ptr[], uint8_t length);
void vSpi3BurstReadFIFO(uint8_t ptr[], uint8_t length);

#endif

