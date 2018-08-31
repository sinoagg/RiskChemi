#ifndef __TFT_H
#define __TFT_H	 

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"	
#include "Delay.h"

/*define----------------------------------------------------------------------------*/
#define SHOW 0x01
#define HIDE 0x00
#define DOOR1_RIGHT	0x60
#define DOOR1_LEFT	0x62
#define DOOR2_RIGHT	0x64						//ƽ��ֵȡ������
#define DOOR2_LEFT	0x66
#define DOOR3_RIGHT	0x68
#define DOOR3_LEFT	0x6A
#define DOOR4_RIGHT	0x6C
#define DOOR4_LEFT	0x6F

#define SEAT_NUM1		0x70
#define SEAT_NUM2 	0x72
#define SEAT_NUM3		0x74
#define SEAT_NUM4 	0x76
#define SEAT_NUM5		0x78
#define SEAT_NUM6 	0x7A	

/*extern----------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern uint8_t TFT_RxBuf[110];
extern uint8_t TFT_ACK[6];
	 
/*export function-------------------------------------------------------------------*/
void TFT_Init(void);
void TFT_StartWairning(uint8_t pos_addr, uint8_t state);  //ǰ���ű��� 60ǰ��1�� 62ǰ��2�� 64����1�� 66����2�� ��1 �ر�0
void TFT_TransTemHumi(uint16_t tem,uint16_t humi);	      //������ʪ��ֵ
void TFT_SendWairning(void);
void TFT_StartDesktop(uint8_t addr);

#endif



