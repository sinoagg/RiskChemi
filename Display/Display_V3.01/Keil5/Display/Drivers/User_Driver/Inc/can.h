#ifndef __CAN_H
#define __CAN_H	 

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


/* Define---------------------------------------------------------------------*/
#define STD_Frame 0                //��׼֡��־
#define EXT_Frame 1                //��չ֡��־
#define CAN_OK    0                
#define CAN_ERROR 1

/*extern----------------------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern CanTxMsgTypeDef TxMessage;
extern CanRxMsgTypeDef RxMessage;
	 
/*Export Function	------------------------------------------------------------*/

void MyCAN_FilterConf(CAN_HandleTypeDef* phcan,CAN_FilterConfTypeDef* psFilterConfig);      //CAN�˲�������
uint8_t MyCAN_TransMessage(CAN_HandleTypeDef* phcan,uint32_t frame_type,uint32_t can_id);   //CAN��������
void CAN_TX_Init(CAN_HandleTypeDef* phcan, uint32_t can_id);


#endif

