#ifndef __CAN_H
#define __CAN_H	 

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


/* Define---------------------------------------------------------------------*/
#define STD_Frame 0                //标准帧标志
#define EXT_Frame 1                //扩展帧标志
#define CAN_OK    0                
#define CAN_ERROR 1

/*extern----------------------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern CanTxMsgTypeDef TxMessage;
extern CanRxMsgTypeDef RxMessage;
	 
/*Export Function	------------------------------------------------------------*/

void MyCAN_FilterConf(CAN_HandleTypeDef* phcan,CAN_FilterConfTypeDef* psFilterConfig);      //CAN滤波器设置
uint8_t MyCAN_TransMessage(CAN_HandleTypeDef* phcan,uint32_t frame_type,uint32_t can_id);   //CAN发送数据
void CAN_TX_Init(CAN_HandleTypeDef* phcan, uint32_t can_id);


#endif

