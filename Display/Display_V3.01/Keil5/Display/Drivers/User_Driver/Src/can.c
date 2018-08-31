#include "can.h"

#define CAN_TX_LEN 8

/*CAN滤波器设置------------------------------------------------------------------------------------------------*/		
void MyCAN_FilterConf(CAN_HandleTypeDef* phcan,CAN_FilterConfTypeDef* psFilterConfig)   
{
	psFilterConfig->FilterNumber=0;                                //滤波器组0
	psFilterConfig->FilterMode=CAN_FILTERMODE_IDMASK;
	psFilterConfig->FilterScale=CAN_FILTERSCALE_32BIT;
	psFilterConfig->FilterIdHigh=0x0000;                           
	psFilterConfig->FilterIdLow=0x0000;
	psFilterConfig->FilterMaskIdHigh=0x0000;
	psFilterConfig->FilterMaskIdLow=0x0000;
	psFilterConfig->BankNumber=14;                                //banknumber接收到的信息从哪一块存取
	psFilterConfig->FilterFIFOAssignment=CAN_FIFO0;
	psFilterConfig->FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&(*phcan), &(*psFilterConfig));
	
	HAL_CAN_Receive_IT(&(*phcan), CAN_FIFO0);                     //使能CAN RX0接收中断
}

/*CAN发送配置---------------------------------------------------------------------------------------------------*/
void CAN_TX_Init(CAN_HandleTypeDef* phcan, uint32_t can_id)
{
	static CanRxMsgTypeDef RxMessage;      //接收消息
	static CanTxMsgTypeDef TxMessage;			//发送消息
	
	hcan.pTxMsg = &TxMessage;
  hcan.pRxMsg = &RxMessage;
	
	hcan.pTxMsg->DLC= CAN_TX_LEN;
	hcan.pTxMsg->IDE = CAN_ID_EXT;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
	hcan.pTxMsg->ExtId = can_id;

}

/*接收中断回调函数-----------------------------------------------------------------------------------------------*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* phcan)
{
	if(phcan->Instance==CAN1)
	{
		
		HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);                     //使能CAN RX0接收中断
	}
}
	

