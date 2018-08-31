#include "tft.h"


void TFT_Init(void)
{
	HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_RESET);
	uint8_t TFT_SWINTERFACE[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x00};					//切换主界面
	HAL_UART_Transmit(&huart2, TFT_SWINTERFACE, 10, 1000);
	Delay_us(100);
	TFT_StartWairning(DOOR1_LEFT, HIDE);
	Delay_us(100);
	TFT_StartWairning(DOOR1_RIGHT, HIDE);
	Delay_us(100);
	TFT_StartWairning(DOOR2_LEFT, HIDE);
	Delay_us(100);
	TFT_StartWairning(DOOR2_RIGHT, HIDE);
//	Delay_us(100);
//	TFT_StartWairning(DOOR3_LEFT, HIDE);
//	Delay_us(100);
//	TFT_StartWairning(DOOR3_RIGHT, HIDE);
//	Delay_us(100);
//	TFT_StartWairning(DOOR4_LEFT, HIDE);
//	TFT_StartWairning(DOOR4_RIGHT, HIDE);
	Delay_us(100);
	TFT_StartWairning(SEAT_NUM1, HIDE);
	Delay_us(100);
	TFT_StartWairning(SEAT_NUM2, HIDE);
	Delay_us(100);
	TFT_StartWairning(SEAT_NUM3, HIDE);
	Delay_us(100);
	TFT_StartWairning(SEAT_NUM4, HIDE);
	Delay_us(100);
	TFT_StartWairning(SEAT_NUM5, HIDE);
	Delay_us(100);
	TFT_StartWairning(SEAT_NUM6, HIDE);
}

/*前后门报警 60前门1号 62前门2号 64后门1号 66后门2号 打开1 关闭0*/
void TFT_StartWairning(uint8_t pos_addr, uint8_t state)
{
	uint8_t TFT_SWCOMMAND[8]={0x5A,0xA5,0x05,0x82,0x10,0x00,0x00,0x01};
	TFT_SWCOMMAND[5]=pos_addr;
	TFT_SWCOMMAND[7]=state;
//	HAL_UART_AbortReceive_IT(&huart2);
	HAL_UART_Transmit(&huart2, TFT_SWCOMMAND, 8, 1000);
//	while(HAL_UART_Receive(&huart2,TFT_ACK,6,2000)!=HAL_OK);
//	HAL_UART_Receive_IT(&huart2, TFT_RxBuf, 11);
}

/*切换发送报警界面*/
void TFT_SendWairning(void)
{
	uint8_t TFT_SWINTERFACE[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x11};					//切换发送界面
	HAL_UART_Transmit(&huart2, TFT_SWINTERFACE, 10, 1000);
}	

/*切换主界面*/
void TFT_StartDesktop(uint8_t addr)
{
	uint8_t TFT_SWINTERFACE[10]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,0x00};					
	TFT_SWINTERFACE[9]=addr;
	HAL_UART_Transmit(&huart2, TFT_SWINTERFACE, 10, 1000);
}	


/*更新温湿度值*/
void TFT_TransTemHumi(uint16_t tem,uint16_t humi)
{
	uint8_t TFT_THCOMMAND[8]={0x5A,0xA5,0x05,0x82,0x10,0x6A,0x00,0x00};
	uint8_t TFT_TTCOMMAND[8]={0x5A,0xA5,0x05,0x82,0x10,0x6B,0x00,0x00};
	TFT_TTCOMMAND[6]=((tem&0xFF00)>>8);
	TFT_TTCOMMAND[7]=(tem&0x00FF);
//	HAL_UART_AbortReceive_IT(&huart2);
	HAL_UART_Transmit(&huart2, TFT_TTCOMMAND, 8, 1000);
//	while(HAL_UART_Receive(&huart2,TFT_ACK,6,2000)!=HAL_OK);
//	HAL_UART_Receive_IT(&huart2, TFT_RxBuf, 11);
	TFT_THCOMMAND[6]=((humi&0xFF00)>>8);
	TFT_THCOMMAND[7]=(humi&0x00FF);
//	HAL_Delay(10);
//	HAL_UART_AbortReceive_IT(&huart2);
	HAL_UART_Transmit(&huart2, TFT_THCOMMAND, 8, 1000);
//	while(HAL_UART_Receive(&huart2,TFT_ACK,6,2000)!=HAL_OK);
//	HAL_UART_Receive_IT(&huart2, TFT_RxBuf, 11);
}

