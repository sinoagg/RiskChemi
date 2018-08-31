
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "can.h"
#include "delay.h"
#include "wirespi.h"
#include "cmt2219a.h"
#include "tft.h"
#include "internalflash.h"
#include "prj_typedef.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define WIFI_RX_LEN 12
#define WIRE_TRUE 1                           //中断发生接收到了数据
#define WIRE_FALSE 0                          //未收到数据
#define ALARM_ON 0                            //开启报警
#define ALARM_OFF 1                           //关闭报警
#define CAN_TX_LEN 8													//CAN传输长度
#define CAN_TX_TIME 3000
#define CAN_ID 0x18FEE9D8

extern uint8_t CfgTbl[];                      //无线配置表数组
extern uint8_t frame_valid;                   //无线接收中断标志

uint8_t WIFI_RxBuf[WIFI_RX_LEN]={0,1,0};          //无线接收缓冲数组
uint8_t TFT_RxBuf[110]={0};                     //串口接收缓冲数组
uint8_t TFT_ACK[6]={0};
uint32_t Alarm_state[2]={ALARM_ON,ALARM_ON};
SensorsTypeDef hsensor;


uint8_t TFTRxComplete=0;


uint8_t alarm_pos=0;                          //出现危化品位置值
uint16_t alarm_cnt=0;													//报警计时	
uint8_t alarm_send=0;
//uint8_t alarm_state=ALARM_ON;                //报警状态
//uint8_t alarm_volume=ALARM_ON;                //报警音量
uint8_t value_xor=0;                          //异或值校验
//uint8_t selfcheck_state=0;                   //can是否自检
uint8_t selfcheck_cnt=0;                     //自检计数因子
//float front_alarm_value=0.01;                 //前门报警阈值
//float rear_alarm_value=0.01;                  //后门报警阈值
//float alarm_value=0.01;                       //报警阈值
//uint16_t tim2_cnt=0;
uint8_t CAN_Tx_Flag=0;
uint8_t CAN_AlarmTxBuf[8]={0,0,0,0,0,0xFF,0xFF,0xFF};
uint8_t CAN_TxBuf[8]={0,0,0,0,0,0xFF,0xFF,0xFF};
cmt2219aClass radio;                          //无线模块配置

CAN_FilterConfTypeDef  sFilterConfig;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void vCMT2219_Setup(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
uint8_t i=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	hsensor.HUMI.number_float=50;
	hsensor.TEMP.number_float=20;
	CAN_TX_Init(&hcan, CAN_ID);
	Delay_Init(72);                           //配置us延时初始化函数
	vCMT2219_Setup();                             //配置无线模块参数
	MyCAN_FilterConf(&hcan,&sFilterConfig);       //配置CAN过滤器组并使能接收中断
	HAL_TIM_Base_Start_IT(&htim3);
	//读flash中报警状态
	LoadSetArray(FLASH_USER_START_ADDR+RISKCHEMI_ALARMSET_ADDR, Alarm_state,2);
	Delay_us(500000);
	TFT_StartDesktop(*Alarm_state);	
	Delay_us(3000);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/**************************串口屏通信**********************************/
		HAL_GPIO_TogglePin(GPIOB, LED_Pin);
		HAL_UART_Receive_DMA(&huart2, TFT_RxBuf, 110);
		if(TFTRxComplete==1)
		{
			TFTRxComplete=0;
			if((TFT_RxBuf[4]==0x30)&&(TFT_RxBuf[7]==0xAA))
			{
				if(TFT_RxBuf[8]==0xB0)      
				{
					if((TFT_RxBuf[9]==0x00)&&(TFT_RxBuf[10]==0x00))                            //关闭报警
					{
						*Alarm_state=ALARM_OFF;
						FlashWrite_ArrayUint32(FLASH_USER_START_ADDR+RISKCHEMI_ALARMSET_ADDR, Alarm_state,2);
					}
					else if((TFT_RxBuf[9]==0x01)&&(TFT_RxBuf[10]==0x00))                      //打开报警
					{
						*Alarm_state=ALARM_ON;
						FlashWrite_ArrayUint32(FLASH_USER_START_ADDR+RISKCHEMI_ALARMSET_ADDR, Alarm_state,2);
					}
				}
				else if(TFT_RxBuf[8]==0xB1) 
				{
					if(TFT_RxBuf[9]==0x01)                            //关闭声音
					{
						*(Alarm_state+1)=ALARM_OFF;
						FlashWrite_ArrayUint32(FLASH_USER_START_ADDR+RISKCHEMI_ALARMSET_ADDR, Alarm_state,2);
					}
					else                                              //打开声音
						{
						*(Alarm_state+1)=ALARM_ON;
						FlashWrite_ArrayUint32(FLASH_USER_START_ADDR+RISKCHEMI_ALARMSET_ADDR, Alarm_state,2);
					}
				}
			}
			if((TFT_RxBuf[4]==0x40)&&(TFT_RxBuf[5]==0x01)&&(TFT_RxBuf[6]==0x02))
				alarm_send=1;																				//发送报警按钮按下
		}

	  /*************************无线数据接收****************************************/
		if(frame_valid==WIRE_TRUE)
		{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          {
			frame_valid=WIRE_FALSE;                        //外部中断标志清零
			HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
			HAL_NVIC_DisableIRQ(EXTI2_IRQn);               //关闭外部中断即无线接收中断
			bCMT2219_GetMessage(WIFI_RxBuf, WIFI_RX_LEN);  //获取包数据
			value_xor=0;
			for(uint8_t i=0;i<WIFI_RX_LEN-1;i++)                      //校验接收到的数据
			{
				value_xor^=WIFI_RxBuf[i];
			}
			if(WIFI_RxBuf[WIFI_RX_LEN-1]==value_xor&&value_xor!=0)//如果接受到正确的帧头信息并且不是全0错误数据
			{
				//只收进来温度湿度数据
				hsensor.TEMP.number_uchar[3]=WIFI_RxBuf[3];
				hsensor.TEMP.number_uchar[2]=WIFI_RxBuf[4];
				hsensor.TEMP.number_uchar[1]=WIFI_RxBuf[5];
				hsensor.TEMP.number_uchar[0]=WIFI_RxBuf[6];
				hsensor.HUMI.number_uchar[3]=WIFI_RxBuf[7];
				hsensor.HUMI.number_uchar[2]=WIFI_RxBuf[8];
				hsensor.HUMI.number_uchar[1]=WIFI_RxBuf[9];
				hsensor.HUMI.number_uchar[0]=WIFI_RxBuf[10];
				TFT_TransTemHumi((uint16_t)hsensor.TEMP.number_float,(uint16_t)hsensor.HUMI.number_float);//更新温湿度
				
				CAN_TxBuf[3]=	(uint16_t)((hsensor.TEMP.number_float+64.f)/0.5);	//温度值
				CAN_TxBuf[4]=(uint16_t)(hsensor.HUMI.number_float/0.5);	//湿度值
				CAN_AlarmTxBuf[3]=	(uint16_t)((hsensor.TEMP.number_float+64.f)/0.5);	//温度值
				CAN_AlarmTxBuf[4]=(uint16_t)(hsensor.HUMI.number_float/0.5);	//湿度值
			}				
			vCMT2219_Setup();
			HAL_NVIC_EnableIRQ(EXTI2_IRQn);                //使能外部中断
		}	
	}
		Delay_us(999);
		/****************************报警处理**********************************/
		if(*Alarm_state==ALARM_ON)
		{
			if(WIFI_RxBuf[1]==0X00)     //如果是报警信号
			{
				WIFI_RxBuf[1]=0XFF;
				alarm_pos=WIFI_RxBuf[2];
				alarm_cnt=5000;           //报警3000ms
			}
			
			if(*(Alarm_state+1)==ALARM_OFF)
					HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_RESET);
			if(alarm_cnt>0)
				alarm_cnt--;
				//报警图像处理	
			if(alarm_cnt==4999) 
			{
				TFT_SendWairning();
				if(*(Alarm_state+1)==ALARM_ON)
					HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_SET);
				Delay_us(100);
				switch (alarm_pos)
				{
					case DOOR1_LEFT:
						TFT_StartWairning(DOOR1_LEFT, SHOW);
						CAN_AlarmTxBuf[0]|=1<<4;
						CAN_AlarmTxBuf[1]|=1<<6;
						break;
					case DOOR1_RIGHT:
						TFT_StartWairning(DOOR1_RIGHT, SHOW);
						CAN_AlarmTxBuf[0]|=1<<4;
						CAN_AlarmTxBuf[1]|=1<<7;
						break;
					case DOOR2_LEFT:
						TFT_StartWairning(DOOR2_LEFT, SHOW);
						CAN_AlarmTxBuf[0]|=1<<3;	
						CAN_AlarmTxBuf[1]|=1<<4;
						break;
					case DOOR2_RIGHT:
						TFT_StartWairning(DOOR2_RIGHT, SHOW);
						CAN_AlarmTxBuf[0]|=1<<3;
						CAN_AlarmTxBuf[1]|=1<<5;
						break;
					case DOOR3_LEFT:
						TFT_StartWairning(DOOR3_LEFT, SHOW);
						CAN_AlarmTxBuf[0]|=1<<2;
						CAN_AlarmTxBuf[1]|=1<<2;
						break;
					case DOOR3_RIGHT:
						TFT_StartWairning(DOOR3_RIGHT, SHOW);
						CAN_AlarmTxBuf[0]|=1<<2;
						CAN_AlarmTxBuf[1]|=1<<3;
						break;
					case DOOR4_LEFT:
						TFT_StartWairning(DOOR4_LEFT, SHOW);
						CAN_AlarmTxBuf[0]|=1<<1;
						CAN_AlarmTxBuf[1]|=1;
						break;
					case DOOR4_RIGHT:
						TFT_StartWairning(DOOR4_RIGHT, SHOW);
						CAN_AlarmTxBuf[0]|=1<<1;
						CAN_AlarmTxBuf[1]|=1<<1;
						break;
					case SEAT_NUM1:
						TFT_StartWairning(SEAT_NUM1, SHOW);
						CAN_AlarmTxBuf[0]|=1<<5;
						CAN_AlarmTxBuf[2]|=1;
						break;
					case SEAT_NUM2:
						TFT_StartWairning(SEAT_NUM2, SHOW);
						CAN_AlarmTxBuf[0]|=1<<5;
						CAN_AlarmTxBuf[2]|=1<<1;
						break;
					case SEAT_NUM3:
						TFT_StartWairning(SEAT_NUM3, SHOW);
						CAN_AlarmTxBuf[0]|=1<<5;
						CAN_AlarmTxBuf[2]|=1<<2;
						break;
					case SEAT_NUM4:
						TFT_StartWairning(SEAT_NUM4, SHOW);
						CAN_AlarmTxBuf[0]|=1<<5;
						CAN_AlarmTxBuf[2]|=1<<3;
						break;
					case SEAT_NUM5:
						TFT_StartWairning(SEAT_NUM5, SHOW);
						CAN_AlarmTxBuf[0]|=1<<5;
						CAN_AlarmTxBuf[2]|=1<<4;
						break;
					case SEAT_NUM6:
						TFT_StartWairning(SEAT_NUM6, SHOW);
						CAN_AlarmTxBuf[0]|=1<<5;
						CAN_AlarmTxBuf[2]|=1<<5;
						break;
					default: break ;
				}
			}
			else if(alarm_cnt==1)															//报警最后消除 
			{
				for(i=0;i<3;i++)
					CAN_AlarmTxBuf[i]=0;
				TFT_Init();
				HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_RESET);
			}
		}				
		else if((alarm_cnt>0)&&(*Alarm_state==ALARM_OFF))
		{
			for(i=0;i<3;i++)
					CAN_AlarmTxBuf[i]=0;
			alarm_cnt=0;
			TFT_Init();
			TFT_StartDesktop(*Alarm_state);
			HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_RESET);
		}
		/***************CAN**********************/
		if(CAN_Tx_Flag==1)
		{
			CAN_Tx_Flag=0;
			if(selfcheck_cnt>19)
			{
				if(alarm_send!=1)
				{
					for(i=0;i<8;i++)
						hcan.pTxMsg->Data[i]=CAN_TxBuf[i];
					HAL_CAN_Transmit(&hcan,100);
				}
				
				else
				{
					TFT_Init();
					for(i=0;i<8;i++)
						hcan.pTxMsg->Data[i]=CAN_AlarmTxBuf[i];
					HAL_CAN_Transmit(&hcan,100);
					alarm_send=0;
				}	
			}
			else
			{
				selfcheck_cnt++;
				hcan.pTxMsg->Data[0]=0x01;
				for(i=1;i<8;i++)
						hcan.pTxMsg->Data[i]=CAN_TxBuf[i];
				HAL_CAN_Transmit(&hcan,100);
			}	
		}
 	
	
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
 } 
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_7TQ;
  hcan.Init.BS2 = CAN_BS2_8TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FCSB_Pin|SCL_Pin|SDA_Pin|CSB_Pin 
                          |Buzzer_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : nRSTO_Pin CLKO_Pin */
  GPIO_InitStruct.Pin = nRSTO_Pin|CLKO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT1_Pin */
  GPIO_InitStruct.Pin = INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DOUT_Pin */
  GPIO_InitStruct.Pin = DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FCSB_Pin SCL_Pin SDA_Pin CSB_Pin */
  GPIO_InitStruct.Pin = FCSB_Pin|SCL_Pin|SDA_Pin|CSB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin LED_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void vCMT2219_Setup(void)            //设置无线接收模块
{
  radio.CrcDisable     = SET;
  radio.FixedPktLength = SET;
  radio.NodeDisable    = SET;
  radio.PktLength      = 16;
  vCMT2219_Init(CfgTbl, &radio);
  vCMT2219_GpioFuncCfg(GPIO1_POR|GPIO2_INT1|GPIO3_CLK|GPIO4_DOUT);     //GPIO1默认为nRST，GPIO2默认为INT1，GPIO3改为DOUT，GPIO4改为DCLK
  
  //vIntSourcCfg((FIFO_WBYTE+OFFSET), 0);                               //Hope自带驱动内容
  //vCMT2219_IntOutputCfg(INT1_PIN, 3);
	vCMT2219_IntOutputCfg(INT1_PIN, INT_SOURCE_RX_PACKET_DONE);           //INT1输出中断信号，中断信号来源为接收packet完成
  vCMT2219_EnableIntSource(0xFF);                                       //使能所有中断
  vCMT2219_GoRx();                                                      //转入接收模式
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	CAN_Tx_Flag=1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
