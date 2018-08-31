/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "sht20.h"
#include "delay.h"
#include "CMT2119.h"
#include "num_to_txpacket.h"
#include "sensors.h"
#include "prj_typedef.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/




#define LOCATION DOOR1_RIGHT				//第一门右侧
//#define LOCATION DOOR1_LEFT		   		//第一门左侧
//#define LOCATION DOOR2_RIGHT				//第二门右侧
//#define LOCATION DOOR2_LEFT					//第二门左侧
//#define LOCATION DOOR3_RIGHT				//第三门右侧
//#define LOCATION DOOR3_LEFT					//第三门左侧
//#define LOCATION DOOR4_RIGHT				//第四门右侧
//#define LOCATION DOOR4_LEFT					//第四门左侧

//#define LOCATION SEAT_NUM1				//座位1号
//#define LOCATION SEAT_NUM2		   	//座位2号
//#define LOCATION SEAT_NUM3				//座位3号
//#define LOCATION SEAT_NUM4				//座位4号
//#define LOCATION SEAT_NUM5				//座位5号
//#define LOCATION SEAT_NUM6				//座位6号









#define TIMEOUT 300									//每TIMEOUT * 100ms传输一次温度  300
#define DOOR1_RIGHT	0x60
#define DOOR1_LEFT	0x62
#define DOOR2_RIGHT	0x64						//平均值取样个数
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
#define SEAT_NUM6 	0x7A						//平均值取样个数
#define PRE_HEAT_TIME	180						//预热时间PRE_HEAT_TIME * 1s  180   （把1改成180）  
#define TX_LEN 21										//传输数据长度
#define ALARM_TIME	5								//ALARM_TIME  报警传输次数	
#define TEST_TIME_INTERVAL	100			//检测周期
#define ALARM_CONFIRM_TIMES 3				//ALARM_CONFIRM_TIMES+1次检测到值再报警   1



#define Alarm_level1_value   2.7
#define Alarm_level2_value   3.0

const uint16_t CfgTbl[21] = {
                     0x007F,         // Mode                = Advanced                                             	
                     0x5000,         // Part Number         = CMT2119A                         
                     0x0000,         // Frequency           = 868.35 MHz                          
                     0x0000,         // Modulation          = FSK                              
                     0x0000,         // Symbol Rate         = 0.5-100.0 ksps                   
                     0xF000,         // Tx Power            = +13 dBm                          
                     0x0000,         // Deviation           = 35.0 kHz                         
                     0xCBCE,         // PA Ramping Time     = NA                               
                     0x4208,         // Xtal Cload          = 15.00 pF                         
                     0x00B0,         // Data Representation = 0:F-low,1:F-high                 
                     0x6401,         // Tx Start by         = DATA Pin Rising Edge            
                     0x0081,         // Tx Stop by          = DATA Pin Holding Low For 20 ms   
                     0x8000,         // Increase XO Current = No                               
                     0x0000,         // FILE CRC            = D150                             
                     0xFFFF,
                     0x0020,
                     0x5FF0,
                     0xA2D6,
                     0x0E13,
                     0x0019,
                     0x0000
                 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void PushData(FIFOTypeDef *pFIFO, float data);
void WirelessTxPacket(uint8_t* pTxPacket, uint8_t timeout, SensorsTypeDef* phsensors);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
SensorsTypeDef hsensors;
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t alarm_times=0;								//检测报警次数， 设定N次之后才认为是报警			
	uint8_t TxPacket[TX_LEN]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x2D,0xD4,0xA5,0xAA};			//前导码 6个0xAA  SYNC WORD 0xAA2DD4 数据
  uint16_t i=0, timeout=0;
	//SensorsTypeDef hsensors;
	uint8_t alarm_cnt=0;	//表示报警状态，只有alrm_cnt非零时表示在报警
	uint8_t beep_cnt=0;
//	uint8_t bit0=0,bit1=0,bit2=0;
//	uint8_t sensitivity=0;
	float TGS2602_WARNING_SLOPE=1.03;	  //TGS2602传感器报警斜率
  float M010_WARNING_SLOPE=1.03;			//M010传感器报警斜率
	float MC101_WARNING_SLOPE=0.9;			//MC101传感器报警斜率
	uint8_t A=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  delay_init(72);												//延时函数初始化
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
//	MX_TIM2_Init();
	

  /* USER CODE BEGIN 2 */
	Delay_ms(2000);																					//给无线芯片上电时间
	vCMT2119AInit(CfgTbl, 21);															//配置无线芯片
	SHT20_init();																						//SHT20传感器初始化
//	for(i=0;i<PRE_HEAT_TIME;i++) Delay_ms(1000);						//延时180秒用于传感器预热，否则开始电压值就很大
//	for(i=0;i<LOCATION*4;i++) Delay_ms(100);								//根据位置差异化时间传输
	
	hsensors.TEMP.number_float=SHT20_GetTempPoll();					//获取初始温度值
	hsensors.HUMI.number_float=SHT20_GetHumiPoll();					//获取初始湿度值
	
	//获取初始化FIFO值和加和值
	SensorsGetValue(&hsensors);															//获取传感器的值
	FIFOTypeDef hFIFO_TGS2602={hsensors.TGS2602_value.number_float*AVG_NUM, hsensors.TGS2602_value.number_float};			//初始化TGS2602 FIFO
	FIFOTypeDef hFIFO_M010={hsensors.M010_value.number_float*AVG_NUM, hsensors.M010_value.number_float};
	FIFOTypeDef hFIFO_MC101={hsensors.MC101_value.number_float*AVG_NUM, hsensors.MC101_value.number_float};	//初始化M010 FIFO
	for(i=0;i<AVG_NUM;i++) 																																														//添加FIFO里的data数据
	{
		hFIFO_TGS2602.data[i]=hsensors.TGS2602_value.number_float;
		hFIFO_M010.data[i]=hsensors.M010_value.number_float;
		hFIFO_MC101.data[i]=hsensors.MC101_value.number_float;
	}
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(hsensors.MC101_value.number_float>0.2&&hsensors.MC101_value.number_float<1.8)
			;
		else
			HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port,LED_STATUS_Pin);
		if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==GPIO_PIN_SET)    //3档爆炸下不报警
		{
		  TGS2602_WARNING_SLOPE=1000;    
		  MC101_WARNING_SLOPE=0.001;
			alarm_times=0;
		}
		else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==GPIO_PIN_SET)   //2档正常工作
		{
		  TGS2602_WARNING_SLOPE=1.1;    
		  MC101_WARNING_SLOPE=0.000000001;
		}
		else if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)==GPIO_PIN_SET)  //1档演示
		{
		  TGS2602_WARNING_SLOPE=1.03;     
		  MC101_WARNING_SLOPE=0.9;
		}
		else
		{
			TGS2602_WARNING_SLOPE=1.1;    
		  MC101_WARNING_SLOPE=0.000000001;
		}
		SensorsGetValue(&hsensors);			//获取传感器模拟电压值
		PushData(&hFIFO_TGS2602, hsensors.TGS2602_value.number_float);
		PushData(&hFIFO_M010, hsensors.M010_value.number_float);
		PushData(&hFIFO_MC101, hsensors.MC101_value.number_float);
		#ifdef TGS2602_ONLY
			if(hsensors.TGS2602_value.number_float>hFIFO_TGS2602.avg*TGS2602_WARNING_SLOPE||timeout==0)
		#endif
		#ifdef M010_ONLY
			if(hsensors.M010_value.number_float>hFIFO_M010.avg*M010_WARNING_SLOPE||timeout==0)
		#endif
		#ifdef MC101_ONLY
			if(hsensors.MC101_value.number_float<hFIFO_MC101.avg*MC101_WARNING_SLOPE||timeout==0)
		#endif
		#ifdef TGS2602_AND_M010_AND_MC101
			if(hsensors.TGS2602_value.number_float>hFIFO_TGS2602.avg*TGS2602_WARNING_SLOPE||hsensors.M010_value.number_float>hFIFO_M010.avg*M010_WARNING_SLOPE||hsensors.MC101_value.number_float<hFIFO_MC101.avg*MC101_WARNING_SLOPE||(timeout==0))
		#endif
		{	
			WirelessTxPacket(TxPacket, alarm_cnt, &hsensors);		//如果alarm_cnt不为0, 发射报警数据

			if(timeout!=0)
						alarm_times++;																	//有报警值
			if(alarm_times>ALARM_CONFIRM_TIMES)	
			{
				alarm_cnt=ALARM_TIME;																//产生确认报警
				alarm_times=0;
//				HAL_GPIO_WritePin(GPIOB, CLK_Pin|DATA_Pin|BEEP_Pin, GPIO_PIN_SET);
				beep_cnt=50;
			}
		}
			Delay_ms(TEST_TIME_INTERVAL);
		
			if(timeout==0) 																				//按照时间间隔获取温湿度值	
			{
				timeout=TIMEOUT;
				hsensors.TEMP.number_float=SHT20_GetTempPoll();			//获取温度值
				hsensors.HUMI.number_float=SHT20_GetHumiPoll();			//获取湿度值
			}
			timeout--;
			if(beep_cnt>1)    
				beep_cnt--;
			else  
				HAL_GPIO_WritePin(GPIOB, CLK_Pin|DATA_Pin|BEEP_Pin, GPIO_PIN_RESET);
			if(alarm_cnt>0)   alarm_cnt--;
			HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port,LED_STATUS_Pin);
			//报警次数衰减
	}
		
  /* USER CODE END WHILE */
   
  /* USER CODE BEGIN 3 */

}
  /* USER CODE END 3 */


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;							//ADC扫描使能
  hadc1.Init.ContinuousConvMode = DISABLE;								//连续转换禁止
  hadc1.Init.DiscontinuousConvMode = ENABLE;							//
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;													//转换通道数量
	hadc1.Init.NbrOfDiscConversion=1;												//每个通道转换数量
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;												//PA1
  sConfig.Rank = 1;																				//AD转换顺序
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SHT_SDA_Pin|SHT_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_Pin|DATA_Pin|BEEP_Pin, GPIO_PIN_RESET);
	
	  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SHT_SDA_Pin|SHT_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_Pin|DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SHT_SDA_Pin SHT_CLK_Pin */
  GPIO_InitStruct.Pin = SHT_SDA_Pin|SHT_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_STATUS_Pin CLK_Pin DATA_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin|CLK_Pin|DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_RE_Pin */
  GPIO_InitStruct.Pin = RS485_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_RE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHT_SDA_Pin SHT_CLK_Pin */
  GPIO_InitStruct.Pin = SHT_SDA_Pin|SHT_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin DATA_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BEEP_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);
	
  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void WirelessTxPacket(uint8_t* pTxPacket, uint8_t timeout, SensorsTypeDef* phsensors)
{
		uint8_t i=0;
		*(pTxPacket+9)=0xA5;
		if(timeout==0)																					//时间到达传温度值
			*(pTxPacket+10)=0xAA;
		else
		{
			*(pTxPacket+10)=0x00;																	//有报警信号
			HAL_GPIO_WritePin(GPIOB, CLK_Pin|DATA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, BEEP_Pin, GPIO_PIN_SET);
		}
		*(pTxPacket+11)=LOCATION;
		*(pTxPacket+12)=phsensors->TEMP.number_uchar[3];
		*(pTxPacket+13)=phsensors->TEMP.number_uchar[2];
		*(pTxPacket+14)=phsensors->TEMP.number_uchar[1];
		*(pTxPacket+15)=phsensors->TEMP.number_uchar[0];
		*(pTxPacket+16)=phsensors->HUMI.number_uchar[3];
		*(pTxPacket+17)=phsensors->HUMI.number_uchar[2];
		*(pTxPacket+18)=phsensors->HUMI.number_uchar[1];
		*(pTxPacket+19)=phsensors->HUMI.number_uchar[0];
		*(pTxPacket+20)=0;
		for(i=9;i<20;i++)
		*(pTxPacket+20)^=*(pTxPacket+i);
		vTxPacket(pTxPacket, TX_LEN);													//发送数据
		HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port,LED_STATUS_Pin);
}

void PushData(FIFOTypeDef *pFIFO, float data)
{
	uint8_t i=0;
	pFIFO->sum=pFIFO->sum - pFIFO->data[0] + data;
	pFIFO->avg=pFIFO->sum/AVG_NUM; 
	for(i=0;i<AVG_NUM-1;i++)
	{
		pFIFO->data[i]=pFIFO->data[i+1];
	}
	pFIFO->data[AVG_NUM-1]=data;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
