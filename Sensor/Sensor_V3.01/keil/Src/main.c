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




#define LOCATION DOOR1_RIGHT				//��һ���Ҳ�
//#define LOCATION DOOR1_LEFT		   		//��һ�����
//#define LOCATION DOOR2_RIGHT				//�ڶ����Ҳ�
//#define LOCATION DOOR2_LEFT					//�ڶ������
//#define LOCATION DOOR3_RIGHT				//�������Ҳ�
//#define LOCATION DOOR3_LEFT					//���������
//#define LOCATION DOOR4_RIGHT				//�������Ҳ�
//#define LOCATION DOOR4_LEFT					//���������

//#define LOCATION SEAT_NUM1				//��λ1��
//#define LOCATION SEAT_NUM2		   	//��λ2��
//#define LOCATION SEAT_NUM3				//��λ3��
//#define LOCATION SEAT_NUM4				//��λ4��
//#define LOCATION SEAT_NUM5				//��λ5��
//#define LOCATION SEAT_NUM6				//��λ6��









#define TIMEOUT 300									//ÿTIMEOUT * 100ms����һ���¶�  300
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
#define SEAT_NUM6 	0x7A						//ƽ��ֵȡ������
#define PRE_HEAT_TIME	180						//Ԥ��ʱ��PRE_HEAT_TIME * 1s  180   ����1�ĳ�180��  
#define TX_LEN 21										//�������ݳ���
#define ALARM_TIME	5								//ALARM_TIME  �����������	
#define TEST_TIME_INTERVAL	100			//�������
#define ALARM_CONFIRM_TIMES 3				//ALARM_CONFIRM_TIMES+1�μ�⵽ֵ�ٱ���   1



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
	uint8_t alarm_times=0;								//��ⱨ�������� �趨N��֮�����Ϊ�Ǳ���			
	uint8_t TxPacket[TX_LEN]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x2D,0xD4,0xA5,0xAA};			//ǰ���� 6��0xAA  SYNC WORD 0xAA2DD4 ����
  uint16_t i=0, timeout=0;
	//SensorsTypeDef hsensors;
	uint8_t alarm_cnt=0;	//��ʾ����״̬��ֻ��alrm_cnt����ʱ��ʾ�ڱ���
	uint8_t beep_cnt=0;
//	uint8_t bit0=0,bit1=0,bit2=0;
//	uint8_t sensitivity=0;
	float TGS2602_WARNING_SLOPE=1.03;	  //TGS2602����������б��
  float M010_WARNING_SLOPE=1.03;			//M010����������б��
	float MC101_WARNING_SLOPE=0.9;			//MC101����������б��
	uint8_t A=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  delay_init(72);												//��ʱ������ʼ��
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
	Delay_ms(2000);																					//������оƬ�ϵ�ʱ��
	vCMT2119AInit(CfgTbl, 21);															//��������оƬ
	SHT20_init();																						//SHT20��������ʼ��
//	for(i=0;i<PRE_HEAT_TIME;i++) Delay_ms(1000);						//��ʱ180�����ڴ�����Ԥ�ȣ�����ʼ��ѹֵ�ͺܴ�
//	for(i=0;i<LOCATION*4;i++) Delay_ms(100);								//����λ�ò��컯ʱ�䴫��
	
	hsensors.TEMP.number_float=SHT20_GetTempPoll();					//��ȡ��ʼ�¶�ֵ
	hsensors.HUMI.number_float=SHT20_GetHumiPoll();					//��ȡ��ʼʪ��ֵ
	
	//��ȡ��ʼ��FIFOֵ�ͼӺ�ֵ
	SensorsGetValue(&hsensors);															//��ȡ��������ֵ
	FIFOTypeDef hFIFO_TGS2602={hsensors.TGS2602_value.number_float*AVG_NUM, hsensors.TGS2602_value.number_float};			//��ʼ��TGS2602 FIFO
	FIFOTypeDef hFIFO_M010={hsensors.M010_value.number_float*AVG_NUM, hsensors.M010_value.number_float};
	FIFOTypeDef hFIFO_MC101={hsensors.MC101_value.number_float*AVG_NUM, hsensors.MC101_value.number_float};	//��ʼ��M010 FIFO
	for(i=0;i<AVG_NUM;i++) 																																														//���FIFO���data����
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
		if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==GPIO_PIN_SET)    //3����ը�²�����
		{
		  TGS2602_WARNING_SLOPE=1000;    
		  MC101_WARNING_SLOPE=0.001;
			alarm_times=0;
		}
		else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==GPIO_PIN_SET)   //2����������
		{
		  TGS2602_WARNING_SLOPE=1.1;    
		  MC101_WARNING_SLOPE=0.000000001;
		}
		else if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)==GPIO_PIN_SET)  //1����ʾ
		{
		  TGS2602_WARNING_SLOPE=1.03;     
		  MC101_WARNING_SLOPE=0.9;
		}
		else
		{
			TGS2602_WARNING_SLOPE=1.1;    
		  MC101_WARNING_SLOPE=0.000000001;
		}
		SensorsGetValue(&hsensors);			//��ȡ������ģ���ѹֵ
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
			WirelessTxPacket(TxPacket, alarm_cnt, &hsensors);		//���alarm_cnt��Ϊ0, ���䱨������

			if(timeout!=0)
						alarm_times++;																	//�б���ֵ
			if(alarm_times>ALARM_CONFIRM_TIMES)	
			{
				alarm_cnt=ALARM_TIME;																//����ȷ�ϱ���
				alarm_times=0;
//				HAL_GPIO_WritePin(GPIOB, CLK_Pin|DATA_Pin|BEEP_Pin, GPIO_PIN_SET);
				beep_cnt=50;
			}
		}
			Delay_ms(TEST_TIME_INTERVAL);
		
			if(timeout==0) 																				//����ʱ������ȡ��ʪ��ֵ	
			{
				timeout=TIMEOUT;
				hsensors.TEMP.number_float=SHT20_GetTempPoll();			//��ȡ�¶�ֵ
				hsensors.HUMI.number_float=SHT20_GetHumiPoll();			//��ȡʪ��ֵ
			}
			timeout--;
			if(beep_cnt>1)    
				beep_cnt--;
			else  
				HAL_GPIO_WritePin(GPIOB, CLK_Pin|DATA_Pin|BEEP_Pin, GPIO_PIN_RESET);
			if(alarm_cnt>0)   alarm_cnt--;
			HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port,LED_STATUS_Pin);
			//��������˥��
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;							//ADCɨ��ʹ��
  hadc1.Init.ContinuousConvMode = DISABLE;								//����ת����ֹ
  hadc1.Init.DiscontinuousConvMode = ENABLE;							//
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;													//ת��ͨ������
	hadc1.Init.NbrOfDiscConversion=1;												//ÿ��ͨ��ת������
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;												//PA1
  sConfig.Rank = 1;																				//ADת��˳��
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
		if(timeout==0)																					//ʱ�䵽�ﴫ�¶�ֵ
			*(pTxPacket+10)=0xAA;
		else
		{
			*(pTxPacket+10)=0x00;																	//�б����ź�
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
		vTxPacket(pTxPacket, TX_LEN);													//��������
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
