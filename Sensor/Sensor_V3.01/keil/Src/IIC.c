#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal_gpio.h"
#include "IIC.h"
#include "Delay.h"
void SDA_OUT_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	/*Configure GPIO pins : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void SDA_IN_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	/*Configure GPIO pins : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void SCL_OUT_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	/*Configure GPIO pins : PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed =GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
}
void SDA_OUT(uint8_t data)
{
	if (data==0)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	else if (data==1)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}
void SCL_OUT(uint8_t data)
{
	if (data==0)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	else if (data==1)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
}
uint8_t SDA_IN(void)
{
	uint8_t x;
	if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==0u)
		x=0;
	else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==1u)
		x=1;
	return x;
}

void IIC_init(void)
{
	SCL_OUT_init();
	SDA_OUT_init();
	
	SCL_OUT(1);
	SDA_OUT(1);
	Delay_ms(10);
}
void IIC_Start(void)
{
	SCL_OUT_init();
	SDA_OUT_init();
	
	SCL_OUT(1);
	SDA_OUT(1);
	Delay_us(10);
	SDA_OUT(0);
	Delay_us(10);
	SCL_OUT(0);
	Delay_us(10);
}
void IIC_Stop(void)
{
	SCL_OUT_init();
	SDA_OUT_init();
	
	SCL_OUT(0);
	SDA_OUT(0);
	Delay_us(10);
	SCL_OUT(1);
	Delay_us(10);
	SDA_OUT(1);
	Delay_us(10);
}
//等待应答信号
//返回值	1 未收到应答
//				0 收到应答
uint8_t IIC_wait_ack(void)
{
	uint8_t qiut_number=0;
	SDA_IN_init();
	Delay_us(1);
	SCL_OUT(1);
	Delay_us(1);
	while(SDA_IN())
	{
		qiut_number++;
		if(qiut_number>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_OUT(0);
	return 0;
}
void IIC_ack(void)
{
	SCL_OUT(0);
	SDA_OUT_init();
	SDA_OUT(0);
	Delay_us(10);
	SCL_OUT(1);
	Delay_us(10);
	SCL_OUT(0);
}
void IIC_noack(void)
{
	SCL_OUT(0);
	SDA_OUT_init();
	SDA_OUT(1);
	Delay_us(10);
	SCL_OUT(1);
	Delay_us(10);
	SCL_OUT(0);
}
void IIC_send_byte(uint8_t data)
{
	uint8_t t;
	SDA_OUT_init();
	SCL_OUT(0);
	for(t=0;t<8;t++)
	{
		SDA_OUT((data&0x80)>>7);
		data<<=1;
		Delay_us(50);
		SCL_OUT(1);
		Delay_us(50);
		SCL_OUT(0);
		Delay_us(50);
	}
}
//ack	0 接收完数据不产生应答
//		1 接收完数据产生应答信号
uint8_t IIC_read_byte(uint8_t ack)
{
	uint8_t i,data;
	SDA_IN_init();
	for(i=0;i<8;i++)
	{
		SCL_OUT(0);
		Delay_us(50);
		SCL_OUT(1);
		Delay_us(50);
		data<<=1;
		if(SDA_IN())
			data++;
		Delay_us(50);
	}
	if(ack)
		IIC_ack();
	else
		IIC_noack();
	
	return data;
}
