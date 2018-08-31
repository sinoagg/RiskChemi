#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "IIC.h"
#include "sht20.h"
#include "Delay.h"
SHT2x_data SHT20;

uint8_t SHT20_reset(void)
{
	uint8_t err=0;
	IIC_send_byte(0x80);
	err=IIC_wait_ack();
	IIC_send_byte(0xfe);
	err=IIC_wait_ack();
	IIC_Stop();
	return err;
}

uint8_t SHT20_init(void)
{
	uint8_t err;
	IIC_init();
	IIC_Start();
	err=SHT20_reset();
	IIC_Stop();
	return err;
}
float SHT20_GetTempPoll(void)
{
	float temp=0;
	uint8_t ack=0,temp1=0,temp2=0;
	uint16_t st,i=0;
	SHT20_reset();
	Delay_ms(5);
	IIC_Start();
	Delay_us(50);
	IIC_send_byte(I2C_ADR_W);
	ack=IIC_wait_ack();
	Delay_us(50);
	IIC_send_byte(TRIG_TEMP_MEASUREMENT_POLL);
	ack=IIC_wait_ack();
	do
	{
		Delay_ms(5);
		IIC_Start();
		Delay_ms(5);
		IIC_send_byte(I2C_ADR_R);
		i++;
		ack=IIC_wait_ack();
		if(i==50) break;
	}
	while (ack!=0);
	Delay_us(100);
  temp1=IIC_read_byte(1);
	temp2=IIC_read_byte(1);
	IIC_read_byte(0);
	Delay_us(50);
	IIC_Stop();
	st=(temp1<<8)|(temp2<<0);
	
	st&=~0x0003;
	temp=((float)st*0.00268127)-46.85;
 	return temp;
	
}
float SHT20_GetHumiPoll(void)
{
	float humi;
	uint8_t ack,humi1,humi2;
	uint16_t srh,i=0;
	IIC_Start();
	Delay_us(50);
	IIC_send_byte(I2C_ADR_W);
	ack=IIC_wait_ack();
	Delay_us(50);
	IIC_send_byte(TRIG_HUMI_MEASUREMENT_POLL);
	ack=IIC_wait_ack();
	do
	{
		Delay_ms(5);
		IIC_Start();
		Delay_ms(5);
		IIC_send_byte(I2C_ADR_R);
		i++;
		ack=IIC_wait_ack();
		if(i==50) break;
	}
	while (ack!=0);
	Delay_us(50);
	humi1=IIC_read_byte(1);
	Delay_us(50);
	humi2=IIC_read_byte(1);
	srh=(humi1<<8)|(humi2<<0);
	srh&=~0x0003;
	humi=((float)srh*0.00190735)-6;
	
	return humi;
	
}
