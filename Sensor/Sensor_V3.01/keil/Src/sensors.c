#include "sensors.h"
#include "prj_typedef.h"
#include "delay.h"

extern ADC_HandleTypeDef hadc1;

void SensorsGetValue(SensorsTypeDef *pSensors)
{
	uint8_t i=0;
	uint16_t MCU_ADC_Raw_Value[SENSOR_NUM];
	for(i=0;i<SENSOR_NUM;i++)
	{						
		HAL_ADC_Start(&hadc1);
		Delay_us(100);
		if(HAL_ADC_PollForConversion(&hadc1, 200)==HAL_OK)
			MCU_ADC_Raw_Value[i]=HAL_ADC_GetValue(&hadc1);
	}
	pSensors->TGS2602_value.number_float=(float)MCU_ADC_Raw_Value[0]*3.3/4096;
	pSensors->M010_value.number_float=(float)MCU_ADC_Raw_Value[1]*3.3/4096;
	
	pSensors->MC101_value.number_float=(float)MCU_ADC_Raw_Value[2]*3.3/4096;
	HAL_ADC_Stop(&hadc1);
}
