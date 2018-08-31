#ifndef __PRJ_TYPEDEF_H
#define __PRJ_TYPEDEF_H

#include "stm32f1xx_hal.h"

#define SENSOR_NUM 3
#define AVG_NUM	30								//平均检测点个数

typedef union										
{
	uint8_t number_uchar[4];			
	float number_float;					
}Float_Union_Data;

typedef struct
{
	Float_Union_Data TGS2602_value;
	Float_Union_Data M010_value;
	Float_Union_Data MC101_value;
	Float_Union_Data TEMP;
	Float_Union_Data HUMI;
}SensorsTypeDef; 

typedef struct 
{
	float sum;				//求和
	float avg;				//平均值
	float data[AVG_NUM];		//数组
}FIFOTypeDef;

#endif



