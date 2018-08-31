#ifndef _SHT20_H
#define _SHT20_H
#include "stdint.h"
#define  I2C_ADR_W 0x80  //SHT2X IIC 读地址
#define  I2C_ADR_R 0x81  //SHT2X IIC 写地址

typedef enum {
    TRIG_TEMP_MEASUREMENT_HM   = 0xE3, // 保持主机 温度测量
    TRIG_HUMI_MEASUREMENT_HM   = 0xE5, // 保持主机 温度测量
    TRIG_TEMP_MEASUREMENT_POLL = 0xF3, // 非保持主机 温度测量
    TRIG_HUMI_MEASUREMENT_POLL = 0xF5, // 非保持主机 湿度测量
    USER_REG_W                 = 0xE6, // 写寄存器命令
    USER_REG_R                 = 0xE7, // 读寄存器命令
    SOFT_RESET                 = 0xFE  // 软件复位命令
} SHT2xCommand;

typedef enum {
    SHT2x_RES_12_14BIT         = 0x00, //RH=12bit, T=14bit 
    SHT2x_RES_8_12BIT          = 0x01, //RH= 8bit, T=12bit
    SHT2x_RES_10_13BIT         = 0x80, //RH=10bit, T=13bit
    SHT2x_RES_11_11BIT         = 0x81, //RH=11bit, T=11bit
    SHT2x_RES_MASK             = 0x81  //Mask for res. bits (7,0) in user reg.
} SHT2xResolution;

typedef enum {
    SHT2x_HEATER_ON            = 0x04, //heater on
    SHT2x_HEATER_OFF           = 0x00, //heater off
    SHT2x_HEATER_MASK          = 0x04  //Mask for Heater bit(2) in user reg.
} SHT2xHeater;

typedef struct{
    float TEMP_POLL;
    float HUMI_POLL;    
} SHT2x_data;

extern SHT2x_data SHT20;


uint8_t SHT20_reset(void);
uint8_t SHT20_init(void);
float SHT20_GetTempPoll(void);
float SHT20_GetHumiPoll(void);
float SHT20_TempPopp(void);
#endif
