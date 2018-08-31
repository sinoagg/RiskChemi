#ifndef _IIC_H
#define _IIC_H
#include "stdint.h"
void SDA_OUT_init(void);
void SDA_IN_init(void);
void SCL_OUT_init(void);
//void SCL_IN_init();
void SDA_OUT(uint8_t data);
void SCL_OUT(uint8_t data);
uint8_t SDA_IN(void);
//uint8_t SCL_IN(void);

void IIC_init(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_wait_ack(void);
void IIC_ack(void);
void IIC_noack(void);
void IIC_send_byte(uint8_t data);
uint8_t IIC_read_byte(uint8_t ack);
#endif
