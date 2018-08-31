#ifndef __INTERNALFLASH_H
#define __INTERNALFLASH_H

#include "stm32f1xx_hal.h"

#define FLASH_USER_START_ADDR        ((uint32_t)0x08008000)   /* Start @ of user Flash area */

#define RISKCHEMI_ALARMSET_ADDR 0x00   	//打开关闭报警建偏移量存储地址
#define RISKCHEMI_VOLSET_ADDR 0x04   	//打开关闭报警建偏移量存储地址

void GetFlashData_U32(uint32_t* pdata, uint32_t flash_addr, uint8_t length);
uint32_t GetFlashData_SingleUint32(uint32_t flash_addr);
void FlashErase(uint32_t start_address,uint32_t end_address);
void FlashWrite_SingleUint32(uint32_t start_address, uint32_t data);
void FlashWrite_ArrayUint32(uint32_t start_address, uint32_t *pData, uint8_t num);
uint32_t FlashRead32bit(uint32_t ReadAddr);
void LoadSetArray(uint32_t ParaFlashAddr, uint32_t *pData, uint8_t num);

#endif
