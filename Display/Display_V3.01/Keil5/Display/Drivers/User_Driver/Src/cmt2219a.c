#include "cmt2219a.h"

cmt2219aClass *pRfm219a;

//Cofiguration Table 
//uint8_t CfgTbl[LEN_CFG_TBL] = {
//                    0x72,0x42,0x44,0x15,0x0D,0x63,0x9A,0x80,0xC6,0x53,0x01,0x00,0x62,0x1E,0x00,    			        
//                    0x10,0x84,0x14,0xE0,0x00,0x27,0x9F,0x00,/*0xD4,0x2D*/0xAA,0xAA,0xAA,0x00,0x38,0x01,0x01,                             
//                    0x60,0x21,0x07,0x84,0x00,0x00,0x19,0x00,0x00,0xAE,0xAC,0xD4,0x53,0xD4,0x40,
//                    0x49,0xFF,0x5D,0x12,0x00,0x90,0xFA,0x00,0x00,0x40,0xC0,0x00,0x00,0x20,0xCA,                      
//                    0x07,0x00 };
uint8_t CfgTbl[LEN_CFG_TBL] = {
                    0x72,0x42,0x44,0x15,0x0D,0x63,0x9A,0x80,0xC6,0x53,0x01,0x00,0x62,0x1E,0x00,    			        
                    0x10,0x84,0x14,0xE0,0x00,0x27,0x9F,0x00,0xD4,0x2D,0xAA,0x00,0x38,0x01,0x01,                             
                    0x20,0x21,0x07,0x84,0x00,0x00,0x19,0x00,0x00,0x00,0xAC,0xAE,0x53,0xD4,0x40,
                    0x49,0xFF,0x5D,0x12,0x00,0x90,0xFA,0x00,0x00,0x40,0xC0,0x00,0x00,0x20,0xCA,                      
                    0x07,0x00 };

									
/**********************************************************
**Name:     vCMT2219_GPIO_Init
**Function: Init GPIO of CMT2219 GPIO pins
**Input:    none
**Output:   none
**********************************************************/
static void vCMT2219_GPIO_Init(void)
{
	HAL_GPIO_WritePin(GPIOB, FCSB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, SCL_Pin, GPIO_PIN_RESET);
}	
/**********************************************************
**Name:     vGoRx
**Function: Entry Rx Mode
**Input:    none
**Output:   none
**********************************************************/
void vCMT2219_GoRx(void)
{
 uint8_t tmp;
 tmp = bSpi3Read((uint8_t)(OP_MODE>>8));		//基本配置
 tmp &= OP_MASK;
 vSpi3Write(OP_MODE+tmp+OP_RX);
 HAL_Delay(10);
 tmp = bSpi3Read((uint8_t)(OP_MODE>>8));		//基本配置
 if(tmp==0) 
	 HAL_Delay(10);
}
/**********************************************************
**Name:     vGoSleep
**Function: Entry Sleep Mode
**Input:    none
**Output:   none
**********************************************************/
static void vGoSleep(void)
{
 uint8_t tmp;	
 tmp = bSpi3Read((uint8_t)(OP_MODE>>8));
 tmp &= OP_MASK;
 vSpi3Write(OP_MODE+tmp+OP_SLEEP);
}

/**********************************************************
**Name:     vGoStandby
**Function: Entry Standby Mode
**Input:    none
**Output:   none
**********************************************************/
static void vGoStandby(void)
{
 uint8_t tmp;	
 tmp = bSpi3Read((uint8_t)(OP_MODE>>8));
 tmp &= OP_MASK;
 vSpi3Write(OP_MODE+tmp+OP_STANDBY);	
}

/**********************************************************
**Name:     vSoftReset
**Function: Software reset Chipset
**Input:    none
**Output:   none
**********************************************************/
/*static void vSoftReset(void)
{
 vSpi3Write(SOFT_RST); 
}*/

/**********************************************************
**Name:     vClearIntFlag
**Function: clear all irq flag
**Input:    none
**Output:   none
**********************************************************/
static void vClearIntFlag(void)
{
 vSpi3Write(INTCTL_B+0xFF);
}

/**********************************************************
**Name:     vClearFIFO
**Function: clear FIFO buffer
**Input:    none
**Output:   none
**********************************************************/
static void vClearFIFO(void)
{
 uint8_t tmp;	
 tmp = bSpi3Read((uint8_t)(INTCTL_D>>8));
 vSpi3Write(INTCTL_D+tmp+FIFO_CLR);
}

/**********************************************************
**Name:     bReadStatus
**Function: read chipset status
**Input:    none
**Output:   none
**********************************************************/
/*static uint8_t bReadStatus(void)
{
 return(0xE0&(bSpi3Read((uint8_t)(OP_MODE>>8))));
}*/

/**********************************************************
**Name:     vInit
**Function: Init. CMT2119A
**Input:    none
**Output:   none
**********************************************************/
void vCMT2219_Init(uint8_t cfg[],  cmt2219aClass *pCmt)
{
	 uint8_t i;
//	 uint8_t tmp;
	 pRfm219a = pCmt;
	 vCMT2219_GPIO_Init();                                  //GPIO pins Init
	 vSpi3Init();                                           //3 wire SPI Init
	 HAL_Delay(20);
	 vGoSleep();					
	 HAL_Delay(20);						
	 vGoStandby();
	 HAL_Delay(20);
	 for(i=0; i<LEN_CFG_TBL; i++)			                      //exp file have 62 byte
		vSpi3Write(((uint16_t)i<<8)|cfg[i]);

	 vClearIntFlag();
	 vGoStandby();					        //进入STB状态

}

/**********************************************************
**Name:     vGpioFuncCfg
**Function: GPIO Function config
**Input:    none
**Output:   none
**********************************************************/
void vCMT2219_GpioFuncCfg(uint8_t io_cfg)
{
 vSpi3Write(IO_SEL+io_cfg);
}

/**********************************************************
**Name:     vIntOutputCfg
**Function: Select which intterupt source is output on which interrupt pin
**          The address of INTCTL_A is 0x41 highest 4 bit is for INT2 PIN, Lowest 4 bit for INT1 PIN
**Input:    int_pin: INT1/INT2, int_source: intterrupt source
**Output:   none
**********************************************************/
void vCMT2219_IntOutputCfg(uint8_t int_pin, uint8_t int_source)
{
  vSpi3Write(INTCTL_A+((int_source)<<(4*int_pin)));
}
/**********************************************************
**Name:     vIntSourceCfg
**Function: config Interrupt source
**Input:    int_1, int_2
**Output:   none
**********************************************************/
/*static void vIntSourcCfg(uint8_t int_1, uint8_t int_2)
{
 vSpi3Write(INTCTL_A+int_2+((int_1)>>4));
}*/

/**********************************************************
**Name:     vEnableIntSource
**Function: enable interrupt source 
**Input:    en_int
**Output:   none
**********************************************************/
void vCMT2219_EnableIntSource(uint8_t en_int)
{
  vSpi3Write(INT_EN+en_int);
}

/**********************************************************
**Name:     bReadIngFlag
**Function: Read interrupt flag(INTCTL_C)
**Input:    none
**Output:   interrupt flag
**********************************************************/
/*static uint8_t bReadIngFlag(void)
{
 return(bSpi3Read((uint8_t)(INTCTL_C>>8)));
}	*/

/**********************************************************
**Name:     bReadRegister
**Function: Read Register
**Input:    Register address
**Output:   value 
**********************************************************/
uint8_t bCMT2219_ReadRegister(uint8_t address)
{
 return(bSpi3Read(address));
}	

/**********************************************************
**Name:     bReadRssi
**Function: Read Rssi
**Input:    none
**Output:   none
**********************************************************/
/*static uint8_t bReadRssi(void)
{
 return(bSpi3Read((uint8_t)(RSSI_ADDR>>8)));
}*/

/**********************************************************
**Name:     bGetMessage
**Function: Read FIFO from CMT2219
**Input:    pointer to the message buffer
**Output:   Status
**********************************************************/
uint8_t bCMT2219_GetMessage(uint8_t* pBuf, uint8_t length)
{
  uint8_t i, status=0;
  for(i=0;i<length;i++)
  {
    *pBuf=bSpi3ReadFIFO();
    Delay_us(10);
    pBuf++;
  }
  vClearFIFO();
  vClearIntFlag();
  return status;
}
	
