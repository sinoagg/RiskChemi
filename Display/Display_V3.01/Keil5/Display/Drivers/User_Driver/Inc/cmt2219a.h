#ifndef __CMT2219A_H
#define __CMT2219A_H	 

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"	
#include "wirespi.h"
/*define----------------------------------------------------------------------------*/
#define LEN_CFG_TBL     62
	 
//Reg mapping of CMT2219
#define INT1_PIN                0x00
#define INT2_PIN                0x01

#define INT_SOURCE_RSSI_VLD        0x01
#define INT_SOURCE_PREAM_PS        0x02
#define INT_SOURCE_SYNC_PS         0x03
#define INT_SOURCE_RX_PACKET_DONE  0x06      
#define INT_SOURCE_FIFO_NETY       0x09
#define INT_SOURCE_FIFO_WBYTE      0x0C     

#define	INT_EN			  0x3F00
#define RX_DONE_EN		0x01
#define CRC_PS_EN		  0x02
#define	NODE_PS_EN		0x04
#define	SYNC_PS_EN		0x08
#define PREAM_PS_EN		0x10
#define	RSSI_VLD_EN		0x20
#define	RX_TMO_EN		  0x40
#define	SL_TMO_EN		  0x80

#define	IO_SEL			  0x4000
#define GPIO1_POR		  0x00
#define GPIO1_INT1		0x01
#define	GPIO1_INT2		0x02
#define	GPIO1_DOUT		0x03

#define	GPIO2_INT1		0x00
#define	GPIO2_INT2		0x04
#define	GPIO2_DCLK		0x08
#define	GPIO2_0			  0x0C	

#define	GPIO3_CLK		  0x00
#define GPIO3_INT1		0x10
#define	GPIO3_INT2		0x20
#define	GPIO3_DOUT		0x30

#define	GPIO4_DOUT		0x00
#define	GPIO4_INT1		0x40
#define	GPIO4_INT2		0x80
#define	GPIO4_DCLK		0xC0
	

#define	INTCTL_A		0x4100
#define	RSSI_VLD		0x010		
        
#define	PREAM_PS		0x020
#define	SYNC_PS			0x030

#define	NODE_PS			0x040
#define	CRC_PS			0x050
#define	RX_DONE			0x060

#define	SL_TMO			0x040		//+OFFSET
#define	RX_TMO			0x050
#define	FIFO_NMTY		0x060
#define	FIFO_TH			0x070
#define	FIFO_FULL		0x080
#define	FIFO_WBYTE	0x090
#define FIFO_OVF		0x0A0
#define	RSSI_INDI		0x0B0
		
#define	OFFSET			0x030

#define	INTCTL_B		0x4200
#define	RX_DONE_CLR		0x01
#define	CRC_PS_CLR		0x02
#define	NODE_PS_CLR		0x04
#define	SYNC_PS_CLR		0x08
#define	PREAM_PS_CLR	0x10
#define	RSSI_VLD_CLR	0x20
#define RX_TMO_CLR		0x40
#define	SL_TMO_CLR		0x80

#define	INTCTL_C		0x4300
#define	RX_DONE_FLG		0x01
#define	CRC_PS_FLG		0x02
#define	NODE_PS_FLG		0x04
#define	SYNC_PS_FLG		0x08
#define	PREAM_PS_FLG	0x10
#define	RSSI_VLD_FLG	0x20
#define	RX_TMO_FLG		0x40			
#define	SL_TMO_FLG		0x80

#define	INTCTL_D		0x4400
#define	FIFO_WR_METH	0x01
#define	FIFO_WR_EN		0x02
#define	FIFO_CLR		  0x04
#define	FIFO_OVF_FLG	0x08
#define	FIFO_TH_FLG		0x10
#define	FIFO_NMTY_FLG	0x20
#define	FIFO_FULL_FLG	0x40
#define	PKT_LEN_ERR_FLG	0x80	

#define	RSSI_ADDR		0x4500

#define	FUNC_EN			0x4600
#define	EEPROM_LOCK		0x01
#define	EEPROM_PWRON 	0x02
#define FAST_SPI_EN		0x04
#define	SA_TEST_EN		0x08

#define	OP_MODE			0x4700
#define OP_EEPROM		0x01
#define	OP_STANDBY	0x02	
#define	OP_FS			  0x04
#define	OP_RX			  0x08
#define	OP_SLEEP		0x10

#define	RD_IDLE			0x00
#define	RD_SLEEP		0x20
#define	RD_STBY			0x40
#define	RD_FS			  0x60
#define	RD_RX			  0x80
#define	RD_EEPROM		0xA0
                        
#define OP_MASK			0xE0

#define	SOFT_RST 	  0x4FFF

#define	EE_DAT_LOW  0x5000
#define EE_DAT_HIGH	0x5100
#define	EE_ADD			0x5200

#define	EE_CTL			0x5300
#define	EE_ERASE		0x01
#define	EE_PROG			0x02
#define	EE_READ			0x04
	 
/*variable--------------------------------------------------------------------------*/	 
typedef struct 
{	
  uint8_t CrcDisable;				//false: CRC enable   true : CRC disable                                        
  uint8_t FixedPktLength;		//false: for contain packet length in Tx message, the same mean with variable lenth                                                                                                                                                                           //true : for doesn't include packet length in Tx message, the same mean with fixed length
  uint8_t NodeDisable;			//false: Node Address Enable   true : Node Address Disable                                         
  uint8_t PktLength;											
}cmt2219aClass;	

/*Private Function------------------------------------------------------------------*/
static void vCMT2219_GPIO_Init(void);
static void vGoSleep(void);
static void vGoStandby(void);
static void vSoftReset(void);
static void vClearIntFlag(void);
static void vClearFIFO(void);
static uint8_t bReadStatus(void);
static void vIntSourcCfg(uint8_t int_1, uint8_t int_2);
static uint8_t bReadIngFlag(void);
static uint8_t bReadRssi(void);
/*Export Function-------------------------------------------------------------------*/
void vCMT2219_GoRx(void);
void vCMT2219_Init(uint8_t cfg[],  cmt2219aClass *pCmt);
void vCMT2219_GpioFuncCfg(uint8_t io_cfg);
void vCMT2219_IntOutputCfg(uint8_t int_pin, uint8_t int_source);
void vCMT2219_EnableIntSource(uint8_t en_int);
uint8_t bCMT2219_ReadRegister(uint8_t address);
uint8_t bCMT2219_GetMessage(uint8_t* pBuf, uint8_t length);
	 
#endif

