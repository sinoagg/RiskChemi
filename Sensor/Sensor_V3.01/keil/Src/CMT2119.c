#include "stm32f1xx_hal.h"
#include "CMT2119.h" 
#include "delay.h"
#include "HopeDuino_TWI.h"

void vOpenLdoAndOsc(void);
void vCloseLdoAndOsc(void);
void vActiveRegsister(void);
void vEnableRegMode(void);
void vWriteReg(uint8_t adr, uint16_t wrPara);
uint16_t wReadReg(uint8_t adr);
void vSoftReset(void);
void vTwiReset(void);
void vTwiOff(void);	

/**********************************************************
**Name:     vCMT2119AInit
**Function: Init. CMT2119A with config 
**Input:    none
**Output:   none
**********************************************************/
void vCMT2119AInit(const uint16_t para[], uint8_t length)
{
	uint8_t i;		
	//uint16_t test[21];
	vTWIInit();
	vTWIReset();						//step 1
	Delay_us(5);
	vSoftReset();						//step 2
	Delay_ms(5);						//wait more than 1ms		
	vOpenLdoAndOsc();				//step 3
	Delay_ms(5);
	vActiveRegsister();			//step 4
	Delay_ms(5);
	vEnableRegMode();				//step 5
	for(i=0;i<length;i++)		//step 6
  {
		vWriteReg(i, para[i]);
		Delay_us(50);
	}
	/*for(i=0;i<length;i++)
	{
		test[i]=wReadReg(i);
		Delay_us(50);
		if(test[i]!=0)
		Delay_us(10);
	}*/	
	Delay_ms(5);	
	vTwiOff();			        //step 7	
	Delay_ms(5);
	ClrTDAT();				//Set Dat to low
	OutputTDAT();				//Read for Tx
}

/**********************************************************
**Name:     vCMT2119ASleep
**Function: set CMT2119A to sleep
**Input:    none
**Output:   none
**********************************************************/
void vCMT2119ASleep(void)
{
  vCloseLdoAndOsc();				//step 1
  vTWIReset();				        //step 2
  vSoftReset();					//step 3
}

/**********************************************************
**Name:     vEncode
**Function: encode 
**Input:    ptr length etype
            length need less than 64
**Output:   TxBuf
**********************************************************/
/*void vEncode(uint8_t ptr[], uint8_t length, uint8_t etype)
{
 uint8_t i, j, k;	
 
 switch(etype)
 	{
	case E527:
		TxBuf[0] = 0x80;			
		TxBuf[1] = 0x00;
		TxBuf[2] = 0x00;
		TxBuf[3] = 0x00;		
		for(i=4; i<(4+(length<<2)); i++)	//1XX0
			TxBuf[i] = 0x88;
 		k = 4;
 		for(j=0; j<length; j++)
 			{
 			for(i=0x80; i!=0; )
 				{
 				if((ptr[j]&i)!=0)
 					TxBuf[k] |= 0x60;
 				i >>= 1;
 				if((ptr[j]&i)!=0)
 					TxBuf[k] |= 0x06;
 				i >>= 1;
 				k++;
 				}
			}
		TxBufLength = ((length<<2)+4);
		break;			
	case E201:
 		for(i=0; i<4; i++)			//
     		TxBuf[i] = 0x00;		        //4 byte 0x00
 		TxBuf[4] = 0x0A;			//10 pulse preamble	
 		for(i=5; i<(5+2); i++)
	 		TxBuf[i] = 0xAA;		
		TxBuf[i++] = 0x00;			//Sync
		TxBuf[i++] = 0x00;
		k = i+(length*3);
		j = i;		
 		for( ; j<k ; )				
 			{
 			TxBuf[j++] = 0x92;		//1x01x01xB
 			TxBuf[j++] = 0x49;		//01x01x01B
 			TxBuf[j++] = 0x24;		//x01x01x0B
			}

 		for(j=0; j<length; j++)
 			{
			if((ptr[j]&0x80)==0x00)
				TxBuf[i] |= 0x40;
			if((ptr[j]&0x40)==0x00)
				TxBuf[i] |= 0x08;
			if((ptr[j]&0x20)==0x00)
				TxBuf[i] |= 0x01;
			i++;
			if((ptr[j]&0x10)==0x00)
				TxBuf[i] |= 0x20;
			if((ptr[j]&0x08)==0x00)
				TxBuf[i] |= 0x04;
			i++;
			if((ptr[j]&0x04)==0x00)
				TxBuf[i] |= 0x80;
			if((ptr[j]&0x02)==0x00)
				TxBuf[i] |= 0x10;
			if((ptr[j]&0x01)==0x00)
				TxBuf[i] |= 0x02;
			i++;
 			}
 		TxBuf[i++] = 0x92;
		TxBufLength = i;
		break;
	case ENRZ: 	
 	default:
 		for(i=0; i<length; i++)		//do nothing
 			TxBuf[i] = ptr[i];
 		TxBufLength = length;
 		break;
 	}
}*/

/**********************************************************
**Name:     vTxPacket
**Function: send TxBuf
**Input:    none
**Output:   none
**********************************************************/
void vTxPacket(uint8_t *pTxBuf, uint16_t Length)
{
 uint8_t i, j;
 	
 vTWIReset();				//step 1
 vTwiOff();					//step 2
 ClrTDAT();

 for(i=0; i<Length; i++)
 {
 	for(j=0; j<8; j++)
 		{
	 		if((*(pTxBuf+i))&1<<(7-j))
	 		SetTDAT();
	 		else
	 		ClrTDAT();
			
			Delay_us(SYMBOL_TIME);
 		}
 }
 ClrTDAT(); 
 Delay_ms(5);
 vTWIReset();				//step 3
 vSoftReset();      //step 4
}


//**********************HAL Layer**************************
/**********************************************************
**Name:     vOpenLdoAndOsc
**Function: Open LDO & Osc(only for CMT2119A)
**Input:    none
**Output:   none
**********************************************************/
void vOpenLdoAndOsc(void)
{
  vTWIWrite(0x02, 0x78);
}

/**********************************************************
**Name:     vCloseLdoAndOsc
**Function: Close LDO & Osc (only for CMT2119A)
**Input:    none
**Output:   none
**********************************************************/
void vCloseLdoAndOsc(void)
{
  vTWIWrite(0x02, 0x7F);
}

/**********************************************************
**Name:     vActiveRegsister
**Function: Active Regsisiter Mode (Step-4)  (only for CMT2119A)
**Input:    none
**Output:   none
**********************************************************/
void vActiveRegsister(void)
{
  vTWIWrite(0x2F, 0x80);
  vTWIWrite(0x35, 0xCA);
  vTWIWrite(0x36, 0xEB);
  vTWIWrite(0x37, 0x37);
  vTWIWrite(0x38, 0x82);
}	
	
/**********************************************************
**Name:     EnableRegMode
**Function: Active Enable Regsisiter Mode (Step-5)  (only for CMT2119A)
**Input:    none
**Output:   none
**********************************************************/
void vEnableRegMode(void)
{
  vTWIWrite(0x12, 0x10);
  vTWIWrite(0x12, 0x00);
  vTWIWrite(0x24, 0x07);
  vTWIWrite(0x1D, 0x20);
}

/**********************************************************
**Name:     vWriteReg
**Function: wirte something to Regsisiter  (only for CMT2119A)
**Input:    none
**Output:   none
**********************************************************/
void vWriteReg(uint8_t adr, uint16_t wrPara)
{
  vTWIWrite(0x18, adr);
  vTWIWrite(0x19, (uint8_t)wrPara);
  vTWIWrite(0x1A, (uint8_t)(wrPara>>8));
  vTWIWrite(0x25, 0x01);
}

/**********************************************************
**Name:     vReadReg
**Function: read something from Regsisiter 	(only for CMT2119A)
**Input:    none
**Output:   none
**********************************************************/
uint16_t wReadReg(uint8_t adr)
{
  uint8_t i_H, i_L;	
  vTWIWrite(0x18, adr);
  i_L = bTWIRead(0x1b);
  i_H = bTWIRead(0x1c);
  return(((uint16_t)i_H)<<8|i_L);
}

/**********************************************************
**Name:     vSoftReset
**Function: Software Reset Chipset
**Input:    none
**Output:   none
**********************************************************/
void vSoftReset(void)
{
 vTWIWrite(0x3D, 0x01);		//0xBD01
}

/**********************************************************
**Name:     vTwiReset
**Function: Twi mode on
**Input:    none
**Output:   none
**********************************************************/
void vTwiReset(void)
{
 vTWIReset();
}

/**********************************************************
**Name:     vTwiOff
**Function: Twi mode off
**Input:    none
**Output:   none
**********************************************************/
void vTwiOff(void)
{
 vTWIWrite(0x0D, 0x02);		//0x8D02
}
