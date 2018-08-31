#ifndef __CMT2119_H
#define __CMT2119_H

#define SYMBOL_TIME 417//416                         //in us

typedef struct 
{	
  char CrcDisable;			        //false: CRC enable & use CCITT 16bit 
						//true : CRC disable
  char FixedPktLength;				//false: for contain packet length in Tx message, the same mean with variable lenth
						//true : for doesn't include packet length in Tx message, the same mean with fixed length
  char NodeDisable;				//false: Node Address Enable
						//true : Node Address Disable
  char PktLength;											
	
}CMT2119_TypeDef;


//enum EncodeType {ENRZ, E527, E201};
	

void vCMT2119AInit(const uint16_t para[], uint8_t length);
void vCMT2119ASleep(void);
void vTxPacket(uint8_t *pTxBuf, uint16_t Length);
//void vInit(uint8_t cfg[],  uint8_t etype);	

void vCloseLdoAndOsc(void);
void vSoftReset(void);
void vEncode(uint8_t ptr[], uint8_t length, uint8_t etype);

#endif
