#ifndef _HopeDuino_TWI_h
#define _HopeDuino_TWI_h
#include "stm32f1xx_hal.h"	
#define	TWI_SPEED	200			//used as delay time. The bigger this number is , the slower interface works

#define CMT2119_DATA_PORT       GPIOB
#define CMT2119_DATA_PIN        GPIO_PIN_13
#define CMT2119_CLK_PORT        GPIOB
#define CMT2119_CLK_PIN         GPIO_PIN_12

void vTWIInit(void);							/** initialize TWI port **/
void vTWIWrite(uint8_t adr, uint8_t dat);		
uint8_t bTWIRead(uint8_t adr);
void vTWIReset(void);
void vTWIWriteByte(uint8_t dat);
uint8_t bTWIReadByte(void);

void	SetTCLK(void);
void	ClrTCLK(void);
void	InputTDAT(void);  
void	OutputTDAT(void);
void	SetTDAT(void);	 
void	ClrTDAT(void);	 
uint8_t TDAT_H(void);	 
uint8_t	TDAT_L(void);

void _delay_us(uint16_t time);

#endif 

