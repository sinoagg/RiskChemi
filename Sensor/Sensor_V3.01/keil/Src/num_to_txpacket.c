#include "num_to_txpacket.h"
void num_to_tx(uint8_t *tx, uint32_t num)
{
	uint8_t i=0;
	while(num!=0)
	{
		tx[i]=num%10;
		num=num/10;
		i++;
	}
}
