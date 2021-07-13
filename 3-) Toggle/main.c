#include "tm4c123gh6pm.h"
#define delay_value 16000000
void delay(unsigned int time);

int main()
{
	SYSCTL_RCGCGPIO_R  |=0x20;
	GPIO_PORTF_LOCK_R  = 0x4C4F434B; 
	GPIO_PORTF_CR_R    = 0x1F;
	GPIO_PORTF_PUR_R   = 0x11;
	GPIO_PORTF_DEN_R   = 0x1F;
	GPIO_PORTF_DIR_R	 = 0x0E;
	while(1)
	{
		if(((GPIO_PORTF_DATA_R & (1<<4)) == 0) && (GPIO_PORTF_DATA_R & (1<<0)) == 0)
		{
			GPIO_PORTF_DATA_R = 0x08;
			delay(10000000);
			GPIO_PORTF_DATA_R = 0x00;
			delay(10000000);
		}
		else if((GPIO_PORTF_DATA_R & (1<<0)) == 0)
		{
			GPIO_PORTF_DATA_R = 0x02;
			delay(10000000);
			GPIO_PORTF_DATA_R = 0x00;
			delay(10000000);
		}
		else if((GPIO_PORTF_DATA_R & (1<<4)) == 0)
		{
			GPIO_PORTF_DATA_R = 0x04;
			delay(10000000);
			GPIO_PORTF_DATA_R = 0x00;
			delay(10000000);
		}
		else
			GPIO_PORTF_DATA_R = 0x00;
	}
}
	unsigned int i,j;
void delay(unsigned int time)
{
	for(j=0; j<=time ;j++)
	{
		for(i=0; i<delay_value ;i++);
	}
}	
