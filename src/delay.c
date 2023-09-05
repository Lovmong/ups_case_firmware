#include "delay.h"
#include "intrins.h"	

/*--------------------------------------------------------------
一般 51 单片机 1机器周期 = 12时钟周期
时钟频率为 12MHz, 则 1机器周期 = 12*1/12MHz = 1us
即一个  _nop_() 耗时 1us										
--------------------------------------------------------------*/
void delayMs(u32 ms)
{
	u32 i;
	while(ms--)
	{
		for(i=0;i<250;i++)
		{
			_nop_();
			_nop_();
			_nop_();
			_nop_();
			
		}	
	}
}


void delayUs(u32 us)
{
	while(us--)
	{
		_nop_();
	}
}

void delayS(u32 s)
{
	while (s--)
	{
		delayMs(1000);
	}
}

