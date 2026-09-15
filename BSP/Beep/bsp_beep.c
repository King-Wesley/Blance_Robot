#include "bsp_beep.h"

u32 beep_time = 0;

void init_beep(void)
{
	BEEP_BEEP = 0;

}


//beep_time 传入1ms的单位即可  Just pass in the unit of 1ms
void open_beep(u32 beep_time) //10ms
{
	beep_time = beep_time/10;
}

