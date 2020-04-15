/*
Description: 系统运行LED显示
*/

#include "led.h"

#define SystemRunLedTime 10

extern uint16_t Tim1Count;

void SystemRunLed(void)
{
	if(Tim1Count % SystemRunLedTime ==0)
	{
		RunLedToggle();
	}
}

