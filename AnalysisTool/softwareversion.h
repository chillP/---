#ifndef __softwareversion_H
#define __softwareversion_H
#include "stm32l0xx_hal.h"

#define SOFTWARE_VERSION_YEAR 	0x20//年
#define SOFTWARE_VERSION_MONTH 	0x04//月
#define SOFTWARE_VERSION_DAY	 	0x01//日
#define SOFTWARE_VERSION_NO	 		0x03//软件版本

void SoftwareVersionShow(void);
#endif
