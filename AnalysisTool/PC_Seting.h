#ifndef __PC_SETING_H
#define __PC_SETING_H

#include "stdint.h"
#include "stdio.h"


#define PC_CID_Connect 0x01
#define PC_CID_SetEUI  0x02
#define PC_CID_SetPower 0x03

#define PC_CID_GetVersion 0x24

int PC_SetingAnalysis(uint8_t * recbuf, uint8_t recLen);
int PC_SetingLegality(uint8_t * recbuf, uint8_t recLen, uint8_t *data);


int PC_SetingSetingEUI(uint8_t *data, uint8_t *EUI);
	

uint8_t PM_CS_check(uint8_t * chackBuf, int len, uint8_t mode);

#endif
