#ifndef __GASSENSORTEST_H
#define __GASSENSORTEST_H

#include "stdint.h"

void TestMode_Init(void);
void DebugMode_Init(void);
void FactoryTestMode_Run(void);
void FactoryDebugMode_Run(void);
void radioDataDecode(void);
void serialDataDecode(void);
uint8_t infoSearch(uint8_t* payload);

typedef struct  
{  
	uint8_t deveui[8];  //DTU�豸�ż�¼
	uint8_t sensorTestResult;  //���������Խ��
	int8_t rssiDTU;  //�ڵ�RSSI
	int8_t rssiGW;  //���ز�RSSI
	int8_t snrDTU;  //�ڵ��SNR
	int8_t snrGW;  //���ز�SNR
	uint8_t lifeCycle; //������������
}DTU_INFO_TYPE;

#endif