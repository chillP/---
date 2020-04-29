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
	uint8_t deveui[8];  //DTU设备号记录
	uint8_t sensorTestResult;  //传感器测试结果
	int8_t rssiDTU;  //节点RSSI
	int8_t rssiGW;  //网关侧RSSI
	int8_t snrDTU;  //节点侧SNR
	int8_t snrGW;  //网关侧SNR
	uint8_t lifeCycle; //数据生命周期
}DTU_INFO_TYPE;

#endif