
#ifndef _LORAGW_USER_H
#define _LORAGW_USER_H

#include "stdio.h"
#include "stdint.h"
#include "loragw_hal.h"
#include "stdlib.h"

#include "loragw_spi.h"
#include "loragw_reg.h"

//########################调试功能开关
#define RTCTimeShow 1//定义之后显示RTC时间
//#define DebugDisplayFromUserData 1//定义之后打印来自串口的数据
#define DebugDisplayTabString 1//定义之后打印表格输出射频性能参数
//#define DebugDisplayTabStringSimple 1//定义之后打印表格输出射频性能参数精简版
#define DebugDisplayAnalyLoRaWAN 1//打开宏之后解析LoRaWAN数据
//#define DebugDisplayToUserRFData 1//打开宏定义输出射频数据给用户/#define DebugDisplayToUserProtocolData 1//打开宏定义输出协议数据给用户
#define DebugDisplayLoraPayload 1 //输出payload
#define DebugDisplayTimestramp 1 //打印数据时间戳
//#define DebugFreqIFCalibration 1 //接收反向时频率校准 网关的数据下行反向发送
//#define DebugDisplayRxCnt 1 //接收到的数据包计数打印
//#define DebugDisplayRXTest 1//测试灵敏度
//#define DisplaySendInfo 1//打开宏定义输出发送时间
//#define DebugCalibrationSkip 1//打开可以跳过校准失败
#define DebugDisplayParameter 1//打开显示接收lora配置参数 crc mac inver

//#define TESTTOOL 1//测试工装代码  有宏定义打开测试工装代码

//拉距开关,打开之后直接开启 1开 0关
#define DISTANCE_TEST 0
#define TimeCalibrationSingleSend 0//只用一个模块发送
#define TimeCalibrationEachOtherSend 0//相互发送


//RTC打印时间间隔分钟
#define RTCTimeDisplayTime 10

//RTC BCD时间格式
#define RTCInitBCDYear 0x18
#define RTCInitBCDDate 0x14
#define RTCInitBCDMonth RTC_MONTH_SEPTEMBER
#define RTCInitBCDWeekDay RTC_WEEKDAY_FRIDAY
#define RTCInitBCDHours 0x00
#define RTCInitBCDMinutes 0x00
#define RTCInitBCDSeconds 0x00

//选择频率
//#define F433 		1//433
#define F470 		2//465-515
//#define F868 		3//862-870
//#define F915 		4//902-930
//#define F470510	5//465-51
//#define F433510	6//430-515

//时钟选择
//#define SX1301CLK0 0//打开宏定义选择时钟源0，关闭选择时钟源1

#ifdef F433
#define MIN_SEND_F			430000000//
#define MAX_SEND_F			436000000//
#define LGW_RADIO_TYPE	LGW_RADIO_TYPE_SX1255
#endif

#ifdef F470
#define MIN_SEND_F			465000000//
#define MAX_SEND_F			515000000//
#define LGW_RADIO_TYPE	LGW_RADIO_TYPE_SX1255
#endif


#ifdef F868
#define MIN_SEND_F			862000000//
#define MAX_SEND_F			870000000//
#define LGW_RADIO_TYPE	LGW_RADIO_TYPE_SX1257
#endif


#ifdef F915
#define MIN_SEND_F			900000000//
#define MAX_SEND_F			930000000//
#define LGW_RADIO_TYPE	LGW_RADIO_TYPE_SX1257
#endif

#ifdef F470510
#define MIN_SEND_F			465000000//
#define MAX_SEND_F			515000000//
#define LGW_RADIO_TYPE	LGW_RADIO_TYPE_SX1255
#endif

#ifdef F433510
#define MIN_SEND_F			430000000//
#define MAX_SEND_F			515000000//
#define LGW_RADIO_TYPE	LGW_RADIO_TYPE_SX1255
#endif

#define MIN_F MIN_SEND_F //Radio中心频率最小值
#define MAX_F MAX_SEND_F //Radio中心频率最大值
#define MAX_B 5000000//单边最大偏移

//
#define RFRXData_LEN 1024//

#define TimeCalibrationCycle 60000000//us  
//电源控制
#define SX1301_PORT GPIOA
#define SX1301_PIN GPIO_PIN_12
//忙状态指示
#define Sx1301Busy()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define Sx1301Free()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
//启动态指示
#define Sx1301StartSucces()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define Sx1301StartFail()			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

//SX1301功能引脚
#define RXBUFF_NOT_EMPTY_PORT 		GPIOA
#define RXBUFF_NOT_EMPTY_PIN		GPIO_PIN_0
#define SENSOR_PKT_PORT 				GPIOC
#define SENSOR_PKT_PIN					GPIO_PIN_3
#define TX_ON_PORT 					GPIOC
#define TX_ON_PIN						GPIO_PIN_6



int fputc(int ch,FILE *f);

uint8_t SX1301Start(struct lgw_conf_rxrf_s *rfconfA,struct lgw_conf_rxrf_s *rfconfB,struct lgw_conf_rxif_s *pifms);
uint8_t SX1301RXData(void);
uint8_t SX1301RXDataQueryPin(void);
uint8_t SX1301TXData(void);
void Sx1301Power(uint8_t OnOrOff);
void DistanceTest(void);
void WaiteTimeDisplay(uint32_t j);
void DispalyTabString(void);
void DispalyTabStringSimple(void);
uint8_t  AnalysisLoRaWANData(void);

//数据域长度
typedef struct  
{  
	char Name[20];//显示字符串名称
	uint32_t DataLen;//数据长度
}DIS_INFO_TYPE;

//LoRaWAN Data
typedef struct  
{  
	uint8_t MHDR;
	uint8_t Fport;
	uint8_t DLSettings  ;
	uint8_t RxDelay   ;
	uint8_t FCTL;
	uint8_t ADR;
	uint8_t ADRACKReq;
	uint8_t ACK;
	uint8_t FOptslen;
	uint16_t Fcnt;	
	uint8_t MType;
	uint8_t RFU;
	uint8_t Major;
	uint8_t FRamePayloadIndex;
	uint8_t FRamePayloadLen;
	uint8_t APPEUI[8];
	uint8_t DEVEUI[8];
	uint8_t DevNonce[3];
	uint8_t NetID[3];
	uint8_t MIC[4];	
	uint8_t AppNonce[3];	
	uint8_t DevAddr[4];
	uint8_t CFList[16];	
	uint8_t Fopts[15];	
	uint8_t MACPayload[256];
}LoRaWAN_TYPE;

typedef struct  
{  
	uint8_t LoRaMAC;
	uint8_t TxContinuous;
	uint8_t LowDatarateOptmize;
	uint8_t ReceiveCRC;//接收CRC配置
	uint8_t ReceiveIQInvert;
	uint8_t SendIQInvert;
	uint8_t CWSendModeEN;
	uint8_t RFRxTxDebugEN;
	uint8_t TimeTrigSendEN;//已经使用时间戳发送模式，而且时间戳已经给出
	uint8_t CalibrationFailFlag;
	uint8_t CalibrationFailTime;//重试次数
	uint32_t TimestampSendTime;//时间戳发送的时间
	uint32_t Timestamp;//SX1301时间
	uint32_t RxCnt;//接接收数据包计数
	uint32_t RxPreambleLen;//接收前导码长度
	uint32_t TXCnt;
}RFSystemParameter_TYPE;

#define CalibrationFailTime_H 3//3校准失败重试次数

#define REC_CRC 0x01//接收无CRC
#define REC_NO_CRC 0x02//接收无CRC
#define REC_ERROR_CRC 0x04//接收错误CRC

//发送最长时间
#define  MAX_SEND_TIME 30000//ms 非连续发送模式

//MType
#define JoinRequest_H 			0
#define JoinAccept_H			1
#define UconfirmedDataUp_H	2
#define UconfirmedDataDown_H	3
#define ConfirmedDataUp_H		4
#define ConfirmedDataDown_H	5
#define RFU_H					6	
#define Proprietary_H			7

#endif


