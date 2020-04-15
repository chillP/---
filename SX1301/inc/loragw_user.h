
#ifndef _LORAGW_USER_H
#define _LORAGW_USER_H

#include "stdio.h"
#include "stdint.h"
#include "loragw_hal.h"
#include "stdlib.h"

#include "loragw_spi.h"
#include "loragw_reg.h"

//########################���Թ��ܿ���
#define RTCTimeShow 1//����֮����ʾRTCʱ��
//#define DebugDisplayFromUserData 1//����֮���ӡ���Դ��ڵ�����
#define DebugDisplayTabString 1//����֮���ӡ��������Ƶ���ܲ���
//#define DebugDisplayTabStringSimple 1//����֮���ӡ��������Ƶ���ܲ��������
#define DebugDisplayAnalyLoRaWAN 1//�򿪺�֮�����LoRaWAN����
//#define DebugDisplayToUserRFData 1//�򿪺궨�������Ƶ���ݸ��û�/#define DebugDisplayToUserProtocolData 1//�򿪺궨�����Э�����ݸ��û�
#define DebugDisplayLoraPayload 1 //���payload
#define DebugDisplayTimestramp 1 //��ӡ����ʱ���
//#define DebugFreqIFCalibration 1 //���շ���ʱƵ��У׼ ���ص��������з�����
//#define DebugDisplayRxCnt 1 //���յ������ݰ�������ӡ
//#define DebugDisplayRXTest 1//����������
//#define DisplaySendInfo 1//�򿪺궨���������ʱ��
//#define DebugCalibrationSkip 1//�򿪿�������У׼ʧ��
#define DebugDisplayParameter 1//����ʾ����lora���ò��� crc mac inver

//#define TESTTOOL 1//���Թ�װ����  �к궨��򿪲��Թ�װ����

//���࿪��,��֮��ֱ�ӿ��� 1�� 0��
#define DISTANCE_TEST 0
#define TimeCalibrationSingleSend 0//ֻ��һ��ģ�鷢��
#define TimeCalibrationEachOtherSend 0//�໥����


//RTC��ӡʱ��������
#define RTCTimeDisplayTime 10

//RTC BCDʱ���ʽ
#define RTCInitBCDYear 0x18
#define RTCInitBCDDate 0x14
#define RTCInitBCDMonth RTC_MONTH_SEPTEMBER
#define RTCInitBCDWeekDay RTC_WEEKDAY_FRIDAY
#define RTCInitBCDHours 0x00
#define RTCInitBCDMinutes 0x00
#define RTCInitBCDSeconds 0x00

//ѡ��Ƶ��
//#define F433 		1//433
#define F470 		2//465-515
//#define F868 		3//862-870
//#define F915 		4//902-930
//#define F470510	5//465-51
//#define F433510	6//430-515

//ʱ��ѡ��
//#define SX1301CLK0 0//�򿪺궨��ѡ��ʱ��Դ0���ر�ѡ��ʱ��Դ1

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

#define MIN_F MIN_SEND_F //Radio����Ƶ����Сֵ
#define MAX_F MAX_SEND_F //Radio����Ƶ�����ֵ
#define MAX_B 5000000//�������ƫ��

//
#define RFRXData_LEN 1024//

#define TimeCalibrationCycle 60000000//us  
//��Դ����
#define SX1301_PORT GPIOA
#define SX1301_PIN GPIO_PIN_12
//æ״ָ̬ʾ
#define Sx1301Busy()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define Sx1301Free()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
//����ָ̬ʾ
#define Sx1301StartSucces()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define Sx1301StartFail()			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

//SX1301��������
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

//�����򳤶�
typedef struct  
{  
	char Name[20];//��ʾ�ַ�������
	uint32_t DataLen;//���ݳ���
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
	uint8_t ReceiveCRC;//����CRC����
	uint8_t ReceiveIQInvert;
	uint8_t SendIQInvert;
	uint8_t CWSendModeEN;
	uint8_t RFRxTxDebugEN;
	uint8_t TimeTrigSendEN;//�Ѿ�ʹ��ʱ�������ģʽ������ʱ����Ѿ�����
	uint8_t CalibrationFailFlag;
	uint8_t CalibrationFailTime;//���Դ���
	uint32_t TimestampSendTime;//ʱ������͵�ʱ��
	uint32_t Timestamp;//SX1301ʱ��
	uint32_t RxCnt;//�ӽ������ݰ�����
	uint32_t RxPreambleLen;//����ǰ���볤��
	uint32_t TXCnt;
}RFSystemParameter_TYPE;

#define CalibrationFailTime_H 3//3У׼ʧ�����Դ���

#define REC_CRC 0x01//������CRC
#define REC_NO_CRC 0x02//������CRC
#define REC_ERROR_CRC 0x04//���մ���CRC

//�����ʱ��
#define  MAX_SEND_TIME 30000//ms ����������ģʽ

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


