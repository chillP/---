#include "stdio.h"
#include "stdlib.h"
#include "GasSensorTest.h"
#include "usart.h"
#include "loragw_user.h"
#include "gpio.h"
#include "protocol_analysis.h"

extern USART_RECEIVETYPE Usart2_RX;
extern RFSystemParameter_TYPE SystemParameter;

extern struct SX1301Status_S SX1301Status;
extern struct lgw_conf_rxrf_s *rfAp;
extern struct lgw_conf_rxrf_s *rfBp;
extern struct lgw_conf_rxif_s *prxifms;
extern	struct lgw_conf_rxif_s ifconfm[8];
extern	struct lgw_conf_rxrf_s rfconfA,rfconfB;

/**
 * @brief   ����DTUģ������ - ����ģʽ��ʼ��
 * @details SX1301���ü�������ʼ��
 * @param   ��
 * @return  ��
 */
void TestMode_Init(void)
{
	//������ʼ��
	SystemParameter.LowDatarateOptmize = 0x60;
	SystemParameter.RxPreambleLen = 15;
	SystemParameter.ReceiveCRC = REC_CRC;
	SystemParameter.CalibrationFailFlag = 0;
	
	//����ṹ��
	memset(&SX1301Status, 0x00, sizeof(SX1301Status));
	
	//ʹ��1301
	SX1301_Enable(1);
	SX1301_ENABLE;
	HAL_Delay(20);	
	
	//1301��������
	rfconfA.enable = 1 ;   //ʹ��radioA
	rfconfA.freq_hz = (uint32_t)(488.6*1e6);  //RadioA����Ƶ������
	
	rfconfB.enable = 1 ;   //ʹ��radioB
	rfconfB.freq_hz = (uint32_t)(489.4*1e6);  //RadioA����Ƶ������
	
	//����ͨ������
    ifconfm[0].enable = true;
    ifconfm[0].rf_chain = 0;
    ifconfm[0].freq_hz = -300000;
    ifconfm[0].datarate = DR_LORA_MULTI;

    ifconfm[1].enable = true;
    ifconfm[1].rf_chain = 0;
    ifconfm[1].freq_hz = -100000;
    ifconfm[1].datarate = DR_LORA_MULTI;

    ifconfm[2].enable = true;
    ifconfm[2].rf_chain = 0;
    ifconfm[2].freq_hz = 100000;
    ifconfm[2].datarate = DR_LORA_MULTI;

    ifconfm[3].enable = true;
    ifconfm[3].rf_chain = 0;
    ifconfm[3].freq_hz = 300000;
    ifconfm[3].datarate = DR_LORA_MULTI;

    ifconfm[4].enable = true;
    ifconfm[4].rf_chain = 1;
    ifconfm[4].freq_hz = -300000;
    ifconfm[4].datarate = DR_LORA_MULTI;

    ifconfm[5].enable = true;
    ifconfm[5].rf_chain = 1;
    ifconfm[5].freq_hz = -100000;
    ifconfm[5].datarate = DR_LORA_MULTI;
		
    ifconfm[6].enable = true;
    ifconfm[6].rf_chain = 1;
    ifconfm[6].freq_hz = 100000;
    ifconfm[6].datarate = DR_LORA_MULTI;


    ifconfm[7].enable = true;
    ifconfm[7].rf_chain = 1;
    ifconfm[7].freq_hz = 300000;
    ifconfm[7].datarate = DR_LORA_MULTI;
	
	SystemParameter.LoRaMAC = 1;	
	
	//1301������־
	lgw_stop();
	SX1301Status.NeedStartFlag = 1;		
	SX1301Status.StartSuccesFlag = 0;//��������״̬���
	
}
	
/**
 * @brief   ����DTUģ������ - ͸��ģʽ��ʼ��
 * @details SX1301���ü�������ʼ��
 * @param   ��
 * @return  ��
 */
void DebugMode_Init(void)
{
	
}

/**
 * @brief   ����DTUģ������ - ����ģʽ����
 * @details �Բ���ָ����Ӧ�ڵ������֡�����DTU��ɲ��Ժ��������Խ��
 * @param   ��
 * @return  ��
 */
void FactoryTestMode_Run(void)
{
	//����
	if(SX1301Status.NeedStartFlag ==1)
	{
		printf("SX1301 Starting...\r\n");
		SX1301Status.StartSuccesFlag = SX1301Start(rfAp,rfBp,prxifms);
		SX1301Status.NeedStartFlag =0;//����������ʶ
		if(SX1301Status.StartSuccesFlag == 0)
		{
			Sx1301StartFail();//IOָʾ
		}
		else
		{
			Sx1301StartSucces();//IOָʾ
		}		
	}
	//�����ɹ���ʼ����
	if(SX1301Status.StartSuccesFlag == 0)
	{
		SX1301RXDataQueryPin();//ʹ�������Ҫ����SX1301�����⹦�����ŵ�ָʾ
		DEBUG_Printf("Test mode runnning!!!\r\n");
		HAL_Delay(1000);
	}
}

/**
 * @brief   ����DTUģ������ - ͸����������
 * @details �Ե���ָ����Ӧ�ڵ������֡������͸��ģʽ��ģ��������DTU�佨��͸��ͨ��
 * @param   ��
 * @return  ��
 */
void FactoryDebugMode_Run(void)
{
	
}
	