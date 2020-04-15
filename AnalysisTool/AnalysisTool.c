/*
Description: ���ļ�Ϊ�����������еı��壬
						 �����������ߵĳ�ʼ�������С�
*/

#include "AnalysisTool.h"
#include "loragw_user.h"
#include "softwareversion.h"
#include "rtc.h"
#include "gpio.h"
#include "PC_Seting.h"

extern USART_RECEIVETYPE Usart2_RX;
extern RFSystemParameter_TYPE SystemParameter;
extern FRAME_RECEIVE_TYPE *pFrame;//���յ��û�����֡������
extern struct ProtocolRespond_S *pMCUTXData;//�ظ�����

extern struct SX1301Status_S SX1301Status;
extern struct lgw_conf_rxrf_s *rfAp;
extern struct lgw_conf_rxrf_s *rfBp;
extern struct lgw_conf_rxif_s *prxifms;

// ��λ�����ӱ�־
uint8_t AT_ConnectUpper;

// �������߳�ʼ��
void AnalysisTool_init(void){
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
	
	SoftwareVersionShow();//����汾
	RTC_TimeShow();//RTCʱ��
	
	AT_ConnectUpper = 0;
	
}


// ������������
void AnalysisTool_Run(void){
	
	
	
	//����
	if(SX1301Status.NeedStartFlag ==1)
	{
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
	if(SX1301Status.StartSuccesFlag == 1)
	{
			SX1301RXDataQueryPin();//ʹ�������Ҫ����SX1301�����⹦�����ŵ�ָʾ
	}
	//���ڽ�������
	if(Usart2_RX.receive_flag == 1)
	{
		if(Usart2_RX.RX_Buf[0] == 0xFE){
			//__HAL_UART_DISABLE_IT(&huart2, Usart2_RX);//�ȹرս��գ��������ٴ�		
			ProtocolAnalysis(Usart2_RX.RX_Buf,Usart2_RX.rx_len,pFrame,pMCUTXData);
			//__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
			
			// �����豸����
		}else if(Usart2_RX.RX_Buf[0] == 0xEE){
			PC_SetingAnalysis(Usart2_RX.RX_Buf, Usart2_RX.rx_len);
		}else{
			if(Usart2_RX.RX_Buf[0] == 'c' && Usart2_RX.RX_Buf[1] == 'o' && Usart2_RX.RX_Buf[2] == 'n' && Usart2_RX.RX_Buf[3] == 'n'){
				AT_ConnectUpper = 1;	
			}
		}
	
		Usart2_RX.receive_flag = 0;
		//Usart2_RX.rx_len = 0;		
	}
}

