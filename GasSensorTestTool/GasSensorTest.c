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
extern struct lgw_conf_rxif_s ifconfm[8];
extern struct lgw_conf_rxrf_s rfconfA,rfconfB;
extern struct lgw_pkt_rx_s rxpkt[4]; 
extern int nb_pkt;

typedef struct  
{  
	uint8_t deveui[8];  //DTU�豸�ż�¼
	float rssiDTU;  //�ڵ�RSSI
	float rssiGW;  //���ز�RSSI
	float snrDTU;  //�ڵ��SNR
	float snrGW;  //���ز�SNR
	uint8_t lifeCycle  //������������
}DTU_INFO_TYPE;

DTU_INFO_TYPE dtuInfoRecordBuf[20];

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
	
	SystemParameter.LoRaMAC = 0;	
	
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
	
	//��Ƶ���ݽ���
	if(SX1301Status.StartSuccesFlag == 1)
	{
		SX1301RXDataQueryPin();//ʹ�������Ҫ����SX1301�����⹦�����ŵ�ָʾ
		//DEBUG_Printf("Test mode runnning!!!\r\n");
		//HAL_Delay(1000);
	}
	
	//��Ƶ���ݴ���
	radioDataDecode();
	
	//����ָ���
	serialDataDecode();
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

/**
 * @brief   ��Ƶ���ݽ���
 * @details ����1301�յ������ݣ�ִ�ж�Ӧ�Ĵ���
 * @param   ��
 * @return  ��
 */
void radioDataDecode(void)
{
	uint8_t testData[10]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
	uint8_t handShakeAcp[12]={0xC8,0xDF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  //2�ֽ�֡ͷ+1�ֽڿ�����+8�ֽ�DEVEUI
	uint8_t frameFaultFlag=0;
	uint8_t send_access_flag=1;
	uint8_t logLevel = 2;
	bool frameCheckOk = true;
	uint8_t i,j=0;
	
	struct lgw_pkt_tx_s tx_packet;
	
	if(nb_pkt!=0)
	{
		//֡У��
		if(rxpkt[0].payload[0]==0xE4 && rxpkt[0].payload[1]==0xFB)  //֡ͷ
		{
			if(rxpkt[0].payload[2]==0x01)  //�����룺���ְ�
			{
				//��Ϣ�Ǽ�
				for(i=0;i<20;i++)  //DTU��Ϣ��洢��
				{
					if(dtuInfoRecordBuf[j].deveui != NULL)  //�������д洢�� #���ȷ��deveui�洢��Ψһ�ԣ�
					{
						for(j=0;j<8;j++)  //�洢DEVEUI
						{
							dtuInfoRecordBuf[j].deveui[j] = rxpkt[0].payload[i+3];  
						}
						dtuInfoRecordBuf[j].lifeCycle = 20;  //�趨��������
					}
				}
				//���ְ�����
				for(i=0;i<8;i++)  //���а�EUI�����а���EUI
				{
					handShakeAcp[i+3]=rxpkt[0].payload[i+3];
				}
				for(i=0;i<12;i++)  //TXPacket��ֵ
				{
					tx_packet.payload[i] = handShakeAcp[i];
				}
				lgw_send(tx_packet);
			}
			else if(rxpkt[0].payload[2]==0xF1)  //�����룺���԰�1
			{
				
			}
			else if(rxpkt[0].payload[2]==0xF3)  //�����룺���԰�3
			{
				
			}
			else
			{
				if(logLevel == 2) printf("Ctrl bit check error\r\n");
				frameCheckOk = false;	
			}
		}
		else
		{
			if(logLevel == 2) printf("Frame head check error\r\n");
			frameCheckOk = false;	
		}
		
		
		
		
		
		
		
		
		
		
		//DEVEUI��¼
		for(uint8_t i=0;i<9;i++)
		{
			//if(rxpkt[0].payload[i] != testData[i])
	
		}
		
		//���ݰ�����
		if(frameCheckOk)
		{
			DEBUG_Printf("Send packet: ");
			
			for(uint8_t i=0;i<9;i++)
			{
				tx_packet.payload[i] = 9-i;
			}
		
			tx_packet.bandwidth		= 	BW_125KHZ;
			tx_packet.coderate		 =	 CR_LORA_4_5;
			tx_packet.datarate 		= 	DR_LORA_SF7;
			tx_packet.freq_hz 		= 	(uint32_t)(488.3*1e6);
			tx_packet.modulation	 	= 	MOD_LORA;
			tx_packet.no_crc 			= 	false;
			tx_packet.preamble		 = 	8;
			tx_packet.rf_chain		 = 	0;
			tx_packet.rf_power		 = 	10;
			tx_packet.tx_mode		 = 	IMMEDIATE;
			tx_packet.size 			 =	9;	
			send_access_flag = lgw_send(tx_packet);
			
			DEBUG_Printf("Send packet: %d\r\n",send_access_flag);
		}
		
		
		
		
	}
}

/**
 * @brief   �������ݽ���
 * @details ������λ�����ڵ�ATָ�ִ�ж�Ӧ�Ĵ���
 * @param   ��
 * @return  ��
 */
void serialDataDecode(void)
{
	
}
	