#include "stdio.h"
#include "stdlib.h"
#include "GasSensorTest.h"
#include "usart.h"
#include "loragw_user.h"
#include "gpio.h"
#include "protocol_analysis.h"

struct lgw_pkt_tx_s tx_packet;
uint8_t powerDtu=0;  //DTUȱʡ���书��0dBm

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
	SystemParameter.RxPreambleLen = 8;
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
	
	//�����������
	tx_packet.bandwidth		= 	BW_125KHZ;
	tx_packet.coderate		 =	 CR_LORA_4_5;
	tx_packet.datarate 		= 	DR_LORA_SF7;
	tx_packet.freq_hz 		= 	(uint32_t)(488.3*1e6);
	tx_packet.modulation	 	= 	MOD_LORA;
	tx_packet.no_crc 			= 	false;
	tx_packet.preamble		 = 	8;
	tx_packet.rf_chain		 = 	0;
	tx_packet.rf_power		 = 	0;
	tx_packet.tx_mode		 = 	IMMEDIATE;
	tx_packet.size 			 =	12;	
	
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
 * @brief   �������ݰ��ڵ�DEVEUI����Ϣ��̬�洢���ڵ�λ��
 * @details 
 * @param   ��
 * @return  ��
 */
uint8_t infoSearch(uint8_t* payload)
{
	uint8_t i,j=0;
	
	for(i=0;i<20;i++)
	{
		for(j=0;j<8+1;j++)
		{
			if(j>=8)  //������ָ��DEVEUI
			{
				return i;  //����DEVEUI�Ĵ洢λ��
			}
			
			if(payload[j+3] != dtuInfoRecordBuf[i].deveui[j])
			{
				break;
			}
		}
	}
	
	return 100;  //δ��������ǰDEVEUI
}


/**
 * @brief   ����DTUģ������ - ͸����������
 * @details �Ե���ָ����Ӧ�ڵ������֡������͸��ģʽ��ģ��������DTU�佨��͸��ͨ��
 * @param   ��
 * @return  ��
 */
void FactoryDebugMode_Run(void)
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
	
	//���ְ�����
	if(SX1301Status.StartSuccesFlag == 1)
	{
		SX1301RXDataQueryPin();//ʹ�������Ҫ����SX1301�����⹦�����ŵ�ָʾ
		//DEBUG_Printf("Test mode runnning!!!\r\n");
		//HAL_Delay(1000);
	}
	
	//
	radioDataDecode();
	
	//����ָ���
	serialDataDecode();	
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
	uint8_t handShakeAcp[16]={0xC8,0xDF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0xB0,0xFB,0x00};  //2�ֽ�֡ͷ+1�ֽڿ�����+8�ֽ�DEVEUI+...
	uint8_t downLink1Pkt[14]={0xC8,0xDF,0xF2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X00,0X00};  //2�ֽ�֡ͷ+1�ֽڿ�����+8�ֽ�DEVEUI
	uint8_t downLink2Pkt[14]={0xC8,0xDF,0xF4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X00,0X00};  //2�ֽ�֡ͷ+1�ֽڿ�����+8�ֽ�DEVEUI
	uint8_t frameFaultFlag=0;
	uint8_t send_access_flag=1;
	uint8_t logLevel = 2;
	bool frameCheckOk = true;
	uint8_t i,j=0;
	uint8_t pkt=0;
	uint8_t freeBuffNum=0;
	uint8_t infoBuffNum=0;
	int8_t txRssi=0;
	int8_t rxRss=0;
	int8_t txSnr=0;
	int8_t rxSnr=0;
	
	for(pkt=0;pkt<nb_pkt;pkt++)
	{
		//֡У��
		if(rxpkt[pkt].payload[0]==0xE4 && rxpkt[pkt].payload[1]==0xFB)  //֡ͷ
		{
			DEBUG_Printf("-get data: ");
			for (i = 0;i<rxpkt[pkt].size;i++) 
			{
				DEBUG_Printf("%02X ", rxpkt[pkt].payload[i]);
			}
			DEBUG_Printf("\r\n");
			
			switch (rxpkt[pkt].payload[2])
			{
				case 0x01:  //�����룺���ְ�
					
					for(i=0;i<21;i++)  //�������д洢�� 
					{
						if(i>=20)  //�޿��д洢�� 
						{
							freeBuffNum = 100;
							break;
						}
						if(dtuInfoRecordBuf[i].lifeCycle == 0)  
						{
							freeBuffNum = i;
							break;
						}
					}
					if(freeBuffNum != 100)
					{
						//�洢DEVEUI
						for(j=0;j<8;j++)  
						{
							dtuInfoRecordBuf[freeBuffNum].deveui[j] = rxpkt[pkt].payload[j+3];  
						}
						
						//�洢�ź�����
						//if(rxpkt[0].rssi < -127) rxpkt[0].rssi = -127;  //�޷�
						//DEBUG_Printf("RSSI: %.2f SNR: %.2f\r\n",rxpkt[pkt].rssi,rxpkt[pkt].snr);
						//DEBUG_Printf("RSSI: %d SNR: %d\r\n",(int8_t)rxpkt[pkt].rssi, (int8_t)rxpkt[pkt].snr);
						//dtuInfoRecordBuf[i].rssiDTU = (int8_t)rxpkt[pkt].rssi;
						//dtuInfoRecordBuf[i].snrDTU = (int8_t)rxpkt[pkt].snr;
						
						//�趨��������
						dtuInfoRecordBuf[i].lifeCycle = 5;  
			
						//handShakeAcp[11] = dtuInfoRecordBuf[i].rssiDTU ;  //�ź�����
						//handShakeAcp[12] = dtuInfoRecordBuf[i].snrDTU ;
						
						//���ְ�����
						for(j=0;j<8;j++)  //���а�EUI�����а���EUI
						{
							handShakeAcp[j+3]=rxpkt[pkt].payload[j+3];
						}
			 			handShakeAcp[12] = powerDtu;  //DTU���书������
						
						DEBUG_Printf("-send data: ");
						for(j=0;j<16;j++)  //TXPacket��ֵ
						{
							tx_packet.payload[j] = handShakeAcp[j];
							DEBUG_Printf("%02X",tx_packet.payload[j]);
						}
						DEBUG_Printf("\r\n");
						tx_packet.size = 16;
						lgw_send(tx_packet);				
					}
					else
					{
						DEBUG_Printf("-infoBuf is full !");
					}
					break;
				case 0xF1:  //�����룺Uplink1
					//CSУ��
				
					//DEVEUI����
					infoBuffNum = infoSearch(rxpkt[pkt].payload);
					
					//��Ϣ����
					if(infoBuffNum != 100)
					{
						dtuInfoRecordBuf[infoBuffNum].sensorTestResult = rxpkt[pkt].payload[13];  //���洮�ڲ��Խ��
						dtuInfoRecordBuf[infoBuffNum].rssiGW = (int8_t)rxpkt[pkt].rssi;  //��¼Uplink1�ź�����
						dtuInfoRecordBuf[infoBuffNum].snrGW = (int8_t)rxpkt[pkt].snr;
					
						//Downlink1����
						for(i=0;i<8;i++)  //д��DEVEUI
						{
							downLink1Pkt[i+3] = dtuInfoRecordBuf[infoBuffNum].deveui[i];
						}
						
						downLink1Pkt[11] = dtuInfoRecordBuf[infoBuffNum].rssiGW;  //д��Uplink1�ź�����
						downLink1Pkt[12] = dtuInfoRecordBuf[infoBuffNum].snrGW;
						
						DEBUG_Printf("-send data: ");
						for(i=0;i<14;i++)  //TXPacket��ֵ
						{
							tx_packet.payload[i] = downLink1Pkt[i];
							DEBUG_Printf("%02X",tx_packet.payload[i]);
						}
						DEBUG_Printf("\r\n");
						tx_packet.size = 14;
						lgw_send(tx_packet);
					}
					else
					{
						DEBUG_Printf("-deveui error: unhandshaked eui");
					}

					break;
				case 0xF3:  //�����룺Uplink2
					//CSУ��
				
					//DEVEUI����
					infoBuffNum = infoSearch(rxpkt[pkt].payload);
					
					//��Ϣ����
					if(infoBuffNum != 100)
					{
						dtuInfoRecordBuf[infoBuffNum].rssiDTU = rxpkt[pkt].payload[11];  //��¼Downlink1���յ��ź�����
						dtuInfoRecordBuf[infoBuffNum].snrDTU = rxpkt[pkt].payload[12];
						
						//Downlink2����
						for(i=0;i<8;i++)  //д��DEVEUI
						{
							downLink2Pkt[i+3] = dtuInfoRecordBuf[infoBuffNum].deveui[i];
						}

						DEBUG_Printf("-send data: ");
						for(i=0;i<14;i++)  //TXPacket��ֵ
						{
							tx_packet.payload[i] = downLink2Pkt[i];
							DEBUG_Printf("%02X",tx_packet.payload[i]);
						}
						DEBUG_Printf("\r\n");
						tx_packet.size = 14;
						lgw_send(tx_packet);
						
						//���Խ�����
						int8_t txRssi = (int8_t)dtuInfoRecordBuf[infoBuffNum].rssiGW;
						int8_t rxRssi = (int8_t)dtuInfoRecordBuf[infoBuffNum].rssiDTU;
						int8_t txSnr = (int8_t)dtuInfoRecordBuf[infoBuffNum].snrGW;
						int8_t rxSnr = (int8_t)dtuInfoRecordBuf[infoBuffNum].snrDTU;
						
						DEBUG_Printf("\r\n***Test Mode***\r\n");
						DEBUG_Printf("&DEVEUI:");  //DEVEUI
						for(i=0;i<8;i++)
						{
							DEBUG_Printf("%02x",dtuInfoRecordBuf[infoBuffNum].deveui[i]);
						}
						DEBUG_Printf("$\r\n");
						
						if(dtuInfoRecordBuf[infoBuffNum].sensorTestResult == 0xff) DEBUG_Printf("&SERIAL:pass$\r\n");  //���ڲ��Խ��
						else printf("&SERIAL:fail$\r\n");
						
						if(txRssi>0) DEBUG_Printf("&TX-RSSI:+%d$\r\n",txRssi);  //�ź��������
						else DEBUG_Printf("&TX-RSSI:%d$\r\n",txRssi);
						if(rxRssi>0) DEBUG_Printf("&RX-RSSI:+%d$\r\n",rxRssi);
						else DEBUG_Printf("&TX-RSSI:%d$\r\n",rxRssi);
						if(txSnr>0) DEBUG_Printf("&TX-SNR:+%d$\r\n",txSnr);
						else DEBUG_Printf("&TX-RSSI:%d$\r\n",txSnr);
						if(rxSnr>0) DEBUG_Printf("&RX-SNR:+%d$\r\n",rxSnr);
						else DEBUG_Printf("&TX-RSSI:%d$\r\n\r\n",rxSnr);					
						
						//��������
						dtuInfoRecordBuf[infoBuffNum].lifeCycle = 0;  //��ɲ��Ժ������������								
					}
					else
					{
						DEBUG_Printf("-deveui error: unhandshaked eui");
					}
									
					break;
				default:
					if(logLevel == 2) printf("Ctrl bit check error\r\n");
					frameCheckOk = false;	
					break;				
			}	
		}
		else
		{
			if(logLevel == 2) printf("Frame head check error\r\n");
			frameCheckOk = false;	
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
	