/*
Description: ���ļ�����������ߵ����������������
						1�������У��
						2�����ͺͽ�������Ľ���������
						3��SX1301������Ϣ����������
						4�����ò����ظ�Ĭ��ֵ
*/

#include "protocol_analysis.h"
#include "loragw_user.h"
#include <stdbool.h>    /* bool type */
#include "usart.h"
#include "stm32l0xx_hal.h"
#include "loragw_spi.h"
#include "led.h"
#include "rtc.h"
#include "softwareversion.h"
#include "rf_rxtx_debug.h"
#include "loragw_reg.h"
#include "loragw_hal.h"
#include "string.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))//�����ܴ�С/һ��Ԫ�ش�С = Ԫ�ظ���


FRAME_RECEIVE_TYPE Frame;

extern struct lgw_pkt_tx_s txpkt; /* configuration and metadata for an outbound packet */

struct lgw_conf_rxrf_s rfconfA,rfconfB;
struct lgw_conf_rxif_s ifconfm[8];
struct user_pkt_tx_s TXPktUserFull; /* �û��������Ͳ��� */
struct user_pkt_tx_s TXPktUserDefault; /* �û�ȱʡ���Ͳ��� */
struct ProtocolRespond_S MCUTXData;//�ظ�����

struct lgw_conf_rxrf_s *rfAp = &rfconfA;
struct lgw_conf_rxrf_s *rfBp = &rfconfB;
struct lgw_conf_rxif_s *prxifms = ifconfm;
struct ProtocolRespond_S *pMCUTXData = &MCUTXData;//�ظ�����
FRAME_RECEIVE_TYPE *pFrame = &Frame;//���յ��û�����֡������

extern	struct SX1301Status_S SX1301Status;//1301״̬
extern	double xd;
extern	USART_INFO Usart2Info;//����2����

extern	RFSystemParameter_TYPE SystemParameter;


//������������ ��չ��������������
const uint8_t CTab[]=
{
	1,//����
	2,//��ѯ
	3//����
	//4 �������� �˴���������
};
//���ö�ٱ���
enum ETab{ETSetupDIDatalenTab,ETQueryDIDatalenTab,ETSendDIDatalenTab};//

//�������ݱ�ʶ�����ݳ��ȱ��
const FRAME_INFO_TYPE SetupDIDatalenTab[]=
{
	{0x01,0,Equal},
	{0x02,0,Equal},
	{0x03,0,Equal},
	{0x04,1,Equal},
	{0x05,1,Equal},
	{0x06,3,Equal},
	{0x07,3,Equal},
	{0x08,7,Equal},
	{0x09,7,Equal},
	{0x0A,7,Equal},
	{0x0B,1,Equal},
	{0x0C,8,Equal},
	{0x0D,11,Equal},
	{0x0E,1,Equal},
	{0x0F,1,Equal},
	{0x10,1,Equal},
	{0x11,1,Equal},
	{0x12,1,Equal},
	{0x13,1,Equal},
	{0x14,1,Equal},
	{0x15,1,Equal},
	{0x16,2,Equal},
	{0xFF,1,Equal}
};
//��ѯ���ݱ�ʶ�����ݳ��ȱ��
const FRAME_INFO_TYPE QueryDIDatalenTab[]=
{
	{0x00,1,Equal},
	{0x04,1,Equal},
	{0x05,1,Equal},
	{0x06,1,Equal},
	{0x07,1,Equal},
	{0x08,1,Equal},
	{0x09,1,Equal},
	{0x0A,1,Equal},
	{0x0B,1,Equal},
	{0x0C,1,Equal},
	{0x0D,1,Equal},
	{0x0E,1,Equal},
	{0x0F,1,Equal},
	{0x10,1,Equal},
	{0x11,1,Equal},
	{0x12,1,Equal},
	{0x13,1,Equal},
	{0x14,1,Equal},
	{0x15,1,Equal},
	{0x16,1,Equal},
	{0xFF,1,Equal}
};

//��ѯ���ݱ�ʶ�����ݳ��ȱ�� ��󳤶�
const FRAME_INFO_TYPE SendDIDatalenTab[]=
{
	{0x01,271,Less},//lora 16+255
	{0x02,255,Less},//lora ȱʡ
	{0x03,2039,Less},//lora ȱʡ���
	{0x04,271,Less},//fsk
	{0x05,255,Less},//fsk ȱʡ
};
void SystemParamterRecovery()
{
	memset(&SystemParameter, 0, sizeof(SystemParameter));
	SystemParameter.LowDatarateOptmize = 0x60;
}
//p����ֱ֡��
//len ����֡�ĳ���
//frame ����֡���� ����ָ���յ��û������ݵ�����
#if 1
//��Ƶ�����ָ���������
uint8_t ParaInit(struct lgw_conf_rxrf_s *rfAp,struct lgw_conf_rxrf_s *rfBp,struct lgw_conf_rxif_s * pifms)
{
	//��������
	#ifdef F470
	rfAp->freq_hz = (uint32_t)(465.6*1e6);
	rfBp->freq_hz = (uint32_t)(466.4*1e6);	
	//����Ƶ����ʱ����Ҫ
	xd = 475.3;
	#endif
	
	#ifdef F915
	rfAp->freq_hz = (uint32_t)(915.6*1e6);
	rfBp->freq_hz = (uint32_t)(916.4*1e6);	
	//����Ƶ����ʱ����Ҫ
	xd = 915.3;
	#endif	
	
	#ifdef F433510
	rfAp->freq_hz = (uint32_t)(467.6*1e6);
	rfBp->freq_hz = (uint32_t)(468.4*1e6);	
	xd = 475.3;
	#endif
	
	rfAp->enable = true;
	rfBp->enable = true;
	/* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
	
	//0
	pifms->enable	 = true ;
	pifms->rf_chain	 = 0;	
	pifms->freq_hz 	= -300000;
	pifms->datarate 	= DR_LORA_MULTI;
	
	//1
	pifms++;
	pifms->enable	 = false ;
	pifms->rf_chain	 = 0;	
	pifms->freq_hz 	= -100000;
	pifms->datarate 	= DR_LORA_MULTI;
	
	//2
	pifms++;
	pifms->enable	 = false ;
	pifms->rf_chain	 = 0;	
	pifms->freq_hz 	= 100000;
	pifms->datarate 	= DR_LORA_MULTI;
	
	//3
	pifms++;
	pifms->enable	 = false ;
	pifms->rf_chain	 = 0;	
	pifms->freq_hz 	= 300000;
	pifms->datarate 	= DR_LORA_MULTI;
	
	//4
	pifms++;
	pifms->enable	 = false ;
	pifms->rf_chain	 = 1;	
	pifms->freq_hz 	= -300000;
	pifms->datarate 	= DR_LORA_MULTI;
	
	//5
	pifms++;
	pifms->enable	 = false ;
	pifms->rf_chain	 = 1;	
	pifms->freq_hz 	= -100000;
	pifms->datarate 	= DR_LORA_MULTI;
	
	//6
	pifms++;
	pifms->enable	 = false ;
	pifms->rf_chain	 = 1;	
	pifms->freq_hz 	= 100000;
	pifms->datarate 	= DR_LORA_MULTI;
	
	//7
	pifms++;
	pifms->enable	 = false ;
	pifms->rf_chain	 = 1;	
	pifms->freq_hz 	= 300000;
	pifms->datarate 	= DR_LORA_MULTI;

	//TXPktUserDefault.freq_hz =(uint32_t)( 485.3*1E6+0.5);
	TXPktUserDefault.freq_hz = rfAp->freq_hz;
	//txpkt.tx_mode = IMMEDIATE;
	TXPktUserDefault.modulation = MOD_LORA;
	//TXPktUserDefault.modulation = MOD_FSK;	
	TXPktUserDefault.rf_power = 30 ;
	TXPktUserDefault.no_crc = false;
	//TXPktUserDefault.no_crc = true;
	TXPktUserDefault.coderate = CR_LORA_4_5;	
	TXPktUserDefault.datarate =DR_LORA_SF9;	
	TXPktUserDefault.tx_mode = IMMEDIATE;//ȱʡ���Ͷ�����������
	TXPktUserDefault.bandwidth = BW_125KHZ;	
	TXPktUserDefault.preamble = 8;
	
	return 1;
}
#else
uint8_t ParaInit()
{
	//��������
	rfconfA.freq_hz = (uint32_t)(475.3*1e6);
	rfconfB.freq_hz = (uint32_t)(485.3*1e6);	
	//����Ƶ����ʱ����Ҫ
	xd = 479.3;
	rfconfA.enable = true;
	rfconfB.enable = true;
	/* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
	
	ifconfm[0].enable	 = true ;
	ifconfm[0].rf_chain	 = 0;	
	ifconfm[0].freq_hz 	= -200000;
	ifconfm[0].datarate 	= DR_LORA_MULTI;

	ifconfm[1].enable	 = true ;
	ifconfm[1].rf_chain	 = 0;	
	ifconfm[1].freq_hz 	= -400000;
	ifconfm[1].datarate 	= DR_LORA_MULTI;

	ifconfm[2].enable	 = true ;
	ifconfm[2].rf_chain	 = 0;	
	ifconfm[2].freq_hz 	= 200000;
	ifconfm[2].datarate 	= DR_LORA_MULTI;

	ifconfm[3].enable	 = true ;
	ifconfm[3].rf_chain	 = 0;	
	ifconfm[3].freq_hz 	= 400000;
	ifconfm[3].datarate 	= DR_LORA_MULTI;

	ifconfm[4].enable	 = true ;
	ifconfm[4].rf_chain	 = 1;	
	ifconfm[4].freq_hz 	= -200000;
	ifconfm[4].datarate 	= DR_LORA_MULTI;

	ifconfm[5].enable	 = true ;
	ifconfm[5].rf_chain	 = 1;	
	ifconfm[5].freq_hz 	= -400000;
	ifconfm[5].datarate 	= DR_LORA_MULTI;

	ifconfm[6].enable	 = true ;
	ifconfm[6].rf_chain	 = 1;	
	ifconfm[6].freq_hz 	= 200000;
	ifconfm[6].datarate 	= DR_LORA_MULTI;

	ifconfm[7].enable	 = true ;
	ifconfm[7].rf_chain	 = 1;	
	ifconfm[7].freq_hz 	= 400000;
	ifconfm[7].datarate 	= DR_LORA_MULTI;	
	
	return 1;
}
#endif
#if 1
//���ұ�����Ƿ��д����ݱ�ʶ �з���1	 �޷���0
uint16_t CheckDI(FRAME_RECEIVE_TYPE *Frame)
{
	uint8_t valid = 0;
	uint8_t size = 0;//�����С
	const FRAME_INFO_TYPE *p;
	switch(Frame->C)
	{
		case SETUP_C:
			valid = 1;
			size = ARRAY_SIZE(SetupDIDatalenTab);
			p = SetupDIDatalenTab;
			break;
		case QUERY_C:
			valid = 1;
			size = ARRAY_SIZE(QueryDIDatalenTab);
			p = QueryDIDatalenTab;
			break;
		case TX_C:
			valid = 1;
			size = ARRAY_SIZE(SendDIDatalenTab);
			p = SendDIDatalenTab;
			break;
		default: valid = 0;
	}
	if(valid)
	{
		for(uint8_t i = 0;i<size;i++)
		{
			if(Frame->DI == p[i].DI)
			{
				Frame->DIIndex = i;
				return 1;
			}
		}
	}
	else
	{
		printf("\n Frame format Check :ERROR Ctrl Code[%d] \n",Frame->C);
		return 0;
	}	
	return 0;
}
//������ݳ���
//(uint8_t cc) ������
uint8_t CheckDataLen(FRAME_RECEIVE_TYPE *Frame)
{
	//uint8_t valid = 0;
	switch(Frame->C)
	{
		case SETUP_C:
			if(SetupDIDatalenTab[Frame->DIIndex].Len == Frame->DataLen)
			{
				//Valid =1;
				return 1;
			}
			else
			{
				//Valid = 0;				
				printf("\n Frame format Check :ERROR>>Datalen: %d is invalid\n",Frame->DataLen);
				return 0;
			}
		case QUERY_C:
			if(QueryDIDatalenTab[Frame->DIIndex].Len == Frame->DataLen)
			{
				//Valid =1;
				return 1;
			}
			else
			{
				//Valid = 0;
				printf("\n Frame format Check :ERROR>>Datalen: %d is invalid\n",Frame->DataLen);
				return 0;
			}
		case TX_C:
			if(SendDIDatalenTab[Frame->DIIndex].Len >= Frame->DataLen)
			{
				//Valid =1;
				return 1;
			}
			else
			{
				//Valid = 0;
				printf("\n Frame format Check :ERROR>>Datalen: %d is invalid\n",Frame->DataLen);
				return 0;
			}
		default: //valid = 0;
				return 0;
	}
	#if 0
	if(valid)
	{
		return 1;
	}
	else
	{		
		return 0;
	}	
	#endif
}

#endif
// ֡У��
uint8_t FrameCheck(uint8_t *p,uint16_t len ,FRAME_RECEIVE_TYPE *Frame)
{
	uint8_t FrameCs= 0;
	uint8_t Valid = 0;//�Ϸ���Ч
//	uint16_t temp;
	
	//֡�����
	#if 1
	if(len-Frame->Findcount < FRAME_MIN_LEN)
	{
		Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
		printf("\n Frame format Check :ERROR>>frame len is too short\n");
		return 0xfe;//����֡�쳣 ���ظ� �в���ѭ������ ֱ���Ƴ�����
	}
	#endif
	if(len>FRAME_MAX_LEN)//�����趨�����ֵ
	{
		Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
		printf("\n Frame format Check :ERROR>>frame len is too long \n");
		return 0xff;//����֡�쳣 ���ظ�
	}
	//����0x68
	Frame->StartIndex = 0xffff;//��ʼ������
	for(uint16_t i = Frame->Findcount; i<len; i++)
	{
		if(*(p + i)  == 0x68) 
		{
			Frame->StartIndex = i;			
			Frame->Findcount = i;//�ҵ������� ���Ż���ѭ��
			break;
		}
		//Frame->Findcount = i;//�ҵ������� ���Ż���ѭ��
	}
	
	if(Frame->StartIndex == 0xffff)//û�ҵ�
	{
		Frame->Findcount = len;//�ҵ������� ���Ż���ѭ��
		printf("\n Frame format Check :ERROR>>not find 0x68\n");
		return 0xfe;//����֡�쳣 ���ظ� �в���ѭ������ ֱ���Ƴ�����
	}
	else
	{

		if(len - Frame->StartIndex < FRAME_MIN_LEN)
		{			
			Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
			printf("\n Frame format Check :ERROR>>0x68 is coming too late\n");
			return 0xfe;//����֡�쳣 ���ظ� �в���ѭ������ ֱ���Ƴ�����
		}
	}
	//�ж�0x16
	//Frame->Findcount = Frame->DataIndex;//�ҵ������� ���Ż���ѭ��
	Frame->DataLen = *(p + Frame->StartIndex + 3)*256 + *(p + Frame->StartIndex + 4 );//�����򳤶�
	if(Frame->StartIndex + Frame->DataLen + 7 > len )//�ܳ��ȴ��ڽ��յ����ݳ�������֡������
	{
		Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
		printf("\n Frame format Check :ERROR>>data len longer than frame len\n");
		return 0xff;//����֡�쳣 ���ظ�
	}
	if(*(p + Frame->StartIndex + Frame->DataLen + 6)!=0x16)
	{
		Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
		printf("\n Frame format Check :ERROR>>not find 0x16\n");
		return 0xff;//����֡�쳣 ���ظ�
	}

	//�ŵ�������ڴ��������֡�ظ�ʱ����
	//���ݱ�ʶ
	Frame->DIIndex = Frame->StartIndex + 2;//���ݱ�ʶ����
	Frame->DI = *(p + Frame->DIIndex);	//���ݱ�ʶ
	Frame->C = *(p + Frame->StartIndex + 1);//������
	Frame->CIndex = Frame->StartIndex + 1;//����������
	Frame->DataIndex = Frame->DIIndex + 3;//����������
	
	//��������
	for(uint8_t i=0;i<ARRAY_SIZE(CTab);i++)
	{
		if(Frame->C == CTab[i])
		{
			Valid =1;
			break;
		}
	}
	//Frame->Findcount = Frame->CIndex;//�ҵ������� ���Ż���ѭ��
	if(Valid == 0)
	{
		Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
		printf("\n Frame format Check :ERROR>>not find this C code\n");		
		return 0xff;//����֡�쳣 ���ظ�
	}
	
	//���ݳ��� ��Ч�Լ��
	//�ж����ݱ�ʶ�Ƕ���
	#if 0
	switch(Frame->C)
	{
		case SETUP_C:			
			for(uint8_t i=0 ;i<ARRAY_SIZE(SetupDIDatalenTab);i++)
			{
				if(Frame->DI == SetupDIDatalenTab[i].DI)
				{
					temp = i;
					break;
				}
			}
			if(temp == ARRAY_SIZE(SetupDIDatalenTab))
			{
				temp = 1000;
			}
			//temp = CheckDI(SetupDIDatalenTab,Frame->DI);//��֤DI��Ч�ԣ�����DI�������߲�����
			if(temp!=1000)
			{
				if(SetupDIDatalenTab[temp].Len == Frame->DataLen)
				{
					Valid =1;
				}
				else
				{
					Valid = 0;
					printf("\n Frame format Check :ERROR>>Datalen: %d is invalid\n",Frame->DataLen);
				}
			}
			else
			{
				Valid = 0;
				printf("\n Frame format Check :ERROR>>DI: %2x is invalid\n",Frame->DI);
			}
			
			break;

		case QUERY_C:
			for(uint8_t i=0 ;i<ARRAY_SIZE(QueryDIDatalenTab);i++)
			{
				if(Frame->DI == QueryDIDatalenTab[i].DI)
				{
					temp = i;
					break;
				}
			}
			if(temp == ARRAY_SIZE(QueryDIDatalenTab))
			{
				temp = 1000;
			}
			//temp = CheckDI(QueryDIDatalenTab,Frame->DI);//��֤DI��Ч�ԣ�����DI�������߲�����
			if(temp!=1000)
			{
				if(QueryDIDatalenTab[temp].Len == Frame->DataLen)
				{
					Valid =1;
				}
				else
				{
					Valid = 0;
					printf("\n Frame format Check :ERROR>>Datalen: %d is invalid\n",Frame->DataLen);
				}
			}
			else
			{
				Valid = 0;
				printf("\n Frame format Check :ERROR>>DI: %2x is invalid\n",Frame->DI);
			}
			break;
		case TX_C:
			//temp = CheckDI(SendDIDatalenTab,Frame->DI);//��֤DI��Ч�ԣ�����DI�������߲�����
			if(temp!=1000)
			{
				if(SendDIDatalenTab[temp].Len >= Frame->DataLen)
				{
					Valid =1;
				}
				else
				{
					Valid = 0;
					printf("\n Frame format Check :ERROR>>Datalen: %d is invalid\n",Frame->DataLen);
				}
			}
			else
			{
				Valid = 0;
				printf("\n Frame format Check :ERROR>>DI: %2x is invalid\n",Frame->DI);
			}			
			break;
		default:
			Valid = 0;
			printf("\n Frame format Check :ERROR>>C: %2x is invalid\n",Frame->C);			
	}
	//Frame->Findcount = Frame->DIIndex;//�ҵ������� ���Ż���ѭ��
	if(Valid == 0)
	{				
		return 0x06;
	}
	#endif
	
	#if 1
	//��֤DI��Ч�ԣ�
	if(CheckDI(Frame) == 0)
	{
		Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
		printf("\n Frame format Check :ERROR>>DI: %2x is invalid\n",Frame->DI);
		return 0xff;//����֡�쳣 ���ظ�
	}
	else//������ݱ�ʶ���ڣ�������ݳ���
	{	
		Valid = CheckDataLen(Frame);
	}	
	if(Valid == 0)
	{		
		Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
		return EC06;//����֡�쳣 ���ظ�
	}
	#endif
	#if 0
	if(Frame->DI== 0xff)//���0xff�����⴦��
	{
		if(1 != Frame->DataLen)
		{
			printf("\n Frame format Check :ERROR>>DI x0ff data len ERROR:%d\n",Frame->DataLen);
			return EC06;
		}
	}
	else
	{
		if(Frame->C == SETUP_C)
		{
			if(FrameInfoTab[Frame->DI].SetLen != Frame->DataLen)
			{
				printf("\n Frame format Check :ERROR>>setup C:%2x,DI %2x ,data len ERROR:%2x  \n",Frame->C,Frame->DI,Frame->DataLen);
				return EC06;
			}
		}
		else  if(Frame->C == QUERY_C)
		{
			if(FrameInfoTab[Frame->DI].QueryLen != Frame->DataLen)
			{
				printf("\n Frame format Check :ERROR>>setup C:%2x,DI %2x ,data len ERROR:%2x  \n",Frame->C,Frame->DI,Frame->DataLen);
				return EC06;
			}
		}
	}
	#endif
#if 0
	//�жϵڶ���0x68
	if(Frame->StartIndex + 2>len)
	{
		return 0;
	}
	if(*(p+Frame->StartIndex+2) != 0x68)
	{
		return 0;
	}
#endif
	
	
	//�ж�У���
	Frame->CSIndex = Frame->StartIndex + Frame->DataLen +5;//CSУ������
	Frame->EndIndex = Frame->CSIndex + 1;//0x16����
	Frame->FrameLen = Frame->EndIndex -Frame->StartIndex + 1 ;//֡����0x68��ʼ	
	for(uint16_t i = Frame->StartIndex; i < Frame->CSIndex  ;i ++ )
	{
		FrameCs += *(p + i);
	}
	//Frame->Findcount = Frame->CSIndex;//�ҵ������� ���Ż���ѭ��
	if(FrameCs != *(p + Frame->CSIndex))	
	{
		Frame->Findcount ++;//�ҵ������� ���Ż���ѭ��
		printf("\n Frame format Check :ERROR>>CS ERROR\n");
		return EC03;
	}
	
	//����֡��λ
	//ռ�ô���
	MCUTXData.BusyFlag = 1;	

	//���²�������λ��
	Frame->Findcount = Frame->EndIndex;//�ҵ������� ���Ż���ѭ��
	
	printf(" Frame format Check OK\n");
	
	return EC00;
	
}
//LoRaȱʡ����д��ṹ��
//������Դ��������֡
//д�����ֱ�Ӻ�SX1301�������ʾ��Ӧ
//���õ�ʱ�򲻻���payload,Ҳ�����д�С,�ڷ��͵�ʱ�����֡�����㸺�س���
uint8_t LoRaDefaultSendParaAnalysis( uint8_t *p ,FRAME_RECEIVE_TYPE *Frame)
{
	uint8_t temp8;
	uint16_t temp16;
	//����Ƶ�ʽ���
	TXPktUserDefault.freq_hz = (*(p + Frame->DataIndex)<<16) + (*(p + Frame->DataIndex + 1) <<8) + *(p + Frame->DataIndex + 2);
	TXPktUserDefault.freq_hz *= 100;
	//TXPktUserDefault.freq_hz = (uint32_t) (TXPktUserDefault.freq_hz+0.5 );
	if((TXPktUserDefault.freq_hz > MAX_SEND_F)||(TXPktUserDefault.freq_hz < MIN_SEND_F))
	{
		return EC01;
	}
	//����ģʽ
	//����ģʽ
	temp8 = *(p + Frame->DataIndex + 3)&0xC0;
	temp8=temp8>>6;
	if(temp8 == 0)
	{
		TXPktUserFull.tx_mode = IMMEDIATE;
	}
	else if(temp8 == 1)
	{
		TXPktUserFull.tx_mode = TIMESTAMPED;
	}
	else if(temp8 == 2)
	{
		TXPktUserFull.tx_mode = ON_GPS;
	}	
	//����ģʽ����
	if((*(p + Frame->DataIndex + 3)&0x20) == 0)
	{
		TXPktUserDefault.modulation = MOD_LORA;
	}
	else
	{
		TXPktUserDefault.modulation = MOD_FSK;
	}
	//���书�ʽ���
	TXPktUserDefault.rf_power = (*(p + Frame->DataIndex + 3)&0x1f) ;
	if(TXPktUserDefault.rf_power < MIN_TX_POWER || TXPktUserDefault.rf_power > MAX_TX_POWER)
	{		
		return EC01;
	}
	#if 0
	if(TXPktUserDefault.rf_power > 20)
	{
		TXPktUserDefault.rf_power = 20;
	}	
	#endif
	//CRC16����
	if( (*(p + Frame->DataIndex + 4)&0x80))
	{
		TXPktUserDefault.no_crc = true;
	}
	else
	{
		TXPktUserDefault.no_crc = false;
	}
	//�����ʽ���
	temp8 = *(p + Frame->DataIndex + 4)&0x70;
	temp8 = temp8>>4;
	if(temp8 > 0 && temp8 <5)
	{
		TXPktUserDefault.coderate = temp8;
	}
	else
	{
		 return EC01;
	}
	//��Ƶ���ӽ���
	temp8 = *(p + Frame->DataIndex + 4)&0x0f; 
	if(temp8 > 12 || temp8 < 7)
	{
		 return EC01;
	}
	temp8 -= 6;
	TXPktUserDefault.datarate = 1<<temp8;
	//����ʱ��̶���������
	TXPktUserDefault.tx_mode = IMMEDIATE;//ȱʡ���Ͷ�����������
	//��ͷ����
	temp8 = *(p + Frame->DataIndex + 5)&0x04;	
	if(temp8 )
	{
		TXPktUserDefault.no_header = true;
	}
	else
	{
		TXPktUserDefault.no_header = false;
	}
	//�������
	if((*(p + Frame->DataIndex + 5)&0x03) == 0x02)
	{
		TXPktUserDefault.bandwidth = BW_250KHZ;
	}
	else if((*(p + Frame->DataIndex + 5)&0x0f) == 0x01)
	{
		TXPktUserDefault.bandwidth = BW_500KHZ;
	}
	else
	{
		TXPktUserDefault.bandwidth = BW_125KHZ;
	}
	TXPktUserDefault.bandwidth = BW_125KHZ;//��ΪLORA�̶���125	
	
	//ǰ�������
	temp16 = (*(p + Frame->DataIndex + 6)<<8) + *(p + Frame->DataIndex + 7);
	if(temp16 == 0 )
	{

	}
	else if(temp16 < 4 )//�������4
	{
		 return EC01;
	}
	TXPktUserDefault.preamble = temp16;
	//ͨ��Ӳ������0
	txpkt.rf_chain = 0;
	return EC00;
}
//���Ͳ���д��ṹ��TXPkt
//������Դ��������֡
uint8_t LoRaSendParaAnalysis( uint8_t *p ,FRAME_RECEIVE_TYPE *Frame)
{

	uint8_t temp8;//����Ƶ�ʽ���
	uint16_t temp16;
	uint32_t temp32;
	TXPktUserFull.freq_hz = (*(p + Frame->DataIndex)<<16) + (*(p + Frame->DataIndex + 1) <<8) + *(p + Frame->DataIndex + 2);
	TXPktUserFull.freq_hz *= 100;
	if((TXPktUserFull.freq_hz > MAX_SEND_F)||(TXPktUserFull.freq_hz < MIN_SEND_F))
	{
		return 0;
	}
	//����ʱ�����
	temp32 = (*(p + Frame->DataIndex + 3)<<24) + (*(p + Frame->DataIndex + 4) <<16) + (*(p + Frame->DataIndex + 5) <<8) + *(p + Frame->DataIndex + 6);
	if(temp32 == 0)
	{
		TXPktUserFull.tx_mode = IMMEDIATE;
	}
	else
	{
		TXPktUserFull.tx_mode = TIMESTAMPED;
	}
	//����ģʽ
	temp8 = *(p + Frame->DataIndex + 7)&0xC0;
	temp8=temp8>>6;
	if(temp8 == 0)
	{
		TXPktUserFull.tx_mode = IMMEDIATE;
	}
	else if(temp8 == 1)
	{
		TXPktUserFull.tx_mode = TIMESTAMPED;
	}
	else if(temp8 == 2)
	{
		TXPktUserFull.tx_mode = ON_GPS;
	}	
	//����ģʽ����
	if((*(p + Frame->DataIndex + 7)&0x20) == 0)
	{
		TXPktUserFull.modulation = MOD_LORA;
	}
	else
	{
		TXPktUserFull.modulation = MOD_FSK;
	}
	TXPktUserFull.modulation = MOD_LORA;
	//���书�ʽ���
	TXPktUserFull.rf_power = (*(p + Frame->DataIndex + 7)&0x1f) ;
	if(TXPktUserFull.rf_power < MIN_TX_POWER)
	{
		TXPktUserFull.rf_power = MIN_TX_POWER;
	}
	if(TXPktUserFull.rf_power > MAX_TX_POWER)
	{
		TXPktUserFull.rf_power = MAX_TX_POWER;
	}	
	//TXPktUserFull.rf_power = 10;
	//CRCʹ��
	if(((*(p + Frame->DataIndex + 9))&0x80)==0)//0 ��CRCУ��
	{
		TXPktUserFull.no_crc = false;
	}
	else
	{
		TXPktUserFull.no_crc = true;
	}
	//�������
	temp8 = *(p + Frame->DataIndex + 9)&0x70;
	temp8 = temp8>>4;
	if(temp8 > 0 && temp8 <5)
	{
		TXPktUserFull.coderate = temp8;
	}
	else
	{
		return 0;
	}
	//��Ƶ���ӽ���
	temp8 = *(p + Frame->DataIndex + 9)&0x0f; 
	if(temp8 > 12 || temp8 < 6)
	{
		return 0;
	}
	temp8 -= 6;
	TXPktUserFull.datarate = 1<<temp8;	
	//���ݰ���С����
	TXPktUserFull.size = *(p + Frame->DataIndex + 10);
	//TXPktUserFull.size = 20;
	temp8 = *(p + Frame->DataIndex + 11)&0x04;	
	if(temp8 )
	{
		TXPktUserDefault.no_header = true;
	}
	else
	{
		TXPktUserDefault.no_header = false;
	}
	//�������
	if((*(p + Frame->DataIndex + 11)&0x03) == 1)
	{
		TXPktUserFull.bandwidth = BW_250KHZ;
	}
	else if((*(p + Frame->DataIndex + 11)&0x03) == 2)
	{
		TXPktUserFull.bandwidth = BW_500KHZ;
	}
	else
	{
		TXPktUserFull.bandwidth = BW_125KHZ;
	}
	//TXPktUserFull.bandwidth = BW_125KHZ;//��ΪLORA�̶���125
	//payload
//	strcpy((char *)TXPktUserFull.payload, "TX.TEST.LORA.GW.????" );
#if 0
	for(uint8_t i=0; i<TXPktUserFull.size; i++)
	{
		TXPktUserFull.payload[i] = *(p + Frame->DataIndex + 16 + i);
	}
#endif
	
	//ǰ�������
	temp16 = (*(p + Frame->DataIndex + 12)<<8) + *(p + Frame->DataIndex + 13);
	if(temp16 == 0 )
	{

	}
	 if(temp16 < 4 )//�������4
	{
		return 0;
	}
	TXPktUserFull.preamble = temp16;
	//TXPktUserFull.preamble = 6;
	//ͨ��ѡ��
	//TXPktUserFull.rf_chain = *(p + Frame->DataIndex + 7)&0x20;
	TXPktUserFull.rf_chain = 0;//�̶�
	return 1;
}
//�û����Ͳ���������������
uint8_t UserSendParaToUserVar(uint8_t *p ,FRAME_RECEIVE_TYPE *Frame)
{
	uint8_t ErrorCode;
	switch(Frame->DI)
	{
		case 1://LoRa
			ErrorCode = LoRaSendParaAnalysis(p ,Frame);
			break;
		case 2://LoRaȱʡ����
			
			break;
		case 3://LoRaȱʡ���Ͷ������
			
			break;
		case 4://FSk����
			
			break;
		case 5://FSkȱʡ����
			
			break;
	}
	return ErrorCode;
	
}
//�û���������ת����SX1301�����У�payload����ͳһSX1301��
uint8_t UserVarToSX1301Var(uint8_t *p ,FRAME_RECEIVE_TYPE *Frame)
{
	uint8_t temp16;
	uint32_t temp32;
	uint32_t temp321;
	switch(Frame->DI)
	{
		case 1://LoRa
			txpkt.bandwidth		=	TXPktUserFull.bandwidth;
			txpkt.coderate		 =	 TXPktUserFull.coderate;
			txpkt.datarate 		= 	TXPktUserFull.datarate;
			txpkt.freq_hz 		= 	TXPktUserFull.freq_hz;
			txpkt.modulation	 	= 	TXPktUserFull.modulation;
			txpkt.no_crc 			= 	TXPktUserFull.no_crc;
			txpkt.preamble		 = 	TXPktUserFull.preamble;
			txpkt.rf_chain		 = 	TXPktUserFull.rf_chain;
			txpkt.rf_power		 = 	TXPktUserFull.rf_power;
			txpkt.tx_mode		 = 	TXPktUserFull.tx_mode;
			temp32 = *(p + Frame->DataIndex +10);//�����еĲ�����Ϣ���ݳ���
		  	if(temp32 > SendDataLenMAX)
			{
				return 0;
			}			
			if(Frame->DataLen != (temp32+16))
			{
				return 0; 
			}
			txpkt.size =	TXPktUserFull.size;
			for(uint8_t i=0;i<txpkt.size;i++)
			{
				txpkt.payload[i] = *(p + Frame->DataIndex +16 + i);
			}
			break;
		case 2://LoRaȱʡ����
			txpkt.bandwidth		=	TXPktUserDefault.bandwidth;
			txpkt.coderate		 =	 TXPktUserDefault.coderate;
			txpkt.datarate 		= 	TXPktUserDefault.datarate;
			txpkt.freq_hz 		= 	TXPktUserDefault.freq_hz;
			txpkt.modulation	 	= 	TXPktUserDefault.modulation;
			txpkt.no_crc 			= 	TXPktUserDefault.no_crc;
			txpkt.preamble		 = 	TXPktUserDefault.preamble;
			txpkt.rf_chain		 = 	TXPktUserDefault.rf_chain;
			txpkt.rf_power		 = 	TXPktUserDefault.rf_power;
			txpkt.tx_mode		 = 	TXPktUserDefault.tx_mode;
			temp32 = Frame->DataLen;
		  	if(temp32 > SendDataLenMAX )
			{
				return 0;
			}	
			txpkt.size 			=	(uint8_t)(temp32);			
			for(uint8_t i=0;i<txpkt.size;i++)
			{
				txpkt.payload[i] = *(p + Frame->DataIndex + i);
			}
			break;
		case 3://LoRaȱʡ���Ͷ��ģʽ
			txpkt.bandwidth		=	TXPktUserDefault.bandwidth;
			txpkt.coderate		 =	 TXPktUserDefault.coderate;
			txpkt.datarate 		= 	TXPktUserDefault.datarate;
			txpkt.freq_hz 		= 	TXPktUserDefault.freq_hz;
			txpkt.modulation	 	= 	TXPktUserDefault.modulation;
			txpkt.no_crc 			= 	TXPktUserDefault.no_crc;
			txpkt.preamble		 = 	TXPktUserDefault.preamble;
			txpkt.rf_chain		 = 	TXPktUserDefault.rf_chain;
			txpkt.rf_power		 = 	TXPktUserDefault.rf_power;
			txpkt.tx_mode		 = 	TXPktUserDefault.tx_mode;
			temp32 = Frame->DataLen;			
		  	if((temp32 >MutiSendDataLenMAX)||(temp32<3))//���С��2���ߴ���1024 ���ģʽ
			{
				return 0;
			}
			temp321 = 0;
			while(temp321<temp32)//�����е��������ݳ���С������֡�����ݳ���
			{
				//temp321+=2;
				temp16 = *(p + Frame->DataIndex+temp321);//��N�����ݵ����ݳ���
				if(temp16<3)
				{
					return 0;
				}
				temp321 += temp16;
			}	
			if(temp321 != temp32)
			{
				return 0;
			}
			break;
		case 4://FSK
			
			break;
		case 5://FSkȱʡ����
			
			break;
	}	
	return 1;
}
uint8_t SX1301SendParameterAnalysis(uint8_t *p,FRAME_RECEIVE_TYPE *Frame,struct ProtocolRespond_S *MCUTXData)
{	
	uint8_t ErrorCode=0;
	//��Ϊ����֡��������������������������SX1301֡��ʽ,��������
	//����֡�����������ݱ�ʶ�������ŵ���ͬ����
	if(SX1301Status.StartSuccesFlag == 0)
	{
		ErrorCode = EC05;
	}
	else
	{
		if(SX1301Status.SendStatus==0)//���1301���Ϳ���
		{
			SX1301Status.SendStatus=1;//���е�ʱ�������������æ״̬			
			//��������
			if(UserSendParaToUserVar(p ,Frame))
			{
				ErrorCode = UserVarToSX1301Var(p ,Frame);
				if(ErrorCode == 1)
				{
					if(Frame->DI == 0x03)//����Ƕ������
					{
						uint32_t NeedSendLen;
						uint32_t SendedLen  = 0;		
						uint8_t Times = 0;
						NeedSendLen = Frame->DataLen;	
						while(SendedLen<NeedSendLen)//�����е��������ݳ���С������֡�����ݳ���
						{
							uint32_t temp32;
							temp32 = *(p + Frame->DataIndex+SendedLen);//��N�����ݵ����ݳ���	
							//�������֡ÿ�������Ƿ�����	
							if(temp32 > DefaultMoutiPacktLenMax)
							{
								ErrorCode = EC06;
								break;
							}
							txpkt.size  = temp32 - 2;
							Times =  *(p + Frame->DataIndex+SendedLen + 1);//��N�����ݵ����ݷ��ʹ���
							for(uint8_t i=0;i<temp32;i++)
							{
								txpkt.payload[i] = *(p + Frame->DataIndex + 2 + SendedLen + i);
							}							
							for(uint8_t i=0;i<Times;i++)
							{
								if( SX1301TXData())//���ͳɹ�
								{
									ErrorCode = EC00;
								}	
								else
								{
									ErrorCode = EC07;
								}
							}
							SendedLen += temp32;
						}												
					}
					else
					{
						if( SX1301TXData())//���ͳɹ�
						{
							ErrorCode = EC00;
						}	
						else
						{
							ErrorCode = EC07;
						}
					}					
				}
				else
				{
					ErrorCode = EC06;
				}				
				//SX1301Status.SendStatus=0;//������ɽ������
			}
			else//�����쳣
			{
				ErrorCode = EC01;
			}		
			SX1301Status.SendStatus=0;//������ɽ������
		}
		else//����æ
		{		
			ErrorCode = EC04;
		}
	}		
	//��ظ�֡
	MCUTXData->frame[0] = 0xfe;
	MCUTXData->frame[1] = 0xfe;
	MCUTXData->frame[2] = 0x68;	
	if(ErrorCode)
	{
		MCUTXData->frame[3] = TX_ERROR_C;
	}
	else
	{
		MCUTXData->frame[3] = TX_OK_C;
	}	
	MCUTXData->frame[4] =Frame->DI;
	MCUTXData->frame[5] =0;
	MCUTXData->frame[6] =1;
	MCUTXData->frame[7] =ErrorCode;
	MCUTXData->frame[8] = 0;
	for(uint8_t i=2;i<8;i++)
	{
		MCUTXData->frame[8] += MCUTXData->frame[i];
	}
	MCUTXData->frame[9] = 0x16;
	MCUTXData->len = 10;
	return 1;
}
//����0-7ͨ������
uint8_t ChainParameterAnalysis(uint8_t *p,FRAME_RECEIVE_TYPE *Frame)
{
	uint8_t temp8;
	uint8_t channel;
	uint8_t temp81;
	int32_t temp32;
	//struct lgw_conf_rxif_s ifconf;
	//ͨ��
	channel = *(p + Frame->DataIndex);
	if(channel > 7)
	{
		return 0;
	}
	//ʹ��
	ifconfm[channel].enable = false;
	if(*(p + Frame->DataIndex + 1) == 0)
	{
		//ifconf.enable = false;
		ifconfm[channel].enable = false;
	}
	else
	{
		//ifconf.enable = true;
		ifconfm[channel].enable = true;
	}
	//chain
	temp8 = *(p + Frame->DataIndex + 2);
	if(temp8 == 0)
	{
		//ifconf.rf_chain = 0;
		ifconfm[channel].rf_chain = 0;
	}
	else if(temp8 == 1)
	{
		//ifconf.rf_chain = 1;
		ifconfm[channel].rf_chain = 1;
	}
	else
	{
		return 0;
	}		
	//ƫ��
	temp8 = 0;	
	if((*(p + Frame->DataIndex + 3))&0x80)//�������λ��λ�Ǹ���
	{
		temp8 = 1;//���ű��   ����
		*(p + Frame->DataIndex + 3)&=0x7F;//ȥ������λ
	}
	temp32 = *(p + Frame->DataIndex + 5) + (*(p + Frame->DataIndex + 4)<<8) + (*(p + Frame->DataIndex + 3)<<16);	
	temp32*=100;
	if(temp32 > MAX_B)
	{
		return 0;
	}
	if(temp8 == 1)
	{
		temp32 = - temp32;
	}
	//ifconf.freq_hz = temp32;
	
	ifconfm[channel].freq_hz = temp32;
	//datarate
	temp8 = *(p + Frame->DataIndex + 6);
	switch(temp8)
	{
		case 0x02://SF7
		temp81 = DR_LORA_SF7;
		break;
		case 0x04://SF8
		temp81 = DR_LORA_SF8;
		break;
		case 0x08://SF9
		temp81 = DR_LORA_SF9;
		break;
		case 0x10://SF10
		temp81 = DR_LORA_SF10;
		break;
		case 0x20://SF11
		temp81 = DR_LORA_SF11;
		break;
		case 0x40://SF12
		temp81 = DR_LORA_SF12;
		break;
		case 0x7e://���
		temp81 = DR_LORA_MULTI;
		break;
		default : return 0;
	}	 
	//ifconf.datarate = temp81;
	ifconfm[channel].datarate = temp81;
	//lgw_rxif_setconf(channel, ifconf); /* chain 3: LoRa 125kHz, all SF, on f0 - 0.4 MHz */
	return 1;
}
//����ֵ 0���� ���� �������
uint8_t SX1301ParameterSet(uint8_t *p,FRAME_RECEIVE_TYPE *Frame,struct ProtocolRespond_S *MCUTXData)
{
	uint8_t temp8;
	//uint16_t temp16;
	uint32_t temp32;
	uint8_t ErrorCode=0;
	ErrorCode = EC00;//��ʼ��
	switch(Frame->DI)
	{
		case 0://����汾
			ErrorCode = EC01;//
			break;
		case 0x01://��λ��Ƶ
			//AbnormalFlag = 0;
			lgw_stop();
			Sx1301Reset();
			SX1301Status.NeedStartFlag = 0;
			SX1301Status.StartSuccesFlag = 0;//��������״̬���
			//����Ҫ���ò���
			
			break;
		case 0x02://�ر���Ƶ��Դ
			//AbnormalFlag = 0;
			//������ԴIO��		
			lgw_stop();
			Sx1301Power(0);
			SX1301Status.NeedStartFlag = 0;
			SX1301Status.StartSuccesFlag = 0;//��������״̬���
			break;
		case 0x03://�ָ���������
			//��������ʼ��
			//ParaInit(rfAp,rfBp,prxifms);	//������ʼ��
			SystemParamterRecovery();
		
			Usart2Info.NowBaudRate = 1;//�û�ʹ�ò�����Ĭ����115200
			Usart2Info.LastBaudRate = 1;//�û�ʹ�ò�����Ĭ����115200
		
			lgw_stop();//ֹͣ��Ƶ
		
			Sx1301Power(0);//�ر���Ƶ��Դ
		
			SX1301Status.NeedStartFlag = 0;
			SX1301Status.StartSuccesFlag = 0;//��������״̬���
			break;
		case 0x04://radioA en
			temp8 = *(p + Frame->DataIndex);
			if(temp8 == 0)
			{
				rfconfA.enable = false;
			}
			else
			{
				rfconfA.enable = true;
			}
			#if 0
			memset(&rfconf, 0, sizeof(rfconf)); 	
			//rfconf.enable = true;
			rfconf.enable = true;
			rfconf.freq_hz = fa;
			rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
			rfconf.type = radio_type;
			rfconf.tx_enable = true;
			rfconf.tx_notch_freq = DEFAULT_NOTCH_FREQ;
			lgw_rxrf_setconf(0, rfconf); /* radio A, f0 */
			#endif
			break;
		case 0x05://radioB en
			temp8 = *(p + Frame->DataIndex);
			if(temp8 == 0)
			{
				rfconfB.enable = false;
			}
			else
			{
				rfconfB.enable = true;
			}
			break;
		case 0x06://radio a f
			temp32 = (*(p + Frame->DataIndex)<<16) + (*(p + Frame->DataIndex + 1)<<8) + *(p + Frame->DataIndex + 2);
			temp32 *=100;
			if(temp32 > MAX_F || temp32 < MIN_F)
			{
				//return 0;
				ErrorCode = EC01;//
			}
			rfconfA.freq_hz = (uint32_t)(temp32 + 0.5);
			break;
		case 0x07://radio B f
			temp32 = (*(p + Frame->DataIndex)<<16) + (*(p + Frame->DataIndex + 1)<<8) + *(p + Frame->DataIndex + 2);
			temp32 *=100;
			if(temp32 > MAX_F || temp32 < MIN_F)
			{
				//return 0;
				ErrorCode = EC01;//
			}
			rfconfB.freq_hz = (uint32_t)(temp32 + 0.5);
			break;
		case 0x08:// chain0-7 parameter set
			temp8 = ChainParameterAnalysis(p,Frame);
			if(temp8 == 0)
			{
				//return 0;
				ErrorCode = EC01;//
			}
			break;
		case 0x09://chain 8 parameter
			ErrorCode = EC01;//
			break;
		case 0x0A://chain 9 parameter
			ErrorCode = EC01;//
			break;
		case 0x0B://����SX1301
			lgw_stop();
			SX1301Status.NeedStartFlag = 1;		
			SX1301Status.StartSuccesFlag = 0;//��������״̬���
			#if 1
			if(SX1301Status.NeedStartFlag ==1)
			{
				SX1301Status.StartSuccesFlag = SX1301Start(rfAp,rfBp,prxifms);
				SX1301Status.NeedStartFlag =0;//����������ʶ
				if(SX1301Status.StartSuccesFlag == 0)
				{
					Sx1301StartFail();//IOָʾ
					ErrorCode = EC05;
				}
				else
				{
					Sx1301StartSucces();//IOָʾ
				}		
			}	
			#endif
			break;
		case 0x0C://����LORAȱʡ���Ͳ���
			//SX1301StartFlag = 1;
			ErrorCode = LoRaDefaultSendParaAnalysis( p , Frame);
			break;
		case 0x0D://FSK����
			ErrorCode = EC01;//
			break;
		case 0x0E://RX TX Debug
			SystemParameter.RFRxTxDebugEN = *(p + Frame->DataIndex);
			break;
		case 0x0F://RX TX Debug
			temp8  =  *(p + Frame->DataIndex);
			if(temp8 > 3)
			{
				ErrorCode = EC01;
			}
			else
			{
				Usart2Info.NowBaudRate = temp8;
			}
			break;
		case 0x10://SystemParameter.LoRaMAC
			temp8  =  *(p + Frame->DataIndex);
			if(temp8 ==0)
			{
				SystemParameter.LoRaMAC = 0;
			}
			else
			{
				SystemParameter.LoRaMAC = 1;
			}
			break;
		case 0x11://��������ģʽ  10S
			temp8  =  *(p + Frame->DataIndex);
		  //printf("SystemParameter.TxContinuous: %d\r\n",SystemParameter.TxContinuous );
			if(temp8 ==0)
			{
				SystemParameter.TxContinuous = 0;//�ر�
				lgw_reg_w(LGW_TX_MODE, 0); /* Tx continuous */
			}
			else
			{
				SystemParameter.TxContinuous = 1;//����
				lgw_reg_w(LGW_TX_MODE, 1); /* Tx continuous */
			}
			//printf("SystemParameter.TxContinuous: %d\r\n",SystemParameter.TxContinuous );
			break;
		case 0x12://����no crc
			temp8  =  *(p + Frame->DataIndex);
			SystemParameter.ReceiveCRC = temp8;
		#if 0
			if(temp8 & REC_CRC)
			{
				SystemParameter.ReceiveCRC |= REC_CRC;
			}
			else
			{
				SystemParameter.ReceiveCRC &= (~REC_CRC);
			}	
		
			temp8  =  *(p + Frame->DataIndex);
			if(temp8 & REC_NO_CRC)
			{
				SystemParameter.ReceiveCRC |= REC_NO_CRC;
			}
			else
			{
				SystemParameter.ReceiveCRC &= (~REC_NO_CRC);
			}	
			
			if(temp8 & REC_ERROR_CRC)
			{
				SystemParameter.ReceiveCRC |= REC_ERROR_CRC;
			}
			else
			{
				SystemParameter.ReceiveCRC &= (~REC_ERROR_CRC);
			}
			#endif
			break;
		case 0x13://����IQ����
			temp8  =  *(p + Frame->DataIndex);
			if(temp8 ==0)
			{
				SystemParameter.ReceiveIQInvert  = 0;
			}
			else
			{
				SystemParameter.ReceiveIQInvert = 1;
			}							
			break;		
		case 0x14://����IQ����
			temp8  =  *(p + Frame->DataIndex);
			if(temp8 ==0)
			{
				SystemParameter.SendIQInvert= 0;
			}
			else
			{
				SystemParameter.SendIQInvert = 1;
			}							
			break;
		case 0x15://CW����
			temp8  =  *(p + Frame->DataIndex);
			if(temp8 ==0)
			{
				SystemParameter.CWSendModeEN= 0;
				//lgw_stop();
				//SX1301Status.NeedStartFlag = 1;
				//SX1301Status.StartSuccesFlag  = 0;
				//lgw_reg_w(LGW_TX_MODE, 0); /* Tx continuous */
				lgw_reg_w(LGW_SIG_GEN_EN, 0);
			}
			else
			{
				SystemParameter.CWSendModeEN = 1;				
			}							
			break;
		case 0x16://����ǰ���볤��  ��󳤶ȴ���
			temp32  =  (*(p + Frame->DataIndex)<<8) + (*(p + Frame->DataIndex + 1));
			if(temp32 < 10)
			{
				SystemParameter.RxPreambleLen = 10;				
			}
			else if(temp32 < 65536)
			{
				SystemParameter.RxPreambleLen = temp32;				
			}
			else
			{
				SystemParameter.RxPreambleLen = 65535;
			}
			break;
		case 0xFF://�����Ż�����
			temp8  =  *(p + Frame->DataIndex);
			if(temp8&0x80)
			{
				ErrorCode = EC01;//
			}
			else
			{
				SystemParameter.LowDatarateOptmize = temp8;
			}			
			break;
		default :
			ErrorCode = EC02;//		
	}
	MCUTXData->frame[0] = 0xfe;
	MCUTXData->frame[1] = 0xfe;
	MCUTXData->frame[2] = 0x68;	
	if(ErrorCode)
	{
		MCUTXData->frame[3] =SETUP_ERROR_C;
	}
	else
	{
		MCUTXData->frame[3] =SETUP_OK_C;
	}
	
	MCUTXData->frame[4] =Frame->DI;
	MCUTXData->frame[5] =0;
	MCUTXData->frame[6] =1;
	MCUTXData->frame[7] =ErrorCode;
	MCUTXData->frame[8] = 0;
	for(uint8_t i=2;i<8;i++)
	{
		MCUTXData->frame[8] += MCUTXData->frame[i];
	}
	MCUTXData->frame[9] = 0x16;
	MCUTXData->len = 10;
	return 1;
}
//����ֵ 
//0���� 
//����  �쳣���ش������
uint8_t SX1301ParameterQuery(uint8_t *p,FRAME_RECEIVE_TYPE *Frame,struct ProtocolRespond_S *MCUTXData)
{
	uint8_t ErrorCode = 0;//�����ʶ	
	uint8_t  temp8;	
	uint16_t  DataLen;//�����򳤶�	
	uint16_t temp16;
	int32_t temp321;
	
	ErrorCode = 0;
	
	switch(Frame->DI)
	{
		case 0://����汾17020101
			DataLen = 4;
			MCUTXData->frame[7] = SOFTWARE_VERSION_YEAR;
			MCUTXData->frame[8] = SOFTWARE_VERSION_MONTH;
			MCUTXData->frame[9] = SOFTWARE_VERSION_DAY;
			MCUTXData->frame[10] = SOFTWARE_VERSION_NO;			
			break;
		case 0x01://��λ��Ƶ
			DataLen = 1;
			ErrorCode = EC01;
			break;
		case 0x02://�ر���Ƶ��Դ
			DataLen = 1;
			ErrorCode = EC01;
			break;
		case 0x03://�ָ���������
			DataLen = 1;
			ErrorCode = EC01;
			break;
		case 0x04://Radio A EN
			DataLen = 1;			
			if(rfconfA.enable == false)
			{
				MCUTXData->frame[7] = 0;
			}
			else
			{
				MCUTXData->frame[7] = 1;				
			}			
			break;
		case 0x05://Radio B EN
			DataLen = 1;			
			if(rfconfB.enable == false)
			{
				MCUTXData->frame[7] = 0;
			}
			else
			{
				MCUTXData->frame[7] = 1;				
			}
			break;
		case 0x06://radio a f 
			DataLen = 3;
			MCUTXData->frame[7] = (rfconfA.freq_hz/100)>>16;
			MCUTXData->frame[8] = ((rfconfA.freq_hz/100)>>8)%256;
			MCUTXData->frame[9] = (rfconfA.freq_hz/100)%256;			
			break;
		case 0x07://radio B f
			DataLen = 3;
			MCUTXData->frame[7] = (rfconfB.freq_hz/100)>>16;
			MCUTXData->frame[8] = ((rfconfB.freq_hz/100)>>8)%256;
			MCUTXData->frame[9] = (rfconfB.freq_hz/100)%256;	
			break;
		case 0x08://ͨ�����ղ���
			temp8 = *(p+Frame->DataIndex);
			if(temp8 > 7)
			{
				DataLen = 1;
				ErrorCode = EC01;				
			}
			else
			{
				DataLen = 7;
				MCUTXData->frame[7] = temp8;
				if(ifconfm[temp8].enable)//bool
				{
					MCUTXData->frame[8] = 1;
				}
				else
				{
					MCUTXData->frame[8] = 0;
				}				
				MCUTXData->frame[9]=ifconfm[temp8].rf_chain;				
				if(ifconfm[temp8].freq_hz <0)
				{
					temp321 = (-ifconfm[temp8].freq_hz)/100;
					MCUTXData->frame[10]=temp321>>16;
					MCUTXData->frame[10]|=0x80;
				}
				else
				{
					temp321 = ifconfm[temp8].freq_hz/100;
					MCUTXData->frame[10]=temp321>>16;
				}	
				MCUTXData->frame[11]=(temp321>>8)%256;
				MCUTXData->frame[12]=temp321%256;					
				MCUTXData->frame[13]=ifconfm[temp8].datarate;	
			}			
			break;
		case 0x09://��8ͨ������lora
			DataLen = 1;
			ErrorCode = EC01;
			break;
		case 0x0a://��9ͨ������fsk
			DataLen = 1;
			ErrorCode = EC01;
			break;
		case 0x0b:
			DataLen = 1;
			MCUTXData->frame[7] = SX1301Status.NeedStartFlag;
			break;
		case 0x0c:
			DataLen = 8;
			MCUTXData->frame[7] = (TXPktUserDefault.freq_hz/100)>>16;
			MCUTXData->frame[8] = ((TXPktUserDefault.freq_hz/100)>>8)%256;
			MCUTXData->frame[9] = (TXPktUserDefault.freq_hz/100)%256;
			MCUTXData->frame[10] =0;
			temp8 = TXPktUserDefault.tx_mode;
			temp8 = temp8<<6;
			MCUTXData->frame[10]|=temp8;
			if(TXPktUserDefault.modulation == MOD_LORA)
			{
				MCUTXData->frame[10]|=0x00;
			}
			else
			{
				MCUTXData->frame[10]|=0x20;
			}
			//���书��
			temp8 = TXPktUserDefault.rf_power;			
			MCUTXData->frame[10]|=temp8;
			//crc
			MCUTXData->frame[11]=0;
			if(TXPktUserDefault.no_crc)
			{
				temp8 = 0x80;
			}
			else
			{
				temp8 = 0x00;
			}
			MCUTXData->frame[11]|=temp8;
			//cr 6:4  76543210 CR_LORA_4_5=0x01
			temp8 = TXPktUserDefault.coderate;
			temp8 =temp8<<4;
			MCUTXData->frame[11]|=temp8;
			//SF 3:0 76543210 DR_LORA_SF7     0x02
			switch(TXPktUserDefault.datarate)
			{				
				case DR_LORA_SF7 :temp8=7;break;
				case DR_LORA_SF8 :temp8=8;break;
				case DR_LORA_SF9 :temp8=9;break;
				case DR_LORA_SF10 :temp8=10;break;
				case DR_LORA_SF11 :temp8=11;break;
				case DR_LORA_SF12 :temp8=12;break;
			}
			MCUTXData->frame[11]|=temp8;
			//����Э��
			MCUTXData->frame[12] = 0;
			//2:500kHz
			//1:250kHz
			//0:125kHz
			//#define BW_500KHZ       0x01
			//#define BW_250KHZ       0x02
			//#define BW_125KHZ       0x03
			switch(TXPktUserDefault.bandwidth)
			{
				case BW_500KHZ:temp8=2;break;
				case BW_250KHZ:temp8=1;break;
				case BW_125KHZ:temp8=0;break;				
			}
			MCUTXData->frame[12] |=temp8;
			//ǰ��������
			temp16 = TXPktUserDefault.preamble;
			MCUTXData->frame[13] =temp16/256;
			MCUTXData->frame[14] =temp16%256;	
			break;
		case 0x0D://fsk qȱʡ����
			DataLen = 1;
			ErrorCode = EC01;
			break;
		case 0x0E:
			DataLen = 1;
			MCUTXData->frame[7] = SystemParameter.RFRxTxDebugEN;
			break;
		case 0x0F:
			DataLen = 1;
			MCUTXData->frame[7] = Usart2Info.NowBaudRate;
			break;
		case 0x10:
			DataLen = 1;
			if(SystemParameter.LoRaMAC)
			{
				MCUTXData->frame[7] = 1;
			}
			else
			{
				MCUTXData->frame[7] = 0;
			}			
			break;
		case 0x11:
			DataLen = 1;
			if(SystemParameter.TxContinuous)
			{
				MCUTXData->frame[7] = 1;
			}
			else
			{
				MCUTXData->frame[7] = 0;
			}			
			break;
		case 0x12:
			DataLen = 1;
			MCUTXData->frame[7] = SystemParameter.ReceiveCRC;
			break;
		case 0x13:
			DataLen = 1;
			if(SystemParameter.ReceiveIQInvert)
			{
				MCUTXData->frame[7] = 1;
			}
			else
			{
				MCUTXData->frame[7] = 0;
			}	
			break;
		case 0x14:
			DataLen = 1;
			if(SystemParameter.SendIQInvert)
			{
				MCUTXData->frame[7] = 1;
			}
			else
			{
				MCUTXData->frame[7] = 0;
			}	
			break;
		case 0x15:
			DataLen = 1;
			if(SystemParameter.CWSendModeEN)
			{
				MCUTXData->frame[7] = 1;
			}
			else
			{
				MCUTXData->frame[7] = 0;
			}	
			break;
		case 0x16:
			DataLen = 2;
		  MCUTXData->frame[7] = SystemParameter.RxPreambleLen>>8;			
			MCUTXData->frame[8] = SystemParameter.RxPreambleLen%256;
			break;
		case 0xFF:
			DataLen = 1;	
			#if 0
			if(SX1301Status.StartSuccesFlag)
			{
				 lgw_reg_r(LGW_PPM_OFFSET, &temp321);  /* as the threshold is 16ms, use 0x60 to enable ppm_offset for SF12 and SF11 @125kHz*/
				temp8 = (uint8_t)(temp321);
			}
			else
			{
				temp8 = SystemParameter.LowDatarateOptmize;	
			}	
			#endif
			temp8 = SystemParameter.LowDatarateOptmize;
			MCUTXData->frame[7] = temp8;
			break;
		default :ErrorCode = EC01;
	}
	//�ظ�֡
	MCUTXData->frame[0] = 0xfe;
	MCUTXData->frame[1] = 0xfe;
	MCUTXData->frame[2] = 0x68;	
	if(ErrorCode)
	{
		MCUTXData->frame[3] =QUERY_ERROR_C;
	}
	else
	{
		MCUTXData->frame[3] =QUERY_OK_C;
	}	
	MCUTXData->frame[4] =Frame->DI;
	MCUTXData->frame[5] = DataLen/256;
	MCUTXData->frame[6] = DataLen%256;
	//����У���
	MCUTXData->frame[DataLen+7] = 0;
	for(uint16_t i = 2;i<DataLen+7;i++)
	{
		MCUTXData->frame[DataLen+7]+=MCUTXData->frame[i];
	}
	MCUTXData->frame[DataLen + 8]=0x16;
	MCUTXData->len = DataLen + 9;
	
	return 1;
}

//����
uint8_t Deny(FRAME_RECEIVE_TYPE *Frame,uint8_t ErrorCode,struct ProtocolRespond_S *MCUTXData)
{
	MCUTXData->frame[0] = 0xfe;
	MCUTXData->frame[1] = 0xfe;
	MCUTXData->frame[2] = 0x68;	
	MCUTXData->frame[3] =Frame->C+0x90;	
	MCUTXData->frame[4] =Frame->DI;
	MCUTXData->frame[5] =0;
	MCUTXData->frame[6] =1;
	MCUTXData->frame[7] =ErrorCode;
	MCUTXData->frame[8] = 0;
	for(uint8_t i=2;i<8;i++)
	{
		MCUTXData->frame[8] += MCUTXData->frame[i];
	}
	MCUTXData->frame[9] = 0x16;
	MCUTXData->len = 10;
	return 1;
}

uint8_t MCUSendData(struct ProtocolRespond_S *MCUTXData)
{

	//Usart3SendData_DMA(MCUTXData->frame, MCUTXData->len);
	if(MCUTXData->BusyFlag ==0x10)
	{
		
	}
	return HAL_UART_Transmit(&huart2,MCUTXData->frame, MCUTXData->len,500);
	//return 1;
}
//p���յ�������
//���յ������ݳ���
//Frame����֡����

uint8_t  ProtocolAnalysis(uint8_t *p,uint16_t len,FRAME_RECEIVE_TYPE *Frame,struct ProtocolRespond_S *MCUTXData)
{
	//֡���
	uint8_t ErrorCode=0;
	Sx1301Busy();//IOָʾæ״̬	
	//UserDataBusyLedOn();//�û�������ʾ
	MCUTXData->BusyFlag = 1;//���㴮��ռ�ñ�ʶ
	Frame->Findcount = 0;//����λ��
	
	//��ӡ��������
	#ifdef DebugDisplayFromUserData
	printf("From user data:\n");
	for(uint16_t k = 0;k<len;k++)
	{
		printf(" %02X", p[k]);
	}
	printf(" #\n");
	#endif
	
	//ѭ������	
	for(;Frame->Findcount<len-1;)
	{
		ErrorCode = FrameCheck(p,len,Frame);
		if(ErrorCode == 0xfe)//
		{
			break;
		}
		else if(ErrorCode == 0xff)//
		{
			continue;
		}
		else if(ErrorCode)
		{
			Deny(Frame,ErrorCode,MCUTXData);
		}
		else
		{			
			switch(Frame->C)
			{
				case SETUP_C:
						//if()
						
						SX1301ParameterSet(p,Frame,MCUTXData);
						break;
				case QUERY_C:
						
						SX1301ParameterQuery(p,Frame,MCUTXData);
						break;
				case TX_C:
						//Sx1301Busy();//IOָʾ
						//RFTXLedOn();//busy ָʾ����ʾ
						SX1301SendParameterAnalysis(p,Frame,MCUTXData);
						//Sx1301Free();//IOָʾ
						break;
				case RX_C:
						ErrorCode = EC01;
						Deny(Frame,ErrorCode,MCUTXData);
						break;
			}
		}	
		//�ظ�����
		//if(ErrorCode != 0xff)//0xff���쳣���ݲ���Ҫ�ظ����쳣����
		{
			MCUSendData(MCUTXData);
			
			//��ӡ������û���ӡ
			#ifdef DebugDisplayToUserProtocolData
			printf("TO user data:\n");
			for(uint16_t k = 0;k<MCUTXData->len;k++)
			{
				printf(" %02X", MCUTXData->frame[k]);
			}
			printf(" #\n");
			#endif
		}		
	}
	
	Sx1301Free();//IOָʾæ״̬
	MCUTXData->BusyFlag = 0;//���㴮��ռ�ñ�ʶ
	//UserDataBusyLedOff();//busy ָʾ����ʾ
	//RFTXLedOff();//�û�������ʾ
	return 1;
}





