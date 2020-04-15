#ifndef __protocol_analysis_H
#define __protocol_analysis_H
#include "stm32l0xx_hal.h"
#include <stdbool.h>    /* bool type */
#include "loragw_hal.h"
#include "usart.h"


#define FRAME_MIN_LEN			7//����0xfe,��������
//#define FRAME_MAX_LEN		(16+256+10)//2��0xfe
//#define FRAME_MAX_LEN		USART1_RX_BUFF_LEN-128//���ڽ��ջ����С���� ���ǵ�������֡���� ��������֡������framecheck��
#define FRAME_MAX_LEN		RECEIVELEN//���ڽ��ջ����С���� ���ǵ�������֡���� ��������֡������framecheck��


//������
#define SETUP_C			0x01//���ò���
#define QUERY_C		0x02//��ѯ����
#define TX_C			0x03//��������
#define RX_C		0x04//��������

#define SETUP_OK_C			0x81//���óɹ�
#define QUERY_OK_C		0x82//��ѯ����
#define TX_OK_C				0x83//��������
#define RX_OK_C				0x84//��������

#define SETUP_ERROR_C		0x91//�����쳣
#define QUERY_ERROR_C	0x92//��ѯ�쳣
#define TX_ERROR_C			0x93//�����쳣
#define RX_ERROR_C		0x94//�����쳣

//�������
#define EC00 0x00//����
#define EC01 0x01//��������
#define EC02 0x02//CRC�쳣
#define EC03 0x03//CS�쳣
#define EC04 0x04//ģ�鷢��æ
#define EC05 0x05//ϵͳ����ʧ��
#define EC06 0x06//����֡�쳣
#define EC07 0x07//����֡����ʧ��


//����������ݳ���
#define DefaultMoutiPacktLenMax 257
#define SendDataLenMAX 255
#define MutiSendDataLenMAX 2039

//���ݳ��ȴ���С����ָ��ֵ
enum Ecompare{Equal,Less,Greater};

//�����򳤶�
typedef struct  
{  
	uint8_t DI;//���ݱ�ʶ
	uint16_t Len;//���ݳ���
	enum Ecompare Comepare;
}FRAME_INFO_TYPE;

//���ò�ѯ���ݱ�ʶ
#define software_DI			0x00//

//���书��
#define MIN_TX_POWER 0
#define MAX_TX_POWER 30


typedef struct  
{  
	uint8_t DI;//���ݱ�ʶ
	uint8_t C;//���ݱ�ʶ
	uint8_t CIndex;//���������ݱ�ʶ
	uint16_t StartIndex;	//֡��ʼ��ַ
	uint16_t DIIndex;	//���ݱ�ʶ����
	uint16_t DataIndex; //����������
	uint16_t DataLen;//�����򳤶�
	uint16_t CSIndex;				//���ճ���
	uint16_t EndIndex;				//16����
	uint16_t FrameLen;				//���ճ���
	uint16_t Findcount;//���Ҽ���
}FRAME_RECEIVE_TYPE;

struct user_pkt_tx_s {
    uint32_t    freq_hz;        /*!> center frequency of TX */
    uint8_t     tx_mode;        /*!> select on what event/time the TX is triggered */
    uint32_t    count_us;       /*!> timestamp or delay in microseconds for TX trigger */
    uint8_t     rf_chain;       /*!> through which RF chain will the packet be sent */
    int8_t      rf_power;       /*!> TX power, in dBm */
    uint8_t     modulation;     /*!> modulation to use for the packet */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint32_t    datarate;       /*!> TX datarate (baudrate for FSK, SF for LoRa) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
    bool        invert_pol;     /*!> invert signal polarity, for orthogonal downlinks (LoRa only) */
    uint8_t     f_dev;          /*!> frequency deviation, in kHz (FSK only) */
    uint16_t    preamble;       /*!> set the preamble length, 0 for default */
    bool        no_crc;         /*!> if true, do not send a CRC in the packet */
    bool        no_header;      /*!> if true, enable implicit header mode (LoRa), fixed length (FSK) */
    uint16_t    size;           /*!> payload size in bytes */
    uint8_t payload_index;//����֡�е���Ч�غ�����
    uint8_t payload_size;//��Ч�غɴ�С
};

 struct   SX1301Status_S
{
	uint8_t NeedStartFlag;//������ʶ
	uint8_t StartSuccesFlag;//�����ɹ���ʶ 1�ɹ���0ʧ��
	uint8_t SendStatus;//����0����1æ
	uint8_t Starting;//��������
	uint8_t err;		//���������־
};

#define NOStart	0
#define StartSuccess 0x01
#define StartFail 0x02


struct ProtocolRespond_S//Э��Ӧ��  ���Ż�ʹ��fw��ָ��ָ�����λ��
{
	uint8_t	C;
	uint8_t	BusyFlag;//æ��ʶ
	uint8_t	DI;
	uint8_t	EC;
	uint8_t	*frame;
	uint16_t	len;//����֡�ܳ���	
	uint16_t RemainSpace;//ʣ��ռ�
};
#define PROTOCOL_BUSY 0x10
#define RF_BUSY 0x01

//extern FRAME_RECEIVE_TYPE Frame;


//#define RX_FRAME_LEN_MAX 
//uint8_t ParaInit(struct lgw_conf_rxrf_s rfA,struct lgw_conf_rxrf_s rfB,struct lgw_conf_rxif_s * pifms);
//uint8_t ParaInit(void);
uint8_t ParaInit(struct lgw_conf_rxrf_s *rfAp,struct lgw_conf_rxrf_s *rfBp,struct lgw_conf_rxif_s *pifms);
uint8_t  ProtocolAnalysis(uint8_t *p,uint16_t len,FRAME_RECEIVE_TYPE *Frame,struct ProtocolRespond_S *MCUTXData);
void SystemParamterRecovery(void);
#endif

