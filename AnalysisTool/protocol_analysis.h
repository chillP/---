#ifndef __protocol_analysis_H
#define __protocol_analysis_H
#include "stm32l0xx_hal.h"
#include <stdbool.h>    /* bool type */
#include "loragw_hal.h"
#include "usart.h"


#define FRAME_MIN_LEN			7//不计0xfe,无数据域
//#define FRAME_MAX_LEN		(16+256+10)//2个0xfe
//#define FRAME_MAX_LEN		USART1_RX_BUFF_LEN-128//串口接收缓存大小限制 不是单个数据帧限制 单个数据帧限制在framecheck中
#define FRAME_MAX_LEN		RECEIVELEN//串口接收缓存大小限制 不是单个数据帧限制 单个数据帧限制在framecheck中


//控制码
#define SETUP_C			0x01//设置参数
#define QUERY_C		0x02//查询参数
#define TX_C			0x03//发送数据
#define RX_C		0x04//接收数据

#define SETUP_OK_C			0x81//设置成功
#define QUERY_OK_C		0x82//查询正常
#define TX_OK_C				0x83//发送正常
#define RX_OK_C				0x84//接收正常

#define SETUP_ERROR_C		0x91//设置异常
#define QUERY_ERROR_C	0x92//查询异常
#define TX_ERROR_C			0x93//发送异常
#define RX_ERROR_C		0x94//接收异常

//错误代码
#define EC00 0x00//正常
#define EC01 0x01//参数错误
#define EC02 0x02//CRC异常
#define EC03 0x03//CS异常
#define EC04 0x04//模块发送忙
#define EC05 0x05//系统启动失败
#define EC06 0x06//数据帧异常
#define EC07 0x07//数据帧发送失败


//多包发送数据长度
#define DefaultMoutiPacktLenMax 257
#define SendDataLenMAX 255
#define MutiSendDataLenMAX 2039

//数据长度带大小等于指定值
enum Ecompare{Equal,Less,Greater};

//数据域长度
typedef struct  
{  
	uint8_t DI;//数据标识
	uint16_t Len;//数据长度
	enum Ecompare Comepare;
}FRAME_INFO_TYPE;

//设置查询数据标识
#define software_DI			0x00//

//发射功率
#define MIN_TX_POWER 0
#define MAX_TX_POWER 30


typedef struct  
{  
	uint8_t DI;//数据标识
	uint8_t C;//数据标识
	uint8_t CIndex;//控制码数据标识
	uint16_t StartIndex;	//帧起始地址
	uint16_t DIIndex;	//数据标识索引
	uint16_t DataIndex; //数据域索引
	uint16_t DataLen;//数据域长度
	uint16_t CSIndex;				//接收长度
	uint16_t EndIndex;				//16索引
	uint16_t FrameLen;				//接收长度
	uint16_t Findcount;//查找计数
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
    uint8_t payload_index;//数据帧中的有效载荷索引
    uint8_t payload_size;//有效载荷大小
};

 struct   SX1301Status_S
{
	uint8_t NeedStartFlag;//启动标识
	uint8_t StartSuccesFlag;//启动成功标识 1成功，0失败
	uint8_t SendStatus;//发送0空闲1忙
	uint8_t Starting;//正在启动
	uint8_t err;		//启动错误标志
};

#define NOStart	0
#define StartSuccess 0x01
#define StartFail 0x02


struct ProtocolRespond_S//协议应答  待优化使用fw用指针指向分配位置
{
	uint8_t	C;
	uint8_t	BusyFlag;//忙标识
	uint8_t	DI;
	uint8_t	EC;
	uint8_t	*frame;
	uint16_t	len;//数据帧总长度	
	uint16_t RemainSpace;//剩余空间
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

