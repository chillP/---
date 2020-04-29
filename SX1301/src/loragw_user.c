#include <stdint.h>        /* C99 types */
#include <stdbool.h>       /* bool type */
#include <stdio.h>         /* printf */
#include <string.h>        /* memset */
//#include <signal.h>        /* sigaction */
//#include <unistd.h>        /* getopt access */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

#include "loragw_user.h"
#include "sx1301_rx_tx.h"
#include "stm32l0xx_hal.h"

#include "loragw_spi.h"
#include "usart.h"
#include "protocol_analysis.h"

#include "stdlib.h"
#include "led.h"
#include "stm32l0xx_hal.h"
#include "rtc.h"
#include "rf_rxtx_debug.h"
#include "test_tool.h"
#include "gpio.h"

#include "p2p.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_RSSI_OFFSET -176
#define DEFAULT_NOTCH_FREQ  129000U

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */


extern	struct lgw_conf_board_s boardconf;
extern	struct lgw_conf_rxrf_s rfconf;
extern	struct lgw_conf_rxif_s ifconf;

extern	struct lgw_pkt_rx_s rxpkt[4]; /* array containing up to 4 inbound packets metadata */
extern	struct lgw_pkt_tx_s txpkt; /* configuration and metadata for an outbound packet */
extern	struct lgw_pkt_rx_s *p; /* pointer on a RX packet */

extern	int i, j;
extern	int nb_pkt;
extern	uint32_t fa, fb, ft;
extern	enum lgw_radio_type_e radio_type;
//enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_SX1255;
extern	uint8_t clocksource; /* Radio B is source by default */

extern	uint32_t tx_cnt;
//unsigned long loop_cnt = 0;

//uint8_t status_var = 0;
extern	uint8_t status_var;
extern	uint32_t loop_cnt;
extern	double xd;
extern	int xi;

#define MCU_AGC_FW_BYTE     8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
extern uint8_t fw_check[MCU_AGC_FW_BYTE];
//#define	OUTPUTINDEX 4096//输出起始索引
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern	struct lgw_conf_rxif_s ifconfm[8];
extern	struct lgw_conf_rxrf_s rfconfA,rfconfB;

extern	struct ProtocolRespond_S MCUTXData;//发送

//00DistanceTest使用
extern	struct lgw_conf_rxrf_s *rfAp;
extern	struct lgw_conf_rxrf_s *rfBp;
extern	struct lgw_conf_rxif_s *prxifms;
extern	USART_INFO Usart2Info;//串口2属性
//00 end

//test tool
extern enum  TestMod TestToolMod;
extern uint8_t TestToolFinish;

struct SX1301Status_S SX1301Status;

uint32_t LastRxdataTimestamp =0 ;//上一包数据的时间戳
uint8_t RFRXData[RFRXData_LEN];//射频收到的数据

LoRaWAN_TYPE LoRaWANData;
RFSystemParameter_TYPE SystemParameter;

#define DISTAB_LEN 10
const DIS_INFO_TYPE Distab[DISTAB_LEN]=
{
	{
		"CHANNEL",
		10
	},
	{
		"freq_hz",
		10
	},	
	{
		"SF",
		10
	},	
	{
		"size",
		10
	},
	{
		"CR",
		10
	},
	{
		"CRC",
		10
	},
	{
		"SNR",
		10
	},
	{
		"SNR min",
		10
	},
	{
		"SNR max",
		10
	},
	{
		"RSSI",
		10
	},
};
#define LoRaWANUpLinkJoinLEN 5
const DIS_INFO_TYPE LoRaWANJoinUpLinkDistab[LoRaWANUpLinkJoinLEN]=
{
	{
		"MHDR",
		6
	},
	{
		"APPEUI",
		18
	},	
	{
		"DEVEUI",
		18
	},	
	{
		"DevNonce",
		10
	},
	{
		"MIC",
		10
	},
};
#define LoRaWANDownLinkJoinLEN 7
const DIS_INFO_TYPE LoRaWANJoinDownLinkDistab[LoRaWANDownLinkJoinLEN]=
{
	{
		"MHDR",
		6
	},
	{
		"AppNonce",
		10
	},	
	{
		"NetID",
		8
	},	
	{
		"DevAddr",
		10
	},
	{
		"DLSettings  ",
		12
	},
	{
		"RxDelay   ",
		10
	},
	{
		"CFList  ",
		32
	},
};

#define LoRaWANUpLinkDataLEN 8
const DIS_INFO_TYPE LoRaWANUpLinkDataDistab[LoRaWANUpLinkDataLEN]=
{
	{
		"MHDR",
		6
	},
	{
		"DEVADDR",
		10
	},	
	{
		"FCTL",
		6
	},	
	{
		"Fcnt",
		6
	},
	{
		"Fopts",
		32
	},
	{
		"Fport",
		7
	},	
	{
		"FRMpayload",
		16
	},
	{
		"MIC",
		10
	},
};
//灵敏度测试使用变量
const uint8_t TestData[]={
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0};
#define PER_Data_LEN 10//PER字节长度
#define SF7TestDataNum 100
#define SF10TestDataNum 100
#define SF12TestDataNum 100


#define SF7Time 50000//实际测试正常时候是56ms，异常时小于这个值 32字节是72ms
#define SF10Time 300000//实际测试正常时候是360ms，异常时小于这个值
#define SF12Time 900000//实际测试正常时候是56ms，异常时小于这个值

uint32_t timerem;//时间记录
uint32_t timecount = 0;//测试接收数据包的时间 距离上一包数据的时间差总和
uint32_t RightCount = 0;//收到正确数据包计数
uint32_t timeavg;//平均包间间隔
uint8_t ReceiveRightFlag = 0;//接收正确标志
uint32_t TimeAvgeTemp;//时间间隔缓存
uint8_t DatarateMem;
float PerValue = 0.0;//成功率
uint8_t RXRadioAB;//0:A 1:B
uint8_t TestToolSFRem;//按下按键记录当前的SF值
uint32_t TestToolTimesTH;


extern void usage(void);
//****************************************************
//功能开启和关闭射频电源
//参数OnOrOff 
//0 关闭射频电源
//1 开启射频电源
void Sx1301Power(uint8_t OnOrOff)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	
	if(OnOrOff == 0)
	{
		//Sx1301PowerOff();		
		GPIO_InitStruct.Pin = SX1301_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(SX1301_PORT, &GPIO_InitStruct);
		//HAL_GPIO_WritePin(SX1301_PORT, SX1301_PIN, GPIO_PIN_RESET);
	}
	else
	{
		//Sx1301PowerOn();
		GPIO_InitStruct.Pin = SX1301_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(SX1301_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(SX1301_PORT, SX1301_PIN, GPIO_PIN_RESET);
	}
}
//把用户配置的参数放到变量中
//需要注意参数检查
//p是指向结构体数组的指针
uint8_t UserParaToVar(struct lgw_conf_rxrf_s *rfconfA,struct lgw_conf_rxrf_s *rfconfB,struct lgw_conf_rxif_s *p)
{
	//参数配置
	fa = (uint32_t)((rfconfA->freq_hz*100) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	fb = (uint32_t)((rfconfB->freq_hz*100) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	//发射频率暂时不需要
	ft = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	//radio_type = LGW_RADIO_TYPE_SX1255;
	radio_type = LGW_RADIO_TYPE;
	xi = 1;/* <int> Concentrator clock source (Radio A or Radio B) */
	clocksource = (uint8_t)xi;//clocksource = 1; /* Radio B is source by default */


	/* check input parameters */
	if ((fa == 0) || (fb == 0) || (ft == 0)) 
	{
		printf("ERROR: missing frequency input parameter:\n");
		printf("  Radio A RX: %u\n", fa);
		printf("  Radio B RX: %u\n", fb);
		printf("  Radio TX: %u\n", ft);
		usage();
		return 0;
	}

	if (radio_type == LGW_RADIO_TYPE_NONE) 
	{
		printf("ERROR: missing radio type parameter:\n");
		usage();
		return 0;
	}

	/* beginning of LoRa concentrator-specific code */
	printf("Beginning of test for loragw_hal.c\n");

	printf("*** Library version information ***\n%s\n\n", lgw_version_info());

	/* set configuration for board */
	memset(&boardconf, 0, sizeof(boardconf));
	boardconf.lorawan_public = false;
	boardconf.clksrc = clocksource;
	lgw_board_setconf(boardconf);
	printf("lorawan_public:%2d\n",boardconf.lorawan_public);

	/* set configuration for RF chains */
	memset(&rfconf, 0, sizeof(rfconf)); 	

	if(rfconfA->enable)
	{
		rfconf.enable = true;
	}
	else
	{
		rfconf.enable = false;
	}

	rfconf.freq_hz = fa;
	rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
	rfconf.type = radio_type;
	rfconf.tx_enable = true;
	rfconf.tx_notch_freq = DEFAULT_NOTCH_FREQ;

	lgw_rxrf_setconf(0, rfconf); /* radio A, f0 */
	
	if(rfconfB->enable)
	{
		rfconf.enable = true;
	}
	else
	{
		rfconf.enable = false;
	}
	rfconf.freq_hz = fb;
	rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
	rfconf.type = radio_type;
	rfconf.tx_enable = false;
	lgw_rxrf_setconf(1, rfconf); /* radio B, f1 */

	/* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
	for(uint8_t i=0;i<8;i++,p++)
	{
		memset(&ifconf, 0, sizeof(ifconf));
		if(p->enable)
		{
			ifconf.enable = true ;
		}
		else
		{
			ifconf.enable = false ;
		}	
		ifconf.rf_chain = p->rf_chain;	
		ifconf.freq_hz = p->freq_hz;
		ifconf.datarate = p->datarate;
		lgw_rxif_setconf(i, ifconf); /* chain 0: LoRa 125kHz, all SF, on f1 - 0.4 MHz */	
	}

	/* set configuration for LoRa 'stand alone' channel */
	memset(&ifconf, 0, sizeof(ifconf));

	//ifconf.enable = true;
	ifconf.enable = false;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = BW_250KHZ;
	ifconf.datarate = DR_LORA_SF10;
	lgw_rxif_setconf(8, ifconf); /* chain 8: LoRa 250kHz, SF10, on f0 MHz */

	/* set configuration for FSK channel */
	memset(&ifconf, 0, sizeof(ifconf));

	//ifconf.enable = true;
	ifconf.enable = false;
	ifconf.rf_chain = 1;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = BW_250KHZ;
	ifconf.datarate = 64000;
	lgw_rxif_setconf(9, ifconf); /* chain 9: FSK 64kbps, on f1 MHz */

	/* set configuration for TX packet */
	memset(&txpkt, 0, sizeof(txpkt));

	txpkt.freq_hz = ft;
	txpkt.tx_mode = IMMEDIATE;
	txpkt.rf_power = 10;
	txpkt.modulation = MOD_LORA;
	txpkt.bandwidth = BW_125KHZ;
	txpkt.datarate = DR_LORA_SF9;
	txpkt.coderate = CR_LORA_4_5;
	strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
	txpkt.size = 20;
	txpkt.preamble = 6;
	txpkt.rf_chain = 0;
	/*
	memset(&txpkt, 0, sizeof(txpkt));
	txpkt.freq_hz = F_TX;
	txpkt.tx_mode = IMMEDIATE;
	txpkt.rf_power = 10;
	txpkt.modulation = MOD_FSK;
	txpkt.f_dev = 50;
	txpkt.datarate = 64000;
	strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
	txpkt.size = 20;
	txpkt.preamble = 4;
	txpkt.rf_chain = 0;
	*/	
	return 1;
}
uint8_t SX1301Start(struct lgw_conf_rxrf_s *rfconfA,struct lgw_conf_rxrf_s *rfconfB,struct lgw_conf_rxif_s *pifms)
{	
	//先关闭串口的接防止串口数据影响启动缓存
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);//先关闭接收，处理完再打开		
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);//先关闭接收，处理完再打开		
	
	Sx1301Power(1);
	//电容充电时间
	//HAL_Delay(1000);
	//
	 //Sx1301Reset();
	//参数配置
	fa = (uint32_t)(rfconfA->freq_hz + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	fb = (uint32_t)(rfconfB->freq_hz + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	
//	fa = (uint32_t)((475.3*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
//	fb = (uint32_t)((485.3*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	//发射频率暂时不需要
	xd = 868.3;
	ft = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	//radio_type = LGW_RADIO_TYPE_SX1255;
	radio_type = LGW_RADIO_TYPE;
#ifdef SX1301CLK0
	xi = 0;/* <int> Concentrator clock source (Radio A or Radio B) */
#else
	xi = 1;/* <int> Concentrator clock source (Radio A or Radio B) */
#endif
	clocksource = (uint8_t)xi;//clocksource = 1; /* Radio B is source by default */


	/* check input parameters */
	if ((fa == 0) || (fb == 0) || (ft == 0)) 
	{
		printf("ERROR: missing frequency input parameter:\n");
		printf("  Radio A RX: %u\n", fa);
		printf("  Radio B RX: %u\n", fb);
		printf("  Radio TX: %u\n", ft);
		usage();
		return 0;
	}

	if (radio_type == LGW_RADIO_TYPE_NONE) 
	{
		printf("ERROR: missing radio type parameter:\n");
		usage();
		return 0;
	}

	/* beginning of LoRa concentrator-specific code */
	printf("Beginning of test for loragw_hal.c\n");

	printf("*** Library version information ***\n%s\n\n", lgw_version_info());

	/* set configuration for board */
	memset(&boardconf, 0, sizeof(boardconf));

	if(SystemParameter.LoRaMAC)
	{
		boardconf.lorawan_public = true;
	}
	else
	{
		boardconf.lorawan_public = false;
	}
	
	boardconf.clksrc = clocksource;
	lgw_board_setconf(boardconf);
	printf("lorawan_public:%2d\n",boardconf.lorawan_public);

	/* set configuration for RF chains */
	memset(&rfconf, 0, sizeof(rfconf));     

	if(rfconfA->enable)
	{
		rfconf.enable = true;
	}
	else
	{
		rfconf.enable = false;
	}

	rfconf.freq_hz = fa;
	rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
	rfconf.type = radio_type;
	rfconf.tx_enable = true;
	rfconf.tx_notch_freq = DEFAULT_NOTCH_FREQ;

	lgw_rxrf_setconf(0, rfconf); /* radio A, f0 */
	if(rfconfB->enable)
	{
		rfconf.enable = true;
	}
	else
	{
		rfconf.enable = false;
	}
	rfconf.freq_hz = fb;
	rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
	rfconf.type = radio_type;
	rfconf.tx_enable = false;
	lgw_rxrf_setconf(1, rfconf); /* radio B, f1 */
	
	/* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
	for(uint8_t i=0;i<8;i++,pifms++)
	{
		memset(&ifconf, 0, sizeof(ifconf));
		if(pifms->enable)
		{
			ifconf.enable = true ;
		}
		else
		{
			ifconf.enable = false ;
		}	
		ifconf.rf_chain = pifms->rf_chain;	
		ifconf.freq_hz = pifms->freq_hz;
		ifconf.datarate = pifms->datarate;		
		lgw_rxif_setconf(i, ifconf); /* chain 0: LoRa 125kHz, all SF, on f1 - 0.4 MHz */	
	}

	/* set configuration for LoRa 'stand alone' channel */
	memset(&ifconf, 0, sizeof(ifconf));

	//ifconf.enable = true;
	ifconf.enable = false;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = BW_250KHZ;
	ifconf.datarate = DR_LORA_SF10;
	lgw_rxif_setconf(8, ifconf); /* chain 8: LoRa 250kHz, SF10, on f0 MHz */

	/* set configuration for FSK channel */
	memset(&ifconf, 0, sizeof(ifconf));

	//ifconf.enable = true;
	ifconf.enable = false;
	ifconf.rf_chain = 1;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = BW_250KHZ;
	ifconf.datarate = 64000;
	lgw_rxif_setconf(9, ifconf); /* chain 9: FSK 64kbps, on f1 MHz */

	/* set configuration for TX packet */
	memset(&txpkt, 0, sizeof(txpkt));

	txpkt.freq_hz = ft;
	txpkt.tx_mode = IMMEDIATE;
	txpkt.rf_power = 24;
	txpkt.modulation = MOD_LORA;
	txpkt.bandwidth = BW_125KHZ;
	txpkt.datarate = DR_LORA_SF9;
	txpkt.coderate = CR_LORA_4_5;
	strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
	txpkt.size = 20;
	txpkt.preamble = 6;
	txpkt.rf_chain = 0;
	/*
	memset(&txpkt, 0, sizeof(txpkt));
	txpkt.freq_hz = F_TX;
	txpkt.tx_mode = IMMEDIATE;
	txpkt.rf_power = 10;
	txpkt.modulation = MOD_FSK;
	txpkt.f_dev = 50;
	txpkt.datarate = 64000;
	strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
	txpkt.size = 20;
	txpkt.preamble = 4;
	txpkt.rf_chain = 0;
	*/
	
	/* connect, configure and start the LoRa concentrator */
	//i = lgw_start();
	//
	#ifdef DebugDisplayParameter
	printf("$$$ system parameter $$$\n");
	if(SystemParameter.LoRaMAC == 1)
	{
		printf("$ LoRaMAC: ON\n");
	}
	else
	{
		printf("$ LoRaMAC: OFF\n");
	}
	
	if(SystemParameter.ReceiveCRC == 1)
	{
		printf("$ ReceiveCRC: only receive crc data\n");
	}
	else if(SystemParameter.ReceiveCRC == 2)
	{
		printf("$ ReceiveCRC: only receive no crc data\n");
	}
	else if(SystemParameter.ReceiveCRC == 3)
	{
		printf("$ ReceiveCRC: receive crc & no crc data\n");
	}
	else if(SystemParameter.ReceiveCRC == 4)
	{
		printf("$ ReceiveCRC: receive crc & no crc & crc error data\n");
	}
	
	if(SystemParameter.ReceiveIQInvert == 1)
	{
		printf("$ ReceiveIQInvert: ON\n");
	}
	else
	{
		printf("$ ReceiveIQInvert: OFF\n");
	}
	
	
	#endif
	for(SystemParameter.CalibrationFailTime = 0;SystemParameter.CalibrationFailTime < CalibrationFailTime_H;SystemParameter.CalibrationFailTime++)
	{
		if (lgw_start() == LGW_HAL_SUCCESS) 
		{
			printf("*** Concentrator started ***\n");
			SystemParameter.CalibrationFailFlag = 0;
			break;
		}
		else
		{
			printf("*** Impossible to start concentrator ***\n");			
			SystemParameter.CalibrationFailFlag = 1;
		}		
	}
	if((SystemParameter.CalibrationFailFlag == 1)&&(CalibrationFailTime_H == SystemParameter.CalibrationFailTime))
	{
		//启动失败关闭射频电源
		Sx1301Power( 0);
		SX1301Status.StartSuccesFlag = 0;
		return 0;
	}
	
	//获取SX1301时间
	lgw_reg_w(LGW_GPS_EN, 0);
	lgw_get_trigcnt(&SystemParameter.Timestamp);
	lgw_reg_w(LGW_GPS_EN, 1);	
	printf("LGW_TIMESTAMP:%010u\n",SystemParameter.Timestamp);
	
	//打开串口接收中断
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); 
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	
	return 1;	
}

uint8_t SX1301RXData()
{
	uint8_t temp;
	uint16_t temp16;
	uint16_t datacount = 0;//
	uint32_t lasttime;//上一包数据的时间戳
	float  lastRSSI;
	uint8_t lastsize;//上一包数据的大小
	uint8_t abstime;//绝对值之差
	uint8_t absRSSI;//绝对值之差
	uint8_t abssize;//数据包大小之差
//	uint32_t temp32;
	//uint8_t samedataflag = 0;//相同数据包标识 1相同0不同

	/* fetch N packets */
	nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);
	
	if (nb_pkt == 0) 
	{
		//wait_ms(300);
		return 0;
	} 
	else 
	{
		LORA_LEDSET(1);
		//printf("---\nRcv pkt #%d >>\n", SystemParameter.RxCnt++);
		//初始化上一次的数据属性
		lasttime = 0;
		lastRSSI = 0;
		lastsize = 0;	
		
		
		/* display received packets */
		for(i=0; i < nb_pkt; ++i)
		{
			//RFRXLedOn();					
			//RTC_TimeShow();
			p = &rxpkt[i];
				
			//CT_Command_Parsing(p);
			
			abstime = abs( p->count_us - lasttime);
			absRSSI = abs(p->rssi -lastRSSI);
			abssize = abs(p->size - lastsize);
			lasttime =  p->count_us;
			lastRSSI = p->rssi;		
			lastsize = p->size ;
			datacount = 0;//清零发送缓存
			
//			DEBUG_Printf("lora payload: ");	

//			for (uint8_t j = 0; j < p->size; ++j) 
//			{
//				DEBUG_Printf("%02X ", p->payload[j]);
//			}
//			DEBUG_Printf(" #\r\n");
//			DEBUG_Printf("RSSI: %.2f SNR: %.2f\r\n",rxpkt[i].rssi,rxpkt[i].snr);
		
			
//测试灵敏度
#ifdef DebugDisplayRXTest
			
			//正确数据包计数
			if(p->status == STAT_CRC_OK)
			{
				//for(uint8_t i = 0;i<sizeof(TestData);i++)
				for(uint8_t i = 0;i<p->size;i++)
				{
					if(p->payload[i]== TestData[i])
					{
						ReceiveRightFlag ++;
					}
					else
					{
						break;
					}
				}		
			}
			if(ReceiveRightFlag == p->size)
			{
				RightCount ++;	
			}		
			
			//数据包间差计算
			if(timecount == 0)
			{
				timerem = p->count_us;
			}
			else
			{
				if(TestToolSFRem == p->datarate)
				{
					timeavg = p->count_us - timerem;
					timerem = p->count_us;
					//printf(" if_chain:%2d", timeavg);
				}				
			}
			
			//接收数据包数量、数据包之间时间间隔选择
			if(timecount == 0)//防止测试一个SF时中间收到干扰数据，但是每次切换SF需要手动按键重新获取，而且是先前换好SF在按键才有效
			{
				TestToolSFRem = p->datarate;
				switch(p->datarate)
				{
					case DR_LORA_SF7:TestToolTimesTH = SF7TestDataNum;TimeAvgeTemp = SF7Time;DatarateMem = 7;break;
					case DR_LORA_SF10:TestToolTimesTH = SF10TestDataNum;TimeAvgeTemp = SF10Time;DatarateMem = 10;break;
					case DR_LORA_SF12:TestToolTimesTH = SF12TestDataNum;TimeAvgeTemp = SF12Time;DatarateMem = 12;break;
				}
			}

			//总包数计数
			if(timecount != 0)
			{
				timecount +=timeavg/TimeAvgeTemp;//时间间隔大于2倍错误或者失败都要加 时间小于统计值不加
			}
			else
			{
				timecount++;
			}
			printf("num:%4d time:%010u right byte count:%4d right frame count:%4d\n", timecount,timeavg,ReceiveRightFlag,RightCount);
			ReceiveRightFlag = 0;
			
			//测试完成打印和数据清零
			if(timecount >= TestToolTimesTH)
			{
				//timeavg = timeavg/temp32;			
				PerValue = (float)RightCount/(float)(timecount);
				printf("SF%2d RightCount:%4u PER:%2.2f\n", DatarateMem,RightCount,PerValue);
				timecount = 0;
				RightCount = 0;
				timeavg = 0;			
			}
#endif
			

			//CRC输出过滤
			//输出数据给用户
			
			if(((SystemParameter.ReceiveCRC & REC_CRC)&& (p->status == STAT_CRC_OK))||//CRC OK
				((SystemParameter.ReceiveCRC & REC_NO_CRC)&&(p->status == STAT_NO_CRC))||// NO CRC
			((SystemParameter.ReceiveCRC &  REC_ERROR_CRC)&&(p->status == STAT_CRC_BAD)))//CRC BAD
			{
				//CRC校验成功
				//初始化
				//memset(fw_check, 0, 500);
				//组帧
				RFRXData[datacount++] = 0xfe;
				RFRXData[datacount++] = 0xfe;
				RFRXData[datacount++] = 0x68;
				RFRXData[datacount++] = RX_C;//接收数据控制码
				//RFRXData[datacount++] = 0x68;//
				RFRXData[datacount++] = 0x00;//接收数据数据标识
				temp16 = p->size + 16;
				RFRXData[datacount++] = temp16/256;//接收数据数据域长度高8位
				RFRXData[datacount++] = temp16%256;//接收数据数据域长度低8位
				RFRXData[datacount++] = p->if_chain;
				switch (p->datarate) 
				{
					case DR_LORA_SF7: 
					temp = 7;
					break;
					case DR_LORA_SF8:
					temp = 8;
					break;
					case DR_LORA_SF9: 
					temp = 9;
					break;
					case DR_LORA_SF10: 
					temp = 10;
					break;
					case DR_LORA_SF11:
					temp = 11;
					break;
					case DR_LORA_SF12: 
					temp = 12;
					break;
					default: 
					temp = 0xff;
				}
				RFRXData[datacount++] = temp;
				RFRXData[datacount++] = p->size;
				switch (p->coderate) 
				{
					case CR_LORA_4_5:  temp = 5;
							break;
					case CR_LORA_4_6: temp = 6; 
							break;
					case CR_LORA_4_7: temp = 7;
							break;
					case CR_LORA_4_8:  temp = 8;
							break;
					default: ;//printf(" coderate?");							
				}
				RFRXData[datacount++] =temp;
				
				//SNR
				if(p->snr < 0)
				{
					RFRXData[datacount++] = 0x80|(uint8_t)(-(p->snr));	
				}
				else
				{
					RFRXData[datacount++] = (uint8_t)(p->snr);
				}
				
				//SNR min
				if(p->snr_min < 0)
				{
					RFRXData[datacount++] =  0x80|(uint8_t)(-( p->snr_min));	
				}
				else
				{
					RFRXData[datacount++] = (uint8_t)( p->snr_min);	
				}
				
				//SNR max
				if(p->snr_max < 0)
				{
					RFRXData[datacount++] = 0x80|((uint8_t)(-( p->snr_max)));	
				}
				else
				{
					RFRXData[datacount++] = (uint8_t)( p->snr_max);	
				}
				
				//RSSI
				if(p->rssi < 0)
				{
					RFRXData[datacount++] =  0x80|((uint8_t)(-(p->rssi)));
				}
				else
				{
					RFRXData[datacount++] = 0;
				}
				
				//时间戳		
				RFRXData[datacount++] =   (uint8_t)(p->count_us>>24);		
				RFRXData[datacount++] =  (uint8_t)( (p->count_us & 0x00ff0000)>>16);
				RFRXData[datacount++] =  (uint8_t)( (p->count_us & 0x0000ff00)>>8);
				RFRXData[datacount++] =  (uint8_t) (p->count_us & 0x000000ff);
				
				//CRC校验
				if(p->status == STAT_CRC_OK)
				{
					RFRXData[datacount++] =   (uint8_t)(p->crc>>8);
					RFRXData[datacount++] =   (uint8_t)(p->crc & 0x00ff);
				}
				else
				{
					RFRXData[datacount++] =   0;
					RFRXData[datacount++] =   0;
				}
				
				//保留
				RFRXData[datacount++] =  0x00;
				RFRXData[datacount++] =  0x00;
				
				//payload
				for(uint8_t i=0; i< p->size; i++)
				{
					RFRXData[datacount++] =   p->payload[i];
				}

				//CS
				RFRXData[datacount] = 0;
				for(uint16_t j = 2; j<datacount; j++)
				{
					RFRXData[datacount] += RFRXData[j];
				}
				datacount++;
				RFRXData[datacount++] = 0x16;

				//发送数据要从OUTPUTINDEX开始
				if(MCUTXData.BusyFlag == 0)
				{
					
				}
				else
				{
					while(MCUTXData.BusyFlag == 1)
					{
						uint8_t temp;
						HAL_Delay(10);
						if(temp++>15)
						{
							break;
						}
					}				
				}
				
				//HAL_UART_Transmit(&huart2,RFRXData,datacount,500);
				//Usart1SendData_DMA(RFRXData, datacount);
			
				//新数据起始 接收包技计数
#ifdef DebugDisplayRxCnt
				SystemParameter.RxCnt += nb_pkt;
				printf(">>>\\r\n Rcv pkt %ld >>\r\n", SystemParameter.RxCnt);
#endif
				// cyw 20190923
				printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
				//打印时间戳
#ifdef DebugDisplayTimestramp
				if(LastRxdataTimestamp > p->count_us)
				{
					printf("timestamp:%010u timestamp error\n", p->count_us);
				}
				else
				{				
					printf("timestamp:%010u lasttime:%010u now-last:%010u\r\n", p->count_us,LastRxdataTimestamp,p->count_us-LastRxdataTimestamp);
				}
				LastRxdataTimestamp = p->count_us;
			
#endif		

	                 //接收反向频率、通道校准
#ifdef DebugFreqIFCalibration
			if(SystemParameter.LoRaMAC && SystemParameter.ReceiveIQInvert)//LoRaWAN数据包,接收反向
			{
				temp = (p->payload[0] & 0xE0)>>5;
				if(temp == JoinAccept_H || temp == UconfirmedDataDown_H || temp == ConfirmedDataDown_H)
				{
					if(p->if_chain < 4 )
					{
						p->if_chain = 3 - p->if_chain;
						if(p->if_chain == 0)
						{
							p->freq_hz = fa - 300000;
						}
						else if(p->if_chain == 1)
						{
							p->freq_hz = fa - 100000;
						}
						else if(p->if_chain == 2)
						{
							p->freq_hz = fa + 100000;
						}
						else if(p->if_chain == 3)
						{
							p->freq_hz = fa + 300000;
						}
					}
					else
					{
						p->if_chain = 11 - p->if_chain;
						if(p->if_chain == 4)
						{
							p->freq_hz = fb - 300000;
						}
						else if(p->if_chain == 5)
						{
							p->freq_hz = fb - 100000;
						}
						else if(p->if_chain == 6)
						{
							p->freq_hz = fb + 100000;
						}
						else if(p->if_chain == 7)
						{
							p->freq_hz = fb + 300000;
						}
					}				
				}
			}
#endif	

				//打印表格射频接收数据精简版
#ifdef DebugDisplayTabStringSimple
				DispalyTabStringSimple();
#endif			
			
			
				//打印表格射频接收数据
#ifdef DebugDisplayTabString
				DispalyTabString();
#endif
			
			

									//解析LoRaWAN数据
#ifdef DebugDisplayAnalyLoRaWAN
				AnalysisLoRaWANData();		
#endif

						//输出打印串口发送数据帧
#ifdef DebugDisplayToUserRFData
				printf("TO user data:\n");
				for(uint16_t k = 0;k<datacount;k++)
				{
					printf(" %02X", RFRXData[k]);
				}
				printf(" #\n");
#endif

			//打印payload
#ifdef DebugDisplayLoraPayload
				printf("lora payload:\n");	

				for (uint8_t j = 0; j < p->size; ++j) 
				{
					printf(" %02X", p->payload[j]);
				}
				printf(" #\n");
#endif

				// cyw 20190923
				printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\r\n");
					//HAL_Delay(3);//用于断帧	
				
				}
			else
			{
				//printf("CRC is unknow,setup crc is %d,data crc is %d\n",SystemParameter.ReceiveCRC,p->status);
			}
			
				//RFRXLedOff();
			}
			//时间校验互相发送
			if(TimeCalibrationSingleSend)
			{
				Sx1301Busy();//IO指示忙状态
				//RFTXLedOn();//busy 指示灯显示
				TimeCalibrationSingleRxTx(nb_pkt);
				Sx1301Free();//IO指示忙状态
				//RFTXLedOff();//用户数据显示
			}	
			if(TimeCalibrationEachOtherSend)
			{
				Sx1301Busy();//IO指示忙状态
				//RFTXLedOn();//busy 指示灯显示
				TimeCalibrationEachOtherRxTx(nb_pkt);
				Sx1301Free();//IO指示忙状态
				//RFTXLedOff();//用户数据显示
			}		
			# if 1
			if(SystemParameter.RFRxTxDebugEN)
			{
				Sx1301Busy();//IO指示忙状态
				//RFTXLedOn();//busy 指示灯显示
				//HAL_Delay(10);
				SX1301RxTxDebug(nb_pkt);//发送数据			
				Sx1301Free();//IO指示忙状态
				//RFTXLedOff();//用户数据显示
			}
			#endif
			LORA_LEDSET(0);
	}   	
#if 0
    if (exit_sig == 1)
    {
        /* clean up before leaving */
        lgw_stop();
    }

    printf("\nEnd of test for loragw_hal.c\n");
#endif
	return 1;
}

uint8_t SX1301RXDataQueryPin()
{
//	if(HAL_GPIO_ReadPin(RXBUFF_NOT_EMPTY_PORT, RXBUFF_NOT_EMPTY_PIN))//不为空时为高电平
//	{
//		HAL_Delay(50);
//		if(HAL_GPIO_ReadPin(RXBUFF_NOT_EMPTY_PORT, RXBUFF_NOT_EMPTY_PIN))//不为空时为高电平
//		{
//			SX1301RXData();
//		}
//	}
	SX1301RXData();
	return 1;
}
uint8_t SX1301TXData()
{
	//用户参数配置
	//持续发送	
	
	//CW发送模式
	if(SystemParameter.CWSendModeEN == 1)
	{
    /* Overwrite settings */
		/* Enable signal generator with DC */
		lgw_reg_w(LGW_SIG_GEN_FREQ, 0);
		lgw_reg_w(LGW_SIG_GEN_EN, 1);
		lgw_reg_w(LGW_TX_OFFSET_I, 0);
		lgw_reg_w(LGW_TX_OFFSET_Q, 0);
  }
	else
	{
		//lgw_reg_w(LGW_SIG_GEN_EN, 0);
	}
	
	/* send a packet */	
	i = lgw_send(txpkt); /* non-blocking scheduling of TX packet */
	j = 0;
	
	#ifdef DisplaySendInfo
	printf("+++\nSending packet #%d, rf path %d, return %d\nstatus -> ", tx_cnt, txpkt.rf_chain, i);
	#endif
	
	if(SystemParameter.TxContinuous == 0)
	{
		#if 0
		do
		{
			++j;
			//wait_ms(100);
			wait_ms(1);
			lgw_status(TX_STATUS, &status_var); /* get TX status */
			printf("%d", status_var);
		} while ((status_var != TX_FREE) && (j < MAX_SEND_TIME));
		#else
		do
		{
			++j;
			wait_ms(1);		
			if(HAL_GPIO_ReadPin(TX_ON_PORT, TX_ON_PIN)==GPIO_PIN_RESET)break;//不为空时为高电平			
		} while ((status_var != TX_FREE) && (j < MAX_SEND_TIME));
		
		#ifdef DisplaySendInfo
		printf("TX wait time:%d ms", j);
		#endif
		#endif
	}
	
	TestToolFinish |= TestToolSendFinish;
	#ifdef DisplaySendInfo
	printf("\nTX finished\n");
	#endif
	if((j > 99)||(status_var != TX_FREE))
	{
		//return 0;
	}
	else
	{
		//return 1;
	}
	if(i == LGW_HAL_ERROR)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

//拉距测试用于直接烧写代码测试，其他形式使用串口发数据
//使用参数是默认的初始化参数通道和频率
void DistanceTest()
{
	ParaInit(rfAp,rfBp,prxifms);	//参数初始化
	Usart2Info.NowBaudRate = 1;//用户使用波特率默认是115200
	Usart2Info.LastBaudRate = 1;//用户使用波特率默认是115200
	lgw_stop();//停止射频
	Sx1301Power(1);//关闭射频电源
	SX1301Status.NeedStartFlag = 1;//需要启动
	SX1301Status.StartSuccesFlag = 0;//清零启动状态标记
	//SystemParameter.RFRxTxDebugEN = 1;//开启拉距模式
	SX1301Status.StartSuccesFlag = SX1301Start(rfAp,rfBp,prxifms);
	SX1301Status.NeedStartFlag =0;//??????
	if(SX1301Status.StartSuccesFlag == 0)
	{
		Sx1301StartFail();//IO??
	}
	else
	{
		Sx1301StartSucces();//IO??
	}	
}
//参数ms
void WaiteTimeDisplay(uint32_t j)
{
	uint32_t i;
	uint8_t quantity = 100;
	i=j;
	i/=quantity;
	for(uint8_t k = 0;k<quantity;k++)
	{
		printf(">");  
		wait_ms(i);	
	}	
	printf("\n");  
}
//格式数据字符串表格
/*
---------------
|DEVEUI|      |
---------------
*/

void DispalyTabString()
{
	uint16_t width=0;//总长度
	uint16_t temp;
	char coderate[10]="CR1(4/5)";
	//计算出长度	
	for(temp = 0;temp<DISTAB_LEN;temp++)
	{
		width+=Distab[temp].DataLen;
	}
	width+= DISTAB_LEN+1;
	
	//for(uint8_t count = 0;count<nb_pkt;count++)	
	{
		uint8_t count = 0;
		//打印第0行
		//printf("timestamp:\n",rxpkt[count].count_us);//打印数据
		//打印第1行
		for(temp = 0;temp<width;temp++)
		{
			printf("-"); 
		}
		printf("\n"); 
		//打印第2行
		printf("|"); 
		printf("%10s",Distab[0].Name); 
		printf("|"); 
		printf("%10s",Distab[1].Name); 
		printf("|");
		printf("%10s",Distab[2].Name); 
		printf("|");
		printf("%10s",Distab[3].Name); 
		printf("|");
		printf("%10s",Distab[4].Name); 
		printf("|"); 
		printf("%10s",Distab[5].Name); 
		printf("|");
		printf("%10s",Distab[6].Name); 
		printf("|");
		printf("%10s",Distab[7].Name); 
		printf("|"); 
		printf("%10s",Distab[8].Name); 
		printf("|");
		printf("%10s",Distab[9].Name); 
		printf("|");
		printf("\n"); 
		//打印第3行
		for(temp = 0;temp<width;temp++)
		{
			printf("-"); 
		}
		printf("\n"); 
		//打印第4行
		// cyw 20190923
		printf("@");
		printf("%10d",rxpkt[count].if_chain);//打印数据
		printf("|");
		printf("%10d",rxpkt[count].freq_hz);//打印数据
		printf("|");
		switch (rxpkt[count].datarate) 
		{
			case DR_LORA_SF7: 
			temp = 7;
			break;
			case DR_LORA_SF8:
			temp = 8;
			break;
			case DR_LORA_SF9: 
			temp = 9;
			break;
			case DR_LORA_SF10: 
			temp = 10;
			break;
			case DR_LORA_SF11:
			temp = 11;
			break;
			case DR_LORA_SF12: 
			temp = 12;
			break;
			default: 
			temp = 7;
		}	
		printf("%10d",temp);//打印数据
		printf("|");
		printf("%10d",rxpkt[count].size);//打印数据
		printf("|");
		switch (rxpkt[count].coderate) 
		{
			case CR_LORA_4_5: strcpy(coderate,"CR1(4/5)");
					break;
			case CR_LORA_4_6: strcpy(coderate,"CR2(2/3)");
					break;
			case CR_LORA_4_7: strcpy(coderate,"CR3(4/7)");
					break;
			case CR_LORA_4_8: strcpy(coderate,"CR4(1/2)");
					break;
			default: strcpy(coderate,"CR1(4/5)");
		}
		printf("%10s",coderate);//打印数据
		printf("|");
		if(rxpkt[count].status == STAT_CRC_OK)
		{
			printf("%10X",rxpkt[count].crc);//打印数据
		}
		else if(rxpkt[count].status == STAT_CRC_BAD)
		{
			printf("%10s","CRC error");//打印数据
		}
		else
		{
			printf("%10s","no CRC");//打印数据
		}	
		printf("|");
		printf("%10.1f",rxpkt[count].snr);//打印数据
		printf("|");
		printf("%10.1f",rxpkt[count].snr_min);//打印数据
		printf("|");
		printf("%10.1f",rxpkt[count].snr_max);//打印数据
		printf("|");
		printf("%10.1f",rxpkt[count].rssi);//打印数据
		// cyw 20190923
		printf("@");
		printf("\n"); 
		//打印第5行
		for(temp = 0;temp<width;temp++)
		{
			printf("-"); 
		}
		printf("\n");
		
	}	
}


void DispalyTabStringSimple()
{
	uint8_t sf;
	char cr[]="CR1(4/5)";
	char crc[]="CRC_BAD";
	for(uint8_t count = 0;count<nb_pkt;count++)	
	{
		switch(rxpkt[count].datarate) 
		{
				case DR_LORA_SF7: 
				sf = 7;
				break;
				case DR_LORA_SF8:
				sf = 8;
				break;
				case DR_LORA_SF9: 
				sf = 9;
				break;
				case DR_LORA_SF10: 
				sf = 10;
				break;
				case DR_LORA_SF11:
				sf = 11;
				break;
				case DR_LORA_SF12: 
				sf = 12;
				break;
				default: 
				sf = 7;
		}	
		switch (rxpkt[count].coderate) 
		{
				case CR_LORA_4_5: strcpy(cr,"CR1(4/5)");
						break;
				case CR_LORA_4_6: strcpy(cr,"CR2(4/6)");
						break;
				case CR_LORA_4_7: strcpy(cr,"CR3(4/7)");
						break;
				case CR_LORA_4_8: strcpy(cr,"CR4(4/8)");
						break;
				default: strcpy(cr,"CR1(4/5)");
		}
		printf("TMST:%10d,FREQ:%10d,Chain:%2d,SF:%2d,CR:%10s,CRC:%6d,RSSI:%6.1f,SNR ave:%4.1f,SNR min:%4.1f,SNR max:%4.1f,Size:%3d,\n",\
						rxpkt[count].count_us,rxpkt[count].freq_hz,rxpkt[count].if_chain,sf,cr,rxpkt[count].crc,rxpkt[count].rssi,rxpkt[count].snr,rxpkt[count].snr_min,rxpkt[count].snr_max,rxpkt[count].size);//打印数据		
		printf("Payload:");
		for(uint8_t num=0;num<rxpkt[count].size;num++)
		{
			printf("%02X",rxpkt[count].payload[num]);
		}
		printf("\n");
	}	
}

//打印
//

uint8_t  AnalysisLoRaWANData()
{
	uint8_t count = 0;//帧序号
	uint8_t temp = 0;
	uint8_t width = 0;//打印位宽
	if(SystemParameter.LoRaMAC == 0)
	{
		return 0;
	}
	#if 1
	if(rxpkt[count]. status == STAT_CRC_BAD)
	{
		return 0;
	}
	#endif 

	//屏蔽SF9 干扰报	
	#if 0
	if(rxpkt[count].datarate == DR_LORA_SF9)
	{
		return 0;
	}	
	#endif
	temp = rxpkt[count].payload[0];
	if(temp &0x1F)
	{
		return 0;
	}
	LoRaWANData.MHDR = rxpkt[count].payload[0];
	temp = LoRaWANData.MHDR;
	temp&=0xE0;
	temp>>=5;
	LoRaWANData.MType  = temp;
	
	LoRaWANData.RFU = (LoRaWANData.MHDR&0x1C)>>2;
	LoRaWANData.Major = LoRaWANData.MHDR&0x03;
	switch(LoRaWANData.MType)
	{
		case 0://join request
			printf("### join request command\n"); 
			//appeui
			for(uint8_t i = 0;i<8;i++)
			{
				LoRaWANData.APPEUI[i] = rxpkt[count].payload[i+1];
			}	
			//deveui
			for(uint8_t i = 0;i<8;i++)
			{
				LoRaWANData.DEVEUI[i] = rxpkt[count].payload[i+9];
			}
			//DevNonce 2
			for(uint8_t i = 0;i<2;i++)
			{
				LoRaWANData.DevNonce[i] = rxpkt[count].payload[i+17];
			}
			//MIC
			for(uint8_t i = 0;i<4;i++)
			{
				LoRaWANData.MIC[i] = rxpkt[count].payload[i+19];
			}
			//
			//打印
			//
			//计算出长度	
			for(uint8_t i = 0;i<LoRaWANUpLinkJoinLEN;i++)
			{
				width+=LoRaWANJoinUpLinkDistab[i].DataLen;
			}
			width+= LoRaWANUpLinkJoinLEN+1;
			
			//打印第1行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第2行
			printf("|"); 
			printf("%6s",LoRaWANJoinUpLinkDistab[0].Name); 
			printf("|"); 
			printf("%18s",LoRaWANJoinUpLinkDistab[1].Name); 
			printf("|");
			printf("%18s",LoRaWANJoinUpLinkDistab[2].Name); 
			printf("|");
			printf("%10s",LoRaWANJoinUpLinkDistab[3].Name); 
			printf("|");
			printf("%10s",LoRaWANJoinUpLinkDistab[4].Name); 
			printf("|");  
			printf("\n"); 
			//打印第3行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第4行
			// cyw 20190923
			printf("$");
			//MHDR
			printf("    ");
			printf("%02X",LoRaWANData.MHDR);
			printf("|");
			//APPEUI
			printf("  ");
			for(uint8_t i=0;i<8;i++)
			{
				printf("%02X",LoRaWANData.APPEUI[7-i]);
			}			
			printf("|");
			//DEVEUI
			printf("  ");
			for(uint8_t i=0;i<8;i++)
			{
				printf("%02X",LoRaWANData.DEVEUI[7-i]);
				
			}			
			printf("|");
			//DevNonce
			printf("      ");//4个
			for(uint8_t i=0;i<2;i++)
			{
				printf("%02X",LoRaWANData.DevNonce[1-i]);
			}			
			printf("|");
			//MIC
			printf("  ");//2个
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.MIC[3-i]);				
			}			
			// cyw 20190923
			printf("$");
			printf("\n");
			//打印第5行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			break;
		case 1://join accept
			printf("### join accept command\n"); 
			//AppNonce
			for(uint8_t i = 0;i<3;i++)
			{
				LoRaWANData.AppNonce[i] = rxpkt[count].payload[i+1];
			}	
			//NetID
			for(uint8_t i = 0;i<3;i++)
			{
				LoRaWANData.NetID[i] = rxpkt[count].payload[i+4];
			}
			//DevAddr
			for(uint8_t i = 0;i<4;i++)
			{
				LoRaWANData.DevAddr[i] = rxpkt[count].payload[i+7];
			}
			//DLSettings  
			LoRaWANData.DLSettings =  rxpkt[count].payload[i+11];
			//DLSettings  
			LoRaWANData.RxDelay =  rxpkt[count].payload[i+12];
			//CFList
			temp = rxpkt[count].size - 13;
			for(uint8_t i=0;i<temp;i++)
			{
				LoRaWANData.CFList[i] =  rxpkt[count].payload[i+13];
			}
			//
			//打印
			//
			//计算出长度	
			for(uint8_t i = 0;i<LoRaWANDownLinkJoinLEN;i++)
			{
				width+=LoRaWANJoinDownLinkDistab[i].DataLen;
			}
			width+= LoRaWANDownLinkJoinLEN+1;
			
			//打印第1行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第2行
			printf("|"); 
			printf("%6s",LoRaWANJoinDownLinkDistab[0].Name); 
			printf("|"); 
			printf("%10s",LoRaWANJoinDownLinkDistab[1].Name); 
			printf("|");
			printf("%8s",LoRaWANJoinDownLinkDistab[2].Name); 
			printf("|");
			printf("%10s",LoRaWANJoinDownLinkDistab[3].Name); 
			printf("|");
			printf("%12s",LoRaWANJoinDownLinkDistab[4].Name); 
			printf("|");  
			printf("%10s",LoRaWANJoinDownLinkDistab[5].Name); 
			printf("|");
			printf("%32s",LoRaWANJoinDownLinkDistab[6].Name); 
			printf("|");  
			printf("\n"); 
			//打印第3行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第4行
			// cyw 20190923
			printf("$");
			//MHDR
			printf("%6X",LoRaWANData.MHDR);
			printf("|");
			//AppNonce
			printf("    ");
			for(uint8_t i=0;i<3;i++)
			{
				printf("%02X",LoRaWANData.AppNonce[2-i]);
			}			
			printf("|");
			//NetID
			printf("  ");
			for(uint8_t i=0;i<3;i++)
			{
				printf("%02X",LoRaWANData.NetID[2-i]);
				
			}			
			printf("|");
			//DevAddr
			printf("  ");//4个
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.DevAddr[3-i]);
			}			
			printf("|");
			//DLSettings  
			printf("%12d",LoRaWANData.DLSettings);	
			printf("|");
			//RxDelay     
			printf("%10d",LoRaWANData.RxDelay);	
			printf("|");
			//CFList
			for(uint8_t i=0;i<16;i++)
			{
				printf("%02X",LoRaWANData.CFList[15-i]);				
			}			
			// cyw 20190923
			printf("$");
			printf("\n");
			//打印第5行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			break;
		case 2://unconfirmed data up
			//devaddr
			printf("### unconfirmed data up command\n"); 
			for(uint8_t i = 0;i<4;i++)
			{
				LoRaWANData.DevAddr[i] = rxpkt[count].payload[i+1];
			}
			LoRaWANData.FCTL = rxpkt[count].payload[5];
			LoRaWANData.ADR = LoRaWANData.FCTL&0x80;
			LoRaWANData.ADRACKReq = LoRaWANData.FCTL&0x40;
			LoRaWANData.ACK = LoRaWANData.FCTL&0x20;
			LoRaWANData.RFU = LoRaWANData.FCTL&0x10;
			LoRaWANData.FOptslen = LoRaWANData.FCTL&0x0F;			
			LoRaWANData.Fcnt = rxpkt[count].payload[6]+(rxpkt[count].payload[7]<<8);
			for(uint8_t i=0;i<LoRaWANData.FOptslen;i++)
			{
				LoRaWANData.Fopts[i] = rxpkt[count].payload[i+8];
			}
			LoRaWANData.Fport = rxpkt[count].payload[8+LoRaWANData.FOptslen];
			LoRaWANData.FRamePayloadIndex = LoRaWANData.FOptslen+9;
			LoRaWANData.FRamePayloadLen = rxpkt[count].size - 13 - LoRaWANData.FOptslen;
			for(uint8_t i=0;i<4;i++)
			{
				LoRaWANData.MIC[i] =  rxpkt[count].payload[rxpkt[count].size -(4 -i)];
			}
			//打印
			//计算出长度	
			for(uint8_t i = 0;i<LoRaWANUpLinkDataLEN;i++)
			{
				width+=LoRaWANUpLinkDataDistab[i].DataLen;
			}
			width+= LoRaWANUpLinkDataLEN+1;			
			//打印第1行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第2行
			printf("|"); 
			printf("%6s",LoRaWANUpLinkDataDistab[0].Name); 
			printf("|"); 
			printf("%10s",LoRaWANUpLinkDataDistab[1].Name); 
			printf("|");
			printf("%6s",LoRaWANUpLinkDataDistab[2].Name); 
			printf("|");
			printf("%6s",LoRaWANUpLinkDataDistab[3].Name); 
			printf("|");
			printf("%32s",LoRaWANUpLinkDataDistab[4].Name); 
			printf("|");  			
			printf("%7s",LoRaWANUpLinkDataDistab[5].Name); 
			printf("|");
			printf("%16s",LoRaWANUpLinkDataDistab[6].Name); 
			printf("|");
			printf("%10s",LoRaWANUpLinkDataDistab[7].Name); 
			printf("|");  	
			printf("\n"); 
			//打印第3行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第4行
			// cyw 20190923
			printf("$");
			//MHDR
			printf("%6X",LoRaWANData.MHDR);
			printf("|");
			//DEVADDR
			printf("  ");
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.DevAddr[3-i]);
			}			
			printf("|");
			//FCTL
			printf("%6X",LoRaWANData.FCTL);
			printf("|");
			//Fcnt
			printf("%6d",LoRaWANData.Fcnt);
			printf("|");
			//Fopts
			//for(uint8_t i=0;i<LoRaWANData.FOptslen;i++)
			if(LoRaWANData.FOptslen !=0)
			{
				printf("  ");
				for(uint8_t i=0;i<15;i++)
				{
					printf("%02X",LoRaWANData.Fopts[14-i]);
				}	
			}
			else
			{
				char stringtem[]="no FOpts";
				printf("%32s",stringtem);
			}						
			printf("|");
			//Fport
			printf("%7d",LoRaWANData.Fport);
			printf("|");
			//FRMpayload
			{
				char stringtem[]="Encrypted data";
				printf("%16s",stringtem);
			}			
			printf("|");
			//MIC
			printf("  ");//2个
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.MIC[3-i]);				
			}			
			// cyw 20190923
			printf("$");
			printf("\n");
			//打印第5行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印payload	
			break;
		case 3://unconfirmed data down
			printf("### unconfirmed data down command\n"); 
			//devaddr
			for(uint8_t i = 0;i<4;i++)
			{
				LoRaWANData.DevAddr[i] = rxpkt[count].payload[i+1];
			}
			LoRaWANData.FCTL = rxpkt[count].payload[5];
			LoRaWANData.ADR = LoRaWANData.FCTL&0x80;
			LoRaWANData.ADRACKReq = LoRaWANData.FCTL&0x40;
			LoRaWANData.ACK = LoRaWANData.FCTL&0x20;
			LoRaWANData.RFU = LoRaWANData.FCTL&0x10;
			LoRaWANData.FOptslen = LoRaWANData.FCTL&0x0F;			
			LoRaWANData.Fcnt = rxpkt[count].payload[6]+(rxpkt[count].payload[7]<<8);
			for(uint8_t i=0;i<LoRaWANData.FOptslen;i++)
			{
				LoRaWANData.Fopts[i] = rxpkt[count].payload[i+8];
			}
			LoRaWANData.Fport = rxpkt[count].payload[8+LoRaWANData.FOptslen];
			LoRaWANData.FRamePayloadIndex = LoRaWANData.FOptslen+9;
			LoRaWANData.FRamePayloadLen = rxpkt[count].size - 13 - LoRaWANData.FOptslen;
			for(uint8_t i=0;i<4;i++)
			{
				LoRaWANData.MIC[i] =  rxpkt[count].payload[rxpkt[count].size -(4 -i)];
			}
			//打印
			//计算出长度	
			for(uint8_t i = 0;i<LoRaWANUpLinkDataLEN;i++)
			{
				width+=LoRaWANUpLinkDataDistab[i].DataLen;
			}
			width+= LoRaWANUpLinkDataLEN+1;			
			//打印第1行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第2行
			printf("|"); 
			printf("%6s",LoRaWANUpLinkDataDistab[0].Name); 
			printf("|"); 
			printf("%10s",LoRaWANUpLinkDataDistab[1].Name); 
			printf("|");
			printf("%6s",LoRaWANUpLinkDataDistab[2].Name); 
			printf("|");
			printf("%6s",LoRaWANUpLinkDataDistab[3].Name); 
			printf("|");
			printf("%32s",LoRaWANUpLinkDataDistab[4].Name); 
			printf("|");  			
			printf("%7s",LoRaWANUpLinkDataDistab[5].Name); 
			printf("|");
			printf("%16s",LoRaWANUpLinkDataDistab[6].Name); 
			printf("|");
			printf("%10s",LoRaWANUpLinkDataDistab[7].Name); 
			printf("|");  	
			printf("\n"); 
			//打印第3行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第4行
			// cyw 20190923
			printf("$");
			//MHDR
			printf("%6X",LoRaWANData.MHDR);
			printf("|");
			//DEVADDR
			printf("  ");
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.DevAddr[3-i]);
			}			
			printf("|");
			//FCTL
			printf("%6X",LoRaWANData.FCTL);
			printf("|");
			//Fcnt
			printf("%6d",LoRaWANData.Fcnt);
			printf("|");
			//Fopts
			//for(uint8_t i=0;i<LoRaWANData.FOptslen;i++)
			if(LoRaWANData.FOptslen !=0)
			{
				printf("  ");
				for(uint8_t i=0;i<15;i++)
				{
					printf("%02X",LoRaWANData.Fopts[14-i]);
				}	
			}
			else
			{
				char stringtem[]="no FOpts";
				printf("%32s",stringtem);
			}						
			printf("|");
			//Fport
			printf("%7d",LoRaWANData.Fport);
			printf("|");
			//FRMpayload
			{
				char stringtem[]="Encrypted data";
				printf("%16s",stringtem);
			}			
			printf("|");
			//MIC
			printf("  ");//2个
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.MIC[3-i]);				
			}			
			// cyw 20190923
			printf("$");
			printf("\n");
			//打印第5行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印payload	
			break;
		case 4://confirmed data up
			printf("### confirmed data up command\n"); 
			//devaddr
			for(uint8_t i = 0;i<4;i++)
			{
				LoRaWANData.DevAddr[i] = rxpkt[count].payload[i+1];
			}
			LoRaWANData.FCTL = rxpkt[count].payload[5];
			LoRaWANData.ADR = LoRaWANData.FCTL&0x80;
			LoRaWANData.ADRACKReq = LoRaWANData.FCTL&0x40;
			LoRaWANData.ACK = LoRaWANData.FCTL&0x20;
			LoRaWANData.RFU = LoRaWANData.FCTL&0x10;
			LoRaWANData.FOptslen = LoRaWANData.FCTL&0x0F;			
			LoRaWANData.Fcnt = rxpkt[count].payload[6]+(rxpkt[count].payload[7]<<8);
			for(uint8_t i=0;i<LoRaWANData.FOptslen;i++)
			{
				LoRaWANData.Fopts[i] = rxpkt[count].payload[i+8];
			}
			LoRaWANData.Fport = rxpkt[count].payload[8+LoRaWANData.FOptslen];
			LoRaWANData.FRamePayloadIndex = LoRaWANData.FOptslen+9;
			LoRaWANData.FRamePayloadLen = rxpkt[count].size - 13 - LoRaWANData.FOptslen;
			for(uint8_t i=0;i<4;i++)
			{
				LoRaWANData.MIC[i] =  rxpkt[count].payload[rxpkt[count].size -(4 -i)];
			}
			
			//打印
			//计算出长度	
			for(uint8_t i = 0;i<LoRaWANUpLinkDataLEN;i++)
			{
				width+=LoRaWANUpLinkDataDistab[i].DataLen;
			}
			width+= LoRaWANUpLinkDataLEN+1;			
			//打印第1行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第2行
			printf("|"); 
			printf("%6s",LoRaWANUpLinkDataDistab[0].Name); 
			printf("|"); 
			printf("%10s",LoRaWANUpLinkDataDistab[1].Name); 
			printf("|");
			printf("%6s",LoRaWANUpLinkDataDistab[2].Name); 
			printf("|");
			printf("%6s",LoRaWANUpLinkDataDistab[3].Name); 
			printf("|");
			printf("%32s",LoRaWANUpLinkDataDistab[4].Name); 
			printf("|");  			
			printf("%7s",LoRaWANUpLinkDataDistab[5].Name); 
			printf("|");
			printf("%16s",LoRaWANUpLinkDataDistab[6].Name); 
			printf("|");
			printf("%10s",LoRaWANUpLinkDataDistab[7].Name); 
			printf("|");  	
			printf("\n"); 
			//打印第3行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第4行
			// cyw 20190923
			printf("$");
			//MHDR
			printf("%6X",LoRaWANData.MHDR);
			printf("|");
			//DEVADDR
			printf("  ");
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.DevAddr[3-i]);
			}			
			printf("|");
			//FCTL
			printf("%6X",LoRaWANData.FCTL);
			printf("|");
			//Fcnt
			printf("%6d",LoRaWANData.Fcnt);
			printf("|");
			//Fopts
			//for(uint8_t i=0;i<LoRaWANData.FOptslen;i++)
			if(LoRaWANData.FOptslen !=0)
			{
				printf("  ");
				for(uint8_t i=0;i<15;i++)
				{
					printf("%02X",LoRaWANData.Fopts[14-i]);
				}	
			}
			else
			{
				char stringtem[]="no FOpts";
				printf("%32s",stringtem);
			}						
			printf("|");
			//Fport
			printf("%7d",LoRaWANData.Fport);
			printf("|");
			//FRMpayload
			{
				char stringtem[]="Encrypted data";
				printf("%16s",stringtem);
			}			
			printf("|");
			//MIC
			printf("  ");//2个
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.MIC[3-i]);				
			}			
			// cyw 20190923
			printf("$");
			printf("\n");
			//打印第5行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印payload			
			break;
		case 5://confirmed down 
			printf("### confirmed down command\n"); 
			//devaddr
			for(uint8_t i = 0;i<4;i++)
			{
				LoRaWANData.DevAddr[i] = rxpkt[count].payload[i+1];
			}
			LoRaWANData.FCTL = rxpkt[count].payload[5];
			LoRaWANData.ADR = LoRaWANData.FCTL&0x80;
			LoRaWANData.ADRACKReq = LoRaWANData.FCTL&0x40;
			LoRaWANData.ACK = LoRaWANData.FCTL&0x20;
			LoRaWANData.RFU = LoRaWANData.FCTL&0x10;
			LoRaWANData.FOptslen = LoRaWANData.FCTL&0x0F;			
			LoRaWANData.Fcnt = rxpkt[count].payload[6]+(rxpkt[count].payload[7]<<8);
			for(uint8_t i=0;i<LoRaWANData.FOptslen;i++)
			{
				LoRaWANData.Fopts[i] = rxpkt[count].payload[i+8];
			}
			LoRaWANData.Fport = rxpkt[count].payload[8+LoRaWANData.FOptslen];
			LoRaWANData.FRamePayloadIndex = LoRaWANData.FOptslen+9;
			LoRaWANData.FRamePayloadLen = rxpkt[count].size - 13 - LoRaWANData.FOptslen;
			for(uint8_t i=0;i<4;i++)
			{
				LoRaWANData.MIC[i] =  rxpkt[count].payload[rxpkt[count].size -(4 -i)];
			}
			
			//打印
			//计算出长度	
			for(uint8_t i = 0;i<LoRaWANUpLinkDataLEN;i++)
			{
				width+=LoRaWANUpLinkDataDistab[i].DataLen;
			}
			width+= LoRaWANUpLinkDataLEN+1;			
			//打印第1行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第2行
			printf("|"); 
			printf("%6s",LoRaWANUpLinkDataDistab[0].Name); 
			printf("|"); 
			printf("%10s",LoRaWANUpLinkDataDistab[1].Name); 
			printf("|");
			printf("%6s",LoRaWANUpLinkDataDistab[2].Name); 
			printf("|");
			printf("%6s",LoRaWANUpLinkDataDistab[3].Name); 
			printf("|");
			printf("%32s",LoRaWANUpLinkDataDistab[4].Name); 
			printf("|");  			
			printf("%7s",LoRaWANUpLinkDataDistab[5].Name); 
			printf("|");
			printf("%16s",LoRaWANUpLinkDataDistab[6].Name); 
			printf("|");
			printf("%10s",LoRaWANUpLinkDataDistab[7].Name); 
			printf("|");  	
			printf("\n"); 
			//打印第3行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印第4行
			// cyw 20190923
			printf("$");
			//MHDR
			printf("%6X",LoRaWANData.MHDR);
			printf("|");
			//DEVADDR
			printf("  ");
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.DevAddr[3-i]);
			}			
			printf("|");
			//FCTL
			printf("%6X",LoRaWANData.FCTL);
			printf("|");
			//Fcnt
			printf("%6d",LoRaWANData.Fcnt);
			printf("|");
			//Fopts
			//for(uint8_t i=0;i<LoRaWANData.FOptslen;i++)
			if(LoRaWANData.FOptslen !=0)
			{
				printf("  ");
				for(uint8_t i=0;i<15;i++)
				{
					printf("%02x",LoRaWANData.Fopts[14-i]);
				}	
			}
			else
			{
				char stringtem[]="no FOpts";
				printf("%32s",stringtem);
			}						
			printf("|");
			//Fport
			printf("%7d",LoRaWANData.Fport);
			printf("|");
			//FRMpayload
			{
				char stringtem[]="Encrypted data";
				printf("%16s",stringtem);
			}			
			printf("|");
			//MIC
			printf("  ");//2个
			for(uint8_t i=0;i<4;i++)
			{
				printf("%02X",LoRaWANData.MIC[3-i]);				
			}			
			// cyw 20190923
			printf("$");
			printf("\n");
			//打印第5行
			for(temp = 0;temp<width;temp++)
			{
				printf("-"); 
			}
			printf("\n"); 
			//打印payload			
			break;
	}	
	//
	//打印
	//
	
	return 1;
}







