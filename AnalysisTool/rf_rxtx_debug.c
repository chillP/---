#include "rf_rxtx_debug.h"
#include "loragw_user.h"
#include "loragw_hal.h"


extern	struct lgw_pkt_tx_s txpkt; /* configuration and metadata for an outbound packet */
extern	struct lgw_pkt_rx_s rxpkt[4]; /* array containing up to 4 inbound packets metadata */
extern RFSystemParameter_TYPE SystemParameter;
uint32_t  TimeCalibrationRXLast,TimeCalibrationTXLast,TimeCalibrationDataTime;
int32_t  LastTimeTimeCalibrationcha;
uint8_t RXTXTimeCalibrationSingleSendFlag;//单独发送开始计算时间差使能

struct RXTXTimeCalibrationParameter_s{
uint8_t 	RxBW;//接收到的BW
uint8_t  	coderate;       /*!> error-correcting code of the packet (LoRa only) */	
uint32_t 	RxFreq;//接收到的频率，发送相同的频率
uint32_t  datarate;       /*!> TX datarate (baudrate for FSK, SF for LoRa) */
uint32_t	TimeCount;
}RXTXTimeCalibrationParameter;


//外部发送一个对时的时间戳命令，开始周期发送对时数据
//void SX1301RxTxDebugSingle(int nb_pkt)
void TimeCalibrationSingleRxTx(int nb_pkt)
{
	uint32_t temp;
	
	if(RXTXTimeCalibrationSingleSendFlag == 0)
	{
			//解析接收数据
		if(nb_pkt != 0)
		{		
			for(uint8_t i=0;i<nb_pkt;i++)
			{
				if(rxpkt[i].status == STAT_CRC_OK)
				{
					//if(rxpkt[i].payload[0] == 0x5A && rxpkt[i].payload[5] == 0xA5)
					if(rxpkt[i].payload[0] == 0x5A && rxpkt[i].payload[18] == 0xA5)
					{					
						//如果是外部触发数据帧，那么标记开始发送，否则只做从机接收
						if(rxpkt[i].payload[1]+rxpkt[i].payload[2]+rxpkt[i].payload[3]+rxpkt[i].payload[4] == 0)
						{
							RXTXTimeCalibrationSingleSendFlag = 1;
							SystemParameter.TimeTrigSendEN = 0;
							
							RXTXTimeCalibrationParameter.coderate = rxpkt[i].coderate;
							RXTXTimeCalibrationParameter.datarate = rxpkt[i].datarate;
							RXTXTimeCalibrationParameter.RxBW = BW_125KHZ;
							RXTXTimeCalibrationParameter.RxFreq = rxpkt[i].freq_hz;								
						}		
						else
						{					
							if(rxpkt[i].count_us != TimeCalibrationRXLast)
							{
								RXTXTimeCalibrationSingleSendFlag = 0;
								SystemParameter.RxCnt++;
								TimeCalibrationDataTime = (rxpkt[i].payload[1]<<24)+(rxpkt[i].payload[2]<<16)+(rxpkt[i].payload[3]<<8)+rxpkt[i].payload[4];
								//printf("count:%06d,T(Raceive):%010u,T(Send):%010u,T(Cycle):%010u,TOA:%010u,T(R-D):%010u,T(R-R):%010d,T(T-R):%010u\r\n",
								printf("count:%010d,T(Raceive):%010u,T(DT):%010u,TOA:%010u,T(R-D-TOA-DT):%010d,T(R-R):%010d\r\n",\
								SystemParameter.RxCnt,\
								rxpkt[i].count_us,\
								TimeCalibrationDataTime,\
								lgw_time_on_air(&txpkt),\
								rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt) - 12,\
								rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt) - 12 - LastTimeTimeCalibrationcha
								);

								TimeCalibrationRXLast = rxpkt[i].count_us;
								TimeCalibrationTXLast = txpkt.count_us;
								LastTimeTimeCalibrationcha = rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt) - 12;
							}						
						}					
					}		
					else//不是对时数据帧
					{
						#if 0
						printf("Receive data is't need\r\n");
						printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", rxpkt[i].rssi,rxpkt[i].snr, rxpkt[i].snr_min, rxpkt[i].snr_max);
						for (uint8_t j = 0; j < rxpkt[i].size; ++j) 
						{
							printf(" %02X", rxpkt[i].payload[j]);
						}
						printf(" #\n");
						#endif
					}
				}	
				else if(rxpkt[i].status == STAT_CRC_BAD)
				{
					//printf("CRC bad\r\n");
					//printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", rxpkt[i].rssi,rxpkt[i].snr, rxpkt[i].snr_min, rxpkt[i].snr_max);
				}
				else
				{
					//printf("no CRC Data\r\n");
				}
			}
		}
	}
	
	
	//是否打开时间戳发送已以及获取发送时间戳
	if(RXTXTimeCalibrationSingleSendFlag == 1)
	{	
		//获取SX1301时间
		lgw_reg_w(LGW_GPS_EN, 0);
		lgw_get_trigcnt(&SystemParameter.Timestamp);
		lgw_reg_w(LGW_GPS_EN, 1);				
		
		if(RXTXTimeCalibrationParameter.TimeCount == 0)
		{			
			txpkt.freq_hz = RXTXTimeCalibrationParameter.RxFreq;
			txpkt.tx_mode = IMMEDIATE;		
			SystemParameter.TimestampSendTime = SystemParameter.Timestamp;
			txpkt.count_us = SystemParameter.TimestampSendTime;		
			txpkt.rf_power = 10;
			txpkt.modulation = MOD_LORA;
			txpkt.bandwidth = BW_125KHZ;
			txpkt.datarate = RXTXTimeCalibrationParameter.datarate;
			txpkt.coderate = RXTXTimeCalibrationParameter.coderate;
			
			txpkt.size = 19;
			txpkt.preamble = 8;//需要在协议中指明
			txpkt.rf_chain = 0;
			
			temp = txpkt.count_us;
			
			txpkt.payload[0] = 0x5A;	
			txpkt.payload[1] = (uint8_t)(temp >> 24);
			txpkt.payload[2] = (uint8_t)((temp<<8)>>24);
			txpkt.payload[3] = (uint8_t)((temp<<16)>>24);
			txpkt.payload[4] = (uint8_t)(temp<<24)>>24;
			txpkt.payload[18] = 0xA5;
			
			SX1301TXData();	
			
			RXTXTimeCalibrationParameter.TimeCount ++;
			
			printf("[TimeCalibrationSend]:SendCount:%06d,NowTime:%010d,SendTime:%010d,SendDataLen:%03d,SendFreq:%010d,SendSF:%02d,SendCoderate:%02d,SendTOA:%010d,SendPreamb:%02d\r\n",
			RXTXTimeCalibrationParameter.TimeCount,
			SystemParameter.Timestamp,
			txpkt.count_us,
			txpkt.size,
			txpkt.freq_hz,
			txpkt.datarate,
			txpkt.coderate,
			lgw_time_on_air(&txpkt),
			txpkt.preamble);
		}
		
		//判断是否进行时间戳发送
		if(SystemParameter.TimeTrigSendEN == 0)
		{	
			if(RXTXTimeCalibrationParameter.TimeCount != 0)
			{
				if(SystemParameter.Timestamp > (SystemParameter.TimestampSendTime + (TimeCalibrationCycle/2)))
				{
					//printf("SystemParameter.Timestamp:%010u,SystemParameter.TimestampSendTime:%010u\r\n",SystemParameter.Timestamp,SystemParameter.TimestampSendTime);
					//printf("RXTXTimeCalibrationParameter.TimeCount: %02d \r\n",RXTXTimeCalibrationParameter.TimeCount);
					SystemParameter.TimestampSendTime = SystemParameter.TimestampSendTime + TimeCalibrationCycle;
					//记录发送标志位
					SystemParameter.TimeTrigSendEN = 1;
				}
				else
				{
					//printf("Waite\r\n");
				}
			}					
		}					
		//填充发送数据
		else		
		{
			SystemParameter.TimeTrigSendEN = 0;
			
			txpkt.freq_hz = RXTXTimeCalibrationParameter.RxFreq;
			txpkt.tx_mode = TIMESTAMPED;		
			txpkt.count_us = SystemParameter.TimestampSendTime;		
			txpkt.rf_power = 10;
			txpkt.modulation = MOD_LORA;
			txpkt.bandwidth = BW_125KHZ;
			txpkt.datarate = RXTXTimeCalibrationParameter.datarate;
			txpkt.coderate = RXTXTimeCalibrationParameter.coderate;
			
			txpkt.size = 19;
			txpkt.preamble = 8;//需要在协议中指明
			txpkt.rf_chain = 0;
			
			temp = txpkt.count_us;
			
			txpkt.payload[0] = 0x5A;	
			txpkt.payload[1] = (uint8_t)(temp >> 24);
			txpkt.payload[2] = (uint8_t)((temp<<8)>>24);
			txpkt.payload[3] = (uint8_t)((temp<<16)>>24);
			txpkt.payload[4] = (uint8_t)(temp<<24)>>24;
			txpkt.payload[18] = 0xA5;
			
			SX1301TXData();	
			
			RXTXTimeCalibrationParameter.TimeCount ++;
			
			printf("[TimeCalibrationSend]:SendCount:%06d,NowTime:%010u,SendTime:%010u,SendDataLen:%03d,SendFreq:%010d,SendSF:%02d,SendCoderate:%02d,SendTOA:%010d,SendPreamb:%02d\r\n",
			RXTXTimeCalibrationParameter.TimeCount,
			SystemParameter.Timestamp,
			txpkt.count_us,
			txpkt.size,
			txpkt.freq_hz,
			txpkt.datarate,
			txpkt.coderate,
			lgw_time_on_air(&txpkt),
			txpkt.preamble);
		}				
	}	
	else
	{
		//printf("RXTXTimeCalibrationSingleSendFlag is 0\r\n");
	}
}

//void SX1301RxTxDebug(int nb_pkt)
void TimeCalibrationEachOtherRxTx(int nb_pkt)
{
	uint32_t temp;
	
	if(nb_pkt != 0)
	{		
		for(uint8_t i=0;i<nb_pkt;i++)
		{
			if(rxpkt[i].status == STAT_CRC_OK)
			{
				if(rxpkt[i].payload[0] == 0x5A && rxpkt[i].payload[18] == 0xA5)
				{					
					txpkt.freq_hz = rxpkt[i].freq_hz;
					//txpkt.tx_mode = IMMEDIATE;
					txpkt.tx_mode = TIMESTAMPED;
					
					txpkt.count_us = rxpkt[i].count_us + TimeCalibrationCycle;
					
					txpkt.rf_power = 10;
					txpkt.modulation = MOD_LORA;
					txpkt.bandwidth = BW_125KHZ;
					txpkt.datarate = rxpkt[i].datarate;
					txpkt.coderate = rxpkt[i].coderate;//CR_LORA_4_5
					//strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );l
					#if 0
					for(uint8_t j=0;j<rxpkt[i].size;++j)
					{
						txpkt.payload[j] = rxpkt[i].payload[j];
					}
					#endif
					
					TimeCalibrationDataTime = ((uint32_t)rxpkt[i].payload[1]<<24) + ((uint32_t)rxpkt[i].payload[2]<<16) + ((uint32_t)rxpkt[i].payload[3]<<8)+ (rxpkt[i].payload[4]);
					
					txpkt.size = 19;
					txpkt.preamble = 8;//需要在协议中指明
					txpkt.rf_chain = 0;
					
					//temp = txpkt.count_us + lgw_time_on_air(&txpkt);
					temp = txpkt.count_us;
					
					txpkt.payload[0] = 0x5A;	
					txpkt.payload[1] = (uint8_t)(temp >> 24);
					txpkt.payload[2] = (uint8_t)((temp<<8)>>24);
					txpkt.payload[3] = (uint8_t)((temp<<16)>>24);
					txpkt.payload[4] = (uint8_t)(temp<<24)>>24;
					txpkt.payload[18] = 0xA5;
					
					//txpkt.size = rxpkt[i].size;
					
					//TimeCalibrationCycle,
					//TimeCalibrationDataTime
					
					SX1301TXData();	
					SystemParameter.RxCnt++;
					//printf("count:%06d,T(Raceive):%010u,T(Send):%010u,T(Cycle):%010u,TOA:%010u,T(R-D):%010u,T(R-R):%010d,T(T-R):%010u\r\n",
					printf("count:%06d,T(Raceive):%010u,T(Send):%010u,T(DT):%010u,TOA:%010u,T(R-D):%011d,T(R-R):%010d,T(R-T):%010d,T(A-B B):%010d\r\n",\
					SystemParameter.RxCnt,\
					rxpkt[i].count_us,\
					txpkt.count_us,\
					TimeCalibrationDataTime,\
					lgw_time_on_air(&txpkt),\
					rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt),\
					rxpkt[i].count_us - TimeCalibrationRXLast - 2*lgw_time_on_air(&txpkt) - 2*TimeCalibrationCycle,\
					rxpkt[i].count_us - TimeCalibrationTXLast - 2*lgw_time_on_air(&txpkt) - TimeCalibrationCycle,\
					rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt) - LastTimeTimeCalibrationcha
					);
					
					TimeCalibrationRXLast = rxpkt[i].count_us;
					TimeCalibrationTXLast = txpkt.count_us;
					LastTimeTimeCalibrationcha = rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt);
				}		
				else
				{
					printf("Receive data is't need\r\n");
					printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", rxpkt[i].rssi,rxpkt[i].snr, rxpkt[i].snr_min, rxpkt[i].snr_max);
					for (uint8_t j = 0; j < rxpkt[i].size; ++j) 
					{
						printf(" %02X", rxpkt[i].payload[j]);
					}
					printf(" #\n");
				}
			}	
			else if(rxpkt[i].status == STAT_CRC_BAD)
			{
				printf("CRC bad\r\n");
				printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", rxpkt[i].rssi,rxpkt[i].snr, rxpkt[i].snr_min, rxpkt[i].snr_max);
				if(abs(rxpkt[i].count_us - TimeCalibrationRXLast-2*TimeCalibrationCycle)<1000 )
				{
					txpkt.freq_hz = rxpkt[i].freq_hz;
					//txpkt.tx_mode = IMMEDIATE;
					txpkt.tx_mode = TIMESTAMPED;
					
					txpkt.count_us = rxpkt[i].count_us + TimeCalibrationCycle;
					
					txpkt.rf_power = 30;
					txpkt.modulation = MOD_LORA;
					txpkt.bandwidth = BW_125KHZ;
					txpkt.datarate = rxpkt[i].datarate;
					txpkt.coderate = rxpkt[i].coderate;//CR_LORA_4_5
					//strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );l
					#if 0
					for(uint8_t j=0;j<rxpkt[i].size;++j)
					{
						txpkt.payload[j] = rxpkt[i].payload[j];
					}
					#endif
					
					TimeCalibrationDataTime = ((uint32_t)rxpkt[i].payload[1]<<24) + ((uint32_t)rxpkt[i].payload[2]<<16) + ((uint32_t)rxpkt[i].payload[3]<<8)+ (rxpkt[i].payload[4]);
					
					txpkt.size = 19;
					txpkt.preamble = 8;//需要在协议中指明
					txpkt.rf_chain = 0;
					
					//temp = txpkt.count_us + lgw_time_on_air(&txpkt);
					temp = txpkt.count_us;
					
					txpkt.payload[0] = 0x5A;	
					txpkt.payload[1] = (uint8_t)(temp >> 24);
					txpkt.payload[2] = (uint8_t)((temp<<8)>>24);
					txpkt.payload[3] = (uint8_t)((temp<<16)>>24);
					txpkt.payload[4] = (uint8_t)(temp<<24)>>24;
					txpkt.payload[18] = 0xA5;
					
					//txpkt.size = rxpkt[i].size;
					
					//TimeCalibrationCycle,
					//TimeCalibrationDataTime
					
					SX1301TXData();	
					SystemParameter.RxCnt++;
					//printf("count:%06d,T(Raceive):%010u,T(Send):%010u,T(Cycle):%010u,TOA:%010u,T(R-D):%010u,T(R-R):%010d,T(T-R):%010u\r\n",
					printf("count:%06d,T(Raceive):%010u,T(Send):%010u,T(DT):%010u,TOA:%010u,T(R-D):%011d,T(R-R):%010d,T(R-T):%010d,T(A-B B):%010d\r\n",\
					SystemParameter.RxCnt,\
					rxpkt[i].count_us,\
					txpkt.count_us,\
					TimeCalibrationDataTime,\
					lgw_time_on_air(&txpkt),\
					rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt),\
					rxpkt[i].count_us - TimeCalibrationRXLast - 2*lgw_time_on_air(&txpkt) - 2*TimeCalibrationCycle,\
					rxpkt[i].count_us - TimeCalibrationTXLast - 2*lgw_time_on_air(&txpkt) - TimeCalibrationCycle,\
					rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt) - LastTimeTimeCalibrationcha
					);
					
					TimeCalibrationRXLast = rxpkt[i].count_us;
					TimeCalibrationTXLast = txpkt.count_us;
					LastTimeTimeCalibrationcha = rxpkt[i].count_us - TimeCalibrationDataTime - lgw_time_on_air(&txpkt);
				}
				else				
				{
					printf("data time is't TimeCalibrationCycle\r\n");
				}
			}
			else
			{
				printf("no CRC Data\r\n");
			}
		}
	}
}


void SX1301RxTxDebug(int nb_pkt)
{
	uint32_t temp;
	
	if(nb_pkt != 0)
	{		
		for(uint8_t i=0;i<nb_pkt;i++)
		{
			if(rxpkt[i].status == STAT_CRC_OK)
			{
				txpkt.freq_hz = rxpkt[i].freq_hz;
				txpkt.tx_mode = IMMEDIATE;
				txpkt.rf_power = 30;
				txpkt.modulation = MOD_LORA;
				txpkt.bandwidth = BW_125KHZ;
				txpkt.datarate = rxpkt[i].datarate;
				txpkt.coderate = rxpkt[i].coderate;//CR_LORA_4_5
				
				for(uint8_t j=0;j<rxpkt[i].size;++j)
				{
					txpkt.payload[j] = rxpkt[i].payload[j];
				}			
				txpkt.size = rxpkt[i].size;
				txpkt.preamble = 8;//需要在协议中指明
				txpkt.rf_chain = 0;

				SX1301TXData();	
				SystemParameter.TXCnt++;
				printf("RxCnt:%06u,TxCnt:%06u,RxRssi:%3.1f,SNR:%3.1f,freq:%010d,size:%02d,Tx/Rx:%2.2f\r\n",\
				SystemParameter.RxCnt,\
				SystemParameter.TXCnt,\
				rxpkt[i].rssi,\
				rxpkt[i].snr,\
				rxpkt[i].freq_hz,\
				rxpkt[i].size,\
				(float)(SystemParameter.TXCnt/SystemParameter.RxCnt)
				);				
			}	
			else if(rxpkt[i].status == STAT_CRC_BAD)
			{
				printf("CRC bad\r\n");
				printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", rxpkt[i].rssi,rxpkt[i].snr, rxpkt[i].snr_min, rxpkt[i].snr_max);						
			}
			else
			{
				printf("no CRC Data\r\n");
			}
		}
	}
}

