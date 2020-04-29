#include "stdio.h"
#include "stdlib.h"
#include "GasSensorTest.h"
#include "usart.h"
#include "loragw_user.h"
#include "gpio.h"
#include "protocol_analysis.h"

struct lgw_pkt_tx_s tx_packet;
uint8_t powerDtu=0;  //DTU缺省发射功率0dBm

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
 * @brief   霍尼DTU模拟网关 - 测试模式初始化
 * @details SX1301配置及变量初始化
 * @param   无
 * @return  无
 */
void TestMode_Init(void)
{
	//参数初始化
	SystemParameter.LowDatarateOptmize = 0x60;
	SystemParameter.RxPreambleLen = 8;
	SystemParameter.ReceiveCRC = REC_CRC;
	SystemParameter.CalibrationFailFlag = 0;
	
	//清零结构体
	memset(&SX1301Status, 0x00, sizeof(SX1301Status));
	
	//使能1301
	SX1301_Enable(1);
	SX1301_ENABLE;
	HAL_Delay(20);	
	
	//1301参数配置
	rfconfA.enable = 1 ;   //使能radioA
	rfconfA.freq_hz = (uint32_t)(488.6*1e6);  //RadioA中心频点设置
	
	rfconfB.enable = 1 ;   //使能radioB
	rfconfB.freq_hz = (uint32_t)(489.4*1e6);  //RadioA中心频点设置
	
	//接收通道配置
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
	
	//发射参数配置
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
	
	//1301启动标志
	lgw_stop();
	SX1301Status.NeedStartFlag = 1;		
	SX1301Status.StartSuccesFlag = 0;//清零启动状态标记
	
}
	
/**
 * @brief   霍尼DTU模拟网关 - 透传模式初始化
 * @details SX1301配置及变量初始化
 * @param   无
 * @return  无
 */
void DebugMode_Init(void)
{
	
}

/**
 * @brief   霍尼DTU模拟网关 - 测试模式运行
 * @details 以测试指令响应节点的握手帧，配合DTU完成测试后输出其测试结果
 * @param   无
 * @return  无
 */
void FactoryTestMode_Run(void)
{
	//启动
	if(SX1301Status.NeedStartFlag ==1)
	{
		printf("SX1301 Starting...\r\n");
		SX1301Status.StartSuccesFlag = SX1301Start(rfAp,rfBp,prxifms);
		SX1301Status.NeedStartFlag =0;//清零启动标识
		if(SX1301Status.StartSuccesFlag == 0)
		{
			Sx1301StartFail();//IO指示
		}
		else
		{
			Sx1301StartSucces();//IO指示
		}		
	}
	
	//射频数据接受
	if(SX1301Status.StartSuccesFlag == 1)
	{
		SX1301RXDataQueryPin();//使用这个需要开启SX1301的特殊功能引脚的指示
		//DEBUG_Printf("Test mode runnning!!!\r\n");
		//HAL_Delay(1000);
	}
	
	//射频数据处理
	radioDataDecode();
	
	//串口指令处理
	serialDataDecode();
}

/**
 * @brief   查找数据包内的DEVEUI在信息动态存储区内的位置
 * @details 
 * @param   无
 * @return  无
 */
uint8_t infoSearch(uint8_t* payload)
{
	uint8_t i,j=0;
	
	for(i=0;i<20;i++)
	{
		for(j=0;j<8+1;j++)
		{
			if(j>=8)  //检索到指定DEVEUI
			{
				return i;  //返回DEVEUI的存储位置
			}
			
			if(payload[j+3] != dtuInfoRecordBuf[i].deveui[j])
			{
				break;
			}
		}
	}
	
	return 100;  //未检索到当前DEVEUI
}


/**
 * @brief   霍尼DTU模拟网关 - 透传调试运行
 * @details 以调试指令响应节点的握手帧，进入透传模式后，模拟网关与DTU间建立透传通道
 * @param   无
 * @return  无
 */
void FactoryDebugMode_Run(void)
{
	//启动
	if(SX1301Status.NeedStartFlag ==1)
	{
		printf("SX1301 Starting...\r\n");
		SX1301Status.StartSuccesFlag = SX1301Start(rfAp,rfBp,prxifms);
		SX1301Status.NeedStartFlag =0;//清零启动标识
		if(SX1301Status.StartSuccesFlag == 0)
		{
			Sx1301StartFail();//IO指示
		}
		else
		{
			Sx1301StartSucces();//IO指示
		}		
	}
	
	//握手包处理
	if(SX1301Status.StartSuccesFlag == 1)
	{
		SX1301RXDataQueryPin();//使用这个需要开启SX1301的特殊功能引脚的指示
		//DEBUG_Printf("Test mode runnning!!!\r\n");
		//HAL_Delay(1000);
	}
	
	//
	radioDataDecode();
	
	//串口指令处理
	serialDataDecode();	
}

/**
 * @brief   射频数据解析
 * @details 解析1301收到的数据，执行对应的处理
 * @param   无
 * @return  无
 */
void radioDataDecode(void)
{
	uint8_t testData[10]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
	uint8_t handShakeAcp[16]={0xC8,0xDF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0xB0,0xFB,0x00};  //2字节帧头+1字节控制码+8字节DEVEUI+...
	uint8_t downLink1Pkt[14]={0xC8,0xDF,0xF2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X00,0X00};  //2字节帧头+1字节控制码+8字节DEVEUI
	uint8_t downLink2Pkt[14]={0xC8,0xDF,0xF4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X00,0X00};  //2字节帧头+1字节控制码+8字节DEVEUI
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
		//帧校验
		if(rxpkt[pkt].payload[0]==0xE4 && rxpkt[pkt].payload[1]==0xFB)  //帧头
		{
			DEBUG_Printf("-get data: ");
			for (i = 0;i<rxpkt[pkt].size;i++) 
			{
				DEBUG_Printf("%02X ", rxpkt[pkt].payload[i]);
			}
			DEBUG_Printf("\r\n");
			
			switch (rxpkt[pkt].payload[2])
			{
				case 0x01:  //控制码：握手包
					
					for(i=0;i<21;i++)  //检索空闲存储区 
					{
						if(i>=20)  //无空闲存储区 
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
						//存储DEVEUI
						for(j=0;j<8;j++)  
						{
							dtuInfoRecordBuf[freeBuffNum].deveui[j] = rxpkt[pkt].payload[j+3];  
						}
						
						//存储信号质量
						//if(rxpkt[0].rssi < -127) rxpkt[0].rssi = -127;  //限幅
						//DEBUG_Printf("RSSI: %.2f SNR: %.2f\r\n",rxpkt[pkt].rssi,rxpkt[pkt].snr);
						//DEBUG_Printf("RSSI: %d SNR: %d\r\n",(int8_t)rxpkt[pkt].rssi, (int8_t)rxpkt[pkt].snr);
						//dtuInfoRecordBuf[i].rssiDTU = (int8_t)rxpkt[pkt].rssi;
						//dtuInfoRecordBuf[i].snrDTU = (int8_t)rxpkt[pkt].snr;
						
						//设定生命周期
						dtuInfoRecordBuf[i].lifeCycle = 5;  
			
						//handShakeAcp[11] = dtuInfoRecordBuf[i].rssiDTU ;  //信号质量
						//handShakeAcp[12] = dtuInfoRecordBuf[i].snrDTU ;
						
						//握手包发送
						for(j=0;j<8;j++)  //下行包EUI即上行包中EUI
						{
							handShakeAcp[j+3]=rxpkt[pkt].payload[j+3];
						}
			 			handShakeAcp[12] = powerDtu;  //DTU发射功率配置
						
						DEBUG_Printf("-send data: ");
						for(j=0;j<16;j++)  //TXPacket赋值
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
				case 0xF1:  //控制码：Uplink1
					//CS校验
				
					//DEVEUI检索
					infoBuffNum = infoSearch(rxpkt[pkt].payload);
					
					//信息处理
					if(infoBuffNum != 100)
					{
						dtuInfoRecordBuf[infoBuffNum].sensorTestResult = rxpkt[pkt].payload[13];  //保存串口测试结果
						dtuInfoRecordBuf[infoBuffNum].rssiGW = (int8_t)rxpkt[pkt].rssi;  //记录Uplink1信号质量
						dtuInfoRecordBuf[infoBuffNum].snrGW = (int8_t)rxpkt[pkt].snr;
					
						//Downlink1发送
						for(i=0;i<8;i++)  //写入DEVEUI
						{
							downLink1Pkt[i+3] = dtuInfoRecordBuf[infoBuffNum].deveui[i];
						}
						
						downLink1Pkt[11] = dtuInfoRecordBuf[infoBuffNum].rssiGW;  //写入Uplink1信号质量
						downLink1Pkt[12] = dtuInfoRecordBuf[infoBuffNum].snrGW;
						
						DEBUG_Printf("-send data: ");
						for(i=0;i<14;i++)  //TXPacket赋值
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
				case 0xF3:  //控制码：Uplink2
					//CS校验
				
					//DEVEUI检索
					infoBuffNum = infoSearch(rxpkt[pkt].payload);
					
					//信息处理
					if(infoBuffNum != 100)
					{
						dtuInfoRecordBuf[infoBuffNum].rssiDTU = rxpkt[pkt].payload[11];  //记录Downlink1接收的信号质量
						dtuInfoRecordBuf[infoBuffNum].snrDTU = rxpkt[pkt].payload[12];
						
						//Downlink2发送
						for(i=0;i<8;i++)  //写入DEVEUI
						{
							downLink2Pkt[i+3] = dtuInfoRecordBuf[infoBuffNum].deveui[i];
						}

						DEBUG_Printf("-send data: ");
						for(i=0;i<14;i++)  //TXPacket赋值
						{
							tx_packet.payload[i] = downLink2Pkt[i];
							DEBUG_Printf("%02X",tx_packet.payload[i]);
						}
						DEBUG_Printf("\r\n");
						tx_packet.size = 14;
						lgw_send(tx_packet);
						
						//测试结果输出
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
						
						if(dtuInfoRecordBuf[infoBuffNum].sensorTestResult == 0xff) DEBUG_Printf("&SERIAL:pass$\r\n");  //串口测试结果
						else printf("&SERIAL:fail$\r\n");
						
						if(txRssi>0) DEBUG_Printf("&TX-RSSI:+%d$\r\n",txRssi);  //信号质量输出
						else DEBUG_Printf("&TX-RSSI:%d$\r\n",txRssi);
						if(rxRssi>0) DEBUG_Printf("&RX-RSSI:+%d$\r\n",rxRssi);
						else DEBUG_Printf("&TX-RSSI:%d$\r\n",rxRssi);
						if(txSnr>0) DEBUG_Printf("&TX-SNR:+%d$\r\n",txSnr);
						else DEBUG_Printf("&TX-RSSI:%d$\r\n",txSnr);
						if(rxSnr>0) DEBUG_Printf("&RX-SNR:+%d$\r\n",rxSnr);
						else DEBUG_Printf("&TX-RSSI:%d$\r\n\r\n",rxSnr);					
						
						//缓存清理
						dtuInfoRecordBuf[infoBuffNum].lifeCycle = 0;  //完成测试后清除生命周期								
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
 * @brief   串口数据解析
 * @details 解析上位机串口的AT指令，执行对应的处理
 * @param   无
 * @return  无
 */
void serialDataDecode(void)
{
	
}
	