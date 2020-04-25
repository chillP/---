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
	uint8_t deveui[8];  //DTU设备号记录
	float rssiDTU;  //节点RSSI
	float rssiGW;  //网关侧RSSI
	float snrDTU;  //节点侧SNR
	float snrGW;  //网关侧SNR
	uint8_t lifeCycle  //数据生命周期
}DTU_INFO_TYPE;

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
	SystemParameter.RxPreambleLen = 15;
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
 * @brief   霍尼DTU模拟网关 - 透传调试运行
 * @details 以调试指令响应节点的握手帧，进入透传模式后，模拟网关与DTU间建立透传通道
 * @param   无
 * @return  无
 */
void FactoryDebugMode_Run(void)
{
	
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
	uint8_t handShakeAcp[12]={0xC8,0xDF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  //2字节帧头+1字节控制码+8字节DEVEUI
	uint8_t frameFaultFlag=0;
	uint8_t send_access_flag=1;
	uint8_t logLevel = 2;
	bool frameCheckOk = true;
	uint8_t i,j=0;
	
	struct lgw_pkt_tx_s tx_packet;
	
	if(nb_pkt!=0)
	{
		//帧校验
		if(rxpkt[0].payload[0]==0xE4 && rxpkt[0].payload[1]==0xFB)  //帧头
		{
			if(rxpkt[0].payload[2]==0x01)  //控制码：握手包
			{
				//信息登记
				for(i=0;i<20;i++)  //DTU信息活动存储区
				{
					if(dtuInfoRecordBuf[j].deveui != NULL)  //检索空闲存储区 #如何确保deveui存储的唯一性？
					{
						for(j=0;j<8;j++)  //存储DEVEUI
						{
							dtuInfoRecordBuf[j].deveui[j] = rxpkt[0].payload[i+3];  
						}
						dtuInfoRecordBuf[j].lifeCycle = 20;  //设定生命周期
					}
				}
				//握手包发送
				for(i=0;i<8;i++)  //下行包EUI即上行包中EUI
				{
					handShakeAcp[i+3]=rxpkt[0].payload[i+3];
				}
				for(i=0;i<12;i++)  //TXPacket赋值
				{
					tx_packet.payload[i] = handShakeAcp[i];
				}
				lgw_send(tx_packet);
			}
			else if(rxpkt[0].payload[2]==0xF1)  //控制码：测试包1
			{
				
			}
			else if(rxpkt[0].payload[2]==0xF3)  //控制码：测试包3
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
		
		
		
		
		
		
		
		
		
		
		//DEVEUI记录
		for(uint8_t i=0;i<9;i++)
		{
			//if(rxpkt[0].payload[i] != testData[i])
	
		}
		
		//数据包发送
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
 * @brief   串口数据解析
 * @details 解析上位机串口的AT指令，执行对应的处理
 * @param   无
 * @return  无
 */
void serialDataDecode(void)
{
	
}
	