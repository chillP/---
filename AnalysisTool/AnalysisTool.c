/*
Description: 该文件为解析工具运行的本体，
						 包括解析工具的初始化和运行。
*/

#include "AnalysisTool.h"
#include "loragw_user.h"
#include "softwareversion.h"
#include "rtc.h"
#include "gpio.h"
#include "PC_Seting.h"

extern USART_RECEIVETYPE Usart2_RX;
extern RFSystemParameter_TYPE SystemParameter;
extern FRAME_RECEIVE_TYPE *pFrame;//接收到用户数据帧的属性
extern struct ProtocolRespond_S *pMCUTXData;//回复数据

extern struct SX1301Status_S SX1301Status;
extern struct lgw_conf_rxrf_s *rfAp;
extern struct lgw_conf_rxrf_s *rfBp;
extern struct lgw_conf_rxif_s *prxifms;

// 上位机连接标志
uint8_t AT_ConnectUpper;

// 解析工具初始化
void AnalysisTool_init(void){
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
	
	SoftwareVersionShow();//软件版本
	RTC_TimeShow();//RTC时间
	
	AT_ConnectUpper = 0;
	
}


// 解析工具运行
void AnalysisTool_Run(void){
	
	
	
	//启动
	if(SX1301Status.NeedStartFlag ==1)
	{
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
	//启动成功开始接受
	if(SX1301Status.StartSuccesFlag == 1)
	{
			SX1301RXDataQueryPin();//使用这个需要开启SX1301的特殊功能引脚的指示
	}
	//串口接收命令
	if(Usart2_RX.receive_flag == 1)
	{
		if(Usart2_RX.RX_Buf[0] == 0xFE){
			//__HAL_UART_DISABLE_IT(&huart2, Usart2_RX);//先关闭接收，处理完再打开		
			ProtocolAnalysis(Usart2_RX.RX_Buf,Usart2_RX.rx_len,pFrame,pMCUTXData);
			//__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
			
			// 配置设备命令
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

