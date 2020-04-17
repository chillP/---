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
extern	struct lgw_conf_rxif_s ifconfm[8];
extern	struct lgw_conf_rxrf_s rfconfA,rfconfB;

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
	
	SystemParameter.LoRaMAC = 1;	
	
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
	//启动成功开始接受
	if(SX1301Status.StartSuccesFlag == 0)
	{
		SX1301RXDataQueryPin();//使用这个需要开启SX1301的特殊功能引脚的指示
		DEBUG_Printf("Test mode runnning!!!\r\n");
		HAL_Delay(1000);
	}
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
	