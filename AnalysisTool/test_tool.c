#include "test_tool.h"
#include "protocol_analysis.h"
#include "stm32l0xx_hal.h"
#include "led.h"
#include "loragw_user.h"
#include "loragw_hal.h"
#include "loragw_reg.h"

uint8_t KeyValue;
uint16_t Tim1Count = 0;//定时器计数

uint8_t TestToolFinish = 0;


//测试模式
enum TestMod TestToolMod = TestToolIdelMod;

extern struct lgw_conf_rxrf_s *rfAp;
extern struct lgw_conf_rxrf_s *rfBp;
extern struct lgw_conf_rxif_s *prxifms;
extern struct SX1301Status_S SX1301Status;

extern uint8_t RXRadioAB;//0:A 1:B
extern float PerValue;//成功率
extern struct lgw_pkt_tx_s txpkt;
extern RFSystemParameter_TYPE SystemParameter;

extern uint32_t timecount;//测试接收数据包的时间 距离上一包数据的时间差总和
extern uint32_t RightCount;//收到正确数据包计数
extern uint32_t timeavg;//平均包间间隔

//工装按键LED初始化函数
void  TestToolKeyLedInit(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin : PB14 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 PC8 */
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB1 PB2 PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin :  PC0 PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_5, GPIO_PIN_RESET);	
	
	//set PC7 PC8 
	//HAL_NVIC_SetPriority(EXTI9_5_IRQn,1,1);	
	//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	//set PB14 PB15 
	//HAL_NVIC_SetPriority(EXTI15_10_IRQn,2,2);	
	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

//按键执行函数
void TestToolKeyImplement(void)
{	
	switch(TestToolMod)
	{
		case TestToolStartMod://启动完成灯亮/灭
			if(SX1301Status.StartSuccesFlag == StartSuccess ||SX1301Status.StartSuccesFlag == StartFail )
			{
				TestToolFinish |= TestToolStartFinish;
			}
			else
			{
				TestToolFinish &= ~TestToolStartFinish;
			}
			
			if(TestToolFinish & TestToolStartFinish)
			{
				if(SX1301Status.StartSuccesFlag == StartSuccess)
				{
					TestToolStartLedOn();
				}
				else
				{
					TestToolStartLedOff();
				}			
				TestToolMod = TestToolIdelMod;		
			}
			break;
		case TestToolSendMod:
			SX1301TXData();
			if(TestToolFinish & TestToolSendFinish)
			{
				TestToolSendLedOn();
				TestToolMod = TestToolIdelMod;
			}
			break;
		case TestToolRXAMod:
			if(SX1301Status.StartSuccesFlag == StartSuccess)
			{
				if((TestToolFinish & TestToolRXAFinish) == 0)
				{
					if(TestToolMod != TestToolRXAMod)
					{
						if(RXRadioAB == 0 && PerValue < 0.9)
						{
							TestToolRXALedOff();
						}
					}
					else
					{
						if(RXRadioAB == 0 && PerValue > 0.9)
						{
							TestToolRXALedOn();
							TestToolMod = TestToolIdelMod;
							TestToolFinish |= TestToolRXAFinish;
						}
					}
				}
				if((TestToolFinish & TestToolRXBFinish) == 0)
				{
					if(RXRadioAB == 1 && PerValue < 0.9)
					{
						TestToolRXBLedOff();
					}
				}
			}			
			break;
		case TestToolRXBMod:
			if(SX1301Status.StartSuccesFlag == StartSuccess)
			{
				if((TestToolFinish & TestToolRXBFinish) == 0)
				{
					if(TestToolMod != TestToolRXBMod)
					{
						if(RXRadioAB == 1 && PerValue < 0.9)
						{
							TestToolRXBLedOff();
						}
					}
					else
					{
						if(RXRadioAB == 1 && PerValue > 0.9)
						{
							TestToolRXBLedOn();
							TestToolMod = TestToolIdelMod;
							TestToolFinish |= TestToolRXBFinish;
						}
					}
				}
				if((TestToolFinish & TestToolRXAFinish) == 0)
				{
					if(RXRadioAB == 0 && PerValue < 0.9)
					{
						TestToolRXALedOff();
					}
				}
			}				
			break;
		case TestToolIdelMod:
			
			break;
	}
	
	//未启动时按了别的按键造成 StartLED异常
	if(TestToolMod != TestToolStartMod)
	{
		if(SX1301Status.StartSuccesFlag == StartFail||SX1301Status.StartSuccesFlag == NOStart)
		{
			TestToolStartLedOff();
		}
	}	
	if(SX1301Status.StartSuccesFlag == StartSuccess)
	{
		TestToolStartLedOn();
	}	
}

//IO口中断服务函数
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

//IO口中断服务函数
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

//按键中断回调函数
void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case StartKeyPIN:
			if(HAL_GPIO_ReadPin(StartKeyPort,StartKeyPIN)==GPIO_PIN_RESET)
			{
				HAL_Delay(50);
				if(HAL_GPIO_ReadPin(StartKeyPort,StartKeyPIN)==GPIO_PIN_RESET)
				{
					KeyValue = StartKeyValue;		
					TestToolFinish = 0;//清零测试完成项目	
					
					ParaInit(rfAp,rfBp,prxifms);
					SX1301Status.NeedStartFlag = 1;	
					SX1301Status.StartSuccesFlag = 0;
					
					TestToolMod = TestToolStartMod;
					
					TestToolStartLedOff();
					TestToolRXALedOff();
					TestToolRXBLedOff();
					TestToolSendLedOff();
				}				
			}
			else
			{
				KeyValue = NoKeyValue;
			}
			break;
		case SendKeyPIN:
			if(HAL_GPIO_ReadPin(SendKeyPort,SendKeyPIN)==0)
			{
				HAL_Delay(50);
				if(HAL_GPIO_ReadPin(SendKeyPort,SendKeyPIN)==0)
				{
					KeyValue = SendKeyValue;
					TestToolMod = TestToolSendMod;
					TestToolFinish &= ~TestToolSendFinish;
					
					txpkt.freq_hz = 475300000;
					txpkt.tx_mode = IMMEDIATE;
					txpkt.rf_power = 30;
					txpkt.modulation = MOD_LORA;
					txpkt.bandwidth = BW_125KHZ;
					txpkt.datarate = DR_LORA_SF7;
					txpkt.coderate = CR_LORA_4_5;
					//strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
					txpkt.size = 0;
					txpkt.preamble = 10;
					txpkt.rf_chain = 0;					
					
					SystemParameter.TxContinuous = 1;
					/* Overwrite settings */
					lgw_reg_w(LGW_TX_MODE, 1); /* Tx continuous */
					//lgw_reg_w(LGW_FSK_TX_GAUSSIAN_SELECT_BT, 2);
					
					/* Enable signal generator with DC */
					lgw_reg_w(LGW_SIG_GEN_FREQ, 0);
					lgw_reg_w(LGW_SIG_GEN_EN, 1);
					lgw_reg_w(LGW_TX_OFFSET_I, 0);
					lgw_reg_w(LGW_TX_OFFSET_Q, 0);
				}				
			}
			else
			{
				KeyValue = NoKeyValue;
			}			
			break;
		case RXAKeyPIN:
			if(HAL_GPIO_ReadPin(RXAKeyPort,RXAKeyPIN)==0)
			{
				HAL_Delay(50);
				if(HAL_GPIO_ReadPin(RXAKeyPort,RXAKeyPIN)==0)
				{
					KeyValue = RXAKeyValue;	
					TestToolMod = TestToolRXAMod;
					TestToolFinish &= ~TestToolRXAFinish;
					
					PerValue = 0;
					
					SystemParameter.TxContinuous = 0;//关闭
					lgw_reg_w(LGW_TX_MODE, 0); /* Tx continuous */
					
					timecount = 0;
					RightCount = 0;
					timeavg = 0;
				}				
			}
			else
			{
				KeyValue = NoKeyValue;
			}	
			break;
		case RXBKeyPIN:
			if(HAL_GPIO_ReadPin(RXBKeyPort,RXBKeyPIN)==0)
			{
				HAL_Delay(50);
				if(HAL_GPIO_ReadPin(RXBKeyPort,RXBKeyPIN)==0)
				{
					KeyValue = RXBKeyValue;	
					TestToolMod = TestToolRXBMod;
					TestToolFinish &= ~TestToolRXBFinish;
					
					PerValue = 0;
					
					SystemParameter.TxContinuous = 0;//关闭
					lgw_reg_w(LGW_TX_MODE, 0); /* Tx continuous */
					
					timecount = 0;
					RightCount = 0;
					timeavg = 0;
				}				
			}
			else
			{
				KeyValue = NoKeyValue;
			}	
			break;
	}	
	TestToolKeyImplement();
}
/*
//定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	Tim1Count++;
	
	if(Tim1Count % 10 ==0)
	{
		RunLedToggle();//系统运行灯
		TestToolRunLedToggle();//工装运行灯闪烁		
	}	
	if(Tim1Count % 5 ==0)
	{
		if(TestToolMod == TestToolStartMod)
		{
			TestToolStartLedToggle();
		}
		else if(TestToolMod == TestToolSendMod)
		{
			TestToolSendLedToggle();
		}
		else if(TestToolMod == TestToolRXAMod)
		{
			TestToolRXALedToggle();
		}
		else if(TestToolMod == TestToolRXBMod)
		{
			TestToolRXBLedToggle();
		}
		#if 0
		else
		{
			TestToolStartLedOff();
			TestToolRXALedOff();
			TestToolRXBLedOff();
			TestToolSendLedOff();
		}
		#endif
	}
}
*/


