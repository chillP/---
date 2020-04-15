#ifndef _test_tool_H
#define _test_tool_H
#include "stm32l0xx_hal.h"

//KEY
#define StartKeyPort GPIOC
#define StartKeyPIN GPIO_PIN_8

#define SendKeyPort GPIOC
#define SendKeyPIN GPIO_PIN_7

#define RXAKeyPort GPIOB
#define RXAKeyPIN GPIO_PIN_15

#define RXBKeyPort GPIOB
#define RXBKeyPIN GPIO_PIN_14

//LED 
#define StartPIN GPIO_PIN_12
#define StartPort GPIOB

#define SendPIN GPIO_PIN_0
#define SendPort GPIOC

#define RXRadioAPIN GPIO_PIN_1
#define RXRadioAPort GPIOB

#define RXRadioBPIN GPIO_PIN_2
#define RXRadioBPort GPIOB

#define RunPIN GPIO_PIN_5
#define RunPort GPIOC

//LED
#define TestToolStartLedOn() HAL_GPIO_WritePin(StartPort, StartPIN, GPIO_PIN_SET)
#define TestToolStartLedOff() HAL_GPIO_WritePin(StartPort, StartPIN, GPIO_PIN_RESET)
#define TestToolStartLedToggle() HAL_GPIO_TogglePin(StartPort, StartPIN)

#define TestToolRXALedOn() HAL_GPIO_WritePin(RXRadioAPort, RXRadioAPIN, GPIO_PIN_SET)
#define TestToolRXALedOff() HAL_GPIO_WritePin(RXRadioAPort, RXRadioAPIN, GPIO_PIN_RESET)
#define TestToolRXALedToggle() HAL_GPIO_TogglePin(RXRadioAPort, RXRadioAPIN)

#define TestToolRXBLedOn() HAL_GPIO_WritePin(RXRadioBPort, RXRadioBPIN, GPIO_PIN_SET)
#define TestToolRXBLedOff() HAL_GPIO_WritePin(RXRadioBPort, RXRadioBPIN, GPIO_PIN_RESET) 
#define TestToolRXBLedToggle() HAL_GPIO_TogglePin(RXRadioBPort, RXRadioBPIN)

#define TestToolSendLedOn() HAL_GPIO_WritePin(SendPort, SendPIN, GPIO_PIN_SET)
#define TestToolSendLedOff() HAL_GPIO_WritePin(SendPort, SendPIN, GPIO_PIN_RESET)
#define TestToolSendLedToggle() HAL_GPIO_TogglePin(SendPort, SendPIN)

#define TestToolRunLedOn() HAL_GPIO_WritePin(RunPort, RunPIN, GPIO_PIN_SET)
#define TestToolRunLedOff() HAL_GPIO_WritePin(RunPort, RunPIN, GPIO_PIN_RESET)
#define TestToolRunLedToggle() HAL_GPIO_TogglePin(RunPort, RunPIN)

//KEY value
#define NoKeyValue 0
#define StartKeyValue 1
#define RXAKeyValue 2
#define RXBKeyValue 3
#define SendKeyValue 4

enum TestMod{
	TestToolIdelMod,
	TestToolStartMod,
	TestToolSendMod,
	TestToolRXAMod,
	TestToolRXBMod
};

#define TestToolStartFinish 0x01
#define TestToolSendFinish 0x02
#define TestToolRXAFinish 0x04
#define TestToolRXBFinish 0x08


void  TestToolKeyLedInit(void);
void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void TestToolKeyImplement(void);
#endif

