#ifndef __led_H
#define __led_H
#include "stm32l0xx_hal.h"

#define RunLedPort 		GPIOC
#define RunLedPin 		GPIO_PIN_10
#define BusyLedPort 		GPIOC
#define BusyLedPin 		GPIO_PIN_11
#define RFLedPort 		GPIOC
#define RFLedPin 		GPIO_PIN_12
#define UserLedPort 		GPIOD
#define UserLedPin 		GPIO_PIN_2

#define RunLedOn() 		HAL_GPIO_WritePin(RunLedPort, RunLedPin, GPIO_PIN_SET)
#define RunLedOff() 		HAL_GPIO_WritePin(RunLedPort, RunLedPin, GPIO_PIN_RESET)
#define RunLedToggle()	 HAL_GPIO_TogglePin(RunLedPort, RunLedPin)

#define UserDataBusyLedOn()       	 HAL_GPIO_WritePin(BusyLedPort, BusyLedPin, GPIO_PIN_SET)
#define UserDataBusyLedOff()      	HAL_GPIO_WritePin(BusyLedPort, BusyLedPin, GPIO_PIN_RESET)
#define UserDataBusyLedToggle()	 HAL_GPIO_TogglePin(BusyLedPort, BusyLedPin)

#define RFRXLedOn()       		 HAL_GPIO_WritePin(RFLedPort, RFLedPin, GPIO_PIN_SET)
#define RFRXLedOff()      		HAL_GPIO_WritePin(RFLedPort, RFLedPin, GPIO_PIN_RESET)
#define RFRXLedToggle()	 	HAL_GPIO_TogglePin(RFLedPort, RFLedPin)

#define RFTXLedOn()       	 HAL_GPIO_WritePin(UserLedPort, UserLedPin, GPIO_PIN_SET)
#define RFTXLedOff()      	HAL_GPIO_WritePin(UserLedPort, UserLedPin, GPIO_PIN_RESET)
#define RFTXLedToggle()	 HAL_GPIO_TogglePin(UserLedPort, UserLedPin)

void SystemRunLed(void);

#endif

