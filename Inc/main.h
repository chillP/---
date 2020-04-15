/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define HIGHT_VER 0  // 是否为高版本

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY3_Pin GPIO_PIN_13
#define KEY3_GPIO_Port GPIOC
#define SX1301_EN_Pin GPIO_PIN_0
#define SX1301_EN_GPIO_Port GPIOH
#define BLE_CONN_IND_Pin GPIO_PIN_1
#define BLE_CONN_IND_GPIO_Port GPIOH
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOA
#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOA
#define SX1301_RST_Pin GPIO_PIN_0
#define SX1301_RST_GPIO_Port GPIOB
#define LORA_LED_Pin GPIO_PIN_1
#define LORA_LED_GPIO_Port GPIOB
#define BLE_LED_Pin GPIO_PIN_8
#define BLE_LED_GPIO_Port GPIOA
#define MODE_LED_Pin GPIO_PIN_8
#define MODE_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

enum SystemStateST{
	Begin = 0,
	AnalysisInit,
	AnalysisRun,
	AnalysisExit,
	ConfigToolInit,
	ConfigToolRun,
	ConfigToolExit,
	
};

struct SystemRunStateSt{
	enum SystemStateST state;
	uint8_t err;
};



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
