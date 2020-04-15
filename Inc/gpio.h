/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
	 
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
	 
#define MODE_LEDSET(enable)do{if(enable == 1){HAL_GPIO_WritePin(MODE_LED_GPIO_Port, MODE_LED_Pin, GPIO_PIN_RESET);}else {HAL_GPIO_WritePin(MODE_LED_GPIO_Port, MODE_LED_Pin, GPIO_PIN_SET);}}while(0)
#define BLE_LEDSET(enable)do{if(enable == 1){HAL_GPIO_WritePin(BLE_LED_GPIO_Port, BLE_LED_Pin, GPIO_PIN_RESET);}else {HAL_GPIO_WritePin(BLE_LED_GPIO_Port, BLE_LED_Pin, GPIO_PIN_SET);}}while(0)
#define LORA_LEDSET(enable)do{if(enable == 1){HAL_GPIO_WritePin(LORA_LED_GPIO_Port, LORA_LED_Pin, GPIO_PIN_RESET);}else {HAL_GPIO_WritePin(LORA_LED_GPIO_Port, LORA_LED_Pin, GPIO_PIN_SET);}}while(0)

#define SX1301_ENABLE HAL_GPIO_WritePin(SX1301_RST_GPIO_Port, SX1301_RST_Pin, GPIO_PIN_RESET)


#define KEYA        HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)  //KEY0°´¼üPE4

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
	 
void SX1301_Enable(uint8_t enble);

void LED_ALLSet(uint8_t Enable);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
