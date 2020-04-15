/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
	 
#include "stdio.h"
#include "string.h"
	 
#define RECEIVELEN 1024 
#define USART_DMA_SENDING 1		//发送未完成 
#define USART_DMA_SENDOVER 0	//发送完成
	 
typedef struct  
{  
	uint8_t receive_flag;				//空闲接收标记 
	uint8_t dmaSend_flag;				//发送完成标记
	uint16_t rx_len;						//接收长度  
	uint8_t RX_Buf[RECEIVELEN];	//DMA接收缓存  
}USART_RECEIVETYPE;

typedef struct  
{  
	uint8_t NowBaudRate;//当前波特率 0-9600 1-115200 2-460600 3-921600
	uint8_t LastBaudRate;//上一次波特率
}USART_INFO;

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void Usart2SendData_DMA(uint8_t *pdata, uint16_t Length);
void Usart2Receive_IDLE(void);
void Clear_UART2_IT(void);
void DEBUG_Printf(char *fmt, ...);
void Usart2SendData_DMA(uint8_t *pdata, uint16_t Length);
int fputc(int ch,FILE *f);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
