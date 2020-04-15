/**
  ******************************************************************************
  * File Name          : RTC.c
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

#include "loragw_user.h"

extern struct SX1301Status_S SX1301Status;
extern RFSystemParameter_TYPE SystemParameter;
/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void RTC_TimeShow(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	 /* Display time Format : hh:mm:ss */
	//if(stimestructureget.Minutes % RTCTimeDisplayTime==0 && stimestructureget.Seconds == 10)
	//if(stimestructureget.Minutes % RTCTimeDisplayTime==0)
	{
		printf("RTC Set Time:%02x-%02x-%02x %02x %02x:%02x:%02x\r\n",
		RTCInitBCDYear,
		RTCInitBCDMonth,
		RTCInitBCDDate,
		RTCInitBCDWeekDay,
		RTCInitBCDHours, 
		RTCInitBCDMinutes, 
		RTCInitBCDSeconds);
		printf("RTC Time now:%02d-%02d-%02d %02d %02d:%02d:%02d\r\n",
		sdatestructureget.Year,
		sdatestructureget.Month,
		sdatestructureget.Date,
		sdatestructureget.WeekDay,
		stimestructureget.Hours, 
		stimestructureget.Minutes, 
		stimestructureget.Seconds);
		/*
		if(SX1301Status.StartSuccesFlag==1)
		{
				lgw_reg_w(LGW_GPS_EN, 0);
				lgw_get_trigcnt(&SystemParameter.Timestamp);
				lgw_reg_w(LGW_GPS_EN, 1);	
				printf("LGW_TIMESTAMP:%010u\n",SystemParameter.Timestamp);
		}
		*/
		
//		printf("Run Time:%02d-%02d-%02d %02d %02d:%02d:%02d\r\n",
//		sdatestructureget.Year - RTCInitBCDYear,
//		sdatestructureget.Month - RTCInitBCDMonth,
//		sdatestructureget.Date - RTCInitBCDDate,
//		sdatestructureget.WeekDay - RTCInitBCDWeekDay,
//		stimestructureget.Hours - RTCInitBCDHours,
//		stimestructureget.Minutes - RTCInitBCDMinutes,
//		stimestructureget.Seconds - RTCInitBCDSeconds);
	}
  
} 

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
