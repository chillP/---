/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Host specific functions to address the LoRa concentrator registers through
    a SPI interface.
    Single-byte read/write and burst read/write.
    Does not handle pagination.
    Could be used with multiple SPI ports in parallel (explicit file descriptor)

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include "loragw_spi.h"
#include "config.h"    /* library configuration options (dynamically generated) */
#include <stdio.h>        /* printf fprintf */
#include <stdlib.h>        /* malloc free */
#include "stm32l0xx_hal.h"
#include "loragw_hal.h"


#define LoRaGW_NSSPIN_UP()        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define LoRaGW_NSSPIN_DOWN()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
//#define LoRaGW_NSSPIN_UP()        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
//#define LoRaGW_NSSPIN_DOWN()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi1;

//uint8_t spi_mux_mode = LGW_SPI_MUX_MODE0;
//uint8_t Sx1301Target;
//void *spi_target = &Sx1301Target;


extern UART_HandleTypeDef huart1;

uint8_t Spi1TxBuf1[SPI1BUFFZIZE];
uint8_t Spi1RxBuf1[SPI1BUFFZIZE];

uint8_t dataout[BURST_TEST_SIZE];
uint8_t datain[BURST_TEST_SIZE];

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_SPI == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_SPI_ERROR;}
#endif
		
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
		
int lgw_spi_open(void **spi_target_ptr)
{
	return LGW_SPI_SUCCESS;
}

int lgw_spi_close(void *spi_target)
{
	return LGW_SPI_SUCCESS;
}

void Sx1301SpiInit(void)
{	
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	//hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		//Error_Handler();
	}
	
	__HAL_SPI_ENABLE(&hspi1);

}

void Sx1301Reset(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct;
	#if 0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(100);//>1ns
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(100);	//?
	#endif
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(100);//>1ns
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(100);	//?
	
	#if 0
	/*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;	
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;	
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	#endif
}
/*******************************************************************************
** 功能：     SPI2发送、接收一字节数据
** 参数：     data:要发送的数据
** 返回值：	  接收到的数据
** 注意：       
** 修改日志： 
*******************************************************************************/
uint8_t SPI1_ReadWriteByte(uint8_t data)
{
    while((SPI1->SR&SPI_SR_TXE) != SPI_SR_TXE);//等待发送寄存器空
    SPI1->DR = data;
    while((SPI1->SR&SPI_SR_RXNE) != SPI_SR_RXNE);//等待接收寄存器非空
    return SPI1->DR;
}

/* Simple write */
int lgw_spi_w(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t data) {
    
	uint8_t out_buf[3];
	uint8_t command_size;
	//int ret;
	
	/* check input variables */
    //CHECK_NULL(spi_target);
    if ((address & 0x80) != 0) {
        DEBUG_MSG("WARNING: SPI address > 127\n");
    }

	/* prepare frame to be sent */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        out_buf[0] = spi_mux_target;
        out_buf[1] = WRITE_ACCESS | (address & 0x7F);
        out_buf[2] = data;
        command_size = 3;
    } else {
        out_buf[0] = WRITE_ACCESS | (address & 0x7F);
        out_buf[1] = data;
        command_size = 2;
    }

	/*SPI发送*/
	LoRaGW_NSSPIN_DOWN();
	for(uint8_t i=0;i<command_size;i++)
	{
		//ret=SPI1_ReadWriteByte(out_buf[i]);
		SPI1_ReadWriteByte(out_buf[i]);
	}	
	LoRaGW_NSSPIN_UP();

	/*返回值*/
	
	return LGW_SPI_SUCCESS;
	
	#if 0
	if(ret==HAL_OK)
	{
		return LGW_SPI_SUCCESS;
	}
	else
	{
		return LGW_SPI_ERROR;
	}	
	#endif
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_spi_r(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data) {
    

	uint8_t out_buf[3];
	uint8_t in_buf[ARRAY_SIZE(out_buf)];
    uint8_t command_size;
	int ret;

	/* check input variables */
    //CHECK_NULL(spi_target);
    if ((address & 0x80) != 0) {
        DEBUG_MSG("WARNING: SPI address > 127\n");
    }

	 /* prepare frame to be sent */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        out_buf[0] = spi_mux_target;
        out_buf[1] = READ_ACCESS | (address & 0x7F);
        out_buf[2] = 0x00;
        command_size = 3;
    } else {
        out_buf[0] = READ_ACCESS | (address & 0x7F);
        out_buf[1] = 0x00;
        command_size = 2;
    }
	
	/*发送数据*/
	
	LoRaGW_NSSPIN_DOWN();
	for(uint8_t i=0;i<command_size;i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, &out_buf[i], &in_buf[i], 1, 5);
		//必须读取处第2位数据
		//ret	=SPI1_ReadWriteByte(out_buf[i]);
		//data[i]	=SPI1_ReadWriteByte(out_buf[i]);
	}
	LoRaGW_NSSPIN_UP();

	*data = in_buf[1];
	//*data = ret;
	
	return LGW_SPI_SUCCESS;
	
	/*返回值*/
	#if 0
	if(ret==HAL_OK)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}	
	#endif
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_spi_wb(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data, uint16_t size) {
    
    uint8_t command[2];
    uint8_t command_size;  
    //int ret;
		int size_to_do, chunk_size;
		//int offset;
    int byte_transfered = 0;

    /* check input parameters */
    //CHECK_NULL(spi_target);
    if ((address & 0x80) != 0) {
        DEBUG_MSG("WARNING: SPI address > 127\n");
    }
    //CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }   

    /* prepare command byte */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        command[0] = spi_mux_target;
        command[1] = WRITE_ACCESS | (address & 0x7F);
        command_size = 2;
    } else {
        command[0] = WRITE_ACCESS | (address & 0x7F);
        command_size = 1;
    } 

	/*SPI发送*/
	LoRaGW_NSSPIN_DOWN();
		
	for(uint8_t i=0;i<command_size;i++)
	{
		//ret	=SPI1_ReadWriteByte(command[i]);	
		SPI1_ReadWriteByte(command[i]);
	}
	size_to_do = size;
	for (int j=0; size_to_do > 0;j++) 
	{
		chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;//单次发送字节数小于1024
		//offset = j * LGW_BURST_CHUNK;//发送位置偏移
		//txcount1 = (unsigned long)(data + offset);//目前的发送指针位置		
			
		for(uint16_t i=0;i<chunk_size;i++)
		{
			//ret	=SPI1_ReadWriteByte(*(data+byte_transfered + i));
			SPI1_ReadWriteByte(*(data+byte_transfered + i));
		}
		//HAL_UART_Transmit(&huart1,command,command_size,1000);
		//HAL_UART_Transmit(&huart1,p+offset,chunk_size,1000);
		byte_transfered += chunk_size;
		//DEBUG_PRINTF("BURST WRITE: to trans %d # chunk %d # transferred %d \n", size_to_do, chunk_size, byte_transfered);
		size_to_do -= chunk_size; /* subtract the quantity of data already transferred */
	}	
	
	LoRaGW_NSSPIN_UP();
	
	return LGW_SPI_SUCCESS;
	
	#if 0
	LoRaGW_NSSPIN_UP();
	
    /* determine return code */
    if (ret != HAL_OK) {
        DEBUG_MSG("ERROR: SPI BURST WRITE FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI burst write success\n");
        return LGW_SPI_SUCCESS;
    }
    #endif
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_spi_rb(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data, uint16_t size) {
//    int spi_device;
    uint8_t command[2];
    uint8_t command_size;
//	uint8_t temp;
//    int ret;
		int size_to_do, chunk_size;
    int byte_transfered = 0;
	
    /* check input parameters */
    //CHECK_NULL(spi_target);
    if ((address & 0x80) != 0) {
        DEBUG_MSG("WARNING: SPI address > 127\n");
    }
    //CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }

    /* prepare command byte */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        command[0] = spi_mux_target;
        command[1] = READ_ACCESS | (address & 0x7F);
        command_size = 2;
    } else {
        command[0] = READ_ACCESS | (address & 0x7F);
        command_size = 1;
    }
    
	LoRaGW_NSSPIN_DOWN();
	//HAL_Delay(1);
	for(uint8_t i=0;i<command_size;i++)
	{
		//ret	=SPI1_ReadWriteByte(command[i]);		
		SPI1_ReadWriteByte(command[i]);
	}
//	HAL_Delay(1);
	size_to_do = size;
	for (int j=0; size_to_do > 0;j++) 
	{
		chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;//单次接收字节数小于等于1024
		//offset = j * LGW_BURST_CHUNK;//发接收位置偏移
		//txcount1 = (unsigned long)(data + offset);//目前的发送指针位置
		for(uint16_t i=0;i<chunk_size;i++)
		{
			*(data+byte_transfered + i)=SPI1_ReadWriteByte(0xff);
			//HAL_UART_Transmit(&huart1,data+byte_transfered + i,1,10);
		}
		//HAL_UART_Transmit(&huart1,command,command_size,1000);
		//HAL_UART_Transmit(&huart1,p+offset,chunk_size,1000);
		byte_transfered += chunk_size;
		//DEBUG_PRINTF("BURST WRITE: to trans %d # chunk %d # transferred %d \n", size_to_do, chunk_size, byte_transfered);
		size_to_do -= chunk_size; /* subtract the quantity of data already transferred */
	}
	
	LoRaGW_NSSPIN_UP();
		
	return LGW_SPI_SUCCESS;
	
    /* determine return code */
    #if 0
    if (ret != HAL_OK) {
        DEBUG_MSG("ERROR: SPI BURST READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI burst read success\n");
        return LGW_SPI_SUCCESS;
    }
    #endif
}

/* --- EOF ------------------------------------------------------------------ */
