#ifndef __CT_FUNTION_H
#define __CT_FUNTION_H

#include "stdint.h"
#include "loragw_hal.h"

#define FREAM_ERROR 2
#define CHECK_ERROR 3
#define FREAM_LEN_ERROR 4
#define true 1
	
#define CT_INFO_CODE 0X11
#define GW_CODE 0X12



enum MCU_CID{
    restart,
    set_success,
    un_start,
		start_error,
		send_error,
		command_error,
		send_success,

};

//发送给MCU的命令
enum Send_MCU_CID{
    SMC_search_gw,
    SMC_start_up,
	  SMC_connect,
};

struct SX1301_ConfPara {
    uint8_t   txdatarate;        //配置工具发送速率
    uint32_t  freq_start;        //发送频点
		uint8_t   power;             //发送功率
};

uint8_t CT_ACK_Pack(uint8_t CID, uint8_t err_code);

int Comand_Pack(uint8_t * send_buf, uint8_t * command_data,uint8_t len);

int Send_PC_Pack(struct lgw_pkt_rx_s rxpkt, uint8_t * Send_to_PC);

int Command_Inspect(uint8_t *recData,uint16_t len);

int GW_Command_Split(uint8_t *PlayLoad);

int CT_Info_Para(uint8_t *PlayLoad, struct SX1301_ConfPara * SX1301_ConfPara);

#endif