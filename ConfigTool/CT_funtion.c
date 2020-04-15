#include "CT_funtion.h"
#include "usart.h"
#include <string.h>
#include "loragw_hal.h"

#include <time.h> 

uint16_t FCNT = 0;
uint8_t SEND_RFU = 0;
uint8_t SEND_RAND = 0;


#define debug 1

uint8_t CT_EUI[8] = {0x11,0x22,0x33,0x44,0x11,0x22,0x33,0x44};

enum MCU_CID MCU_CID;

//打包命令
int Comand_Pack(uint8_t * send_buf, uint8_t * command_data,uint8_t len)
{
	uint8_t i,add = 0;
	//uint8_t send_buf[256] = "";
	uint16_t com_index = 0;
	
  SEND_RAND = (rand()%255);
	
	send_buf[0] = 0xE5;
	send_buf[1] = 0xFC;
	send_buf[2] = 0x29;
	
	com_index +=3;
	
	//CTEUI
	for(i = 0; i<8; i++){
		send_buf[com_index + i] = CT_EUI[i];
	}
	
	com_index += 8;
	
	for(i = 0; i<len; i++){
		send_buf[com_index + i] = command_data[i];
	}
	
	com_index += len;
	
	//RFU
	send_buf[com_index] = SEND_RFU;
	com_index += 1;
	
	//RAND
	send_buf[com_index] = SEND_RAND;
	com_index += 1;
	
	//FCNT
	FCNT++;
	//FCNT += SEND_RAND;
	send_buf[com_index] = (FCNT & 0xff);
	com_index += 1;
	send_buf[com_index] = (FCNT & 0xff00) >> 8;
	com_index += 1;
	
	//E5 FC 29 [内容相加] CS 18
	//CS
	for(i = 0; i< com_index-3; i++){
			add += send_buf[i+3];
	}
	
	send_buf[com_index] = add;
	
	com_index += 1;
	send_buf[com_index] = 0X18;
	
	return com_index;
}

//命令合法性
int Command_Inspect(uint8_t *recData,uint16_t len)
{
	uint16_t i;
	uint8_t add = 0;
	
		
		for(i = 0; i< len; i++){
			add += recData[i];
		}

		if(add == recData[len])
		{
			
			return 1;
			
		}else{
			
			//printf("check error add:%x rec:%x",add,recData[len]);
			
			return -1;
		}

}


// MCU ACK命令打包
uint8_t CT_ACK_Pack(uint8_t CID, uint8_t err_code)
{
	static uint8_t Ack_Pack_buf[6] = "";
	uint8_t add = 0 , i = 0;
	add =0;
	i = 0;
	
	memset(Ack_Pack_buf,0,6);
	// 加上帧头
	Ack_Pack_buf[0] = 0xF5;
	Ack_Pack_buf[1] = 0xFF;
	Ack_Pack_buf[2] = CID;
	Ack_Pack_buf[3] = err_code;
	
	
	for(i = 0; i<3; i++)
	{
		add += Ack_Pack_buf[i + 1];
	}
	
	
	Ack_Pack_buf[4] = add;
	
	
	Ack_Pack_buf[5] = 0x5F;
	
	
	//HAL_Delay(10);
	//HAL_UART_Transmit_DMA(&huart1, Ack_Pack_buf, 6);
	Usart2SendData(Ack_Pack_buf, 6);
	
}


// 网关命令解析
int GW_Command_Split(uint8_t *PlayLoad)
{

    if(PlayLoad[0] == 0xA5)
		{
			return CT_INFO_CODE;
		}else if(PlayLoad[0] == 0xE5 && PlayLoad[1] == 0xFC && PlayLoad[2] == 0x29){
			CT_EUI[0] = PlayLoad[3];
			CT_EUI[1] = PlayLoad[4];
			CT_EUI[2] = PlayLoad[5];
			CT_EUI[3] = PlayLoad[6];
			CT_EUI[4] = PlayLoad[7];
			CT_EUI[5] = PlayLoad[8];
			CT_EUI[6] = PlayLoad[9];
			CT_EUI[7] = PlayLoad[10];
			
			return GW_CODE;
		}

    return -1;
}

// 发送到上位机数据打包
int Send_PC_Pack(struct lgw_pkt_rx_s rxpkt, uint8_t * Send_to_PC)
{
	int i;
	int index = 0;
	uint8_t add = 0;
	char temp[4];
	
	
	for(i = 0; i<rxpkt.size; i++)
	{
		Send_to_PC[i] = rxpkt.payload[i];
	}
	
	index = rxpkt.size;
	
	sprintf(temp, "%4.0f", rxpkt.rssi);
	
	//printf("RSSI:%s ", temp);
	
	for(i = 0; i< 4; i++)
	{
		Send_to_PC[index++] = temp[i];
	}
	
	sprintf(temp, "%3.0f", rxpkt.snr);
	//printf("SNR:%s\n", temp);
	
	for(i = 0; i< 3; i++)
	{
		Send_to_PC[index++] = temp[i];
	}
	//printf("add: ");
	for(i = 0; i< (index - 3); i++)
	{
			//printf("%02x ",Send_to_PC[i+3]);
			add += Send_to_PC[i+3];
	}
	
	Send_to_PC[index++] = add;
	
	return index;
}


// 设置配置工具参数
int CT_Info_Para(uint8_t *PlayLoad, struct SX1301_ConfPara * SX1301_ConfPara)
{
	memset(SX1301_ConfPara, 0x00, sizeof(SX1301_ConfPara));
	
	int i;
	SX1301_ConfPara->freq_start  = PlayLoad[2] << 24;
	SX1301_ConfPara->freq_start += PlayLoad[3] << 16;
	SX1301_ConfPara->freq_start += PlayLoad[4] << 8;
	SX1301_ConfPara->freq_start += PlayLoad[5];
	
	
	
	SX1301_ConfPara->power = PlayLoad[6];
	
	SX1301_ConfPara->txdatarate = PlayLoad[7];
	
	CT_EUI[0] = PlayLoad[8];
	CT_EUI[1] = PlayLoad[9];
	CT_EUI[2] = PlayLoad[10];
	CT_EUI[3] = PlayLoad[11];
	CT_EUI[4] = PlayLoad[12];
	CT_EUI[5] = PlayLoad[13];
	CT_EUI[6] = PlayLoad[14];
	CT_EUI[7] = PlayLoad[15];
	
	/*
	printf("freq_start:%d\n",SX1301_ConfPara->freq_start);
	printf("power:%d\n",SX1301_ConfPara->power);
	printf("txdatarate:%d\n",SX1301_ConfPara->txdatarate);
	*/
	
	
}