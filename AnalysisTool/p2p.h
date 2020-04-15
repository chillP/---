#ifndef __P2P_H
#define __P2P_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "aes.h"
#include "loragw_hal.h"
#include "loragw_user.h"


// �㲥����
#define Radio_COMMAND 		0x10  

// �����������
#define FREAM_ERROR 		-1
#define CHECK_ERROR 		-2
#define FREAM_LEN_ERROR 	-3
#define CT_GW_EUI_ERR 		-4
#define GW_COMMAND_LEN_ERR 	-5
#define DATA_COMMAND_EMPLY  -6
#define MSGTYPE_ERROR 		-7
#define CT_RECFCNT_ERR		-8

// ���ô���
#define CF_PARA_ERR			-0x01

//Emtyp test
#define RF_TestReset  		0x60
#define RF_RadioTest  		0x61

// ����EUI����
#define CT_GW_EUI_LEN 		8

//ACK֡ͷ
#define RF_ACK_Head 		0xAC

#define RFU_DEFAULT			0x66


// ������
#define GW_CTRL_CH          0x01  //�������ù���
#define GW_CTRL_ERR         0x02  //�����쳣�ϱ�
#define GW_CTRL_PRO         0x03  //��������
#define GW_CTRL_READ        0x04  //ֻ��Ȩ�� ���ù���->����
#define GW_CTRL_DEF         0x05  //��ͨĬ�� ���ù���->����
#define GW_CTRL_TEST        0x06  //�տڲ���

#define GW_CTRL_ADMIN       0x0F  //�����û� ���ù���->����

// ��Ϣ����
#define GW_MSG_REC          0x00  //���ù���->���� / �쳣����->�������� / ���Խڵ�->��������
#define GW_MSG_SEND         0x0F  //����->���ù��� / ��������->�쳣���� / ��������->���Խڵ�
#define GW_MSG_CON          0x03  //�������� ���ù���->���� ����FCNT
#define GW_MSG_DISFOREVER   0x04  //�ָ�д�� ���ù���->����
#define GW_MSG_SET_PMI		0x05  //����Ȩ�� ���ù���->����       �Զ���CTRL��Ȩ��д�뵽join
#define GW_MSG_PAR_SET      0x06  //�������� ���ù���->����
#define GW_MSG_CT_RADIO     0x07  //���߹㲥 ���ù���->����
#define GW_MSG_GW_RADIO     0x08  //���ܹ㲥 ����->���ù���
#define GW_MSG_Forever      0x0E  //����д�� ���ù���->����

//ACK֡ͷ
#define RF_ACK_Head 		0xAC

#define RFU_DEFAULT			0x66

#define ACK_Restarting		0xEE  //����

#define ACK_SUCCESS 		0XFF


typedef struct p2p_param_ST{
	int CT_Send_power;
	uint32_t CT_Send_freq_hz;
	uint16_t CT_Send_datarate;
	uint8_t CT_GW_EUI[CT_GW_EUI_LEN];
}p2p_param_ST;


struct Control_Code{
	unsigned CTRL:4;
	unsigned MSGTYPE:4;
};

struct Config_Frame{
	uint8_t Preamble;
	uint8_t Frame_head[2];
	uint8_t CTEUI[8];
	uint8_t GWEUI[8];
	struct Control_Code Control_Code;
	uint8_t DLEN;
	uint8_t DATA[236];
	uint8_t RFU;
	uint8_t RAND;
	uint16_t FCNT;
	uint8_t CS;
	uint8_t Frame_Tail;
	
};

struct Test_set{
	unsigned TestSF:3;
	unsigned TestPower:5;
};

struct TestResetReq{
	struct Test_set Test_set;
	uint8_t RadioMode;
	uint32_t TestFreq;
	uint32_t TestInterval;
};

struct RadioTestReq{
	uint8_t DataLen;
	uint16_t Fcntup;
	uint8_t Payload[217];
	uint8_t RSSI;
	uint8_t SNR;
};

int p2p_init();
int CT_Command_Parsing(struct lgw_pkt_rx_s * lgw_pkt_rx_s);

int CT_Command_AES_Decrypt(uint8_t * PlayLoad, uint8_t * command_buf, uint16_t len, struct Config_Frame * config_frame);
int CT_Command_Inspect(uint8_t *PlayLoad,uint16_t len);
int CT_Command_Split(uint8_t *PlayLoad, struct Config_Frame * config_frame);
int CT_Aes_Encrypt_Pack(char * data, char * send_buf, int len);

int CT_Data_Parsing(uint8_t *PlayLoad_Data, uint8_t DLEN);

int CT_Produck_Key(uint8_t RAND, uint16_t FCNT, uint8_t * eui, uint8_t * token);

int p2p_Hex_printf(char * remarks, uint8_t * Hex_data, int len, int offset);

int CT_ACK_Ans(uint8_t * command_data, uint8_t ACK_ST, uint8_t CID);

#endif
