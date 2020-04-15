#include "PC_Seting.h"
#include "p2p.h"
#include "softwareversion.h"

#define debug 0

extern struct p2p_param_ST p2p_param; // P2P接受eui
// 上位机连接标志
extern uint8_t AT_ConnectUpper;

int PC_SetingPack(uint8_t * sendbuf, uint8_t *data, uint8_t sendlen){
	
}

/*
@brief: 	数据合法性判断
					命令格式：0xEE CID data cs
@param: 	recbuf 需要解析的数据
				data 解析出的命令data数据
@return:	CID
 */
int PC_SetingAnalysis(uint8_t * recbuf, uint8_t recLen){
	uint8_t commandData[200] = "";
	
	
	switch(PC_SetingLegality(recbuf, recLen, commandData)){
		
		case PC_CID_Connect:
				AT_ConnectUpper = 1;
				break;
		
		case PC_CID_SetEUI:
				PC_SetingSetingEUI(commandData, p2p_param.CT_GW_EUI);
				break;
		
		case PC_CID_SetPower:
				p2p_param.CT_Send_power = commandData[0];
				printf("CT_Send_power:%d\n", p2p_param.CT_Send_power);
				break;
		
		case PC_CID_GetVersion:
				SoftwareVersionShow();
			break;
		
		case 0: return 0; break;
		default:return 0; break;
	}
	return 1;
}

/*
@brief: 	数据合法性判断
@param: 	recbuf 需要解析的数据
				data 解析出的命令data数据
@return:	CID
 */

int PC_SetingLegality(uint8_t * recbuf, uint8_t recLen, uint8_t *data){
	if(recbuf[0] == 0xEE){
		if(PM_CS_check(recbuf+1, recLen-1, 1)){
			for(int i = 0; i< recLen - 3; i++){
				data[i] = recbuf[i+2];
			}
			
			return recbuf[1];
		}else{
			printf("cs err recbuf[0] add : %x \r\n");
			return 0;
		}
	}
}

/*
@brief: 	数据合法性判断
@param: 	data 命令有效数据
					EUI  需要设置的EUI
@return:	CID
 */
int PC_SetingSetingEUI(uint8_t *data, uint8_t *EUI){
	for(int i = 0; i<8; i++){
		EUI[i] = data[i];
	}
	
	printf("EUI seting success!\r\n");
	
	for(int i = 0; i<8; i++){
		printf("%02 ", EUI[i]);
	}
	printf("\r\n");
	
	return 1;
}

/**
  *@brief CS检验
  		  用于最后一位为CS的字符串 
  *@param 
  		  chackBuf: 需要校验的字符串 
  		  len	  : 字符串长度
				mode:
		  		0	返回cs校验
					1	进行cs校验判断是否成功 

  *@return 
             mode:
							0	返回cs校验
							1	进行cs校验判断是否成功 
  */

uint8_t PM_CS_check(uint8_t * chackBuf, int len, uint8_t mode){
	int i = 0;
	uint8_t cs = 0;
	
	if(mode == 0){
		//printf("chackBuf:")
		for( i = 0; i<len-1; i++){
			cs += chackBuf[i];
			//printf(" %02x");
		}
		//printf("\n");
		return cs;
	}else if(mode == 1){
		for( i = 0; i<len-1; i++){
			cs = cs + chackBuf[i];
		}
		//printf("cs :%02x   sendBuf[len-1] : %02x \n",cs , sendBuf[len-1]);
		if(cs == chackBuf[len-1]){
			return 1;
		}else{
			return 0;
		}
	}
	
}