#include "ConfigTool.h"
#include "usart.h"
#include "CT_funtion.h"
#include "loragw_hal.h"
#include "loragw_spi.h"
#include "loragw_fpga.h"
#include "loragw_reg.h"
#include "gpio.h"
#include "loragw_user.h"
#include "protocol_analysis.h"

extern USART_RECEIVETYPE Usart2_RX;
struct SX1301_ConfPara ConfPara = {9,475300000,20}; // 缺省配置 Datarate=SF9,Freq=470300000,power:20;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

uint8_t send_command[256];
uint8_t send_len;

//网关是否配置的标志
uint8_t Start_UP = 0;

//1301Config
extern	struct lgw_conf_rxrf_s rfconfA,rfconfB;

static struct lgw_conf_rxif_s ifconf;
static struct lgw_conf_board_s boardconf;
static struct lgw_pkt_rx_s rxpkt[2];	//最多存储两包数据
static struct lgw_pkt_tx_s tx_packet;

extern struct lgw_tx_gain_lut_s txgain_lut;

extern struct lgw_conf_rxif_s *prxifms;
extern struct lgw_conf_rxif_s ifconfm[8];
extern	uint8_t clocksource; /* Radio B is source by default */

extern RFSystemParameter_TYPE SystemParameter;

extern uint8_t CT_EUI[8];

extern struct SX1301Status_S SX1301Status;

// 上位机连接标志
uint8_t CT_ConnectUpper;


//1301启动函数
int User_SX1301_Start()
{
    uint8_t ConfDR=9;
		uint8_t CRC_byte = 0;
    uint8_t StartSuccessTemp=1;
	
    //重启射频板电源
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(1000);

    //1301硬件复位
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(100);//>1ns
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(100);

    //SX1301启动配置
    //Board Configuration
    //boardconf.lorawan_public = true;
		//470：1  923：0
    //boardconf.clksrc = 0;
    //lgw_board_setconf(boardconf);
		
		SystemParameter.LoRaMAC = true;
		SystemParameter.ReceiveCRC = 3;
		clocksource = 1;

    //RadioA RF Configuration
    rfconfA.enable = 1;
    rfconfA.freq_hz = (uint32_t)(ConfPara.freq_start+ 300000);
    rfconfA.tx_enable = 1;
    rfconfA.type = LGW_RADIO_TYPE_SX1257;
    rfconfA.rssi_offset = -176;
    rfconfA.tx_notch_freq = LGW_DEFAULT_NOTCH_FREQ;
		
		
    //lgw_rxrf_setconf(0,rfconfA);

    //RadioB RF Configuration
    rfconfB.enable = 1;
    rfconfB.freq_hz = (uint32_t)(ConfPara.freq_start+ 1100000);
    rfconfB.tx_enable = 1;
    rfconfB.type = LGW_RADIO_TYPE_SX1257;
    rfconfB.rssi_offset = -176;
    rfconfB.tx_notch_freq = LGW_DEFAULT_NOTCH_FREQ;
		
		//lgw_rxrf_setconf(1,rfconfB);

		lgw_reg_w(LGW_FRAME_SYNCH_PEAK1_POS,3); /* default 1 */
    lgw_reg_w(LGW_FRAME_SYNCH_PEAK2_POS,4); /* default 2 */
		 

    //IF Configration
    ifconfm[0].enable = true;
    ifconfm[0].rf_chain = 0;
    ifconfm[0].freq_hz = -300000;
    ifconfm[0].datarate = DR_LORA_MULTI;

    ifconfm[1].enable = true;
    ifconfm[1].rf_chain = 0;
    ifconfm[1].freq_hz = -100000;
    ifconfm[1].datarate = DR_LORA_MULTI;

    ifconfm[2].enable = true;
    ifconfm[2].rf_chain = 0;
    ifconfm[2].freq_hz = 100000;
    ifconfm[2].datarate = DR_LORA_MULTI;

    ifconfm[3].enable = true;
    ifconfm[3].rf_chain = 0;
    ifconfm[3].freq_hz = 300000;
    ifconfm[3].datarate = DR_LORA_MULTI;

    ifconfm[4].enable = true;
    ifconfm[4].rf_chain = 1;
    ifconfm[4].freq_hz = -300000;
    ifconfm[4].datarate = DR_LORA_MULTI;

    ifconfm[5].enable = true;
    ifconfm[5].rf_chain = 1;
    ifconfm[5].freq_hz = -100000;
    ifconfm[5].datarate = DR_LORA_MULTI;
		
    ifconfm[6].enable = true;
    ifconfm[6].rf_chain = 1;
    ifconfm[6].freq_hz = 100000;
    ifconfm[6].datarate = DR_LORA_MULTI;


    ifconfm[7].enable = true;
    ifconfm[7].rf_chain = 1;
    ifconfm[7].freq_hz = 300000;
    ifconfm[7].datarate = DR_LORA_MULTI;


    //调用启动函数
    //StartSuccess = lgw_start();
		lgw_stop();
		StartSuccessTemp = SX1301Start(&rfconfA, &rfconfB, prxifms);
		//StartSuccess = CT_lgw_start();
#if TEST_MODE 
    //输出启动结果
    if (StartSuccessTemp == 1)
    {
        //printf("*** Concentrator started ***\r\n");
    }
    else
    {
        //printf("*** Impossible to start concentrator ***\r\n");
    }
#endif

    //SX1301设置GPIO-mapping
    lgw_spi_w(NULL, 0, 0x0, 0x1C, 0x00);

    //TX参数设置
    switch(ConfPara.txdatarate)
    {
    case 12:
        ConfDR= DR_LORA_SF12;
        break;
    case 11:
        ConfDR= DR_LORA_SF11;
        break;
    case 10:
        ConfDR= DR_LORA_SF10;
        break;
    case 9:
        ConfDR= DR_LORA_SF9;
        break;
    case 8:
        ConfDR= DR_LORA_SF8;
        break;
    case 7:
        ConfDR=	DR_LORA_SF7;
        break;
		default:
			ConfDR= DR_LORA_SF7;
			break;
    }

    tx_packet.bandwidth		= 	BW_125KHZ;
    tx_packet.coderate		 =	 CR_LORA_4_5;
    tx_packet.datarate 		= 	ConfDR;
    tx_packet.freq_hz 		= 	(uint32_t)(ConfPara.freq_start);
    tx_packet.modulation	 	= 	MOD_LORA;
    tx_packet.no_crc 			= 	false;
    tx_packet.preamble		 = 	8;
    tx_packet.rf_chain		 = 	0;
    tx_packet.rf_power		 = 	ConfPara.power;
    tx_packet.tx_mode		 = 	IMMEDIATE;

		
		return StartSuccessTemp;

}

//串口接收命令解析
uint8_t DataResolve_Upside(void)
{
	uint8_t i;
	uint8_t send_data[150];
	uint8_t result = 0;
	uint8_t com_result = 0;
	int err_code = 0;
	
  result = Command_Inspect(Usart2_RX.RX_Buf,Usart2_RX.rx_len-1);
	
	if(result == 1)
	{
		
		com_result = GW_Command_Split(Usart2_RX.RX_Buf);
		
		if(Start_UP == 1 && com_result == GW_CODE)
		{

			//send_len = Comand_Pack(send_command, usart1_rx_data + 3,usart1_rx_data_size-4);
			
			memset(send_command, 0x00, sizeof(send_command));
			
			send_len = Usart2_RX.rx_len-2;
			
			for(i = 0; i< Usart2_RX.rx_len; i++)
			{
				send_command[i] = Usart2_RX.RX_Buf[i];
			}
			
	#if 0
			printf("[send command]： ");
			
			for(i = 0; i<send_len + 1; i++)
			{
					printf("%02x ",send_command[i]);
			}
				printf("\n");
	#endif
		}else if(com_result == CT_INFO_CODE)
		{
				Start_UP = 0;
				CT_ACK_Pack(restart,0);
				switch(Usart2_RX.RX_Buf[1])
				{
					case SMC_start_up:
						//	解析串口下来的设置命令
						CT_Info_Para(Usart2_RX.RX_Buf, &ConfPara);
					
						err_code = User_SX1301_Start();
					
						if(err_code == 1)
						{
							CT_ACK_Pack(set_success,0);
							Start_UP = 1;
							SX1301Status.StartSuccesFlag  = 1;
						}else{
							CT_ACK_Pack(start_error,err_code);
						}
						
						break;
					case SMC_connect:
						
						CT_ConnectUpper = 1;
						break;
					
					default:
						break;
				}
				
			
		}else if(Start_UP == 0 && com_result == GW_CODE){
			
			CT_ACK_Pack(un_start,0);
			
		}else{
			CT_ACK_Pack(command_error,0);
			
		}
		
	}
	
	return com_result;

}

// 接受LoRa数据
int Rec_LoRa(void){
		uint8_t i,j;
    int nb_pkt = 0;
    nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);	//1301接收缓冲区-->rxpkt
		
    return nb_pkt;
}

// 发送数据至上位机
int data_forUp(uint8_t nb_pkt){
		int i;
    int pktnum = nb_pkt;
    uint8_t checkpass=1;
    uint8_t pkt_check[4]= {0xE5,0xFC,0x29,0x18};
		uint8_t Send_to_PC [300] = "";
		int len;
		
		//分别处理缓冲区多包数据（如果有）
		while(nb_pkt>0){
			nb_pkt--;
			//头尾校验
			for(uint8_t i=0; i<3; i++){
					if(rxpkt[pktnum-1].payload[i]!=pkt_check[i]) checkpass=0;
			}
			//校验eui
			for(i = 0; i<8; i++){
				
				 if(rxpkt[pktnum-1].payload[i+3] != CT_EUI[i]){
						//printf("EUI error !\n");
						return -1;
				 }
				 
				 
			}
			
			//串口输出
			if(checkpass){
				LORA_LEDSET(1);
				len = Send_PC_Pack(rxpkt[pktnum-1], Send_to_PC);
				
				Usart2SendData(Send_to_PC, len);
				LORA_LEDSET(0);
				/*
					printf("[ LoRa Data ]: ");
					for(uint8_t i=0; i<rxpkt[pktnum-1].size; i++)
					{
							printf("%02X ",rxpkt[pktnum-1].payload[i]);
					}
					printf("\r\n");
				*/
			}
		}
		
}

// 发送LoRa数据
void LoRaSend(struct lgw_pkt_tx_s tx_packet){
	//射频转发上位机数据
    uint8_t send_access_flag=1 ;

		send_access_flag = lgw_send(tx_packet);

		if(send_access_flag==0)
		{
#if 0
				printf("[ Send Success ]: ");
				for(uint8_t i=0; i< tx_packet.size+1; i++)
				{
						printf("%02X ",tx_packet.payload[i]);
				}
				printf("\r\n");
#endif
			
			CT_ACK_Pack(send_success,0);

		}
		else
		{
			CT_ACK_Pack(send_error,0);
			/*
				printf("Error in Sending\r\n");
			*/
		}
}

// 初始化配置工具
int ConfigTool_Init(void){
	
		SX1301_Enable(1);
		SX1301_ENABLE;
		HAL_Delay(20);

		SX1301Status.StartSuccesFlag = 0;
		CT_ConnectUpper = 0;
	
		return 1;
	
}

// 运行配置工具
void ConfigTool_Run(void){
	
	uint8_t recTemp = 0;
	uint8_t com_result = 0;
	uint8_t i = 0;
	
	printf("ConfigTool V1.2.2 Runing ");
	
	if(Usart2_RX.receive_flag == 1){
		com_result = DataResolve_Upside();
		Usart2_RX.receive_flag = 0;
		if(Start_UP == 1)
		{
			if(com_result == GW_CODE)
			{
				memset(tx_packet.payload,0x00,255);
				for(i = 0; i<send_len+1; i++)
				{
					tx_packet.payload[i] = send_command[i];
				}
				//strcpy(tx_packet.payload,usart1_rx_data);
				tx_packet.size = Usart2_RX.rx_len-2;
				
				//射频发送
				LORA_LEDSET(1);
				LoRaSend(tx_packet);			
				LORA_LEDSET(0);
			
			}
		}
	}
	
	// 配置工具是否启动
	if(Start_UP == 1){
		recTemp = Rec_LoRa();
		if(recTemp != 0){
			
			//射频接收
			data_forUp(recTemp);

		
		}
	}

}