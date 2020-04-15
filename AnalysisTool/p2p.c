#include "p2p.h"


#define debug 0

extern struct lgw_pkt_tx_s txpkt;

uint8_t Radio_Eui[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
//dongle eui
static uint8_t CT_EUI[8] = {0x11,0x22,0x33,0x44,0x11,0x22,0x33,0x44};

uint8_t GW_EUI[8] = {0x12,0x23,0x34,0x45,0x56,0x67,0x78,0x89};

uint16_t Send_FCNT = 0;
uint16_t Rec_FCNT = 0;
uint8_t REC_RFU = 0;
static uint8_t SEND_RFU = 0x66;
static uint8_t SEND_RAND = 0;

// 网关的发送数据参数


// aes加密变量
aes_context AesContext;


// p2p参数
p2p_param_ST p2p_param;

// 发送的信息类别
uint8_t CT_Send_MSGTYPE = GW_MSG_SEND;
uint8_t CT_Send_CTRLTYPE = GW_CTRL_DEF;

int TestDownFcnt = 0;

int LastRSSI = 0;
int LastSNR = 0;


int p2p_init(){
	p2p_param.CT_GW_EUI[0] = 0x00;
	p2p_param.CT_GW_EUI[1] = 0x95;
	p2p_param.CT_GW_EUI[2] = 0x69;
	p2p_param.CT_GW_EUI[3] = 0x01;
	p2p_param.CT_GW_EUI[4] = 0x00;
	p2p_param.CT_GW_EUI[5] = 0x00;
	p2p_param.CT_GW_EUI[6] = 0x00;
	p2p_param.CT_GW_EUI[7] = 0xBB;
	
	p2p_param.CT_Send_power = 20;
	
	return 1;
}

/**
  * @brief  发送给配置工具数据包函数
  			通过入队形式
  * @param  ans_comman:回复给配置工具的数据
  			ans_comman:回复给配置工具的数据长度
  * @retval -1: 入队失败
  */
int CT_Send_Command_Funtion(uint8_t *ans_comman, uint16_t command_len)
{
	int i = 0;

	
	txpkt.freq_hz 		= p2p_param.CT_Send_freq_hz;
	txpkt.tx_mode 		= IMMEDIATE;
	txpkt.rf_power 		= p2p_param.CT_Send_power;
	txpkt.modulation 	= MOD_LORA;
	txpkt.bandwidth 	= BW_125KHZ;
	txpkt.datarate 		= p2p_param.CT_Send_datarate;
	txpkt.coderate 		= CR_LORA_4_5;
	txpkt.no_crc  		= false;
	txpkt.preamble 		= 8;
	txpkt.invert_pol	= false;
	
	//lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS,1); /* default 1 */
    //lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS,2); /* default 2 */

	for(i = 0; i<command_len; i++)
	{
		txpkt.payload[i] = ans_comman[i];
	}

	txpkt.size = command_len;
/*	
	send_access_flag = lgw_send(txpkt);
	
	if(send_access_flag==0)
    {
        printf("[ Send Success ]: ");
        for(uint8_t i=0; i<txpkt.size ; i++)
        {
            printf("%02X ",txpkt.payload[i]);
        }
        printf("\r\n");

    }
    */

#if debug
	printf("CT_txpkt.freq_hz:%d\n", txpkt.freq_hz);
	printf( "CT_txpkt.datarate:%d\n", txpkt.datarate);
#endif
	SX1301TXData();
	
}

int CT_Command_Parsing(struct lgw_pkt_rx_s * lgw_pkt_rx_s) {
    int result = 0;
    int i = 0;
    int pare_bit = 0;
    int rand_temp = 0;
    uint8_t command_buf[300] = "";
    uint8_t comm_data_len = 0;

    struct Config_Frame config_frame_rec;
    memset(command_buf, 0x00, sizeof(command_buf));
    memset(&config_frame_rec, 0x00, sizeof(config_frame_rec));

    p2p_param.CT_Send_freq_hz = lgw_pkt_rx_s->freq_hz;
    p2p_param.CT_Send_datarate = lgw_pkt_rx_s->datarate;
    LastRSSI = ceil(lgw_pkt_rx_s->rssi);
    LastSNR = ceil(lgw_pkt_rx_s->snr);

    //判断命令合法性
    result = CT_Command_Inspect(lgw_pkt_rx_s->payload, lgw_pkt_rx_s->size);
    if(result == true) {
        //命令解密
        result = CT_Command_AES_Decrypt(lgw_pkt_rx_s->payload, command_buf,lgw_pkt_rx_s->size, &config_frame_rec);
    }
		
		if( (result == true) && (config_frame_rec.RFU == RFU_DEFAULT)){
			if(CT_Command_Split(command_buf,&config_frame_rec) == true)
			{
				CT_Data_Parsing(config_frame_rec.DATA, config_frame_rec.DLEN);
			}
		}

}

int TestResetReq_Parsing(uint8_t *PlayLoad_Data, struct TestResetReq * testResetReq)
{
	testResetReq->Test_set.TestPower = PlayLoad_Data[1] & 0X1F;

	if(testResetReq->Test_set.TestPower > 30 || testResetReq->Test_set.TestPower < 0){
		#if debug
		printf("TestPower PARA_ERR:%d\n",testResetReq->Test_set.TestPower);
		#endif
		return CF_PARA_ERR;
	}
	
	testResetReq->Test_set.TestSF = (PlayLoad_Data[1] & 0XE0) >> 5;
	if(testResetReq->Test_set.TestSF > 8 || testResetReq->Test_set.TestSF < 0){
		#if debug
		printf("TestSF PARA_ERR:%d\n",testResetReq->Test_set.TestSF);
		#endif
		return CF_PARA_ERR;
	}
	
	testResetReq->TestFreq  = PlayLoad_Data[2] & 0xFF;
	testResetReq->TestFreq += PlayLoad_Data[3] << 8;
	testResetReq->TestFreq += PlayLoad_Data[4] << 16;
	testResetReq->TestFreq += PlayLoad_Data[5] << 24;

	

	if(testResetReq->TestFreq > 928000000 || testResetReq->TestFreq < 470000000){
		#if debug
		printf("TestFreq PARA_ERR:%d\n",testResetReq->TestFreq);
		#endif
		return CF_PARA_ERR;
	}

	testResetReq->RadioMode = PlayLoad_Data[6];
							
	testResetReq->TestInterval  = PlayLoad_Data[7];
	testResetReq->TestInterval += PlayLoad_Data[8] << 8;
	testResetReq->TestInterval += PlayLoad_Data[9] << 16;

#if debug
	
	printf( "TestSF:%X\n",testResetReq->Test_set.TestSF);
	printf( "TestPower:%d\n",testResetReq->Test_set.TestPower);
	printf( "TestFreq:%d\n",testResetReq->TestFreq);
	printf( "RadioMode:%X\n",testResetReq->RadioMode);
	printf( "TestInterval:%d\n",testResetReq->TestInterval);
#endif

	return true;
}

int RF_RadioTestReq_Parsing(uint8_t *PlayLoad_Data, struct RadioTestReq *RadioTestReq)
{
	uint8_t i;
	RadioTestReq->DataLen = PlayLoad_Data[1];
	
	RadioTestReq->Fcntup = PlayLoad_Data[2];
	RadioTestReq->Fcntup = PlayLoad_Data[3] << 8;

	for(i = 0; i<RadioTestReq->DataLen; i++)
	{
		RadioTestReq->Payload[i] = PlayLoad_Data[4 + i];
	}
	
	printf( "---RadioConfReq_Parsing---\n");
#if debug
	printf( "Fcntup:%X\n",RadioTestReq->Fcntup);
#endif

	return true;
}

int CT_ACK_Ans(uint8_t * command_data, uint8_t ACK_ST, uint8_t CID)
{
	command_data[0] = CID;
	command_data[1] = ACK_ST;
#if debug
	p2p_Hex_printf("ACK", command_data, 2, 0);
#endif
	return 2;
}

int RF_RadioTestAns(uint8_t *command_Data, struct RadioTestReq RadioTestReq){
	uint8_t index = 0;
	int i = 0;
	TestDownFcnt++;
	index++;
	command_Data[index++] = TestDownFcnt & 0xFF;
	command_Data[index++] = TestDownFcnt & 0xFF00 >> 8;

	for(i = 0; i<RadioTestReq.DataLen; i++)
	{
		command_Data[index++] = RadioTestReq.Payload[i];
	}
	
	command_Data[index++] = LastRSSI;
	command_Data[index++] = LastSNR;

	printf( "---RF_RadioTestAns---\n");
	#if debug
	printf( "rssi:%d\n", LastRSSI);
	printf( "snr:%d\n", LastSNR);
	#endif
	return index;
	
}


/**
  * @brief  配置工具打包发送命令
  			格式化打包
  * @param  ans_command: 打包后数据
  			ans_data: 需要打包的数据
  			ans_data_len: 需要打包的数据长度
  * @retval index: 打包后数据长度
  */
int CT_Ans_Pack(uint8_t *ans_command, uint8_t *ans_data, uint16_t ans_data_len, uint8_t *CTEUI, uint8_t Command_CID)
{
	//uint8_t ans_command[300];
	uint8_t add = 0,i;
	uint16_t com_index = 0;
	uint8_t data_buf[300];
	uint16_t FCNT_temp = 0;
	uint16_t temp_len = 0;

	
	//memset(ans_command,0,sizeof(ans_command));

	//获取随机数
	SEND_RAND = (uint8_t)((rand()%255)&0xff);

	data_buf[com_index++] = 0xE5;
	data_buf[com_index++] = 0xFC;
	data_buf[com_index++] = 0x29;
	
	//CTEUI
	for(i = 0; i<8; i++){
		data_buf[com_index++] = CTEUI[i];
		//printf("%X",CTEUI[i]);
	}

	// GWEUI
	for(i = 0; i<8; i++){
		data_buf[com_index++] = p2p_param.CT_GW_EUI[i];
		//printf("%x ",ans_command[com_index]);
	}

	//DLEN
	data_buf[com_index++] = ans_data_len+2;
	//printf("ans_data_len:%d\n",ans_data_len);

	//CTRL
	data_buf[com_index] =  CT_Send_CTRLTYPE << 4;
	//MSGTYPE
	data_buf[com_index++] += (CT_Send_MSGTYPE&0x0f);

	//printf("CTRL:%02x\n",data_buf[com_index-1]);

	//DATA {
	//CID
	data_buf[com_index++] = Command_CID;
	
	//data
	for(i = 0; i<ans_data_len;i++){
		data_buf[com_index++] = ans_data[i];
	}
	//}

	//RFU
	data_buf[com_index++] = SEND_RFU;

	//RAND
	data_buf[com_index++] = SEND_RAND;
	
	//FCNT
	Send_FCNT++;	
	FCNT_temp = Send_FCNT + SEND_RAND;
	data_buf[com_index++] = (FCNT_temp & 0xff);
	data_buf[com_index++] = (FCNT_temp & 0xff00) >> 8;

	#if debug
	p2p_Hex_printf("data_buf", data_buf, com_index, 0);

	printf("com_index:%d\n", com_index);
	#endif
	

	temp_len = CT_Aes_Encrypt_Pack(data_buf, ans_command, com_index);

	#if debug
	p2p_Hex_printf("ans_command", ans_command, temp_len, 0);
	#endif

	
	return temp_len;
	
	
}

int CT_Data_Parsing(uint8_t *PlayLoad_Data, uint8_t DLEN)
{
	int result = 0;
	
	uint8_t command_data[255] = "";
	uint8_t command_buf[255] = "";
	uint8_t command_len = 0;
	uint8_t ACK_bit = 0;
	int comm_data_len = 0;
	
	struct TestResetReq testResetReq;
	struct RadioTestReq radioTestReq;
	switch (PlayLoad_Data[0])
	{
		//--------------P2P test --------------------------
		case RF_TestReset:
			result = TestResetReq_Parsing(PlayLoad_Data,&testResetReq);
			if(result == true)
			{
				CT_Send_CTRLTYPE = GW_CTRL_TEST;
				CT_ACK_Ans(command_data, ACK_SUCCESS, RF_TestReset);

			}else{
				CT_ACK_Ans(command_data, -result, RF_TestReset);
			}
			break;
		

		case RF_RadioTest:
			comm_data_len = RF_RadioTestReq_Parsing(PlayLoad_Data,&radioTestReq);
			if(comm_data_len > 0)
			{
				CT_Send_CTRLTYPE = GW_CTRL_TEST;
				comm_data_len = RF_RadioTestAns(command_data, radioTestReq);
				comm_data_len =  CT_Ans_Pack(command_buf, command_data, comm_data_len, CT_EUI,RF_RadioTest);
				CT_Send_Command_Funtion(command_buf,comm_data_len);
				
				ACK_bit = 1;
				
			}else{
				//comm_data_len为负转正就是错误代码
				CT_ACK_Ans(command_data, -comm_data_len, RF_RadioTest);
			}
			break;
			
		default: break;
	}
	
	if(ACK_bit == 0)
	{

		comm_data_len = CT_Ans_Pack(command_buf, command_data, 3, CT_EUI,RF_ACK_Head);
		CT_Send_Command_Funtion(command_buf,comm_data_len);

		CT_Send_CTRLTYPE = GW_CTRL_DEF;
	}
}
		
/**
  * @brief  数据解析
  			在解密且合法性后调用
  * @param  PlayLoad: 接受到数据
  			config_frame: 配置工具参数结构体

  * @retval true
  */
int CT_Command_Split(uint8_t *PlayLoad, struct Config_Frame * config_frame)
{
	uint8_t i;
	uint8_t check = 0;
	
	memset(config_frame, 0x00, sizeof(config_frame));
	
	config_frame->CTEUI[0]  = PlayLoad[3];
	config_frame->CTEUI[1]  = PlayLoad[4];
	config_frame->CTEUI[2]  = PlayLoad[5];
	config_frame->CTEUI[3]  = PlayLoad[6];
	config_frame->CTEUI[4]  = PlayLoad[7];
	config_frame->CTEUI[5]  = PlayLoad[8];
	config_frame->CTEUI[6]  = PlayLoad[9];
	config_frame->CTEUI[7]  = PlayLoad[10];

	config_frame->GWEUI[0]  = PlayLoad[11];
	config_frame->GWEUI[1]  = PlayLoad[12];
	config_frame->GWEUI[2]  = PlayLoad[13];
	config_frame->GWEUI[3]  = PlayLoad[14];
	config_frame->GWEUI[4]  = PlayLoad[15];
	config_frame->GWEUI[5]  = PlayLoad[16];
	config_frame->GWEUI[6]  = PlayLoad[17];
	config_frame->GWEUI[7]  = PlayLoad[18];

	for(i = 0; i<8; i++)
	{
		CT_EUI[i] = config_frame->CTEUI[i];
	}

	config_frame->DLEN = PlayLoad[19] - 1;
	
	config_frame->Control_Code.CTRL    = (PlayLoad[20] & 0xF0)>>4;
	config_frame->Control_Code.MSGTYPE = (PlayLoad[20] & 0x0F);
	

	if(config_frame->Control_Code.MSGTYPE == GW_MSG_CON)
	{
		Rec_FCNT = 0;
	}

	for(i = 0; i<config_frame->DLEN; i++)
	{
		config_frame->DATA[i] = PlayLoad[21+i];
	}

	config_frame->RAND  = PlayLoad[22+config_frame->DLEN];

	config_frame->FCNT  = PlayLoad[23+config_frame->DLEN];
	config_frame->FCNT += PlayLoad[24+config_frame->DLEN] << 8;

	config_frame->FCNT -= config_frame->RAND;

	//判断是否是广播信号
	if(config_frame->Control_Code.MSGTYPE != GW_MSG_CT_RADIO)
	{
		if(config_frame->FCNT <= Rec_FCNT && config_frame->FCNT != 0)
		{
			//printf("config_frame->FCNT:%d Rec_FCNT:%d  CT_RECFCNT_ERR!\n", config_frame->FCNT, Rec_FCNT);
			return CT_RECFCNT_ERR;
		}else{
			Rec_FCNT = config_frame->FCNT;
		}
	}
	

	config_frame->Frame_Tail = PlayLoad[26+config_frame->DLEN];

#if debug
	printf("CTEUI:");
	for(i = 0; i<8; i++ ){
		printf("%X",config_frame->CTEUI[i]);
	}
	printf("\n");
	
	printf("GWEUI:");
	for(i = 0; i<8; i++ ){
		printf("%X",config_frame->GWEUI[i]);
	}
	printf("\n");
	
	printf("DLEN:%X\n",config_frame->DLEN);
	printf("CTRL:%X\n",config_frame->Control_Code.CTRL);
	printf("MSGTYPE:%X \n",config_frame->Control_Code.MSGTYPE);
	printf("RAND:%X\n",config_frame->RAND);
	printf("FCNT:%X\n",config_frame->FCNT);
#endif
	return true;
	
}

/**
  * @brief  数据合法性判断
  * @param  PlayLoad: 接受到数据
  			len: 接受到长度

  * @retval Radio_COMMAND
  			CT_GW_EUI_ERR
  			CHECK_ERROR
  			FREAM_ERROR
  */
int CT_Command_Inspect(uint8_t *PlayLoad,uint16_t len)
{
    uint16_t i;
    uint8_t add = 0;
    uint8_t eui_temp[8] = "";
    uint8_t check = 0;
#if debug
    printf("PlayLoad:");
    for(i = 0; i<len; i++)
    {
        printf("%02X ",PlayLoad[i]);
    }
    printf("\n");
#endif
    if(PlayLoad[0] == 0xE5 && PlayLoad[1] == 0xFC && PlayLoad[2] == 0x29)
    {

        //printf("add:");
        for(i = 0; i< (len - 4); i++) {
            add += PlayLoad[i+3];
            //printf("%02x ",PlayLoad[i+3]);
        }
        //printf("\n");

        if(add == PlayLoad[len-1]) {

            //判断网关eui
            eui_temp[0]  = PlayLoad[11];
            eui_temp[1]  = PlayLoad[12];
            eui_temp[2]  = PlayLoad[13];
            eui_temp[3]  = PlayLoad[14];
            eui_temp[4]  = PlayLoad[15];
            eui_temp[5]  = PlayLoad[16];
            eui_temp[6]  = PlayLoad[17];
            eui_temp[7]  = PlayLoad[18];

            for(i = 0; i<8; i++) {
                if(eui_temp[i] == Radio_Eui[i]) {
                    check++;
                }
            }

            if(check == 8)
            {
                return Radio_COMMAND;
            }

            for(i = 0; i<8; i++)
            {
								#if debug
                printf("GWEUI[%d]:%x,CTGWEUI[%d]:%x\n",i ,eui_temp[i], i, p2p_param.CT_GW_EUI[i]);
								#endif
                if(eui_temp[i] != p2p_param.CT_GW_EUI[i]) {
									#if 1
                    printf("EUI error !\n");
									#endif
                    return CT_GW_EUI_ERR;
                }
            }
            return true;


        } else {
					#if debug
            printf("frean check error !\n");
					#endif
            return CHECK_ERROR;
        }
    } else {
        //printf("frean error !\n");
        return FREAM_ERROR;
    }

}

/**
  * @brief  配置工具数据加密函数
  			对完整命令包有需要的内容加密
  * @param  data: 未加密数据
  			send_buf: 加密后数据
  			len: 需要加密数据长度
  * @retval index: 加密后数据长度
  */
int CT_Aes_Encrypt_Pack(char * data, char * send_buf, int len)
{
    //aes数据长度
    int aes_len = 0;
    int len_temp = 0;
    uint8_t aes_token[17] = "";
    int i = 0;
    int index = 0;
    int add = 0;

    //需要加密的数据
    char data_temp[300] = "";
    //加密后的数据
    char aes_data[300] = "";

    for(i = 0; i<20; i++)
    {
        send_buf[index++] = data[i];
    }

    for(i = 0; i<(len-23); i++)
    {
        data_temp[i] = data[i+20];
        aes_len ++;
    }
    //p2p_Hex_printf("aes", data_temp, (len-23), 0);

    len_temp = aes_len;

    CT_Produck_Key(SEND_RAND, SEND_RAND+Send_FCNT, p2p_param.CT_GW_EUI, aes_token);


    //p2p_Hex_printf("eui:", CT_User_ST.CT_GW_EUI, 8, 0);

    //p2p_Hex_printf("key:", aes_token, 16, 0);

    //初始化AES
    aes_set_key(aes_token, 16, &AesContext);

    do {
        aes_encrypt((uint8_t *)data_temp+(aes_len - len_temp), (uint8_t *)aes_data+(aes_len - len_temp), &AesContext);

        for(i = 0; i<16; i++)
        {
            send_buf[index++] = aes_data[i+(aes_len - len_temp)];
        }

        len_temp -= 16;
    } while(len_temp > 0);

    for(i = 0 ; i<3; i++)
    {
        send_buf[index++] = data[(len - 3) + i];
    };

    //p2p_Hex_printf("tail", send_buf, 4, index-4);

    //printf("add:");

    //E5 FC 29 [内容相加] CS 18
    for(i = 0; i< (index - 3); i++) {
        add += send_buf[i+3];
        //printf("%02x ",send_buf[i+3]);
        //printf("%x \n",add);
    }
    //printf("\n");
    send_buf[index++] = add;

    //send_buf[index++] = 0X18;

    return index;
}

int CT_Command_AES_Decrypt(uint8_t * PlayLoad, uint8_t * command_buf, uint16_t len, struct Config_Frame * config_frame)
{
	uint8_t aes_token[17] = "";
	int aes_len = 0;
	int len_temp = 0;
	int i = 0;
	int index = 0;
	uint8_t check = 0;

	//需要解密的数据
    char data_temp[300] = "";
	//解密密后的数据
    char aes_data[300] = "";

	for(i = 0; i<20; i++)
    {
        command_buf[index++] = PlayLoad[i];
    }
	//p2p_Hex_printf("head", command_buf, 20, 0);
	
	for(i = 0; i<(len-23); i++)
    {
        data_temp[i] = PlayLoad[i+20];
        aes_len ++;
    }
	//printf("aes_len: %d", aes_len);
	//p2p_Hex_printf("aes", data_temp, (len-24), 0);

	len_temp = aes_len;

	config_frame->CTEUI[0]  = PlayLoad[3];
	config_frame->CTEUI[1]  = PlayLoad[4];
	config_frame->CTEUI[2]  = PlayLoad[5];
	config_frame->CTEUI[3]  = PlayLoad[6];
	config_frame->CTEUI[4]  = PlayLoad[7];
	config_frame->CTEUI[5]  = PlayLoad[8];
	config_frame->CTEUI[6]  = PlayLoad[9];
	config_frame->CTEUI[7]  = PlayLoad[10];

	config_frame->GWEUI[0]  = PlayLoad[11];
	config_frame->GWEUI[1]  = PlayLoad[12];
	config_frame->GWEUI[2]  = PlayLoad[13];
	config_frame->GWEUI[3]  = PlayLoad[14];
	config_frame->GWEUI[4]  = PlayLoad[15];
	config_frame->GWEUI[5]  = PlayLoad[16];
	config_frame->GWEUI[6]  = PlayLoad[17];
	config_frame->GWEUI[7]  = PlayLoad[18];
	
	config_frame->DLEN      = PlayLoad[19];

	config_frame->RAND  = PlayLoad[len-4];

	config_frame->FCNT  = PlayLoad[len-3];
	config_frame->FCNT += PlayLoad[len-2] << 8;

	config_frame->FCNT -= config_frame->RAND;

	CT_Produck_Key(config_frame->RAND, config_frame->FCNT, config_frame->GWEUI, aes_token);

	//p2p_Hex_printf("key:", aes_token, 16, 0);

	//初始化密钥
    aes_set_key(aes_token, 16, &AesContext);

	do{
        aes_decrypt((uint8_t *)data_temp+(aes_len - len_temp), (uint8_t *)aes_data+(aes_len - len_temp), &AesContext);

        len_temp -= 16;
    }while(len_temp > 0);

	for(i = 0; i<config_frame->DLEN+1; i++)
    {
        command_buf[index++] = aes_data[i];
    }

	for(i = 0 ; i<4; i++)
    {
        command_buf[index++] = PlayLoad[(len - 4) + i];
    }

	//p2p_Hex_printf("command_buf", command_buf, index, 0);

	config_frame->RFU   = command_buf[20+config_frame->DLEN];

	return true;

	
}

int CT_Produck_Key(uint8_t RAND, uint16_t FCNT, uint8_t * eui, uint8_t * token)
{
    int i = 0;
    uint8_t key1_temp;
    uint8_t key2_temp;

    key1_temp = RAND ^ FCNT;
    key2_temp = RAND ^ (FCNT >> 8);

    for(i = 0; i <8; i++)
    {
        token[i] = key1_temp ^ eui[i];
    }

    for(i = 0; i <8; i++)
    {
        token[8+i] = key1_temp ^ eui[i];
    }

}

int p2p_Hex_printf(char * remarks, uint8_t * Hex_data, int len, int offset)
{
    int i = 0;
    printf("%s:", remarks);
    for(i = 0 ; i<len; i++)
    {
        printf("%02X ", Hex_data[i+offset]);
    }
    printf("\n");
}