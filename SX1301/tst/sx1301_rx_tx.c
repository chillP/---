/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Minimum test program for the loragw_hal 'library'

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>        /* C99 types */
#include <stdbool.h>       /* bool type */
#include <stdio.h>         /* printf */
#include <string.h>        /* memset */
//#include <signal.h>        /* sigaction */
//#include <unistd.h>        /* getopt access */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

#include "loragw_user.h"
#include "sx1301_rx_tx.h"
#include "stm32l0xx_hal.h"

#include "usart.h"
#include "protocol_analysis.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_RSSI_OFFSET -176
#define DEFAULT_NOTCH_FREQ  129000U

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

struct lgw_conf_board_s boardconf;
struct lgw_conf_rxrf_s rfconf;
struct lgw_conf_rxif_s ifconf;

struct lgw_pkt_rx_s rxpkt[4]; /* array containing up to 4 inbound packets metadata */
struct lgw_pkt_tx_s txpkt; /* configuration and metadata for an outbound packet */
struct lgw_pkt_rx_s *p; /* pointer on a RX packet */

int i, j;
int nb_pkt;
uint32_t fa = 0, fb = 0, ft = 0;
enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_NONE;
//enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_SX1255;
uint8_t clocksource = 1; /* Radio B is source by default */

uint32_t tx_cnt = 0;
//unsigned long loop_cnt = 0;

//uint8_t status_var = 0;
uint8_t status_var = TX_STATUS_UNKNOWN;
uint32_t loop_cnt = 0;
double xd = 0.0;
int xi = 0;

#define MCU_AGC_FW_BYTE     8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
extern uint8_t fw_check[MCU_AGC_FW_BYTE];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

//static void sig_handler(int sigio);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */
#if 0
static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = 1;;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}
#endif
/* describe command line options */
void usage(void) {
    printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -a <float> Radio A RX frequency in MHz\n");
    printf( " -b <float> Radio B RX frequency in MHz\n");
    printf( " -t <float> Radio TX frequency in MHz\n");
    printf( " -r <int> Radio type (SX1255:1255, SX1257:1257)\n");
    printf( " -k <int> Concentrator clock source (0: radio_A, 1: radio_B(default))\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int Sx1301RadioToMcu(void)
{
	//参数配置
	xd = 475.3;/* <float> Radio A RX frequency in MHz */
	fa = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	xd = 476.4;/* <float> Radio B RX frequency in MHz */
	fb = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	xd = 868.1;/* <float> Radio TX frequency in MHz */
	ft = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
	//radio_type = LGW_RADIO_TYPE_SX1255;
	radio_type = LGW_RADIO_TYPE_SX1257;
	xi = 1;/* <int> Concentrator clock source (Radio A or Radio B) */
	clocksource = (uint8_t)xi;//clocksource = 1; /* Radio B is source by default */
		

    /* check input parameters */
    if ((fa == 0) || (fb == 0) || (ft == 0)) 
    {
        printf("ERROR: missing frequency input parameter:\n");
        printf("  Radio A RX: %u\n", fa);
        printf("  Radio B RX: %u\n", fb);
        printf("  Radio TX: %u\n", ft);
        usage();
        return -1;
    }

    if (radio_type == LGW_RADIO_TYPE_NONE) 
    {
        printf("ERROR: missing radio type parameter:\n");
        usage();
        return -1;
    }

    /* beginning of LoRa concentrator-specific code */
    printf("Beginning of test for loragw_hal.c\n");

    printf("*** Library version information ***\n%s\n\n", lgw_version_info());

    /* set configuration for board */
    memset(&boardconf, 0, sizeof(boardconf));
    boardconf.lorawan_public = false;
    boardconf.clksrc = clocksource;
    lgw_board_setconf(boardconf);
    printf("lorawan_public:%2d\n",boardconf.lorawan_public);

    /* set configuration for RF chains */
    memset(&rfconf, 0, sizeof(rfconf));     
    rfconf.enable = true;
    rfconf.freq_hz = fa;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    rfconf.tx_enable = true;
    rfconf.tx_notch_freq = DEFAULT_NOTCH_FREQ;
    lgw_rxrf_setconf(0, rfconf); /* radio A, f0 */

    rfconf.enable = true;
    rfconf.freq_hz = fb;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    rfconf.tx_enable = false;
    lgw_rxrf_setconf(1, rfconf); /* radio B, f1 */

    /* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
    memset(&ifconf, 0, sizeof(ifconf));

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = -400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(0, ifconf); /* chain 0: LoRa 125kHz, all SF, on f1 - 0.4 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = -200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(1, ifconf); /* chain 1: LoRa 125kHz, all SF, on f1 - 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = 0;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(2, ifconf); /* chain 2: LoRa 125kHz, all SF, on f1 - 0.0 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = -400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(3, ifconf); /* chain 3: LoRa 125kHz, all SF, on f0 - 0.4 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = -200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(4, ifconf); /* chain 4: LoRa 125kHz, all SF, on f0 - 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 0;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(5, ifconf); /* chain 5: LoRa 125kHz, all SF, on f0 + 0.0 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(6, ifconf); /* chain 6: LoRa 125kHz, all SF, on f0 + 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(7, ifconf); /* chain 7: LoRa 125kHz, all SF, on f0 + 0.4 MHz */

    /* set configuration for LoRa 'stand alone' channel */
    memset(&ifconf, 0, sizeof(ifconf));
    
    //ifconf.enable = true;
ifconf.enable = false;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 0;
    ifconf.bandwidth = BW_250KHZ;
    ifconf.datarate = DR_LORA_SF10;
    lgw_rxif_setconf(8, ifconf); /* chain 8: LoRa 250kHz, SF10, on f0 MHz */

    /* set configuration for FSK channel */
    memset(&ifconf, 0, sizeof(ifconf));
    
    //ifconf.enable = true;
ifconf.enable = false;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = 0;
    ifconf.bandwidth = BW_250KHZ;
    ifconf.datarate = 64000;
    lgw_rxif_setconf(9, ifconf); /* chain 9: FSK 64kbps, on f1 MHz */

    /* set configuration for TX packet */
    memset(&txpkt, 0, sizeof(txpkt));
    
    txpkt.freq_hz = ft;
    txpkt.tx_mode = IMMEDIATE;
    txpkt.rf_power = 10;
    txpkt.modulation = MOD_LORA;
    txpkt.bandwidth = BW_125KHZ;
    txpkt.datarate = DR_LORA_SF9;
    txpkt.coderate = CR_LORA_4_5;
    strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
    txpkt.size = 20;
    txpkt.preamble = 6;
    txpkt.rf_chain = 0;
/*
    memset(&txpkt, 0, sizeof(txpkt));
    txpkt.freq_hz = F_TX;
    txpkt.tx_mode = IMMEDIATE;
    txpkt.rf_power = 10;
    txpkt.modulation = MOD_FSK;
    txpkt.f_dev = 50;
    txpkt.datarate = 64000;
    strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
    txpkt.size = 20;
    txpkt.preamble = 4;
    txpkt.rf_chain = 0;
*/

    /* connect, configure and start the LoRa concentrator */
    //i = lgw_start();
    if (lgw_start() == LGW_HAL_SUCCESS) {
        printf("*** Concentrator started ***\n");
    } else {
        printf("*** Impossible to start concentrator ***\n");
        return -1;
    }

    /* once configured, dump content of registers to a file, for reference */
    // FILE * reg_dump = NULL;
    // reg_dump = fopen("reg_dump.log", "w");
    // if (reg_dump != NULL) {
        // lgw_reg_check(reg_dump);
        // fclose(reg_dump);
    // }

    while ((quit_sig != 1) && (exit_sig != 1))
    {
        loop_cnt++;
	
        /* fetch N packets */
        nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);

        if (nb_pkt == 0) 
        {
            wait_ms(300);
        } 
        else 
        {
            /* display received packets */
            for(i=0; i < nb_pkt; ++i)
            {
                p = &rxpkt[i];
                printf("---\nRcv pkt #%d >>", i+1);
                if (p->status == STAT_CRC_OK) 
                {
                	uint8_t temp;
                	uint16_t temp16;
                	uint16_t datacount = 0;
		//CRC校验成功
		//初始化
		memset(fw_check, 0, 500);
		//组帧
		fw_check[datacount++] = 0xfe;
		fw_check[datacount++] = 0xfe;
		fw_check[datacount++] = 0x68;
		fw_check[datacount++] = RX_C;//接收数据控制码
		//fw_check[datacount++] = 0x68;//
		fw_check[datacount++] = 0x00;//接收数据数据标识
		temp16 = p->size + 16;
		fw_check[datacount++] = temp16/256;//接收数据数据域长度高8位
		fw_check[datacount++] = temp16%256;//接收数据数据域长度低8位				
		
		printf(" if_chain:%2d", p->if_chain);
		fw_check[datacount++] = p->if_chain;
		
		switch (p->datarate) 
                    {
                        case DR_LORA_SF7: 
                        		printf(" SF7"); 
                        		temp = 7;
				break;
                        case DR_LORA_SF8:
                        		printf(" SF8"); 
                        		temp = 8;
				break;
                        case DR_LORA_SF9: 
                        		printf(" SF9"); 
                        		temp = 9;
				break;
                        case DR_LORA_SF10: 
                        		printf(" SF10"); 
                        		temp = 10;
				break;
                        case DR_LORA_SF11:
                        		printf(" SF11"); 
                        		temp = 11;
				break;
                        case DR_LORA_SF12: 
                        		printf(" SF12"); 
                        		temp = 12;
				break;
                        default: 
                        		printf(" datarate?");
                        		temp = 0xff;
													;
                    }
                    fw_check[datacount++] = temp;
                    
                    printf(" size:%3u", p->size);
                    fw_check[datacount++] = p->size;
                    
                    printf(" tstamp:%010u", p->count_us);
                    
                    switch (p-> modulation) 
                    {
                        case MOD_LORA: printf(" LoRa");
													break;
                        case MOD_FSK: printf(" FSK"); 
												break;
                        default: printf(" modulation?");
													;
                    }
                    
                    switch (p->coderate) 
                    {
                        case CR_LORA_4_5: printf(" CR1(4/5)"); temp = 5;
											break;
                        case CR_LORA_4_6: printf(" CR2(2/3)");temp = 6; 
											break;
                        case CR_LORA_4_7: printf(" CR3(4/7)");temp = 7;
											break;
                        case CR_LORA_4_8: printf(" CR4(1/2)"); temp = 8;
											break;
                        default: printf(" coderate?");
													;
                    }
                    fw_check[datacount++] =temp;
                    
                    printf("\n");
                    printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", p->rssi, p->snr, p->snr_min, p->snr_max);
		if(p->snr < 0)
		{
			fw_check[datacount++] = 0x80|(uint8_t)(-(p->snr));	
		}
		else
		{
			fw_check[datacount++] = (uint8_t)(p->snr);
		}
		
		if(p->snr_min < 0)
		{
			fw_check[datacount++] =  0x80|(uint8_t)(-( p->snr_min));	
		}
		else
		{
			fw_check[datacount++] = (uint8_t)( p->snr_min);	
		}

		if(p->snr_min < 0)
		{
			fw_check[datacount++] = 0x80|(uint8_t)(-( p->snr_max));	
		}
		else
		{
			fw_check[datacount++] = (uint8_t)( p->snr_max);	
		}
		//RSSI
		if(p->rssi < 0)
		{
			fw_check[datacount++] =  (uint8_t)(-(p->rssi));
		}
		else
		{
			fw_check[datacount++] = 0;
		}
		//时间戳		
		fw_check[datacount++] =   (uint8_t)(p->count_us>>24);		
		fw_check[datacount++] =  (uint8_t)( (p->count_us & 0x00ff0000)>>16);
		fw_check[datacount++] =  (uint8_t)( (p->count_us & 0x0000ff00)>>8);
		fw_check[datacount++] =  (uint8_t) (p->count_us & 0x000000ff);
		
		//CRC校验
		fw_check[datacount++] =   (uint8_t)(p->crc>>8);
		fw_check[datacount++] =   (uint8_t)(p->crc & 0x00ff);
		
		//保留
		fw_check[datacount++] =  0x00;
		fw_check[datacount++] =  0x00;
		//payload
		for(uint8_t i=0; i< p->size; i++)
		{
			fw_check[datacount++] =   p->payload[i];
		}
		//CS
		for(uint8_t j=2; j<datacount; j++)
		{
			fw_check[datacount] += fw_check[j];
		}
		datacount++;
		fw_check[datacount++] = 0x16;
		//HAL_UART_Transmit(&huart1,fw_check,datacount,500);
		Usart2SendData_DMA(fw_check, datacount);
		
                    for (j = 0; j < p->size; ++j) 
                    {
                        printf(" %02X", p->payload[j]);
                    }
                    printf(" #\n");
                } 
                else if (p->status == STAT_CRC_BAD) 
                {
                    printf(" if_chain:%2d", p->if_chain);
                    printf(" tstamp:%010u", p->count_us);
                    printf(" size:%3u\n", p->size);
                    printf(" CRC error, damaged packet\n\n");
                } 
                else if (p->status == STAT_NO_CRC)
                {
                    printf(" if_chain:%2d", p->if_chain);
                    printf(" tstamp:%010u", p->count_us);
                    printf(" size:%3u\n", p->size);
                    printf(" no CRC\n\n");
                } 
                else 
                {
                    printf(" if_chain:%2d", p->if_chain);
                    printf(" tstamp:%010u", p->count_us);
                    printf(" size:%3u\n", p->size);
                    printf(" invalid status ?!?\n\n");
                }
            }
        }
//wyk send data 

//receive mcu data and 
	#if 0
	if(Usart1_RX.receive_flag)
	{
		Usart1_RX.receive_flag = 0;
		//Usart1SendData_DMA(Usart1_RX.RX_Buf, Usart1_RX.rx_len);
	}
	#endif

#if 0
        /* send a packet every X loop */
        if (loop_cnt%16 == 0)
        {
            /* 32b counter in the payload, big endian */
            txpkt.payload[16] = 0xff & (tx_cnt >> 24);
            txpkt.payload[17] = 0xff & (tx_cnt >> 16);
            txpkt.payload[18] = 0xff & (tx_cnt >> 8);
            txpkt.payload[19] = 0xff & tx_cnt;
            i = lgw_send(txpkt); /* non-blocking scheduling of TX packet */
            j = 0;
            printf("+++\nSending packet #%d, rf path %d, return %d\nstatus -> ", tx_cnt, txpkt.rf_chain, i);
            do {
                ++j;
                wait_ms(100);
                lgw_status(TX_STATUS, &status_var); /* get TX status */
                printf("%d:", status_var);
            } while ((status_var != TX_FREE) && (j < 100));
            ++tx_cnt;
            printf("\nTX finished\n");
        }
#endif
    }

    if (exit_sig == 1)
    {
        /* clean up before leaving */
        lgw_stop();
    }

    printf("\nEnd of test for loragw_hal.c\n");
    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
