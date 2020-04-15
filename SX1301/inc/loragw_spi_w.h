#include <stdint.h>        /* C99 types */
#include <stdio.h>        /* printf fprintf */
#include <stdlib.h>        /* malloc free */
#include "stm32f1xx_hal.h"
#include "loragw_spi.h"
#include "loragw_hal.h"





/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */
#define LoRaGW_NSSPIN_UP()        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define LoRaGW_NSSPIN_DOWN()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)

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

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80
#define SPI_SPEED       8000000
#define SPI_DEV_PATH    "/dev/spidev0.0"
//#define SPI_DEV_PATH    "/dev/spidev32766.0"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */


