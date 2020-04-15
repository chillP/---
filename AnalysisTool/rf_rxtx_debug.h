#ifndef __rf_rxtx_debug_H
#define __rf_rxtx_debug_H
#include "stm32l0xx_hal.h"
void SX1301RxTxDebug(int nb_pkt);
void TimeCalibrationSingleRxTx(int nb_pkt);
void TimeCalibrationEachOtherRxTx(int nb_pkt);
#endif
