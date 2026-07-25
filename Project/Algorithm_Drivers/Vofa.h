#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

extern float VOFA_fdata[VOFA_TXDATA_SIZE];
extern uint8_t VOFA_tail[4];
extern uint8_t VOFA_message[VOFA_TXDATA_SIZE * 4 + 4];
extern float VOFA_RxData[VOFA_RXDATA_SIZE];

void VOFA_Rx_CallBack(uint8_t *VOFA_RxDataBuff);

#endif