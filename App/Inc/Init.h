#ifndef __INIT_H__
#define __INIT_H__

#include "common.h"
#include "include.h"

#define NRF_OFF 0
#define NRF_ON 1

extern uint8 * imgbuff;
extern uint8 nrf_rx_buff[];
extern uint8 nrf_tx_buff[];

void All_init();
#endif