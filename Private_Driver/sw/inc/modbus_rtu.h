/*
 * modbus_rtu.h
 *
 *  Created on: 19. nov 2018
 *      Author: Toftan2-elekter
 */

#ifndef SW_INC_MODBUS_RTU_H_
#define SW_INC_MODBUS_RTU_H_

/*
 * Includes
 * */
#include "FreeRTOS.h"
#include "semphr.h"

#include "stdint.h"
#include "hw_usart3.h"

/*
 * Defines, macros
 * */
#define MB_RTU_BUFFER_SIZE 256

/*
 * GLOBAL
 * */

/*
 * Functions
 * */

void mb_rtu_init();


void hw_usart3_rx_dma_handler(void);
void hw_usart3_tx_dma_handler(void);
void hw_usart3_rx_idle_handler(void);
/*
 * NVIC functions
 * */

/*
 * Threads
 * */

#endif /* SW_INC_MODBUS_RTU_H_ */
