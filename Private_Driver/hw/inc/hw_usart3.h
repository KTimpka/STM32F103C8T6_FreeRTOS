/*
 * hw_usart3.h
 *
 *  Created on: 12. nov 2018
 *      Author: Sad
 */

#ifndef HW_INC_HW_USART3_H_
#define HW_INC_HW_USART3_H_


/*
 * Includes
 * */
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "stdint.h"

/*
 * Defines, macros
 * */
#define HW_USART3_NVIC_TX_HANDLER		DMA1_Channel2_IRQHandler
#define HW_USART3_NVIC_RX_HANDLER		DMA1_Channel3_IRQHandler
#define HW_USART3_NVIC_USART_HANDLER	USART3_IRQHandler
/*
 * GLOBAL
 * */

/*
 * Functions
 * */
void hw_usart3_init(uint32_t baud);
void hw_usart3_dma_rx_init(uint32_t buffer_address, uint32_t buffer_size);
void hw_usart3_dma_tx_init(uint32_t buffer_address, uint32_t buffer_size);
uint32_t hw_usart3_send_dma(uint32_t buffer, uint32_t size);
void hw_usart3_receive_dma(uint32_t buffer, uint32_t size);
/*
 * NVIC functions
 * */
void HW_USART3_NVIC_TX_HANDLER(void);
void HW_USART3_NVIC_RX_HANDLER(void);
void HW_USART3_NVIC_USART_HANDLER(void);

extern void hw_usart3_rx_dma_handler(void);
extern void hw_usart3_rx_idle_handler(void);
/*
 * Threads
 * */

#endif /* HW_INC_HW_USART3_H_ */
