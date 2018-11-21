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
#include "stm32f10x.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "stdint.h"

/*
 * Defines, macros
 * */
#define HW_USART3_NVIC_TX_HANDLER		DMA1_Channel2_IRQHandler		//USART3 IRQ for TX DMA is DMA1 Channel 2
#define HW_USART3_NVIC_RX_HANDLER		DMA1_Channel3_IRQHandler		//USART3 IRQ for RX DMA is DMA1 Channel 3
#define HW_USART3_NVIC_USART_HANDLER	USART3_IRQHandler				//USART3 IRQ
/*
 * GLOBAL
 * */

/*
 * Functions
 * */
void hw_usart3_init(uint32_t baud);											//Init USART
void hw_usart3_dma_rx_init(uint32_t buffer_address, uint32_t buffer_size);	//Init RX DMA
void hw_usart3_dma_tx_init(uint32_t buffer_address, uint32_t buffer_size);	//Init TX DMA
uint32_t hw_usart3_send_dma(uint32_t buffer, uint32_t size);				//Send data using DMA
void hw_usart3_receiver_dma(uint32_t buffer, uint32_t size);				//Start RX and wait for data with DMA
void hw_usart3_rx_stop();													//Stop RX
void hw_usart3_mute();														//Mute RX
uint16_t hw_usart3_rx_data_count(uint16_t buffer_size);						//Read DMA data transfered count
/*
 * NVIC functions
 * */
void HW_USART3_NVIC_TX_HANDLER(void);			//IRQ handler for TX if TC transfer complete
void HW_USART3_NVIC_RX_HANDLER(void);			//IRQ handler for RX if TC transfer complete
void HW_USART3_NVIC_USART_HANDLER(void);		//IRQ handler for USART if RX line is idle

extern void hw_usart3_tx_dma_handler(void);		//IRQ USER handler for TX if TC transfer complete
extern void hw_usart3_rx_dma_handler(void);		//IRQ USER handler for RX if TC transfer complete
extern void hw_usart3_rx_idle_handler(void);	//IRQ USER handler for USART if RX line is idle
/*
 * Threads
 * */

#endif /* HW_INC_HW_USART3_H_ */
