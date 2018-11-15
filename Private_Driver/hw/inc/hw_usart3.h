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

#define USART_USED			USART3					//USART in use APB1PeriphClock
#define PORT_USED			GPIOB					//Port in use
#define	USART_RCC			RCC_APB1Periph_USART3	//USART Clock
#define PORT_RCC			RCC_APB2Periph_GPIOB	//Port clock
#define PIN_TX				GPIO_Pin_11				//TX port number
#define PIN_RX				GPIO_Pin_10				//RX poer number
#define DMA_RCC				RCC_AHBPeriph_DMA1		//DMA clock
#define DMA_RX_CHANNEL		DMA1_Channel3			//DMA RX channel
#define DMA_TX_CHANNEL		DMA1_Channel2			//DMA TX channel
#define DMA_NVIC_TX_CHANNEL	DMA1_Channel2_IRQn		//NVIC DMA interruption
#define DMA_NVIC_RX_CHANNEL	DMA1_Channel3_IRQn		//NVIC DMA interruption
#define DMA_NVIC_TX_FLAG	DMA1_FLAG_TC2			//NVIC DMA Flag to check
#define DMA_NVIC_RX_FLAG	DMA1_FLAG_TC3			//NVIC DMA Flag to check


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
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);

extern void DMA1_Channel3_IRQHandler_user(void);
/*
 * Threads
 * */

#endif /* HW_INC_HW_USART3_H_ */
