/*
 * hw_usart.h
 *
 *  Created on: 4. nov 2018
 *      Author: Sad
 */

#ifndef HW_INC_HW_USART_SIMPLE_H_
#define HW_INC_HW_USART_SIMPLE_H_

/*
 *
 * */

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


typedef struct
{
	USART_TypeDef 	*USARTx;			//USART used	USART1,2,3,...
	GPIO_TypeDef 	*GPIOx;				//Port used 	GPIOA,B,C,...
	uint32_t		Periph_GPIOx;		//Port clock	RCC_APB2Periph_GPIOA,B,C,...
	uint32_t		Periph_USARTx;		//USART clock	RCC_APB2Periph_USART1,2,3,...
	uint16_t		pin_rx;				//Pin for RX
	uint16_t		pin_tx;				//Pin for TX
	uint32_t		baudrate;			//Baudrate used
} HW_USART_TypeDef;

typedef struct
{
	USART_TypeDef 	*USARTx;			//USART used	USART1,2,3,...
	uint32_t		address_usart;		//USART rx address
	uint32_t		address_memory;		//RX buffer address
	uint32_t		memory_size;		//RX buffer size

} HW_USART_DMA_RX_TypeDef;

typedef struct
{
	USART_TypeDef 	*USARTx;			//USART used	USART1,2,3,...
	uint32_t		address_usart;		//USART tx address
} HW_USART_DMA_TX_TypeDef;

//#define HW_USART_IRQ_4
//#define HW_USART_IRQ_7
#define HW_USART_IRQ_2

/*
 * GLOBAL
 * */

/*
 * Functions
 * */
void hw_usart_init(	HW_USART_TypeDef *HW_USART_Conf);
void hw_usart_dma_rx_init(HW_USART_DMA_RX_TypeDef *HW_USART_DMA_Conf);
void hw_usart_dma_tx_init(HW_USART_DMA_TX_TypeDef *HW_USART_DMA_Conf);
uint8_t	hw_usart_send_dma(USART_TypeDef *USARTx, uint8_t *data, uint32_t size);

/*
 * NVIC functions
 * */
#ifdef HW_USART_IRQ_4
void DMA1_Channel4_IRQHandler(void);
#endif
#ifdef HW_USART_IRQ_7
void DMA1_Channel7_IRQHandler(void);
#endif
#ifdef HW_USART_IRQ_2
void DMA1_Channel2_IRQHandler(void);
#endif

/*
 * Threads
 * */

#endif /* HW_INC_HW_USART_SIMPLE_H_ */
