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
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include "stdint.h"

/*
 * Defines, macros
 * */


typedef struct
{
	USART_TypeDef 	*USARTx;			//USART used USART1,2,3,...
	RCC_TypeDef 	*GPIOx;				//Port used, GPIOA,B,C,...
	uint32_t		Periph_GPIOx;		//Port clock
	uint16_t		pin_rx;				//Pin for RX
	uint16_t		pin_tx;				//Pin for TX
	uint32_t		baudrate;			//Baudrate used
} HW_USART_TypeDef;

typedef struct
{

} HW_USART_DMA_RX_TypeDef;
/*
 * GLOBAL
 * */

/*
 * Functions
 * */
void hw_usart_init(	HW_USART_TypeDef *HW_USART_Conf);


/*
 * NVIC functions
 * */

/*
 * Threads
 * */

#endif /* HW_INC_HW_USART_SIMPLE_H_ */
