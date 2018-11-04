/*
 * hw_usart.c
 *
 *  Created on: 4. nov 2018
 *      Author: Sad
 */
#include <hw_usart_simple.h>

/*
 * Configure USART hardware
 * 		USART		->	USER_DEFINED
 * 		PIN_TX		->	USER_DEFINED
 * 		PIN_RX		->	USER_DEFINED
 * 		BAUDRATE	->	USER_DEFINED
 * 		DATA_LENGTH	->	8Bit
 * 		PARITY		->	EVEN
 * 		STOP		->	1Bit
 * */
void hw_usart_init(HW_USART_TypeDef *HW_USART_Conf)
{
	//Start Port, AF clock
	RCC_APB2PeriphClockCmd(HW_USART_Conf->Periph_GPIOx, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//Pin config struct
	GPIO_InitTypeDef GPIO_Conf;

	//Configure Port
	GPIO_Conf.GPIO_Pin 		= HW_USART_Conf->pin_rx;
	GPIO_Conf.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_Conf.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
	GPIO_Init(HW_USART_Conf->GPIOx, &GPIO_Conf);

	//Configure Port
	GPIO_Conf.GPIO_Pin 		= HW_USART_Conf->pin_tx;
	//GPIO_Conf.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_Conf.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(HW_USART_Conf->GPIOx, &GPIO_Conf);

	//USART config struct
	USART_InitTypeDef USART_Conf;

	USART_Conf.USART_BaudRate				=	HW_USART_Conf->baudrate;
	USART_Conf.USART_WordLength				=	USART_WordLength_9b;
	USART_Conf.USART_StopBits				=	USART_StopBits_1;
	USART_Conf.USART_Parity					=	USART_Parity_Even;
	USART_Conf.USART_Mode					=	USART_Mode_Rx | USART_Mode_Tx;
	USART_Conf.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
	USART_Init(HW_USART_Conf->USARTx, &USART_Conf);
}

void hw_usart_dma_rx_init(HW_USART_DMA_RX_TypeDef *HW_USART_DMA_Conf)
{

}
