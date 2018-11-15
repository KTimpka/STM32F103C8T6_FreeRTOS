/*
 * hw_usart3.c
 *
 *  Created on: 12. nov 2018
 *      Author: Sad
 */

#include "hw_usart3.h"

static SemaphoreHandle_t G_WRITE_LOCK;

/*
 *
 * Configure USART hardware
 * 		USART		->	USART3
 * 		PIN_TX		->	GPIO_Pin_10
 * 		PIN_RX		->	GPIO_Pin_11
 * 		BAUDRATE	->	USER_DEFINED
 * 		DATA_LENGTH	->	8Bit
 * 		PARITY		->	EVEN
 * 		STOP		->	1Bit
 * */
void hw_usart3_init(uint32_t baud)
{
	//Start USART, Port, AF clock
	RCC_APB1PeriphClockCmd(USART_RCC, ENABLE);				//USART3
	RCC_APB2PeriphClockCmd(PORT_RCC, ENABLE);				//PORTB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//AF Port

	//Pin config struct
	GPIO_InitTypeDef GPIO_Conf;

	//Configure Port
	GPIO_Conf.GPIO_Pin 		= PIN_TX;
	GPIO_Conf.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_Conf.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
	GPIO_Init(PORT_USED, &GPIO_Conf);

	//Configure Port
	GPIO_Conf.GPIO_Pin 		= PIN_RX;
	//GPIO_Conf.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_Conf.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(PORT_USED, &GPIO_Conf);

	//USART config struct
	USART_InitTypeDef USART_Conf;

	USART_Conf.USART_BaudRate				=	baud;
	USART_Conf.USART_WordLength				=	USART_WordLength_9b;
	USART_Conf.USART_StopBits				=	USART_StopBits_1;
	USART_Conf.USART_Parity					=	USART_Parity_Even;
	USART_Conf.USART_Mode					=	USART_Mode_Rx | USART_Mode_Tx;
	USART_Conf.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
	USART_Init(USART_USED, &USART_Conf);		//Apply to hardware
	USART_WakeUpConfig(USART_WakeUp_IdleLine);	//Wake up if idle line is detected
	USART_Cmd(USART_USED, ENABLE);				//Start USART
}

/*
 * Enable USART RX DMA
 *
 * @param uint32_t	buffer_address 	: Buffer start address
 * @param uint32_t buffer_size		: Buffer size, can't be zero
 * */
void hw_usart3_dma_rx_init(uint32_t buffer_address, uint32_t buffer_size)
{
	//Start DMA
	RCC_AHBPeriphClockCmd(DMA_RCC, ENABLE);

	DMA_InitTypeDef DMA_Conf;

	DMA_Conf.DMA_PeripheralBaseAddr	=	(uint32_t)&(USART_USED->DR);//USART read address
	DMA_Conf.DMA_MemoryBaseAddr		=	buffer_address;				//Memory array address
	DMA_Conf.DMA_DIR				=	DMA_DIR_PeripheralSRC;		//Peripheral as source
	DMA_Conf.DMA_BufferSize			=	buffer_size;				//Buffer size
	DMA_Conf.DMA_PeripheralInc		=	DMA_PeripheralInc_Disable;	//Disable peripheral address increase
	DMA_Conf.DMA_MemoryInc			=	DMA_MemoryInc_Enable;		//Enable memory address increase
	DMA_Conf.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte; //Take 1 byte
	DMA_Conf.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;	//Put 1 byte
	DMA_Conf.DMA_Mode				=	DMA_Mode_Normal;			//Normal mode
	DMA_Conf.DMA_Priority			=	DMA_Priority_Low;			//USART is not that important
	DMA_Conf.DMA_M2M				=	DMA_M2M_Disable;			//Peripheral to memory

	DMA_Init(DMA_RX_CHANNEL, &DMA_Conf);							//Init DMA for RX
	USART_DMACmd(USART_USED, USART_DMAReq_Rx, ENABLE);				//Enable USART DMA request
	DMA_ITConfig(DMA_RX_CHANNEL, DMA_NVIC_RX_FLAG, ENABLE);			//Enable DMA NVIC transfer complete

	NVIC_InitTypeDef NVIC_conf;

	NVIC_conf.NVIC_IRQChannel						=	DMA_NVIC_RX_CHANNEL;//NVIC interrupt vector
	NVIC_conf.NVIC_IRQChannelPreemptionPriority		=	15;					//Priority 0...15	Not important at all
	NVIC_conf.NVIC_IRQChannelSubPriority			=	0;					//Sub Priority not used
	NVIC_conf.NVIC_IRQChannelCmd					=	ENABLE;				//Enable DMA1_Channel3 interrupt
	NVIC_Init(&NVIC_conf);
}

/*
 * Enable USART TX DMA
 *
 * @param uint32_t	buffer_address 	: Buffer start address
 * @param uint32_t buffer_size		: Buffer size, can't be zero
 * */
void hw_usart3_dma_tx_init(uint32_t buffer_address, uint32_t buffer_size)
{
	//Start DMA
	RCC_AHBPeriphClockCmd(DMA_RCC, ENABLE);

	DMA_InitTypeDef DMA_Conf;

	DMA_Conf.DMA_PeripheralBaseAddr	=	(uint32_t)&(USART_USED->DR);//USART read address
	DMA_Conf.DMA_MemoryBaseAddr		=	buffer_address;				//Buffer address will be set when there is data to send
	DMA_Conf.DMA_DIR				=	DMA_DIR_PeripheralDST;		//Peripheral as destination
	DMA_Conf.DMA_BufferSize			=	buffer_size;				//Buffer size will be set when there is data to send
	DMA_Conf.DMA_PeripheralInc		=	DMA_PeripheralInc_Disable;	//Disable peripheral address increase
	DMA_Conf.DMA_MemoryInc			=	DMA_MemoryInc_Enable;		//Enable memory address increase
	DMA_Conf.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;//Take 1 byte
	DMA_Conf.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;	//Put 1 byte
	DMA_Conf.DMA_Mode				=	DMA_Mode_Normal;			//Normal mode
	DMA_Conf.DMA_Priority			=	DMA_Priority_Low;			//USART is not that important
	DMA_Conf.DMA_M2M				=	DMA_M2M_Disable;			//Peripheral to memory

	DMA_Init(DMA_TX_CHANNEL, &DMA_Conf);							//Configure DMA
	USART_DMACmd(USART_USED, USART_DMAReq_Tx, ENABLE);				//Enable USART DMA request
	DMA_ITConfig(DMA_TX_CHANNEL, DMA_NVIC_TX_FLAG, ENABLE);			//Enable DMA NVIC transfer complete


	NVIC_InitTypeDef NVIC_conf;

	NVIC_conf.NVIC_IRQChannel						=	DMA_NVIC_TX_CHANNEL;//NVIC interrupt vector
	NVIC_conf.NVIC_IRQChannelPreemptionPriority		=	15;					//Priority 0...15	Not important at all
	NVIC_conf.NVIC_IRQChannelSubPriority			=	0;					//Sub Priority not used
	NVIC_conf.NVIC_IRQChannelCmd					=	ENABLE;				//Enable DMA1_Channel3 interrupt
	NVIC_Init(&NVIC_conf);

	G_WRITE_LOCK = xSemaphoreCreateBinary();//Create semaphore to lock sending if DMA is still running
	xSemaphoreGive(G_WRITE_LOCK);			//Enable semaphore
}

/*
 * Send out buffer using DMA
 * @param uint32_t buffer		=	buffer address
 * @param uint32_t size			=	buffer size
 *
 * @return	size if DMA was free, 0 if previous sent was in action
 * */
uint32_t hw_usart3_send_dma(uint32_t buffer, uint32_t size)
{
	//If previous send is still active return 0
	if (xSemaphoreTake(G_WRITE_LOCK, 0) == pdFAIL)
		return 0;

	//DMA_Cmd(DMA_TX_CHANNEL, DISABLE);	//Stop DMA
	DMA_TX_CHANNEL->CNDTR	=	size;	//Update buffer size
	DMA_TX_CHANNEL->CMAR	=	buffer;	//Update buffer address
	DMA_Cmd(DMA_TX_CHANNEL, ENABLE);	//Start DMA

	return size;
}

/*
 * Wait for USART package using DMA
 * If buffer is full transfer complete interruption is generated
 * @param uint32_t buffer		=	buffer address
 * @param uint32_t size			=	buffer size
 *
 * */
void hw_usart3_receive_dma(uint32_t buffer, uint32_t size)
{
	//DMA_Cmd(DMA_RX_CHANNEL, DISABLE);	//Stop DMA
	DMA_RX_CHANNEL->CNDTR	=	size;	//Update buffer size
	DMA_RX_CHANNEL->CMAR	=	buffer;	//Update buffer address
	DMA_Cmd(DMA_RX_CHANNEL, ENABLE);	//Start DMA
}

/*
 * Mute channel until idle line is detected
 * */
void hw_usart3_mute()
{
	//USART_USED->CR1
}

/*
 * DMA1 Channel2 Interruption handler
 * If data is sent out, transfer complete interruption is generated.
 * With this we can reset G_WRITE_LOCK.
 * */
void DMA1_Channel2_IRQHandler(void)
{
	if (DMA_GetFlagStatus(DMA_NVIC_TX_FLAG)) {
		DMA_ClearFlag(DMA_NVIC_TX_FLAG);		//Clear flag
		DMA_Cmd(DMA_TX_CHANNEL, DISABLE);	//Stop DMA
		xSemaphoreGive(G_WRITE_LOCK);		//Reset lock
	}
}

/*
 * DMA1 Channel3 Interruption handler
 * If DMA buffer is full, transfer complete interruption is generated.
 * User has to define DMA1_Channel3_IRQHandler_user() function.
 * */
void DMA1_Channel3_IRQHandler(void)
{
	if (DMA_GetFlagStatus(DMA_NVIC_RX_FLAG)) {
		DMA_ClearFlag(DMA_NVIC_RX_FLAG);	//Clear flag
		DMA_Cmd(DMA_RX_CHANNEL, DISABLE);	//Stop DMA
		DMA1_Channel3_IRQHandler_user();	//Reset lock
	}
}
