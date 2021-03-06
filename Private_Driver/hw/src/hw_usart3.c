/*
 * hw_usart3.c
 *
 *  Created on: 12. nov 2018
 *      Author: Sad
 */

#include "hw_usart3.h"

#define USART_USED				USART3					//USART in use APB1PeriphClock
#define PORT_USED				GPIOB					//Port in use
#define	USART_RCC				RCC_APB1Periph_USART3	//USART Clock
#define PORT_RCC				RCC_APB2Periph_GPIOB	//Port clock
#define PIN_RX					GPIO_Pin_11				//RX port number
#define PIN_TX					GPIO_Pin_10				//TX port number
#define DMA_RCC					RCC_AHBPeriph_DMA1		//DMA clock
#define DMA_RX_CHANNEL			DMA1_Channel3			//DMA RX channel
#define DMA_TX_CHANNEL			DMA1_Channel2			//DMA TX channel
#define DMA_NVIC_TX_CHANNEL		DMA1_Channel2_IRQn		//NVIC DMA interruption
#define DMA_NVIC_RX_CHANNEL		DMA1_Channel3_IRQn		//NVIC DMA interruption
#define DMA_NVIC_TX_FLAG		DMA1_FLAG_TC2			//NVIC DMA Flag to check
#define DMA_NVIC_RX_FLAG		DMA1_FLAG_TC3			//NVIC DMA Flag to check
#define USART_NVIC_IDLE_CHANNEL	USART3_IRQn				//NVIC USART3 interruption



static SemaphoreHandle_t G_WRITE_LOCK;	//Lock TX if DMA is still in action

/*
 * @param uint32_t baud	: User defined BAUDRATE
 *
 * Configure USART hardware
 * 		USART		->	USART3
 * 		PIN_RX		->	GPIO_Pin_10
 * 		PIN_TX		->	GPIO_Pin_11
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
	GPIO_Conf.GPIO_Pin 		= PIN_RX;
	GPIO_Conf.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_Conf.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
	GPIO_Init(PORT_USED, &GPIO_Conf);

	//Configure Port
	GPIO_Conf.GPIO_Pin 		= PIN_TX;
	//GPIO_Conf.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_Conf.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(PORT_USED, &GPIO_Conf);

	//USART config struct
	USART_InitTypeDef USART_Conf;

	USART_Conf.USART_BaudRate				=	baud;								//BAUD RATE
	USART_Conf.USART_WordLength				=	USART_WordLength_9b;				//8bits of data + 1 bit of parity
	USART_Conf.USART_StopBits				=	USART_StopBits_1;					//1bit for stop
	USART_Conf.USART_Parity					=	USART_Parity_Even;					//Parity even
	USART_Conf.USART_Mode					=	USART_Mode_Rx | USART_Mode_Tx;		//Enable RX and TX
	USART_Conf.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;		//No HW control
	USART_Init(USART_USED, &USART_Conf);					//Apply to hardware
	USART_WakeUpConfig(USART_USED, USART_WakeUp_IdleLine);	//Wake up if idle line is detected
	USART_Cmd(USART_USED, ENABLE);							//Start USART
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
	USART_ITConfig(USART_USED, USART_IT_IDLE, ENABLE);				//Enable USART idle line interruption
	DMA_ITConfig(DMA_RX_CHANNEL, DMA_IT_TC, ENABLE);				//Enable DMA NVIC transfer complete

	NVIC_InitTypeDef NVIC_conf;

	NVIC_conf.NVIC_IRQChannel						=	DMA_NVIC_RX_CHANNEL;//NVIC interrupt vector
	NVIC_conf.NVIC_IRQChannelPreemptionPriority		=	15;					//Priority 0...15	Not important at all
	NVIC_conf.NVIC_IRQChannelSubPriority			=	0;					//Sub Priority not used
	NVIC_conf.NVIC_IRQChannelCmd					=	ENABLE;				//Enable DMA1_Channel3 interrupt
	NVIC_Init(&NVIC_conf);

	NVIC_conf.NVIC_IRQChannel						=	USART_NVIC_IDLE_CHANNEL;//NVIC interrupt vector
	//NVIC_conf.NVIC_IRQChannelPreemptionPriority	=	15;						//Priority 0...15	Not important at all
	//NVIC_conf.NVIC_IRQChannelSubPriority			=	0;						//Sub Priority not used
	//NVIC_conf.NVIC_IRQChannelCmd					=	ENABLE;					//Enable DMA1_Channel3 interrupt
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
	DMA_ITConfig(DMA_TX_CHANNEL, DMA_IT_TC, ENABLE);				//Enable DMA NVIC transfer complete


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

	DMA_Cmd(DMA_TX_CHANNEL, DISABLE);	//Stop DMA
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
void hw_usart3_receiver_dma(uint32_t buffer, uint32_t size)
{
	DMA_Cmd(DMA_RX_CHANNEL, DISABLE);			//Stop DMA
	DMA_RX_CHANNEL->CNDTR	=	size;			//Update buffer size
	DMA_RX_CHANNEL->CMAR	=	buffer;			//Update buffer address
	DMA_Cmd(DMA_RX_CHANNEL, ENABLE);			//Start DMA
	USART_USED->CR1			|=	USART_CR1_RE;	//Enable receiver
}

/*
 * Disable receiver
 * */
void hw_usart3_rx_stop()
{
	USART_USED->CR1	&= ~USART_CR1_RE;	//Disable receiver
}

/*
 * Return rx data count
 * @param uint16_t buffer_size :	buffer used for DMA to calculate data count,
 * 									(buffer_size - dma_free_space)
 * */
uint16_t hw_usart3_rx_data_count(uint16_t buffer_size)
{
	return buffer_size - DMA_GetCurrDataCounter(DMA_RX_CHANNEL);
}

/*
 * Mute channel until idle line is detected
 * Before selecting Mute mode (by setting the RWU bit) the USART must first receive a
 * data byte, otherwise it cannot function in Mute mode with wakeup by Idle line detection.
 * */
void hw_usart3_mute()
{
	USART_USED->CR1 |= USART_CR1_RWU;	//Receiver to mute mode
}

/*
 * DMA1 Channel2 Interruption handler
 * If data is sent out, transfer complete interruption is generated.
 * With this we can reset G_WRITE_LOCK.
 * */
void HW_USART3_NVIC_TX_HANDLER(void)
{
	if (DMA_GetFlagStatus(DMA_NVIC_TX_FLAG)) {
		DMA_ClearFlag(DMA_NVIC_TX_FLAG);	//Clear flag
		DMA_Cmd(DMA_TX_CHANNEL, DISABLE);	//Stop DMA
		xSemaphoreGive(G_WRITE_LOCK);		//Reset lock
		hw_usart3_tx_dma_handler();			//User defined handler after flag is removed
	}
}

/*
 * DMA1 Channel3 Interruption handler
 * If DMA buffer is full, transfer complete interruption is generated.
 * User has to define DMA1_Channel3_IRQHandler_user() function.
 * */
void HW_USART3_NVIC_RX_HANDLER(void)
{
	if (DMA_GetFlagStatus(DMA_NVIC_RX_FLAG)) {
		DMA_ClearFlag(DMA_NVIC_RX_FLAG);	//Clear flag
		DMA_Cmd(DMA_RX_CHANNEL, DISABLE);	//Stop DMA
		hw_usart3_rx_dma_handler();			//User defined handler after flag is removed
	}
}

/*
 * USART3 Interruption handler
 * */
void HW_USART3_NVIC_USART_HANDLER(void)
{
	if (USART_GetITStatus(USART_USED, USART_IT_IDLE)) {
		/*
		 * The PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun error) and IDLE
		 * (Idle line detected) flags are cleared by a software sequence: a read operation to the
		 * USART_SR register (USART_GetFlagStatus()) followed by a read operation to the
		 * USART_DR register (USART_ReceiveData()).
		 * */
		USART_ReceiveData(USART_USED);	//Clear flag
		hw_usart3_rx_idle_handler();	//User defined handler after flag is removed
	}
}
