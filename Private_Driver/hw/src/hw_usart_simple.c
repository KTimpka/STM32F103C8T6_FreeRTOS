/*
 * hw_usart.c
 *
 *  Created on: 4. nov 2018
 *      Author: Sad
 */
#include <hw_usart_simple.h>


SemaphoreHandle_t G_SEND_LOCK_1;
SemaphoreHandle_t G_SEND_LOCK_2;
SemaphoreHandle_t G_SEND_LOCK_3;


/*
 * Configure USART hardware
 * 		USART		->	USER_DEFINED
 * 		PIN_TX		->	USER_DEFINED
 * 		PIN_RX		->	USER_DEFINED
 * 		BAUDRATE	->	USER_DEFINED
 * 		DATA_LENGTH	->	8Bit
 * 		PARITY		->	EVEN
 * 		STOP		->	1Bit
 *
 * 	For USART3
 * 	HW_USART_TypeDef HW_USART_Conf;
 * 	HW_USART_Conf.USARTx 		= 	USART3;
 *	HW_USART_Conf.Periph_USARTx	=	RCC_APB1Periph_USART3;
 *	HW_USART_Conf.GPIOx			=	GPIOB;
 *	HW_USART_Conf.Periph_GPIOx	=	RCC_APB2Periph_GPIOB;
 *	HW_USART_Conf.pin_rx		=	GPIO_Pin_11;
 *	HW_USART_Conf.pin_tx		=	GPIO_Pin_10;
 *	HW_USART_Conf.baudrate		=	9600;
 *
 * */

void hw_usart_init(HW_USART_TypeDef *HW_USART_Conf)
{
	//Start USART, Port, AF clock
	if (HW_USART_Conf->USARTx == USART1)
		RCC_APB2PeriphClockCmd(HW_USART_Conf->Periph_USARTx, ENABLE);
	else
		RCC_APB1PeriphClockCmd(HW_USART_Conf->Periph_USARTx, ENABLE);
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
	USART_Cmd(HW_USART_Conf->USARTx, ENABLE);
}

/*
 * Enable USART RX DMA
 *
 * For USART3
 * HW_USART_DMA_RX_TypeDef HW_USART_DMA_RX_Conf;
 * HW_USART_DMA_RX_Conf.USARTx				=	USART3;
 * HW_USART_DMA_RX_Conf.address_usart		=	&(USART3->DR);
 * HW_USART_DMA_RX_Conf.address_memory		=	buffer;
 * HW_USART_DMA_RX_Conf.memory_size		=	BUFFER_SIZE;
 * */
void hw_usart_dma_rx_init(HW_USART_DMA_RX_TypeDef *HW_USART_DMA_Conf)
{
	//Start DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_Conf;

	DMA_Conf.DMA_PeripheralBaseAddr	=	HW_USART_DMA_Conf->address_usart;	//USART read address
	DMA_Conf.DMA_MemoryBaseAddr		=	HW_USART_DMA_Conf->address_memory;	//Memory array address
	DMA_Conf.DMA_DIR				=	DMA_DIR_PeripheralSRC;				//Peripheral as source
	DMA_Conf.DMA_BufferSize			=	HW_USART_DMA_Conf->memory_size;		//Buffer size
	DMA_Conf.DMA_PeripheralInc		=	DMA_PeripheralInc_Disable;			//Disable peripheral address increase
	DMA_Conf.DMA_MemoryInc			=	DMA_MemoryInc_Enable;				//Enable memory address increase
	DMA_Conf.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte; 		//Take 1 byte
	DMA_Conf.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;			//Put 1 byte
	DMA_Conf.DMA_Mode				=	DMA_Mode_Normal;					//Normal mode
	DMA_Conf.DMA_Priority			=	DMA_Priority_Low;					//USART is not that important
	DMA_Conf.DMA_M2M				=	DMA_M2M_Disable;					//Peripheral to memory


	//Configure channel and enable it
	if (HW_USART_DMA_Conf->USARTx == USART1) {
		DMA_Init(DMA1_Channel5, &DMA_Conf);
	}else if (HW_USART_DMA_Conf->USARTx == USART2){
		DMA_Init(DMA1_Channel6, &DMA_Conf);
	}
	else {	//USART3
		DMA_Init(DMA1_Channel3, &DMA_Conf);
	}

	//Enable USART DMA
	USART_DMACmd(HW_USART_DMA_Conf->USARTx, USART_DMAReq_Rx, ENABLE);
}

/*
 * Enable USART TX DMA
 *
 * For USART3
 * HW_USART_DMA_TX_TypeDef HW_USART_DMA_TX_Conf;
 * HW_USART_DMA_TX_Conf.USARTx				=	USART3;
 * HW_USART_DMA_TX_Conf.address_usart		=	&(USART3->DR);
 * */
void hw_usart_dma_tx_init(HW_USART_DMA_TX_TypeDef *HW_USART_DMA_Conf)
{
	SemaphoreHandle_t *lock;
	DMA_Channel_TypeDef	*dma;

	//Start DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_Conf;

	DMA_Conf.DMA_PeripheralBaseAddr	=	HW_USART_DMA_Conf->address_usart;	//USART read address
	DMA_Conf.DMA_MemoryBaseAddr		=	0;									//Buffer address will be set when there is data to send
	DMA_Conf.DMA_DIR				=	DMA_DIR_PeripheralDST;				//Peripheral as destination
	DMA_Conf.DMA_BufferSize			=	1;									//Buffer size will be set when there is data to send
	DMA_Conf.DMA_PeripheralInc		=	DMA_PeripheralInc_Disable;			//Disable peripheral address increase
	DMA_Conf.DMA_MemoryInc			=	DMA_MemoryInc_Enable;				//Enable memory address increase
	DMA_Conf.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte; 		//Take 1 byte
	DMA_Conf.DMA_MemoryDataSize		=	DMA_MemoryDataSize_Byte;			//Put 1 byte
	DMA_Conf.DMA_Mode				=	DMA_Mode_Normal;					//Normal mode
	DMA_Conf.DMA_Priority			=	DMA_Priority_Low;					//USART is not that important
	DMA_Conf.DMA_M2M				=	DMA_M2M_Disable;					//Peripheral to memory


	NVIC_InitTypeDef NVIC_conf;

	//NVIC_conf.NVIC_IRQChannel						=	DMA1_Channel3_IRQn;	//NVIC interrupt vector
	NVIC_conf.NVIC_IRQChannelPreemptionPriority		=	15;					//Priority 0...15	Not important at all
	NVIC_conf.NVIC_IRQChannelSubPriority			=	0;					//Sub Priority not used
	NVIC_conf.NVIC_IRQChannelCmd					=	ENABLE;				//Enable DMA1_Channel3 interrupt


	//Configure channel
	if (HW_USART_DMA_Conf->USARTx == USART1) {
		lock 						= &G_SEND_LOCK_1;		//Semaphore used
		dma 						= DMA1_Channel4;		//Channel used
		NVIC_conf.NVIC_IRQChannel	= DMA1_Channel4_IRQn;	//Channel used
	}else if (HW_USART_DMA_Conf->USARTx == USART2){
		lock 						= &G_SEND_LOCK_2;		//Semaphore used
		dma 						= DMA1_Channel7;		//Channel used
		NVIC_conf.NVIC_IRQChannel	= DMA1_Channel7_IRQn;	//Channel used
	}else {	//USART3
		lock 						= &G_SEND_LOCK_3;		//Semaphore used
		dma 						= DMA1_Channel2;		//Channel used
		NVIC_conf.NVIC_IRQChannel	= DMA1_Channel2_IRQn;	//Channel used
	}

	USART_DMACmd(HW_USART_DMA_Conf->USARTx, USART_DMAReq_Tx, ENABLE);	//Enable USART DMA
	*lock = xSemaphoreCreateBinary();		//Create semaphore to lock sending if DMA is still running
	xSemaphoreGive(*lock);					//Enable semaphore
	DMA_Init(dma, &DMA_Conf);				//Configure DMA
	DMA_ITConfig(dma, DMA_IT_TC, ENABLE);	//Enable DMA NVIC transfer complete
	NVIC_Init(&NVIC_conf);					//Enable interrupt
}

/*
 * Send out buffer using DMA
 * @param USART_TypeDef *USARTx	=	USART1,2,3,...
 * @param uint32_t *data		=	buffer address
 * @param uint32_t size			=	buffer size\
 *
 * @return	size if DMA was free, 0 if previous sent was in action
 * */
uint8_t	hw_usart_send_dma(USART_TypeDef *USARTx, uint8_t *data, uint32_t size)
{
	DMA_Channel_TypeDef *DMAx;
	SemaphoreHandle_t	lock;

	if (USARTx == USART1) {
		DMAx = DMA1_Channel4;
		lock = G_SEND_LOCK_1;
	}else if (USARTx== USART2){
		DMAx = DMA1_Channel7;
		lock = G_SEND_LOCK_2;
	}
	else {	//USART3
		DMAx = DMA1_Channel2;
		lock = G_SEND_LOCK_3;
	}

	//Previous send is still active
	if (xSemaphoreTake(lock, 0) == pdFAIL)
		return 0;

	DMA_Cmd(DMAx, DISABLE);			//Stop DMA
	DMAx->CNDTR	=	size;			//Update buffer size
	DMAx->CMAR	=	(uint32_t)data;	//Update buffer address
	DMA_Cmd(DMAx, ENABLE);			//Start DMA
	//USARTx->SR &=	!(1<<6);

	return size;
}

void DMA1_Channel4_IRQHandler(void)
{
	if (DMA_GetFlagStatus(DMA1_FLAG_TC4)) {
		DMA_ClearFlag(DMA1_FLAG_TC4);
		DMA_Cmd(DMA1_Channel4, DISABLE);
		xSemaphoreGive(G_SEND_LOCK_1);
	}
}
void DMA1_Channel7_IRQHandler(void)
{
	if (DMA_GetFlagStatus(DMA1_FLAG_TC7)) {
		DMA_ClearFlag(DMA1_FLAG_TC7);
		DMA_Cmd(DMA1_Channel7, DISABLE);
		xSemaphoreGive(G_SEND_LOCK_2);
	}
}
/*
void DMA1_Channel2_IRQHandler(void)
{
	if (DMA_GetFlagStatus(DMA1_FLAG_TC2)) {
		DMA_ClearFlag(DMA1_FLAG_TC2);
		DMA_Cmd(DMA1_Channel2, DISABLE);
		xSemaphoreGive(G_SEND_LOCK_3);
	}
}
*/

