/*
 * modbus_rtu.c
 *
 *  Created on: 19. nov 2018
 *      Author: Toftan2-elekter
 */

#include "modbus_rtu.h"


static volatile uint8_t G_IO_BUFFER[MB_RTU_BUFFER_SIZE];	//Buffer for Modbus frame
static uint8_t			G_FRAME_SIZE;						//Frame size that was sent to us
static uint8_t			G_LAST_BUFFER_SIZE;					//Buffer size used
static uint8_t			G_MODBUS_ADDRESS;					//Local Modbus address

static SemaphoreHandle_t G_WAIT_FOR_FRAME;

static void mb_rtu_thread(void *pvParameters);


/*
 * Initialize hardware and buffer for Modbus RTU.
 * */
void mb_rtu_init()
{
	G_MODBUS_ADDRESS 	= 	0x08;	//Default address is 8
	G_FRAME_SIZE		=	0x00;	//Clean frame size
	G_LAST_BUFFER_SIZE	=	0x01;	//Fist get address then data if for us
	G_WAIT_FOR_FRAME	=	xSemaphoreCreateBinary();	//Make semaphore

	hw_usart3_init(9600);	//Default baud rate is 9600
	hw_usart3_dma_rx_init((uint32_t)G_IO_BUFFER, (uint32_t)MB_RTU_BUFFER_SIZE);			//Start dma
	hw_usart3_dma_tx_init((uint32_t)G_IO_BUFFER, (uint32_t)MB_RTU_BUFFER_SIZE);			//Start dma
	hw_usart3_receiver_dma((uint32_t)G_IO_BUFFER, (uint32_t)G_LAST_BUFFER_SIZE);		//Start receiver

	xTaskCreate( mb_rtu_thread, "Modbus thread", configMINIMAL_STACK_SIZE, 0, 0, NULL );//Start thread
}

static void mb_rtu_thread(void *pvParameters)
{
	/*
	 * Main loop for Modbus
	 * */
	while (1) {


	}
	for (;;);
}


void hw_usart3_rx_dma_handler(void)
{
	if (G_LAST_BUFFER_SIZE == 1) {
		/*
		 * If last buffer size was 1 then we are looking for address.
		 * If address matches then expose full buffer for rx.
		 * If address does not match mute rx and leave buffer 1 for new address.
		 * */
		if (G_IO_BUFFER[0] == G_MODBUS_ADDRESS) {
			/*
			 * If address matches then allow buffer to be filled.
			 * */
			G_LAST_BUFFER_SIZE = MB_RTU_BUFFER_SIZE - 1;											//Set last buffer to max - 1
			hw_usart3_receiver_dma((uint32_t)(&G_IO_BUFFER + sizeof(uint8_t)), G_LAST_BUFFER_SIZE);	//Enable RX DMA, start buffer after address
		} else {
			/*
			 * If address did not match then mute rx and set last buffer to 1 for new address.
			 * */
			hw_usart3_mute();
			G_LAST_BUFFER_SIZE = 1;
			hw_usart3_receiver_dma((uint32_t)G_IO_BUFFER, (uint32_t)G_LAST_BUFFER_SIZE);		//Start rx
		}

	} else {
		/*
		 * Last buffer size wasn't 1, meaning we have data in buffer and buffer is full.
		 * Disable rx and set last buffer size to max.
		 * */
		hw_usart3_rx_stop();						//Stop RX
		G_LAST_BUFFER_SIZE = MB_RTU_BUFFER_SIZE;	//Set last buffer to max
		xSemaphoreGive(G_WAIT_FOR_FRAME);			//Activate thread to process frame
	}
}
void hw_usart3_tx_dma_handler(void)
{
	/*
	 * Data is sent out
	 * Set last buffer to 1
	 * Enable RX
	 * */
	G_LAST_BUFFER_SIZE = 1;
	hw_usart3_receiver_dma((uint32_t)G_IO_BUFFER, (uint32_t)G_LAST_BUFFER_SIZE);		//Start rx

}
void hw_usart3_rx_idle_handler(void)
{
	/*
	 * Idle line was detected.
	 * Now assume that full frame is transfered
	 * Disable RX
	 * Get last buffer size
	 * */
	hw_usart3_rx_stop();				//Stop rx
	G_LAST_BUFFER_SIZE = 1 + hw_usart3_rx_data_count(G_LAST_BUFFER_SIZE);	//Get frame size without address and add address
	xSemaphoreGive(G_WAIT_FOR_FRAME);	//Activate thread to process frame

}
