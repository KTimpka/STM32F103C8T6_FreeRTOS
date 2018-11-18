/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "FreeRTOS.h"
#include "task.h"

#include "stdint.h"

#include "stm32f10x.h"

#include "hw_start.h"
#include "blinky.h"
//#include "hw_usart_simple.h"
#include "hw_usart3.h"
			
static void test_thread(void *pvParameters);


int main(void)
{
	/*
	 * Start 72Mhz system clock, 1Khz SysTick, NVIC configure
	 * */
	hw_start_init();

	/*Start blinky*/
	blinky_init(500);

	xTaskCreate( test_thread, "Test Thread", configMINIMAL_STACK_SIZE, 0, 0, NULL );	//Start thread

	/*Start FreeRTOS*/
	vTaskStartScheduler();

	for(;;);
}


static void test_thread(void *pvParameters)
{
/*
	HW_USART_TypeDef HW_USART_Conf;
	HW_USART_Conf.USARTx 		= 	USART3;
	HW_USART_Conf.Periph_USARTx	=	RCC_APB1Periph_USART3;
	HW_USART_Conf.GPIOx			=	GPIOB;
	HW_USART_Conf.Periph_GPIOx	=	RCC_APB2Periph_GPIOB;
	HW_USART_Conf.pin_rx		=	GPIO_Pin_11;
	HW_USART_Conf.pin_tx		=	GPIO_Pin_10;
	HW_USART_Conf.baudrate		=	9600;

	hw_usart_init(&HW_USART_Conf);

	HW_USART_DMA_TX_TypeDef HW_USART_DMA_TX_Conf;
	HW_USART_DMA_TX_Conf.USARTx				=	USART3;
	HW_USART_DMA_TX_Conf.address_usart		=	(uint32_t)&(USART3->DR);

	hw_usart_dma_tx_init(&HW_USART_DMA_TX_Conf);

	while (1){
		uint8_t buffer[] = "KollaneKala!\n";
		hw_usart_send_dma(USART3, buffer, 13);

		vTaskDelay((TickType_t) (500 / portTICK_PERIOD_MS));
	}
	*/
	uint8_t buffer[] = "KollaneKala!\n";

	hw_usart3_init(9600);
	hw_usart3_dma_tx_init((uint32_t)buffer, 13);
	hw_usart3_dma_rx_init((uint32_t)buffer, 13);

	while (1){
		hw_usart3_send_dma((uint32_t)buffer, 13);

		vTaskDelay((TickType_t) (500 / portTICK_PERIOD_MS));
	}

	for(;;);
}

void hw_usart3_rx_dma_handler(void)
{

}

void hw_usart3_rx_idle_handler(void)
{

}

void assert_failed(uint8_t* file, uint32_t line)
{
	while (1);
}
