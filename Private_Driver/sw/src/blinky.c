/*
 * blinky.c
 *
 *  Created on: 2. nov 2018
 *      Author: Sad
 */

#include "blinky.h"

#include "stdint.h"

#include "FreeRTOS.h"
#include "task.h"


#include "stm32f10x_gpio.h"


/*Thread delay in ms*/
static TickType_t THREAD_DELAY;

/*FreeRTOS thread*/
static void blinky_thread(void *pvParameters);

/*
 * blinky_init(uint32_t delay);
 *
 * Simple function to flash LED on pin C13
 *
 * @param uint32_t delay :	function dealy in ms before pin state is changed
 *
 * */
void blinky_init(uint32_t delay)
{
	/*If delay is missing use 500ms*/
	if (delay)
		THREAD_DELAY = (TickType_t) (delay / portTICK_PERIOD_MS);
	else
		THREAD_DELAY = (TickType_t) (500 / portTICK_PERIOD_MS);

	/*Start PortC clock*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/*Pin configure structure*/
	GPIO_InitTypeDef GPIO_Conf;

	/*Port values*/
	GPIO_Conf.GPIO_Pin 		= GPIO_Pin_13;		//LED is connected to pin C13
	GPIO_Conf.GPIO_Speed 	= GPIO_Speed_2MHz;	//Port IO speed
	GPIO_Conf.GPIO_Mode 	= GPIO_Mode_Out_OD;	//Port in open drain mode
	GPIO_Init(GPIOC, &GPIO_Conf);				//Init port

	xTaskCreate( blinky_thread, "Blinky", configMINIMAL_STACK_SIZE, 0, 0, NULL );	//Start thread

}

/*
 * FreeRTOS thread for blinky
 * */
static void blinky_thread(void *pvParameters)
{
	//Thread loop
	while (1) {
		uint8_t temp;

		/*Read pin state*/
		temp = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
		/*Flip pin state*/
		temp ^= 1;
		/*Set new pin state*/
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, temp);

		/*Wait*/
		vTaskDelay(THREAD_DELAY);
	}

	/*Never exit thread*/
	while (1);
}

