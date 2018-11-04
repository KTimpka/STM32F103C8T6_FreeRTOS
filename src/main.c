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

#include "stm32f10x.h"

#include "hw_start.h"
#include "blinky.h"
			

int main(void)
{
	/*
	 * Start 72Mhz system clock, 1Khz SysTick, NVIC configure
	 * */
	hw_start_init();

	/*Start blinky*/
	blinky_init(500);


	/*Start FreeRTOS*/
	vTaskStartScheduler();

	for(;;);
}
