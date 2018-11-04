/*
 * hw_start.h
 *
 *  Created on: 2. nov 2018
 *      Author: Sad
 */

#ifndef HW_INC_HW_START_H_
#define HW_INC_HW_START_H_


/*
 * Includes
 * */
#include <stdint.h>

/*
 * Defines, macros
 * */
#define SYSTICK_RELOAD	(uint32_t)9000	//24bit value for stm32fc8t6

/*
 * GLOBAL
 * */

/*
 * Functions
 * */
void hw_start_init();

/*
 * NVIC functions
 * */

/*
 * Threads
 * */

#endif /* HW_INC_HW_START_H_ */
