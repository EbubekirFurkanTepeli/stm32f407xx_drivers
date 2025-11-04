/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Nov 1, 2025
 *      Author: Furkan
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

// APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

// APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock();

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
