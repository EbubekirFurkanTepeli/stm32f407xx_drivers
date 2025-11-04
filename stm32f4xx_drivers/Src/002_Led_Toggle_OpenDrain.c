/*
 * 002_Led_Toggle.c
 *
 *  Created on: Oct 15, 2025
 *      Author: Furkan
 */


#include "stm32f407xx.h"


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{
	GPIO_Handle_t Gpioled;

	Gpioled.pGPIOx = GPIOD;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPTType = GPIO_OP_TYPE_OD;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControlr = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&Gpioled);


	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
}
