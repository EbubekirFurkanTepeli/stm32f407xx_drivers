/*
 * 003_Led_Button.c
 *
 *  Created on: Oct 15, 2025
 *      Author: Furkan
 */


#include "stm32f407xx.h"

#define HIGH				1
#define BUTTON_PRESSED		HIGH


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 / 2 ; i ++);
}


int main(void)
{
	GPIO_Handle_t GPIOled, GPIObtn;

	GPIOled.pGPIOx = GPIOD;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOled.GPIO_PinConfig.GPIO_PinOPTType = GPIO_OP_TYPE_OD;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControlr = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOled);


	GPIObtn.pGPIOx = GPIOA;
	GPIObtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIObtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObtn.GPIO_PinConfig.GPIO_PinPuPdControlr = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIObtn);

	while(1)
	{

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BUTTON_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

	}


}
