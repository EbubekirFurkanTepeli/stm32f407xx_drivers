/*
 * 004_Button_Interrupt.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Furkan
 */
#include <string.h>
#include "stm32f407xx.h"

#define HIGH 1
#define LOW	 0
#define BUTTON_PRESSED LOW


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 / 2 ; i ++);
}

int main(void)
{

	GPIO_Handle_t GPIOled, GPIObtn;

	memset(&GPIOled,0,sizeof(GPIOled));
	memset(&GPIObtn,0,sizeof(GPIObtn));

	GPIOled.pGPIOx = GPIOD;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOled.GPIO_PinConfig.GPIO_PinOPTType = GPIO_OP_TYPE_PP;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControlr = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOled);


	GPIObtn.pGPIOx = GPIOD;
	GPIObtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIObtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIObtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObtn.GPIO_PinConfig.GPIO_PinPuPdControlr = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIObtn);


	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
}


	void EXTI9_5_IRQHandler(void)
	{
		delay();
		GPIO_IRQHandling(GPIO_PIN_NO_5);
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
	}































