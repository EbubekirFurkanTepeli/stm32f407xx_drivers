/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Nov 1, 2025
 *      Author: Furkan
 */


#include "stm32f407xx_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t cloksource, temp, ahbp, apb1p;

	cloksource = ((RCC->CFGR >> 2) & 0X3);

	if(cloksource == 0)
	{
		SystemClk = 16000000; 	//16MHz
	}
	else if(cloksource == 1)
	{
		SystemClk = 8000000;	//8MHz
	}
	else if(cloksource == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}


	//For AHB
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//For APB1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}



uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock = 0, tmp, pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}
	else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	//For AHBP
	if(tmp < 0x08)
	{
		ahbp = 1;
	}
	else
	{
	   ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;

	//For APB2
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}



uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}
