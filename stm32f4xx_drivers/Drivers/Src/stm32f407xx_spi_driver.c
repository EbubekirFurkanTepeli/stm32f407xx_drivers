/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 17, 2025
 *      Author: Furkan
 */
#include "stm32f407xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);



/*
 * Timeout
 */
uint8_t Timeout_spi(volatile uint32_t timeout_count)
{
    while(timeout_count--);
    return 1;
}


/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}

	}
	else if(ENorDI == DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}

	}
}



/*
 * Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Configure the SPI_CR! register
	uint32_t tempr = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure the device mode
	tempr |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		tempr &= ~( 1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tempr |= ( 1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMP_RX)
	{
		tempr &= ~( 1 << 15);
		tempr |= ( 1 << 10);
	}

	tempr |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;
	tempr |= pSPIHandle->SPIConfig.SPI_DFF << 11;
	tempr |= pSPIHandle->SPIConfig.SPI_CPOL << 1;
	tempr |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	pSPIHandle->pSPIx->CR1 = tempr;

}



/*
 *  De-init
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if(pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if(pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if(pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
}



uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}




/*
 * Data Send and Receive
 */
uint8_t  SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
		{
		    if(Timeout_spi(20000000))
		    {
		        return 1;
		    }
		}

		if((pSPIx->CR1 & ( 1 << SPI_CR1_DFF)))
		{
			pSPIx->DR =  *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			pSPIx->DR =  *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
	return 0;
}



uint8_t SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
		{
		    if(Timeout_spi(20000000))
		    {
		        return 1;
		    }
		}

		if((pSPIx->CR1 & ( 1 << SPI_CR1_DFF)))
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR  ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
	return 0;
}



/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32);
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			*NVIC_ISER3 |= ( 1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32);
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64);
		}
	}
}



void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_CR2_RXNEIE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}
}



uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);
	}
	return state;
}



uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);
	}
	return state;
}



/*
 * Other Functions
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}



void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}



void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE);
	}
}


/*
 * Only this source file functions
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF)))
	{
		pSPIHandle->pSPIx->DR =  *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}



static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & 1 << SPI_CR1_DFF)
	{
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else
	{
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}



static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);
}




void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}



void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen--;
	pSPIHandle->TxState = SPI_READY;
}



void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen--;
	pSPIHandle->RxState = SPI_READY;
}



__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	    /* This is a week implementation. The application may override this function. */
}
