/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Oct 22, 2025
 *      Author: Furkan
 */


#include "stm32f407xx_i2c_driver.h"


static void I2C_GeneraterStartCond(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddrPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddrPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);



static void I2C_GeneraterStartCond(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}


void I2C_GeneraterStopCond(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}


static void I2C_ExecuteAddrPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);// SlaveAddr is (Slave addres + read/write bit 0)
	pI2Cx->DR = SlaveAddr;
}



static void I2C_ExecuteAddrPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;// SlaveAddr is (Slave addres + read/write bit 1 )
	pI2Cx->DR = SlaveAddr;
}




static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t onlyRead;

	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				I2C_ControlAcking(pI2CHandle->pI2Cx, DISABLE);

				onlyRead = pI2CHandle->pI2Cx->SR1;
				onlyRead = pI2CHandle->pI2Cx->SR2;
				(void)onlyRead;
			}
		}
		else
		{
			onlyRead = pI2CHandle->pI2Cx->SR1;
			onlyRead = pI2CHandle->pI2Cx->SR2;
			(void)onlyRead;
		}
	}
	else
	{
		onlyRead = pI2CHandle->pI2Cx->SR1;
		onlyRead = pI2CHandle->pI2Cx->SR2;
		(void)onlyRead;
	}
}



void I2C_SlaveEnableDisableCallBackEvents(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}



/*
 * Timeout
 */
uint8_t Timeout_i2c(volatile uint32_t timeout_count)
{
    while(timeout_count--);
    return 1;
}



uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE);
	}
}



void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else if(ENorDI == DISABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}



/*
 * Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//Enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ACK control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14 );
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Standart mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else if(pI2CHandle->I2C_Config.I2C_SCLSpeed >= I2C_SCL_SPEED_FM2)
	{
		//Fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//Configure the trise
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Trise in standart mode
		tempreg = (uint16_t)(RCC_GetPCLK1Value() + 1) & (0x3F);
	}
	else
	{
		//Trise in fast mode
		tempreg = (uint16_t)(((RCC_GetPCLK1Value() * 300) / 1000) + 1) & (0x3F);
	}
	pI2CHandle->pI2Cx->TRISE |= tempreg;
}



/*
 * De-init
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    if(pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if(pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
    else if(pI2Cx == I2C3)
    {
        I2C3_REG_RESET();
    }
}



void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	/* Generating start condition */
	I2C_GeneraterStartCond(pI2CHandle->pI2Cx);

	/* Confirming start generation is completed by checking SB flag in SR1 */
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	/* Send the address of the slave with r/nm bit set to w(0) (total 8 bits) */
	I2C_ExecuteAddrPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	/* Confirming address phase is completed by checking ADDR flag in SR1 */
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	/* Clearing the address flag according to its software sequence */
	I2C_ClearADDRFlag(pI2CHandle);

	/* Send the data until length reaches 0 value */
	while(Len > 0)
	{
		/* Waiting untill TX is set */
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	/* Waiting for TXE=1 and BTF=1 before generating STOP condition */
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	/* Generate STOP condition and master not need to wait for the completion of stop condition */
	if(Sr == I2C_SR_DISABLE)
	{
		I2C_GeneraterStopCond(pI2CHandle->pI2Cx);
	}
}



void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	//1. Generate the START condition
	I2C_GeneraterStartCond(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddrPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ControlAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		//generate STOP condition
		if(Sr == I2C_SR_DISABLE)
			I2C_GeneraterStopCond(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ControlAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_SR_DISABLE )
					I2C_GeneraterStopCond(pI2CHandle->pI2Cx);
			}
			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}
	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ControlAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}



void I2C_ControlAcking(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == I2C_ACK_ENABLE)
	{
		//Enable the ACK
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}
	else if(ENorDI == I2C_ACK_DISABLE)
	{
		//Disable the ACK
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}



uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GeneraterStartCond(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	return busystate;
}



uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GeneraterStartCond(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}



void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{

	if(ENorDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}



void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		pI2CHandle->TxLen--;

		pI2CHandle->pTxBuffer++;
	}
}


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			I2C_ControlAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		if(pI2CHandle->Sr == I2C_SR_DISABLE)
		{
			I2C_GeneraterStopCond(pI2CHandle->pI2Cx);
		}

		I2C_CloseReceiveData(pI2CHandle);

		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}



void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR= data;
}



uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t) pI2C->DR;
}



void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);

	if(temp1 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddrPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddrPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);

	if(temp1 && temp3)
	{
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);

	if(temp1 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE))
			{
				if(pI2CHandle->TxLen == 0)
				{
					if(pI2CHandle->Sr == I2C_SR_DISABLE)
					{
						I2C_GeneraterStopCond(pI2CHandle->pI2Cx);
					}

					I2C_CloseSendData(pI2CHandle);

					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);

	if(temp1 && temp3)
	{
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);

	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}

	}
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);

	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}



void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

	   I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}



void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    /* Disabling ITBUFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    /* Disabling ITEVFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ControlAcking(pI2CHandle->pI2Cx,ENABLE);
    }

}



void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    /* Disabling ITBUFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    /* Disabling ITEVFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}



__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
    /* This is a week implementation. The application may override this function. */
}
