/*
 * 005_Spi_Tx.c
 *
 *  Created on: Oct 18, 2025
 *      Author: Furkan
 */
#include <string.h>
#include "stm32f407xx.h"

/*
 *  PB14 --> SPI2_MISO
 *  PB15 --> SPI2_MOSI
 *  PB13 --> SPI2_SCLK
 *  PB12 --> SPI2_NSS
 *  Alternative function mode : 5
 */


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIpins;

	SPIpins.pGPIOx = GPIOB;
	SPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIpins.GPIO_PinConfig.GPIO_PinOPTType = GPIO_OP_TYPE_PP;
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControlr = GPIO_NO_PUPD;
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIpins);

	//MOSI
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIpins);

	//MISO
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIpins);

	//NSS
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIpins);
}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8_BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}




int main()
{
	char user_data[] = "This is SPI data";

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 peripherals
	SPI2_Inits();

	// This makes NSS signal internally high and avoid MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	// Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// To send data
	SPI_SendData(SPI2,(uint8_t*) user_data, strlen(user_data));

	// Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;


}
