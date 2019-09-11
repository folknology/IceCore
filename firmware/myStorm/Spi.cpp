#include "spi.h"

void Spi::Boot_Enable(void){

	__HAL_RCC_SPI3_CLK_DISABLE();

	HAL_GPIO_DeInit(GPIOB, SPI3_SCK_Pin|SPI3_MISO_Pin|SPI3_MOSI_Pin);
}

void Spi::Enable(void){

	HAL_GPIO_DeInit(GPIOB, SPI3_SCK_Pin|SPI3_MISO_Pin|SPI3_MOSI_Pin);

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	__HAL_RCC_SPI3_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = SPI3_SCK_Pin|SPI3_MISO_Pin|SPI3_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void Spi::Disable(void){

	__HAL_RCC_SPI3_CLK_DISABLE();

	HAL_GPIO_DeInit(GPIOB, SPI3_SCK_Pin|SPI3_MISO_Pin|SPI3_MOSI_Pin);

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = SPI3_MISO_Pin | SPI3_SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SPI3_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}