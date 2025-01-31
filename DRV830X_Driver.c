/*
 * DRV830X_Driver.c
 *
 *  Created on: Jan 14, 2025
 *      Author: TDM
 */
#include "DRV830X_Driver.h"

void DRV8303_Unit(void) {
// ***************** Сброс и включение DRV8303 ***********************
	HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
// ******************* Инициализация DRV8303 *************************
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
	uint16_t SPI_DATA_SEND[1];
	uint16_t SPI_DATA_READ[1];

	SPI_DATA_SEND[0] = 0b0001000000110000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) SPI_DATA_SEND, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b1001000000000000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) SPI_DATA_SEND, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b0000000000000000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi3, (uint8_t*) SPI_DATA_READ, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b0001100000110000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) SPI_DATA_SEND, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b1001100000000000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*) SPI_DATA_SEND, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

	SPI_DATA_SEND[0] = 0b0000000000000000;
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi3, (uint8_t*) SPI_DATA_READ, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);
}

