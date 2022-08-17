/*
 *	Biblioteca do protocolo de comunicacao SPI para STM32
 *	Versao em C
 *
 *	Feito por: setor de Software Baixo Nivel - Zenith Aerospace
 *	Criado em: 18 jan 2020
 */

#include "STM32_SPI.h"

HAL_StatusTypeDef SPI_read_register(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t reg_addr, uint8_t* pvalue, uint32_t timeout){
	HAL_StatusTypeDef status = HAL_OK;																// status of the execution (checks if any error occurs)	
	reg_addr = reg_addr & 0x7f;

	/* Changes SS pin to enable comunication */
	HAL_GPIO_WritePin(ss_gpio_port, ss_pin, GPIO_PIN_RESET);

	/* Writes Register Address in SPI */
	status = HAL_SPI_Transmit (&spi_bus, &reg_addr, 1, timeout);
	RETURN_ON_ERROR(status);

	/* Reads Register Value from SPI */
	status = HAL_SPI_Receive (&spi_bus, pvalue, 1, timeout);
	RETURN_ON_ERROR(status);

	/* Changes SS pin to disable comunication */
	HAL_GPIO_WritePin(ss_gpio_port, ss_pin, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef SPI_burst_read(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t start_addr, uint8_t* pvalue, uint16_t size, uint32_t timeout){
	HAL_StatusTypeDef status = HAL_OK;																// status of the execution (checks if any error occurs)	
	start_addr = start_addr & 0x7f;
	
	/* Changes SS pin to enable comunication */
	HAL_GPIO_WritePin(ss_gpio_port, ss_pin, GPIO_PIN_RESET);
	
	/* Writes Register Address in SPI */
	status = HAL_SPI_Transmit (&spi_bus, &start_addr, 1, timeout);
	RETURN_ON_ERROR(status);
	
	/* Reads Register Value from SPI */
	status = HAL_SPI_Receive (&spi_bus, pvalue, size, timeout);
	RETURN_ON_ERROR(status);
	
	/* Changes SS pin to disable comunication */
	HAL_GPIO_WritePin(ss_gpio_port, ss_pin, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef SPI_write_register(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t reg_addr, uint8_t value, uint32_t timeout){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t sent_data[2];
	sent_data[0] = reg_addr | 0x80;
	sent_data[1] = value;

	/* Changes SS pin to enable comunication */
	HAL_GPIO_WritePin(ss_gpio_port, ss_pin, GPIO_PIN_RESET);

	/* Writes in Register via SPI */
	status = HAL_SPI_Transmit(&spi_bus, sent_data, 2, timeout);
	RETURN_ON_ERROR(status);

	/* Changes SS pin to disable comunication */
	HAL_GPIO_WritePin(ss_gpio_port, ss_pin, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef SPI_burst_write(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t start_addr, uint8_t* values, uint16_t size, uint32_t timeout){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t* sent_data;
	
	sent_data = (uint8_t*)malloc(size+1);
	sent_data[0] = start_addr | 0x80;
	for(int i = 0; i < size; i++){
		sent_data[i+1] = values[i];
	}
	
	/* Changes SS pin to enable comunication */
	HAL_GPIO_WritePin(ss_gpio_port, ss_pin, GPIO_PIN_RESET);

	/* Writes in Register via SPI */
	status = HAL_SPI_Transmit(&spi_bus, sent_data, size+1, timeout);
	RETURN_ON_ERROR(status);

	free(sent_data);

	/* Changes SS pin to disable comunication */
	HAL_GPIO_WritePin(ss_gpio_port, ss_pin, GPIO_PIN_SET);
	return status;
}
