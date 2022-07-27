/*
 *  This library was stolen from Zenith Aerospace :)
 *  STM32 SPI interace library.
 *
 *  Made by:Low Level Software Department- Zenith Aerospace
 *  Created on: 18 jan 2020
 */

#ifndef STM32_SPI_H
#define STM32_SPI_H
#ifdef __cplusplus

extern "C" {
#endif

/* Platform Specific Includes */
#include "stm32.h"

/* SPI Functions */
HAL_StatusTypeDef SPI_read_register(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t reg_addr, uint8_t* pvalue, uint32_t timeout);					// received data is stored in value
HAL_StatusTypeDef SPI_burst_read(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t start_addr, uint8_t* pvalue, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI_write_register(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t reg_addr, uint8_t value, uint32_t timeout);
HAL_StatusTypeDef SPI_burst_write(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t start_addr, uint8_t* value, uint16_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif
#endif//STM32_SPI_H
