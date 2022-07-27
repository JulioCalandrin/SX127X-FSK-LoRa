/*
 *  This library was stolen from Zenith Aerospace :)
 *  Generic interace library.
 *
 *  Made by:Low Level Software Department - Zenith Aerospace
 *  Created on: 20 sept 2019
 */
 
#ifndef STM32_H
#define STM32_H

#include "stm32l4xx_hal.h" // Nucleo Board
#include "stm32f1xx_hal.h" // Bluepill Board

#define RETURN_ON_ERROR(status);\
	if(status != HAL_OK){\
		return status;\
	}

#endif//LIB_ZENITH_GENERICS
