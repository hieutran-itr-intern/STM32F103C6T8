/*
 * uart.h
 *
 *  Created on: Aug 6, 2023
 *      Author: Hieu
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;

/* Timeout UART*/
//#define UART_TIMEOUT ((uint16_t)1000u)
#define UART_TIMEOUT HAL_MAX_DELAY

/* Status of UART functions*/
typedef enum
{
	UART_OK = 0x00u,
	UART_ERROR = 0x01u
} uart_status;

uart_status uart_receive(uint8_t *data, uint16_t length);
uart_status uart_transmit_char(uint8_t data);
uart_status uart_transmit_string(uint8_t *data);

#endif /* INC_UART_H_ */
