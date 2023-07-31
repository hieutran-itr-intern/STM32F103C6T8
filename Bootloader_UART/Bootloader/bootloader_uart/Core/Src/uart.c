/*
 * uart.c
 *
 *  Created on: Aug 6, 2023
 *      Author: Hieu
 */

#include "uart.h"

/**
 * @brief Receive data from host
 *
 * @param data
 * @param length
 * @return uart_status
 */
uart_status uart_receive(uint8_t *data, uint16_t length)
{
	uart_status status = UART_ERROR;

	if (HAL_UART_Receive(&huart1, data, length, UART_TIMEOUT) == HAL_OK)
	{
		status = UART_OK;
	}

	return status;
}

/**
 * @brief Send a char to host
 *
 * @param data
 * @return uart_status
 */
uart_status uart_transmit_char(uint8_t data)
{
	uart_status status = UART_ERROR;

	if (HAL_UART_Transmit(&huart1, &data, 1u, UART_TIMEOUT) == HAL_OK)
	{
	    status = UART_OK;
	}

	return status;
}


/**
 * @brief Send a string to host
 *
 * @param *data
 * @return uart_status
 */
uart_status uart_transmit_string(uint8_t *data)
{
	uart_status status = UART_ERROR;
	uint16_t length = 0u;

	/* Check the length of string */
	while ('\0' != data[length])
	{
	  length++;
	}

	if (HAL_UART_Transmit(&huart1, data, length, UART_TIMEOUT) == HAL_OK)
	{
	    status = UART_OK;
	}

	return status;

}

