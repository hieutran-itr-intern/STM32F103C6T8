/*
 * bootloader.h
 *
 *  Created on: Aug 7, 2023
 *      Author: Hieu
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include "uart.h"
#include "flash.h"
#include "stdbool.h"

/* Define command in frame received*/
#define READ_MEMORY 0x11
#define WRITE_MEMORY 0x31
#define ERASE_MEMORY 0x43

/* Bootloader_status*/
typedef enum
{
	BOOTLOADER_OK = 0x00,
	BOOTLOADER_ERROR = 0x01
} bootloader_status;


bootloader_status bootloader_handle(uint8_t *data_frame);
uint16_t CRC16_Caculation(uint8_t Num);

#endif /* INC_BOOTLOADER_H_ */
