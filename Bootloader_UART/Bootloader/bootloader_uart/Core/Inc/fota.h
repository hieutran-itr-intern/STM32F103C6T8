/*
 * fota.h
 *
 *  Created on: Aug 7, 2023
 *      Author: Hieu
 */

#ifndef INC_FOTA_H_
#define INC_FOTA_H_

#include "uart.h"
#include "flash.h"
#include "stdbool.h"

/*
 * Data frame:
 * Start code |	Byte count |  Address |	Record type	Data  |	Checksum
 * */

typedef struct
{
	uint8_t start_code;
	uint8_t byte_count;
	uint16_t address;
	uint8_t record_type;
	uint8_t data[16];
	uint8_t checksum;
} hexa_struct;

#define START_CODE ':'
#define RECORD_DATA 0x00u
#define RECORD_EX_LIR_ADDRESS 0x04u
#define RECORD_EOF 0x01u
#define RECORD_EX_SEG_ADDRESS 0X02u
#define RECORD_EX_LIR_ADDRESS_START 0x05u

/* FOTA_status*/
typedef enum
{
	FOTA_OK = 0xFFu,
	FOTA_ERROR = 0xF1u
} fota_status;

void handle_data(uint8_t *rx_data, hexa_struct data_frame);

#endif /* INC_FOTA_H_ */
