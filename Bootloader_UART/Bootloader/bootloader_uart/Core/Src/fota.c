/*
 * fota.c
 *
 *  Created on: Aug 7, 2023
 *      Author: Hieu
 */

#include "fota.h"

/* Step 1: Coppy data from buffer to struct
 * Step 2: Check sum for each case ()
 * Step 3: Flash into memory or update value
 *
 * */

void handle_data(uint8_t *rx_data, hexa_struct data_frame)
{
	uint16_t high_address = 0x0000;

	/* Coppy data to struct */
	data_frame.start_code = *rx_data;

	data_frame.byte_count = *(rx_data + 1);

	data_frame.address = ((*(rx_data + 2) << 4) & 0xF0) | (*(rx_data + 3));

	data_frame.record_type = *(rx_data + 4);


	/* Handle */
	if (data_frame.record_type == RECORD_DATA)
	{
		for (uint8_t i = 0; i < data_frame.byte_count; i++)
		{
			data_frame.data[i] = *(rx_data + 5);
		}

		data_frame.checksum = *(rx_data + 21);
	}
	else if (data_frame.record_type == RECORD_EX_LIR_ADDRESS)
	{
		high_address = ((*(rx_data + 6) << 4) & 0xF0) | (*(rx_data + 7));
		data_frame.checksum = *(rx_data + 7);
	}
	else if (data_frame.record_type == RECORD_EX_LIR_ADDRESS_START)
	{
		/* Do nothing */
	}
	else
	{
		/* Do nothing */
	}

	/* Summary address */
	//uint32_t address = (uint32_t)(((high_address << 16) & 0xFF00) | ( data_frame.address & 0x00FF));

	/* Flash memory */
	// flash_write(address, data_frame.data, data_frame.byte_count);
}
