/*
 * bootloader.c
 *
 *  Created on: Aug 7, 2023
 *      Author: Hieu
 */

/*
 * Data frame:
 * WRITE_MEMORY: 1 byte length command | 0x31 | 4 byte write address | (max 118 byte) data |1 byte Write Length | 2 byte CRC
 * ERASE_MEMORY: 1 byte length command | 0x43 | 4 byte erase address | 2 byte CRC
 * */

#include "bootloader.h"

typedef struct
{
	uint8_t length_cm;
	uint8_t command;
	uint8_t add_write_byte_4;  /* 4-> 3-> 2-> 1*/
	uint8_t add_write_byte_3;
	uint8_t add_write_byte_2;
	uint8_t add_write_byte_1;
	uint8_t length_write;
	uint8_t crc_high; /* high -> low */
	uint8_t crc_low;
	uint8_t write_data[118];

} write_command;

typedef struct
{
	uint8_t length_cm;
	uint8_t command;
	uint8_t crc_high; /* high -> low */
	uint8_t crc_low;
	uint8_t add_erase_byte_4;
	uint8_t add_erase_byte_3;
	uint8_t add_erase_byte_2;
	uint8_t add_erase_byte_1;

} erase_command;

/**
 * @brief Handle command from data frame received from host
 * */
bootloader_status bootloader_handle(uint8_t *data_frame)
{
	bootloader_status status = BOOTLOADER_OK;

	/* Init frame command */
	write_command write;
	erase_command erase;

	/* Check length data_frame and only handle data in range 0 --> length - 1*/
	uint8_t length_frame = 0x00u;
	length_frame = *data_frame;

	/* Save value receive from Host to temp struct */
	while (status == BOOTLOADER_OK)
	{
		/* Check CRC*/

		/* Handle command type */
		uint8_t type_command = *(data_frame + 1);

		switch (type_command)
		{
			case READ_MEMORY:
				/* Do somthing*/
				break;

			case WRITE_MEMORY:
				write.length_cm = length_frame;
				write.command = 0x31;
				write.add_write_byte_4 = *(data_frame + 2);
				write.add_write_byte_3 = *(data_frame + 3);
				write.add_write_byte_2 = *(data_frame + 4);
				write.add_write_byte_1 = *(data_frame + 5);

				for (uint8_t i = 0; i < (write.length_cm - 9); i++)
				{
					write.write_data[i] = *(data_frame + 6 + i);
				}

				write.length_write = *(data_frame + (write.length_cm - 3));
				write.crc_high = *(data_frame + (write.length_cm - 2));
				write.crc_low = *(data_frame + (write.length_cm - 1));

				break;

			case ERASE_MEMORY:
				erase.length_cm = length_frame;
				erase.command = 0x43u;
				erase.add_erase_byte_4 = *(data_frame + 2);
				erase.add_erase_byte_3 = *(data_frame + 3);
				erase.add_erase_byte_2 = *(data_frame + 4);
				erase.add_erase_byte_1 = *(data_frame + 5);
				erase.crc_high = *(data_frame + 6);
				erase.crc_low = *(data_frame + 7);
				break;

			default:
				status = BOOTLOADER_ERROR;
				break;
		}

		/* Check CRC */


	}
}

uint16_t CRC16_Caculation(uint8_t Num) //Ham CRC 16
 {
    uint8_t x,y,i; uint16_t Crc; //Bien Cuc Bo

    Crc=0xFFFF; //Init

    for(x=0;x<Num;++x) //Lap Vong
    {
       y=Prepare_data_send[x]; //Lay Byte Trong Bo Dem CRC 16
       Crc=Crc^y; //Calculate the CRC

       for(i=8;i!=0;--i) //Shift Right
       {
          if((Crc&0x0001)!=0) //If the LSB is set
          {
             Crc>>=1; //Shift right and XOR 0xA001
             Crc^=0xA001;
          }
          else //Else LSB is not set
          {
             Crc>>=1; //Just shift right
          }
       }
    }

    //Swapping of the high and low CRC bytes
    return Crc; //Tra Ve CRC_16 Tinh Duoc
 }

