/*
 * flash.h
 *
 *  Created on: Aug 6, 2023
 *      Author: Hieu
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stm32f1xx_hal.h"

/*Start and end address of user */
#define FLASH_APP_START_ADDRESS (uint32_t)0x08004000u
#define FLASH_APP_END_ADDRESS (uint32_t)FLASH_BANK1_END - 0x10u /**Leave a little space*/

/*Status of function*/
typedef enum
{
	FLASH_OK = 0x00u,
	FLASH_ERROR_WRITE = 0x01u,
	FLASH_ERROR = 0x02u

} flash_status;

flash_status flash_erase(uint32_t address);
flash_status flash_write(uint32_t address, uint32_t *data, uint32_t length);
void flash_jump_to_application(void);

#endif /* INC_FLASH_H_ */
