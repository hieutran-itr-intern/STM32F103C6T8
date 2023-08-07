/*
 * flash.c
 *
 *  Created on: Aug 6, 2023
 *      Author: Hieu
 */

#include "flash.h"


/* Function pointer for jumping to user application. */
typedef void (*funtion_pointer)(void);

/**
 * @brief   Erases the memory
 * @param   address: First address to be erased
 * @return  flash_status
 */
flash_status flash_erase(uint32_t address)
{
	/* Unlock flash control register access */
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef erase;
	flash_status status = FLASH_ERROR;
	uint32_t error = 0u;

	/*Set up mode erase*/
	erase.TypeErase = FLASH_TYPEERASE_PAGES; /* Not erase all flash memory*/
	erase.PageAddress = address;
	erase.Banks = FLASH_BANK_1;
	erase.NbPages = (FLASH_BANK1_END - address) / FLASH_PAGE_SIZE;

	/* Erasing*/
	if (HAL_FLASHEx_Erase(&erase, &error) == HAL_OK)
	{
		status = FLASH_OK;
	}

	/* Lock flash control register access */
	HAL_FLASH_Lock();

	return status;

}

flash_status flash_write(uint32_t address, uint32_t *data, uint32_t length)
{
	flash_status status = FLASH_OK;

	HAL_FLASH_Unlock();

	/* Loop the array */
	for (uint32_t i = 0; (i < length) && (status == FLASH_OK); i++)
	{
		/* Check if data is reached of the memory*/
		if (address >= FLASH_APP_END_ADDRESS)
		{
			status = FLASH_ERROR_WRITE;
		}
		else
		{
			/* Write data */
			if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]))
			{
			    status = FLASH_ERROR_WRITE;
			}
			/* Check data if not read correct*/
		    if (((data[i])) != (*(volatile uint32_t*)address))
		    {
		        status = FLASH_ERROR;
		    }

		    /* Shift address*/
		    address += 4;
		}

	}

	HAL_FLASH_Lock();

}

/**
 * @brief   Jumps to the user application.
 * @param   void
 * @return  void
 */
void flash_jump_to_app(void)
{
	funtion_pointer jump_to_application;
	jump_to_application = (funtion_pointer)(*(volatile uint32_t*) (FLASH_APP_START_ADDRESS + 4u));

	HAL_DeInit();
	SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);
	/* Change MSP */
	__set_MSP(*(volatile uint32_t*)FLASH_APP_START_ADDRESS);
	jump_to_application();

}

