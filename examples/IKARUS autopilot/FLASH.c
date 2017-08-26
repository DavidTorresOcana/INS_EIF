#include "FLASH.h"


FLASH_Status Flash_Clear(uint32_t Address)
{
	FLASH_Status status;
	
	FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	status = FLASH_ErasePage(Address & 0xfffffc00); // FLASH_COMPLETE)
  FLASH_Lock(); 

	return status;
}

FLASH_Status Flash_Write(uint32_t Address, uint32_t data32)
{
	FLASH_Status status; 
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

	status = FLASH_ProgramWord(Address, data32); // ==FLASH_COMPLETE)

	FLASH_Lock(); 
	return status;
}

FLASH_Status Flash_Write_Buff(uint32_t Address, uint32_t *data32, uint32_t size)
{
	int i;
	FLASH_Status status; 
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	
	for(i=0;i<size/4;i++)
	{
		status = FLASH_ProgramWord(Address+4*i, data32[i]); // ==FLASH_COMPLETE)
	}
	
	FLASH_Lock(); 
	return status;
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  

  return sector;
}
