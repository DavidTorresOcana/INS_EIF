#include "stm32f10x_flash.h"

FLASH_Status Flash_Clear(uint32_t Address);
FLASH_Status Flash_Write(uint32_t Address, uint32_t data32);
FLASH_Status Flash_Write_Buff(uint32_t Address, uint32_t *data32, uint32_t size);
