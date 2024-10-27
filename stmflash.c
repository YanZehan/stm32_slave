#include "stmflash.h"

FLASH_ProcessTypeDef p_Flash; 
uint16_t STMFLASH_BUF[STM_SECTOR_SIZE/2]; // 缓存数组

uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
	return *(volatile uint16_t*)faddr; 
}

#if STM32_FLASH_WREN // 如果使能了写   

void STMFLASH_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{ 
	for(uint16_t i = 0; i < NumToWrite; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
	    WriteAddr += 2; // 地址增加2
	}  
} 

void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
	if(WriteAddr < STM32_FLASH_BASE || (WriteAddr >= (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
		return; // 非法地址

	HAL_FLASH_Unlock(); // 解锁

	uint32_t offaddr = WriteAddr - STM32_FLASH_BASE;
	uint32_t secpos = offaddr / STM_SECTOR_SIZE; 
	uint16_t secoff = (offaddr % STM_SECTOR_SIZE) / 2; 
	uint16_t secremain = STM_SECTOR_SIZE / 2 - secoff; 

	if(NumToWrite <= secremain)
		secremain = NumToWrite;

	while(1) 
	{
		STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);

		bool needErase = false;
		for(uint16_t i = 0; i < secremain; i++)
		{
			if(STMFLASH_BUF[secoff + i] != 0xFFFF)
			{
				needErase = true;
				break;
			}
		}

		if(needErase)
		{
			Flash_PageErase(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE);	
			FLASH_WaitForLastOperation(FLASH_WAITETIME);			
			CLEAR_BIT(FLASH->CR, FLASH_CR_PER); 

			for(uint16_t i = 0; i < secremain; i++)
			{
				STMFLASH_BUF[i + secoff] = pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);
		}
		else 
		{
			FLASH_WaitForLastOperation(FLASH_WAITETIME);       
			STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain);
		}

		if(NumToWrite == secremain) break;

		secpos++;
		secoff = 0;	 
	    pBuffer += secremain;  
		WriteAddr += secremain * 2;  
	    NumToWrite -= secremain;
		secremain = (NumToWrite > (STM_SECTOR_SIZE / 2)) ? STM_SECTOR_SIZE / 2 : NumToWrite;
	}	

	HAL_FLASH_Lock(); // 上锁
}

#endif

void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)   	
{
	for(uint16_t i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr); 
		ReadAddr += 2; 
	}
}

void Flash_PageErase(uint32_t PageAddress)
{
  p_Flash.ErrorCode = HAL_FLASH_ERROR_NONE;

#if defined(FLASH_BANK2_END)
  if(PageAddress > FLASH_BANK1_END)
  { 
    SET_BIT(FLASH->CR2, FLASH_CR2_PER);
    WRITE_REG(FLASH->AR2, PageAddress);
    SET_BIT(FLASH->CR2, FLASH_CR2_STRT);
  }
  else
  {
#endif 
    SET_BIT(FLASH->CR, FLASH_CR_PER);
    WRITE_REG(FLASH->AR, PageAddress);
    SET_BIT(FLASH->CR, FLASH_CR_STRT);
#if defined(FLASH_BANK2_END)
  }
#endif 
}
