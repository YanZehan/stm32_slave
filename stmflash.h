#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "stm32f1xx_hal.h" // 根据你的具体芯片型号选择合适的 HAL 库头文件

// 定义 STM32 Flash 的基础地址和大小（根据实际芯片型号调整）
#define STM32_FLASH_BASE  0x08000000    // Flash 基地址
#define STM32_FLASH_SIZE  64            // Flash 大小（以 KB 为单位，STM32F103C8T6 为 64KB）
#define STM_SECTOR_SIZE   1024          // 扇区大小，STM32F103 系列每个扇区为 1KB

#define FLASH_WAITETIME   50000         // Flash 操作的最大等待时间

// 宏定义，控制是否启用写功能
#define STM32_FLASH_WREN  1             // 1: 使能写功能，0: 禁用写功能

// 外部变量声明
extern FLASH_ProcessTypeDef p_Flash;
extern uint16_t STMFLASH_BUF[STM_SECTOR_SIZE / 2];

// 函数声明
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);
void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead);

#if STM32_FLASH_WREN
void STMFLASH_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);
void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);
#endif

void Flash_PageErase(uint32_t PageAddress);

#endif // __STMFLASH_H
