/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : bootloader_flag.c
 * @brief          : Bootloader flag management implementation
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bootloader_flag.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef flash_erase_flag_sector(void);
static HAL_StatusTypeDef flash_write_flag_data(const bootloader_flag_t* flag_data);
static const bootloader_flag_t* get_bootloader_flag_ptr(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Get pointer to bootloader flag structure in Flash
 * @retval Pointer to bootloader flag structure in Flash
 */
static const bootloader_flag_t* get_bootloader_flag_ptr(void) {
    return (const bootloader_flag_t*)BOOTLOADER_FLAG_ADDRESS;
}

/**
 * @brief  Erase the Flash sector containing bootloader flag
 * @retval HAL_OK if successful, HAL_ERROR otherwise
 */
static HAL_StatusTypeDef flash_erase_flag_sector(void) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    // 解锁Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        printf("Flash unlock failed: %d\r\n", status);
        return status;
    }

    // 配置擦除参数
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 2.7V-3.6V
    EraseInitStruct.Sector = BOOTLOADER_FLAG_SECTOR;
    EraseInitStruct.NbSectors = 1;

    // 执行扇区擦除
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        printf("Flash erase failed: %d, sector error: %u\r\n", status, (unsigned int)SectorError);
        HAL_FLASH_Lock();
        return status;
    }

    // 锁定Flash
    HAL_FLASH_Lock();
    
    printf("Flash sector %lu erased successfully\r\n", (unsigned long)BOOTLOADER_FLAG_SECTOR);
    return HAL_OK;
}

/**
 * @brief  Write bootloader flag data to Flash
 * @param  flag_data: Pointer to flag data structure
 * @retval HAL_OK if successful, HAL_ERROR otherwise
 */
static HAL_StatusTypeDef flash_write_flag_data(const bootloader_flag_t* flag_data) {
    HAL_StatusTypeDef status;
    uint32_t address = BOOTLOADER_FLAG_ADDRESS;

    // 解锁Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        printf("Flash unlock failed: %d\r\n", status);
        return status;
    }

    // 写入magic字段 (32位)
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, flag_data->magic);
    if (status != HAL_OK) {
        printf("Flash write magic failed: %d\r\n", status);
        HAL_FLASH_Lock();
        return status;
    }

    // 写入flag字段 (32位)
    address += 4;
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, flag_data->flag);
    if (status != HAL_OK) {
        printf("Flash write flag failed: %d\r\n", status);
        HAL_FLASH_Lock();
        return status;
    }

    // 锁定Flash
    HAL_FLASH_Lock();
    
    printf("Bootloader flag written to Flash at 0x%08lX\r\n", (unsigned long)BOOTLOADER_FLAG_ADDRESS);
    return HAL_OK;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Set bootloader upgrade flag in Flash
 * @retval None
 */
void set_bootloader_upgrade_flag(void) {
    bootloader_flag_t flag_data;
    HAL_StatusTypeDef status;

    // 准备标志位数据
    flag_data.magic = BOOTLOADER_FLAG_MAGIC;
    flag_data.flag = BOOTLOADER_FLAG_UPGRADE;

    printf("Setting bootloader upgrade flag...\r\n");

    // 先擦除Flash扇区
    status = flash_erase_flag_sector();
    if (status != HAL_OK) {
        printf("Failed to erase Flash sector for bootloader flag\r\n");
        return;
    }

    // 写入标志位数据
    status = flash_write_flag_data(&flag_data);
    if (status != HAL_OK) {
        printf("Failed to write bootloader flag to Flash\r\n");
        return;
    }

    printf("Bootloader upgrade flag set successfully at address 0x%08lX\r\n", 
           (unsigned long)BOOTLOADER_FLAG_ADDRESS);
}

/**
 * @brief  Clear bootloader flag in Flash
 * @retval None
 */
void clear_bootloader_flag(void) {
    HAL_StatusTypeDef status;

    printf("Clearing bootloader flag...\r\n");

    // 擦除整个扇区即可清除标志位
    status = flash_erase_flag_sector();
    if (status != HAL_OK) {
        printf("Failed to clear bootloader flag\r\n");
        return;
    }

    printf("Bootloader flag cleared successfully\r\n");
}

/**
 * @brief  Check if bootloader upgrade flag is set
 * @retval 1 if upgrade flag is set, 0 otherwise
 */
uint8_t check_bootloader_upgrade_flag(void) {
    const bootloader_flag_t* flag_ptr = get_bootloader_flag_ptr();

    // 检查Flash中的标志位
    if (flag_ptr->magic == BOOTLOADER_FLAG_MAGIC &&
        flag_ptr->flag == BOOTLOADER_FLAG_UPGRADE) {
        printf("Bootloader upgrade flag detected in Flash\r\n");
        return 1;
    }

    return 0;
}

/**
 * @brief  Trigger system reset to bootloader
 * @note   This function sets the bootloader flag and performs system reset
 * @retval None (function does not return)
 */
void trigger_system_reset_to_bootloader(void) {
    printf("Triggering system reset to bootloader...\r\n");

    // 设置bootloader升级标志位到Flash
    set_bootloader_upgrade_flag();

    // 短暂延时确保Flash写入完成
    // HAL_Delay(10);

    printf("System will reset now...\r\n");

    // 执行系统软复位
    NVIC_SystemReset();

    // 这里不会执行到，因为系统已经复位
    while (1) {
        // 永远不会到达这里
    }
}