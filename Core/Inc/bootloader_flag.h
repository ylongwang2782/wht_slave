/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : bootloader_flag.h
 * @brief          : Bootloader flag management header
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

#ifndef __BOOTLOADER_FLAG_H__
#define __BOOTLOADER_FLAG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private defines -----------------------------------------------------------*/
// STM32F429 Flash扇区布局 (最后几个扇区):
// Sector 21: 0x081A0000-0x081BFFFF (128KB)
// Sector 22: 0x081C0000-0x081DFFFF (128KB)  
// Sector 23: 0x081E0000-0x081FFFFF (128KB) <- 用于标志位存储
// 使用Flash Sector 23的末尾存储bootloader标志位
#define BOOTLOADER_FLAG_SECTOR      23                    // Flash扇区23
#define BOOTLOADER_FLAG_ADDRESS     0x081FFF00UL          // 扇区23末尾预留256字节用于标志位
#define BOOTLOADER_FLAG_MAGIC       0x12345678UL          // 魔数，用于验证标志位的有效性
#define BOOTLOADER_FLAG_UPGRADE     0xABCDEF00UL          // 升级标志

/* Bootloader flag structure */
typedef struct {
    uint32_t magic;    // 魔数，用于验证结构的有效性
    uint32_t flag;     // 标志位，指示是否进入bootloader
} bootloader_flag_t;

/* Function prototypes -------------------------------------------------------*/
void set_bootloader_upgrade_flag(void);
void clear_bootloader_flag(void);
uint8_t check_bootloader_upgrade_flag(void);
void trigger_system_reset_to_bootloader(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_FLAG_H__ */
