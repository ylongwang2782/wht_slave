/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : uart_cmd_handler.h
 * @brief          : UART command handler header
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

#ifndef __UART_CMD_HANDLER_H__
#define __UART_CMD_HANDLER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define UART_CMD_BUFFER_SIZE    64
#define UART_CMD_TIMEOUT_MS     1000

/* Command definitions */
#define CMD_BOOTLOADER_UPGRADE  "upgrade"
#define CMD_HELP                "help"
#define CMD_VERSION             "version"

/* Forward declarations -------------------------------------------------------*/
struct UART_HandleTypeDef;

/* Function prototypes -------------------------------------------------------*/
void uart_cmd_handler_init(void);
void uart_cmd_handler_restart_interrupt(void);
void uart_cmd_handler_task(void *argument);
void process_uart_command(char* command);

#ifdef __cplusplus
}
#endif

#endif /* __UART_CMD_HANDLER_H__ */
