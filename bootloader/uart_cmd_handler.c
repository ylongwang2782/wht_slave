/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : uart_cmd_handler.c
 * @brief          : UART command handler implementation
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
#include "uart_cmd_handler.h"

#include <stdio.h>
#include <string.h>

#include "bootloader_flag.h"
#include "cmsis_os2.h"
#include "factory_test.h"
#include "usart.h"

/* Private variables ---------------------------------------------------------*/
static char uart_cmd_buffer[UART_CMD_BUFFER_SIZE];
static uint8_t uart_cmd_index = 0;
 uint8_t uart_rx_char;

/* Task handles */
osThreadId_t uartCmdTaskHandle;
const osThreadAttr_t uartCmdTask_attributes = {
    .name = "uartCmdTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
static void trim_string(char* str);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Trim whitespace from string
 * @param  str: String to trim
 * @retval None
 */
static void trim_string(char* str) {
    char* end;

    // Trim leading space
    while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;

    if (*str == 0) return;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while (end > str &&
           (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n'))
        end--;

    // Write new null terminator
    *(end + 1) = 0;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Restart UART receive interrupt (call after baudrate change)
 * @retval None
 */
void uart_cmd_handler_restart_interrupt(void) {
    // 重新启动UART接收中断
    RS485_RX_EN();
    HAL_UART_Receive_IT(&RS485_UART, &uart_rx_char, 1);
}

/**
 * @brief  Initialize UART command handler
 * @retval None
 */
void uart_cmd_handler_init(void) {
    // 清空命令缓冲区
    memset(uart_cmd_buffer, 0, sizeof(uart_cmd_buffer));
    uart_cmd_index = 0;

    // 创建UART命令处理任务
    uartCmdTaskHandle =
        osThreadNew(uart_cmd_handler_task, NULL, &uartCmdTask_attributes);

    if (uartCmdTaskHandle != NULL) {
        printf("UART command handler initialized successfully\r\n");
        printf("UART Command Handler Ready\r\n");
        printf("Type 'help' for available commands\r\n");
    } else {
        printf("Failed to create UART command handler task\r\n");
    }

    // 启动第一次UART接收中断
    RS485_RX_EN();
    HAL_UART_Receive_IT(&RS485_UART, &uart_rx_char, 1);
}

/**
 * @brief  UART command handler task
 * @param  argument: Not used
 * @retval None
 */
void uart_cmd_handler_task(void* argument) {
    printf("UART command handler task started\r\n");

    for (;;) {
        // 定期检查，避免过度占用CPU
        osDelay(100);
    }
}

/**
 * @brief  Process received UART command
 * @param  command: Command string to process
 * @retval None
 */
void process_uart_command(char* command) {
    trim_string(command);

    printf("Processing command: '%s'\r\n", command);

    if (strcmp(command, CMD_BOOTLOADER_UPGRADE) == 0) {
        printf(
            "Upgrading firmware... System will reset to bootloader mode.\r\n");
        printf("Firmware upgrade command received\r\n");

        // 触发系统复位进入bootloader
        trigger_system_reset_to_bootloader();

    } else if (strcmp(command, CMD_HELP) == 0) {
        printf("Available commands:\r\n");
        printf("  upgrade  - Enter bootloader mode for firmware upgrade\r\n");
        printf("  version  - Show firmware version\r\n");
        printf("  help     - Show this help message\r\n");

    } else if (strcmp(command, CMD_VERSION) == 0) {
        printf("Firmware Version: 1.0.0\r\n");
        printf("Build Date: " __DATE__ " " __TIME__ "\r\n");
        printf("MCU: STM32F429ZI\r\n");

    } else if (strlen(command) > 0) {
        printf("Unknown command: '%s'. Type 'help' for available commands.\r\n",
               command);
    }
}

/**
 * @brief  UART receive complete callback
 * @param  huart: UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == RS485_UART.Instance) {
        // 检查是否是工厂测试入口指令（仅在检测期间有效）
        factory_test_process_entry_byte(uart_rx_char);

        if (factory_test_is_enabled()) {
            factory_test_process_data(uart_rx_char);
        } else {
            // 正常的命令处理逻辑
            if (uart_rx_char == '\r' || uart_rx_char == '\n') {
                // 接收到回车或换行，处理命令
                if (uart_cmd_index > 0) {
                    uart_cmd_buffer[uart_cmd_index] = '\0';
                    process_uart_command(uart_cmd_buffer);
                    uart_cmd_index = 0;
                    memset(uart_cmd_buffer, 0, sizeof(uart_cmd_buffer));
                }
            } else if (uart_rx_char == '\b' || uart_rx_char == 127) {
                // 退格键处理
                if (uart_cmd_index > 0) {
                    uart_cmd_index--;
                    uart_cmd_buffer[uart_cmd_index] = '\0';
                    // 发送退格、空格、退格来清除终端上的字符
                    RS485_TX_EN();
                    HAL_UART_Transmit(&RS485_UART, (uint8_t*)"\b \b", 3,
                                      HAL_MAX_DELAY);
                    RS485_RX_EN();
                }
            } else if (uart_cmd_index < (UART_CMD_BUFFER_SIZE - 1)) {
                // 普通字符，添加到缓冲区
                uart_cmd_buffer[uart_cmd_index] = uart_rx_char;
                uart_cmd_index++;

                // 回显字符到终端
                RS485_TX_EN();
                HAL_UART_Transmit(&RS485_UART, &uart_rx_char, 1, HAL_MAX_DELAY);
                RS485_RX_EN();
            }
        }

        // 继续接收下一个字符
        RS485_RX_EN();
        HAL_UART_Receive_IT(&RS485_UART, &uart_rx_char, 1);
    }
}
