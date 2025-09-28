/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "elog.h"
#include "factory_test.h"
#include "gpio.h"
#include "uart_cmd_handler.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* Definitions for elog */
osThreadId_t elogHandle;
uint32_t elogBuffer[512];
osStaticThreadDef_t elogControlBlock;
const osThreadAttr_t elog_attributes = {
    .name = "elog",
    .stack_mem = &elogBuffer[0],
    .stack_size = sizeof(elogBuffer),
    .cb_mem = &elogControlBlock,
    .cb_size = sizeof(elogControlBlock),
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for elog_lock */
osSemaphoreId_t elog_lockHandle;
const osSemaphoreAttr_t elog_lock_attributes = {.name = "elog_lock"};
/* Definitions for elog_async */
osSemaphoreId_t elog_asyncHandle;
const osSemaphoreAttr_t elog_async_attributes = {.name = "elog_async"};
/* Definitions for elog_dma_lock */
osSemaphoreId_t elog_dma_lockHandle;
const osSemaphoreAttr_t elog_dma_lock_attributes = {.name = "elog_dma_lock"};

extern int main_app(void);
extern void elog_entry(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* creation of elog_lock */
    elog_lockHandle = osSemaphoreNew(1, 1, &elog_lock_attributes);

    /* creation of elog_async */
    elog_asyncHandle = osSemaphoreNew(1, 1, &elog_async_attributes);

    /* creation of elog_dma_lock */
    elog_dma_lockHandle = osSemaphoreNew(1, 1, &elog_dma_lock_attributes);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

    /* creation of elog */
    elogHandle = osThreadNew(elog_entry, NULL, &elog_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    osDelay(50);    // 等待系统稳定

    // 阻塞式检测工厂测试入口指令（1秒）
    if (factory_test_blocking_check_entry()) {
        // 进入工厂测试模式
        factory_test_enter_mode();

        // 工厂测试模式下的主循环
        for (;;) {
            factory_test_task_process();
            osDelay(10);
        }
    } else {
        // 没有检测到工厂测试指令，进入正常应用程序

        // increase RS485 uart baudrate to 460800 for a better debug experience
        RS485_UART.Init.BaudRate = 460800;
        HAL_UART_Init(&RS485_UART);
        // 重新启动UART接收中断，因为波特率更改后中断会失效
        uart_cmd_handler_restart_interrupt();

        main_app();

        // 正常应用程序完成后的循环
        for (;;) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            osDelay(500);
        }
    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

int __io_putchar(int ch) {
    RS485_TX_EN();
    HAL_UART_Transmit(&RS485_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    RS485_RX_EN();
    return ch;
}

/* USER CODE END Application */

