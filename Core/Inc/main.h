/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIP8_Pin GPIO_PIN_5
#define DIP8_GPIO_Port GPIOF
#define DIP7_Pin GPIO_PIN_8
#define DIP7_GPIO_Port GPIOF
#define DIP6_Pin GPIO_PIN_9
#define DIP6_GPIO_Port GPIOF
#define DIP5_Pin GPIO_PIN_10
#define DIP5_GPIO_Port GPIOF
#define DIP4_Pin GPIO_PIN_0
#define DIP4_GPIO_Port GPIOC
#define DIP3_Pin GPIO_PIN_1
#define DIP3_GPIO_Port GPIOC
#define DIP2_Pin GPIO_PIN_2
#define DIP2_GPIO_Port GPIOC
#define DIP1_Pin GPIO_PIN_3
#define DIP1_GPIO_Port GPIOC
#define CLR_SENSOR_Pin GPIO_PIN_7
#define CLR_SENSOR_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_10
#define KEY1_GPIO_Port GPIOG
#define UNLOCK_BTN_Pin GPIO_PIN_11
#define UNLOCK_BTN_GPIO_Port GPIOG
#define AUX_BTN1_Pin GPIO_PIN_13
#define AUX_BTN1_GPIO_Port GPIOG
#define AUX_BTN2_Pin GPIO_PIN_14
#define AUX_BTN2_GPIO_Port GPIOG
#define P_SENSOR_Pin GPIO_PIN_5
#define P_SENSOR_GPIO_Port GPIOB
#define VALVE1_Pin GPIO_PIN_1
#define VALVE1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
