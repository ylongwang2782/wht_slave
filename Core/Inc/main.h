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
#define UWB_RST_Pin GPIO_PIN_3
#define UWB_RST_GPIO_Port GPIOE
#define SPI4_NSS_Pin GPIO_PIN_4
#define SPI4_NSS_GPIO_Port GPIOE
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define UWB_EN_Pin GPIO_PIN_2
#define UWB_EN_GPIO_Port GPIOF
#define UWB_PULSE_Pin GPIO_PIN_3
#define UWB_PULSE_GPIO_Port GPIOF
#define RS485_CTRL_Pin GPIO_PIN_4
#define RS485_CTRL_GPIO_Port GPIOF
#define DIP8_Pin GPIO_PIN_5
#define DIP8_GPIO_Port GPIOF
#define RS485_RX_Pin GPIO_PIN_6
#define RS485_RX_GPIO_Port GPIOF
#define RS485_TX_Pin GPIO_PIN_7
#define RS485_TX_GPIO_Port GPIOF
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
#define DEBUG_TX_Pin GPIO_PIN_0
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_1
#define DEBUG_RX_GPIO_Port GPIOA
#define VBAT_DET_Pin GPIO_PIN_2
#define VBAT_DET_GPIO_Port GPIOA
#define IO1_Pin GPIO_PIN_3
#define IO1_GPIO_Port GPIOA
#define IO2_Pin GPIO_PIN_4
#define IO2_GPIO_Port GPIOA
#define IO3_Pin GPIO_PIN_5
#define IO3_GPIO_Port GPIOA
#define IO4_Pin GPIO_PIN_6
#define IO4_GPIO_Port GPIOA
#define IO5_Pin GPIO_PIN_7
#define IO5_GPIO_Port GPIOA
#define IO6_Pin GPIO_PIN_4
#define IO6_GPIO_Port GPIOC
#define IO7_Pin GPIO_PIN_5
#define IO7_GPIO_Port GPIOC
#define IO8_Pin GPIO_PIN_0
#define IO8_GPIO_Port GPIOB
#define IO9_Pin GPIO_PIN_1
#define IO9_GPIO_Port GPIOB
#define IO10_Pin GPIO_PIN_11
#define IO10_GPIO_Port GPIOF
#define IO11_Pin GPIO_PIN_12
#define IO11_GPIO_Port GPIOF
#define IO12_Pin GPIO_PIN_13
#define IO12_GPIO_Port GPIOF
#define IO13_Pin GPIO_PIN_14
#define IO13_GPIO_Port GPIOF
#define IO14_Pin GPIO_PIN_15
#define IO14_GPIO_Port GPIOF
#define IO15_Pin GPIO_PIN_0
#define IO15_GPIO_Port GPIOG
#define IO16_Pin GPIO_PIN_1
#define IO16_GPIO_Port GPIOG
#define IO17_Pin GPIO_PIN_7
#define IO17_GPIO_Port GPIOE
#define IO18_Pin GPIO_PIN_8
#define IO18_GPIO_Port GPIOE
#define IO19_Pin GPIO_PIN_9
#define IO19_GPIO_Port GPIOE
#define IO20_Pin GPIO_PIN_10
#define IO20_GPIO_Port GPIOE
#define IO21_Pin GPIO_PIN_11
#define IO21_GPIO_Port GPIOE
#define IO22_Pin GPIO_PIN_12
#define IO22_GPIO_Port GPIOE
#define IO23_Pin GPIO_PIN_13
#define IO23_GPIO_Port GPIOE
#define IO24_Pin GPIO_PIN_14
#define IO24_GPIO_Port GPIOE
#define IO25_Pin GPIO_PIN_15
#define IO25_GPIO_Port GPIOE
#define IO26_Pin GPIO_PIN_10
#define IO26_GPIO_Port GPIOB
#define IO27_Pin GPIO_PIN_11
#define IO27_GPIO_Port GPIOB
#define IO28_Pin GPIO_PIN_12
#define IO28_GPIO_Port GPIOB
#define IO29_Pin GPIO_PIN_13
#define IO29_GPIO_Port GPIOB
#define IO30_Pin GPIO_PIN_14
#define IO30_GPIO_Port GPIOB
#define IO31_Pin GPIO_PIN_15
#define IO31_GPIO_Port GPIOB
#define IO32_Pin GPIO_PIN_8
#define IO32_GPIO_Port GPIOD
#define IO33_Pin GPIO_PIN_9
#define IO33_GPIO_Port GPIOD
#define IO34_Pin GPIO_PIN_10
#define IO34_GPIO_Port GPIOD
#define IO35_Pin GPIO_PIN_11
#define IO35_GPIO_Port GPIOD
#define IO36_Pin GPIO_PIN_12
#define IO36_GPIO_Port GPIOD
#define IO37_Pin GPIO_PIN_13
#define IO37_GPIO_Port GPIOD
#define IO38_Pin GPIO_PIN_14
#define IO38_GPIO_Port GPIOD
#define IO39_Pin GPIO_PIN_15
#define IO39_GPIO_Port GPIOD
#define IO40_Pin GPIO_PIN_2
#define IO40_GPIO_Port GPIOG
#define IO41_Pin GPIO_PIN_3
#define IO41_GPIO_Port GPIOG
#define IO42_Pin GPIO_PIN_4
#define IO42_GPIO_Port GPIOG
#define IO43_Pin GPIO_PIN_5
#define IO43_GPIO_Port GPIOG
#define IO44_Pin GPIO_PIN_6
#define IO44_GPIO_Port GPIOG
#define IO45_Pin GPIO_PIN_7
#define IO45_GPIO_Port GPIOG
#define IO46_Pin GPIO_PIN_8
#define IO46_GPIO_Port GPIOG
#define IO47_Pin GPIO_PIN_6
#define IO47_GPIO_Port GPIOC
#define IO48_Pin GPIO_PIN_7
#define IO48_GPIO_Port GPIOC
#define IO49_Pin GPIO_PIN_8
#define IO49_GPIO_Port GPIOC
#define IO50_Pin GPIO_PIN_9
#define IO50_GPIO_Port GPIOC
#define IO51_Pin GPIO_PIN_8
#define IO51_GPIO_Port GPIOA
#define IO52_Pin GPIO_PIN_11
#define IO52_GPIO_Port GPIOA
#define IO53_Pin GPIO_PIN_12
#define IO53_GPIO_Port GPIOA
#define IO54_Pin GPIO_PIN_15
#define IO54_GPIO_Port GPIOA
#define IO55_Pin GPIO_PIN_10
#define IO55_GPIO_Port GPIOC
#define IO56_Pin GPIO_PIN_11
#define IO56_GPIO_Port GPIOC
#define IO57_Pin GPIO_PIN_12
#define IO57_GPIO_Port GPIOC
#define IO58_Pin GPIO_PIN_0
#define IO58_GPIO_Port GPIOD
#define IO59_Pin GPIO_PIN_1
#define IO59_GPIO_Port GPIOD
#define IO60_Pin GPIO_PIN_2
#define IO60_GPIO_Port GPIOD
#define IO61_Pin GPIO_PIN_3
#define IO61_GPIO_Port GPIOD
#define IO62_Pin GPIO_PIN_4
#define IO62_GPIO_Port GPIOD
#define IO63_Pin GPIO_PIN_5
#define IO63_GPIO_Port GPIOD
#define IO64_Pin GPIO_PIN_6
#define IO64_GPIO_Port GPIOD
#define CLR_SENSOR_Pin GPIO_PIN_7
#define CLR_SENSOR_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOG
#define KEY1_Pin GPIO_PIN_10
#define KEY1_GPIO_Port GPIOG
#define UNLOCK_BTN_Pin GPIO_PIN_11
#define UNLOCK_BTN_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOG
#define AUX_BTN1_Pin GPIO_PIN_13
#define AUX_BTN1_GPIO_Port GPIOG
#define AUX_BTN2_Pin GPIO_PIN_14
#define AUX_BTN2_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOG
#define KEY5_Pin GPIO_PIN_3
#define KEY5_GPIO_Port GPIOB
#define KEY6_Pin GPIO_PIN_4
#define KEY6_GPIO_Port GPIOB
#define P_SENSOR_Pin GPIO_PIN_5
#define P_SENSOR_GPIO_Port GPIOB
#define UWB_INT_Pin GPIO_PIN_6
#define UWB_INT_GPIO_Port GPIOB
#define UWB_INT_EXTI_IRQn EXTI9_5_IRQn
#define UWB_RDY_Pin GPIO_PIN_7
#define UWB_RDY_GPIO_Port GPIOB
#define ELV4_Pin GPIO_PIN_8
#define ELV4_GPIO_Port GPIOB
#define ELV3_Pin GPIO_PIN_9
#define ELV3_GPIO_Port GPIOB
#define ELV2_Pin GPIO_PIN_0
#define ELV2_GPIO_Port GPIOE
#define ELV1_Pin GPIO_PIN_1
#define ELV1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
