/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "elog.h"

extern void uwb_int_handler_wrapper(void);
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, UWB_RST_Pin|SPI4_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RUN_LED_GPIO_Port, RUN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UWB_EN_GPIO_Port, UWB_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port, RS485_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ELV3_GPIO_Port, ELV3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ELV2_Pin|VALVE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : UWB_RST_Pin SPI4_NSS_Pin */
  GPIO_InitStruct.Pin = UWB_RST_Pin|SPI4_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RUN_LED_Pin */
  GPIO_InitStruct.Pin = RUN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RUN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UWB_EN_Pin RS485_CTRL_Pin */
  GPIO_InitStruct.Pin = UWB_EN_Pin|RS485_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : UWB_PULSE_Pin DIP8_Pin DIP7_Pin DIP6_Pin
                           DIP5_Pin IO10_Pin IO11_Pin IO12_Pin
                           IO13_Pin IO14_Pin */
  GPIO_InitStruct.Pin = UWB_PULSE_Pin|DIP8_Pin|DIP7_Pin|DIP6_Pin
                          |DIP5_Pin|IO10_Pin|IO11_Pin|IO12_Pin
                          |IO13_Pin|IO14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP4_Pin DIP3_Pin DIP2_Pin DIP1_Pin
                           IO6_Pin IO7_Pin IO47_Pin IO48_Pin
                           IO49_Pin IO50_Pin IO55_Pin IO56_Pin
                           IO57_Pin */
  GPIO_InitStruct.Pin = DIP4_Pin|DIP3_Pin|DIP2_Pin|DIP1_Pin
                          |IO6_Pin|IO7_Pin|IO47_Pin|IO48_Pin
                          |IO49_Pin|IO50_Pin|IO55_Pin|IO56_Pin
                          |IO57_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IO1_Pin IO2_Pin IO3_Pin IO4_Pin
                           IO5_Pin IO51_Pin IO52_Pin IO53_Pin
                           IO54_Pin */
  GPIO_InitStruct.Pin = IO1_Pin|IO2_Pin|IO3_Pin|IO4_Pin
                          |IO5_Pin|IO51_Pin|IO52_Pin|IO53_Pin
                          |IO54_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IO8_Pin IO9_Pin IO26_Pin IO27_Pin
                           IO28_Pin IO29_Pin IO30_Pin IO31_Pin
                           KEY5_Pin KEY6_Pin P_SENSOR_Pin UWB_RDY_Pin
                           ELV4_Pin */
  GPIO_InitStruct.Pin = IO8_Pin|IO9_Pin|IO26_Pin|IO27_Pin
                          |IO28_Pin|IO29_Pin|IO30_Pin|IO31_Pin
                          |KEY5_Pin|KEY6_Pin|P_SENSOR_Pin|UWB_RDY_Pin
                          |ELV4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IO15_Pin IO16_Pin IO40_Pin IO41_Pin
                           IO42_Pin IO43_Pin IO44_Pin IO45_Pin
                           IO46_Pin KEY1_Pin UNLOCK_BTN_Pin AUX_BTN1_Pin
                           AUX_BTN2_Pin */
  GPIO_InitStruct.Pin = IO15_Pin|IO16_Pin|IO40_Pin|IO41_Pin
                          |IO42_Pin|IO43_Pin|IO44_Pin|IO45_Pin
                          |IO46_Pin|KEY1_Pin|UNLOCK_BTN_Pin|AUX_BTN1_Pin
                          |AUX_BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : IO17_Pin IO18_Pin IO19_Pin IO20_Pin
                           IO21_Pin IO22_Pin IO23_Pin IO24_Pin
                           IO25_Pin */
  GPIO_InitStruct.Pin = IO17_Pin|IO18_Pin|IO19_Pin|IO20_Pin
                          |IO21_Pin|IO22_Pin|IO23_Pin|IO24_Pin
                          |IO25_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : IO32_Pin IO33_Pin IO34_Pin IO35_Pin
                           IO36_Pin IO37_Pin IO38_Pin IO39_Pin
                           IO58_Pin IO59_Pin IO60_Pin IO61_Pin
                           IO62_Pin IO63_Pin IO64_Pin CLR_SENSOR_Pin */
  GPIO_InitStruct.Pin = IO32_Pin|IO33_Pin|IO34_Pin|IO35_Pin
                          |IO36_Pin|IO37_Pin|IO38_Pin|IO39_Pin
                          |IO58_Pin|IO59_Pin|IO60_Pin|IO61_Pin
                          |IO62_Pin|IO63_Pin|IO64_Pin|CLR_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : UWB_INT_Pin */
  GPIO_InitStruct.Pin = UWB_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UWB_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ELV3_Pin */
  GPIO_InitStruct.Pin = ELV3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ELV3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ELV2_Pin VALVE1_Pin */
  GPIO_InitStruct.Pin = ELV2_Pin|VALVE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == UWB_INT_Pin)
  {
    uwb_int_handler_wrapper();
  }
}

/* USER CODE END 2 */
