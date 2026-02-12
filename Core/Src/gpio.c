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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RUN_LED_Pin|MAIN_BELL_LED_Pin|MENU_POPUP_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS485_DE_Pin|RS485_RE_Pin|SUB_BELL_LED_Pin|LOCAL_BELL_LED_Pin
                          |SIREN_LED_Pin|EMERGENCY_BD_LED_Pin|FIRE_LED_Pin|ERR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_3_Pin|TX_LED_Pin|RX_LED_Pin|BUZZER_1_Pin
                          |BUZZER_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RUN_LED_Pin MAIN_BELL_LED_Pin MENU_POPUP_LED_Pin */
  GPIO_InitStruct.Pin = RUN_LED_Pin|MAIN_BELL_LED_Pin|MENU_POPUP_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_DE_Pin RS485_RE_Pin SUB_BELL_LED_Pin LOCAL_BELL_LED_Pin
                           SIREN_LED_Pin EMERGENCY_BD_LED_Pin */
  GPIO_InitStruct.Pin = RS485_DE_Pin|RS485_RE_Pin|SUB_BELL_LED_Pin|LOCAL_BELL_LED_Pin
                          |SIREN_LED_Pin|EMERGENCY_BD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_3_Pin TX_LED_Pin RX_LED_Pin BUZZER_1_Pin
                           BUZZER_2_Pin */
  GPIO_InitStruct.Pin = BUZZER_3_Pin|TX_LED_Pin|RX_LED_Pin|BUZZER_1_Pin
                          |BUZZER_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MENU_POPUP_Pin MAIN_BELL_STOP_Pin SUB_BELL_STOP_Pin LOCAL_BELL_STOP_Pin
                           SIREN_STOP_Pin EMERGENCY_STOP_Pin LOCAL_PHONE_IN_Pin PHONE_JACK_IN_Pin */
  GPIO_InitStruct.Pin = MENU_POPUP_Pin|MAIN_BELL_STOP_Pin|SUB_BELL_STOP_Pin|LOCAL_BELL_STOP_Pin
                          |SIREN_STOP_Pin|EMERGENCY_STOP_Pin|LOCAL_PHONE_IN_Pin|PHONE_JACK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FIRE_LED_Pin ERR_LED_Pin */
  GPIO_InitStruct.Pin = FIRE_LED_Pin|ERR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT_0_Pin */
  GPIO_InitStruct.Pin = BOOT_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOOT_0_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
