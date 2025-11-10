/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f3xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/* === I2C1 MSP INIT: PB6=SCL, PB7=SDA (AF4), OD + PULLUP === */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  if (hi2c->Instance == I2C1) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* PB6 -> I2C1_SCL, PB7 -> I2C1_SDA */
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if (hi2c->Instance == I2C1) {
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    __HAL_RCC_I2C1_CLK_DISABLE();
  }
}

/* === USART2 MSP INIT: PA2=TX, PA15=RX (AF7) === */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART2) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /* PA2 -> USART2_TX (AF7) */
    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA15 -> USART2_RX (AF7)  (NUCLEO-F303K8 VCP) */
    GPIO_InitStruct.Pin       = GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART2) {
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_15);
    __HAL_RCC_USART2_CLK_DISABLE();
  }
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
