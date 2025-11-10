/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART2 init + jednoduché DMA RX/ blocking TX helper
  ******************************************************************************
  */

/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  */

#include "usart.h"
#include "dma.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_dma.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

uint8_t bufferUSART2dma[DMA_USART2_BUFFER_SIZE];

/* USART2 init function ------------------------------------------------------*/
void MX_USART2_UART_Init(void)
{
    LL_USART_InitTypeDef u = {0};
    LL_GPIO_InitTypeDef  g = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    /* USART2 GPIO Configuration
       PA2   -> USART2_TX
       PA15  -> USART2_RX  */
    g.Pin        = LL_GPIO_PIN_2 | LL_GPIO_PIN_15;
    g.Mode       = LL_GPIO_MODE_ALTERNATE;
    g.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    g.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    g.Pull       = LL_GPIO_PULL_UP;
    g.Alternate  = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &g);

    /* USART2 configuration --------------------------------------------------*/
    u.BaudRate            = 115200;
    u.DataWidth           = LL_USART_DATAWIDTH_8B;
    u.StopBits            = LL_USART_STOPBITS_1;
    u.Parity              = LL_USART_PARITY_NONE;
    u.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    u.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    u.OverSampling        = LL_USART_OVERSAMPLING_16;

    LL_USART_Disable(USART2);
    LL_USART_Init(USART2, &u);

    /* --- Nastavenie správneho baudrate podľa PCLK1 --- */
    LL_RCC_ClocksTypeDef rcc;
    LL_RCC_GetSystemClocksFreq(&rcc);
    uint32_t pclk1 = rcc.PCLK1_Frequency;

    LL_USART_SetBaudRate(USART2,
                         pclk1,
                         LL_USART_OVERSAMPLING_16,
                         115200);
    /* --------------------------------------------------- */

    /* DMA RX configuration */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_6,
                           LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE),
                           (uint32_t)bufferUSART2dma,
                           LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, DMA_USART2_BUFFER_SIZE);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);

    LL_USART_EnableDMAReq_RX(USART2);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_6);

    /* Enable USART2 peripheral */
    LL_USART_Enable(USART2);
    while((!LL_USART_IsActiveFlag_TEACK(USART2)) || (!LL_USART_IsActiveFlag_REACK(USART2))) {}
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */


/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
