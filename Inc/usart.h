/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef USART_H
#define USART_H

#include "main.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_utils.h"

/* --- Konštanty --- */

/* Veľkosť RX DMA kruhového bufferu (prispôsobená tvojmu projektu) */
#define DMA_USART2_BUFFER_SIZE 64U

/* --- Globálne dáta --- */

/* Verejný RX buffer (môžeš čítať prijaté dáta aj mimo DMA callbacku) */
extern uint8_t bufferUSART2dma[DMA_USART2_BUFFER_SIZE];

/* --- Funkcie --- */

/**
  * @brief Inicializácia USART2 (115200 8N1) s DMA pre RX a TX.
  *        - RX: DMA1 Channel 6
  *        - TX: DMA1 Channel 7
  *        - Piny: PA2 (TX), PA15 (RX)
  */
void MX_USART2_UART_Init(void);

/**
  * @brief Zaregistruje callback funkciu pre spracovanie prijatých dát.
  * @param callback  Funkcia typu void f(uint8_t* data, uint16_t len, uint16_t pos)
  */
void USART2_RegisterCallback(void *callback);

/**
  * @brief Skontroluje stav DMA RX prenosu (volaj napr. v hlavnom cykle alebo ISR).
  *        Ak je registrovaný callback, zavolá sa s prijatými dátami.
  */
void USART2_CheckDmaReception(void);

/**
  * @brief Odošle buffer cez DMA.
  * @param buffer  Ukazovateľ na dáta, ktoré sa majú poslať
  * @param length  Počet bajtov na odoslanie
  */
void USART2_PutBuffer(uint8_t *buffer, uint8_t length);

/**
  * @brief Resetuje RX DMA buffer (zastaví, resetuje adresy a znova spustí).
  */
void resetBuffer(void);

#endif /* USART_H */


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
