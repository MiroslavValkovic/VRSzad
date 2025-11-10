#ifndef I2C_H
#define I2C_H

#include "main.h"
#include "stm32f3xx_ll_i2c.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_utils.h"

/* Signál pre ISR o ukončení čítania */
extern uint8_t end_of_read_flag;

/* Inicializácia I2C1 na PB6/PB7 (AF4), 100 kHz @ HSI 8 MHz */
void MX_I2C1_Init(void);

/* Zápis jedného registra + 1 byte dát (ak read_flag=1, nastaví auto-inc bit v adrese registra) */
void i2c_master_write(uint8_t data,
                      uint8_t register_addr,
                      uint8_t slave_addr,
                      uint8_t read_flag);

/* Sekvenčné čítanie 'length' bytov od 'register_addr' do 'buffer'
   (ak read_flag=1, zapne auto-increment v adrese registra) */
uint8_t* i2c_master_read(uint8_t* buffer,
                         uint8_t length,
                         uint8_t register_addr,
                         uint8_t slave_addr,
                         uint8_t read_flag);

#endif /* I2C_H */
