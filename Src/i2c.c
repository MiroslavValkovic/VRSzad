/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   I2C1 init + jednoduché master read/write (LL)
  ******************************************************************************
  */

#include "i2c.h"
#include <stddef.h>

/* Vnútorné stavové premenné */
static uint8_t *rx_dst = NULL;
uint8_t end_of_read_flag = 0;

/* Pomocné timeout makro (jednoduchý soft timeout ~1 ms pri 8 MHz) */
#define I2C_WAIT_WHILE(expr)                     \
  do {                                           \
    uint32_t __guard = 8000;                     \
    while ((expr) && __guard--) { __NOP(); }     \
  } while (0)

/* --- Low-level: GPIO + periféria ------------------------------------------------ */
static void i2c1_pins_and_clock_init(void)
{
    /* PB6=SCL, PB7=SDA, AF4, open-drain, pull-up, high speed */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_InitTypeDef g = {0};
    g.Pin        = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    g.Mode       = LL_GPIO_MODE_ALTERNATE;
    g.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    g.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    g.Pull       = LL_GPIO_PULL_UP;
    g.Alternate  = LL_GPIO_AF_4;
    LL_GPIO_Init(GPIOB, &g);

    /* I2C1 clock */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    /* Event IRQ (RXNE a pod.) */
    NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(I2C1_EV_IRQn);
}

void MX_I2C1_Init(void)
{
    i2c1_pins_and_clock_init();

    /* Základná konfigurácia I2C */
    LL_I2C_Disable(I2C1);

    LL_I2C_EnableAutoEndMode(I2C1);
    LL_I2C_DisableOwnAddress2(I2C1);
    LL_I2C_DisableGeneralCall(I2C1);
    LL_I2C_EnableClockStretching(I2C1);

    LL_I2C_InitTypeDef cfg = {0};
    cfg.PeripheralMode   = LL_I2C_MODE_I2C;
    cfg.Timing           = 0x2000090E;   /* ~100 kHz pri HSI 8 MHz */
    cfg.AnalogFilter     = LL_I2C_ANALOGFILTER_ENABLE;
    cfg.DigitalFilter    = 0;
    cfg.OwnAddress1      = 0;
    cfg.TypeAcknowledge  = LL_I2C_ACK;
    cfg.OwnAddrSize      = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &cfg);

    LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);

    LL_I2C_Enable(I2C1);
}

/* --- High-level: zápis/čítanie -------------------------------------------------- */
void i2c_master_write(uint8_t data, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag)
{
    /* Auto-increment bit v adrese registra (ak si to klient želá) */
    if (read_flag) {
        register_addr |= (1u << 7);
    }

    /* Posun adresy doľava – 7bit adresa v HW očakáva R/W na bit0 */
    const uint32_t addr7 = (uint32_t)(slave_addr << 1);

    /* START + adresa + 2 byty (reg + data), AUTOEND => STOP po druhom byte */
    LL_I2C_HandleTransfer(I2C1,
                          addr7,
                          LL_I2C_ADDRSLAVE_7BIT,
                          2,
                          LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_WRITE);

    /* Najprv reg */
    I2C_WAIT_WHILE(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, register_addr);

    /* Potom data */
    I2C_WAIT_WHILE(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, data);

    /* Po AUTOEND očakávaj STOP a zmaž ho */
    I2C_WAIT_WHILE(!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);
}

uint8_t* i2c_master_read(uint8_t* buffer, uint8_t length, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag)
{
    rx_dst = buffer;
    end_of_read_flag = 0;

    /* Auto-increment pri sekvenčnom čítaní z viacerých registrov */
    if (read_flag) {
        register_addr |= (1u << 7);
    }

    /* Fáza 1: zapíš adresu registra (write) */
    LL_I2C_HandleTransfer(I2C1,
                          (uint32_t)(slave_addr << 1),
                          LL_I2C_ADDRSLAVE_7BIT,
                          1,
                          LL_I2C_MODE_SOFTEND,           /* po 1 byte nevygeneruj STOP */
                          LL_I2C_GENERATE_START_WRITE);

    I2C_WAIT_WHILE(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, register_addr);

    /* Počkám na TC (transfer complete), bez STOP (repeated START) */
    I2C_WAIT_WHILE(!LL_I2C_IsActiveFlag_TC(I2C1));

    /* Fáza 2: repeated START a čítanie 'length' bytov */
    LL_I2C_HandleTransfer(I2C1,
                          (uint32_t)(slave_addr << 1),
                          LL_I2C_ADDRSLAVE_7BIT,
                          length,
                          LL_I2C_MODE_AUTOEND,           /* STOP po poslednom byte */
                          LL_I2C_GENERATE_START_READ);

    /* Zapnem RX interrupt, nech ISR vyzdvihuje byty */
    LL_I2C_EnableIT_RX(I2C1);

    /* Čakám na STOP (ukončenie sekvencie), ISR medzitým plní buffer */
    I2C_WAIT_WHILE(!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);

    return buffer;
}

/* --- I2C1 EV IRQ: vyzdvihnutie RX dát ----------------------------------------- */
void I2C1_EV_IRQHandler(void)
{
    if (LL_I2C_IsActiveFlag_RXNE(I2C1))
    {
        *rx_dst++ = LL_I2C_ReceiveData8(I2C1);
    }

    if (LL_I2C_IsActiveFlag_STOP(I2C1))
    {
        /* Koniec transakcie */
        LL_I2C_ClearFlag_STOP(I2C1);
        LL_I2C_DisableIT_RX(I2C1);
        end_of_read_flag = 1;
    }
}
