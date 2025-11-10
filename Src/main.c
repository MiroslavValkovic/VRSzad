/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (kompiluje bez LPS25HB drivera)
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (kompiluje bez LPS25HB drivera)
  ******************************************************************************
  */
/* USER CODE END Header */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body -- HTS221 + LPS25HB (inline driver)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body – HTS221 + LPS25HB (inline)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "i2c.h"
#include "hts221.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* --- LPS25HB inline definície --------------------------------------------- */
#define LPS25HB_ADDR_SA0_LOW     0x5C
#define LPS25HB_ADDR_SA0_HIGH    0x5D
#define LPS25HB_REG_WHO_AM_I     0x0F
#define LPS25HB_REG_CTRL1        0x20
#define LPS25HB_REG_CTRL2        0x21
#define LPS25HB_REG_PRESS_OUT_XL 0x28
#define LPS25HB_SUB_AUTOINC      0x80
#define LPS25HB_WHO_AM_I_VAL     0xBD

static uint8_t g_lps25hb_addr = 0x00;

static void lps25hb_write_u8(uint8_t reg, uint8_t val)
{
  i2c_master_write(val, reg, g_lps25hb_addr, 0);
}

static uint8_t lps25hb_read_u8(uint8_t reg)
{
  uint8_t v = 0;
  i2c_master_read(&v, 1, reg, g_lps25hb_addr, 0);
  return v;
}

static uint8_t lps25hb_detect_address(void)
{
  uint8_t who = 0;

  i2c_master_read(&who, 1, LPS25HB_REG_WHO_AM_I, LPS25HB_ADDR_SA0_LOW, 0);
  if (who == LPS25HB_WHO_AM_I_VAL) { g_lps25hb_addr = LPS25HB_ADDR_SA0_LOW;  return 1; }

  who = 0;
  i2c_master_read(&who, 1, LPS25HB_REG_WHO_AM_I, LPS25HB_ADDR_SA0_HIGH, 0);
  if (who == LPS25HB_WHO_AM_I_VAL) { g_lps25hb_addr = LPS25HB_ADDR_SA0_HIGH; return 1; }

  g_lps25hb_addr = 0x00;
  return 0;
}

static uint8_t lps25hb_init(void)
{
  if (!lps25hb_detect_address())
    return 0;

  lps25hb_write_u8(LPS25HB_REG_CTRL1, 0xA4);  // PD=1, ODR=1Hz, BDU=1
  lps25hb_write_u8(LPS25HB_REG_CTRL2, 0x04);
  LL_mDelay(2);
  lps25hb_write_u8(LPS25HB_REG_CTRL2, 0x00);

  return 1;
}

static float lps25hb_get_pressure(void)
{
  if (g_lps25hb_addr == 0x00) return -1.0f;

  uint8_t raw[3] = {0};
  i2c_master_read(raw, 3, (LPS25HB_REG_PRESS_OUT_XL | LPS25HB_SUB_AUTOINC), g_lps25hb_addr, 0);

  int32_t press_raw = ((int32_t)raw[2] << 16) | ((int32_t)raw[1] << 8) | raw[0];
  if (press_raw & 0x00800000) press_raw |= 0xFF000000;

  return ((float)press_raw) / 4096.0f;  // hPa
}
/* --- Koniec LPS25HB časti -------------------------------------------------- */

/* Globálne premenné -------------------------------------------------------- */
static float g_tempC = 0.0f;
static int8_t g_hum   = 0;
static float g_press  = 0.0f;
static float g_h0     = 0.0f;
static float g_p0     = 1013.25f;
static const float ALT_CONST = 44330.0f;
static char  uart_buf[96];

/* Prototypy ---------------------------------------------------------------- */
void SystemClock_Config(void);

static void uart_tx_blocking(const char *s, int n)
{
  for (int i = 0; i < n; ++i) {
    while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
    LL_USART_TransmitData8(USART2, (uint8_t)s[i]);
  }
  while (!LL_USART_IsActiveFlag_TC(USART2)) {}
}

static float pressure_to_altitude(float p_hPa, float p0_hPa)
{
  if (p_hPa <= 0.0f || p0_hPa <= 0.0f) return 0.0f;
  float ratio = p_hPa / p0_hPa;
  return ALT_CONST * (1.0f - powf(ratio, 1.0f / 5.255f));
}

/* -------------------------------------------------------------------------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  LL_mDelay(20);

  uint8_t hts_ok  = hts221_init();
  uint8_t lps_ok  = lps25hb_init();

  uint8_t who = 0;

  i2c_master_read(&who, 1, HTS221_WHO_AM_I_ADDRESS, HTS221_DEVICE_ADDRESS, 0);
  int n = snprintf(uart_buf, sizeof(uart_buf),
                   "HTS221 WHO_AM_I=0x%02X (ok=%u)\r\n", who, hts_ok);
  uart_tx_blocking(uart_buf, n);

  who = 0;
  if (g_lps25hb_addr)
    i2c_master_read(&who, 1, LPS25HB_REG_WHO_AM_I, g_lps25hb_addr, 0);
  n = snprintf(uart_buf, sizeof(uart_buf),
               "LPS25HB addr=0x%02X WHO_AM_I=0x%02X (ok=%u)\r\n",
               g_lps25hb_addr, who, lps_ok);
  uart_tx_blocking(uart_buf, n);

  if (lps_ok) {
    g_p0 = lps25hb_get_pressure();
    if (g_p0 <= 0.0f) g_p0 = 1013.25f;
  }

  while (1)
  {
    if (hts_ok) {
      g_tempC = hts221_get_temperature();
      g_hum   = hts221_get_humidity();
    }

    if (lps_ok) {
      g_press = lps25hb_get_pressure();
    }

    float altitude = pressure_to_altitude(g_press, g_p0) - g_h0;

    int n = snprintf(uart_buf, sizeof(uart_buf),
                     "T=%.2f C  H=%d %%  P=%.2f hPa  alt=%.2f m\r\n",
                     g_tempC, (int)g_hum, g_press, altitude);
    uart_tx_blocking(uart_buf, n);

    LL_mDelay(500);
  }
}

/**
  * @brief System Clock Configuration
  *        HSE 8 MHz → PLL x9 = 72 MHz, I2C1 clock = HSI
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  LL_RCC_HSE_Enable();
  while (LL_RCC_HSE_IsReady() != 1) {}

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();
  while (LL_RCC_PLL_IsReady() != 1) {}

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

  LL_Init1msTick(72000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(72000000);

  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1) {}
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* Error Handler */
void Error_Handler(void)
{
  while (1) { }
}


/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
