#include "hts221.h"

/* Interné aliasy na I2C */
static const uint8_t DEV = HTS221_DEVICE_ADDRESS;

/* Jednobyte čítanie/zápis */
static uint8_t rd8(uint8_t reg)
{
    uint8_t v = 0;
    i2c_master_read(&v, 1, reg, DEV, 0);
    return v;
}
static void wr8(uint8_t reg, uint8_t val)
{
    i2c_master_write(val, reg, DEV, 0);
}

/* Blokové čítanie s auto-increment */
static void rdn(uint8_t reg_start, uint8_t *dst, uint8_t n)
{
    i2c_master_read(dst, n, reg_start, DEV, 1);
}

uint8_t hts221_init(void)
{
    /* Einschalten: PD=1, BDU=0, ODR=1 Hz (001) => 0b10000101 = 0x85 */
    wr8(HTS221_ADDRESS_CTRL1, 0x85);

    /* over whoami pre debug/diagnostiku */
    return rd8(HTS221_WHO_AM_I_ADDRESS);
}

int8_t hts221_get_humidity(void)
{
    /* Surové dáta vlhkosti */
    uint8_t raw2[2]; rdn(HTS221_ADDRESS_HUMIDITY_L, raw2, 2);
    int16_t H_OUT = (int16_t)((raw2[1] << 8) | raw2[0]);

    /* Kalibračné body senzora (výstupné hodnoty) */
    uint8_t h0xy[2], h1xy[2];
    rdn(HTS221_ADDRESS_H0_T0_OUT_L, h0xy, 2);
    rdn(HTS221_ADDRESS_H1_T0_OUT_L, h1xy, 2);
    const int16_t H0_OUT = (int16_t)((h0xy[1] << 8) | h0xy[0]);
    const int16_t H1_OUT = (int16_t)((h1xy[1] << 8) | h1xy[0]);

    /* Kalibrované %RH*2 v registroch */
    const uint8_t H0_rH_x2 = rd8(HTS221_ADDRESS_H0_rH_x2);
    const uint8_t H1_rH_x2 = rd8(HTS221_ADDRESS_H1_rH_x2);

    /* Lineárna interpolácia (v x2 mierke) */
    const float k  = (float)((int)H1_rH_x2 - (int)H0_rH_x2) / (float)(H1_OUT - H0_OUT);
    const float b  = (float)H0_rH_x2 - k * (float)H0_OUT;
    float rh2      = k * (float)H_OUT + b;     /* %RH * 2 */

    /* Prevedieme do %RH, ohraničíme 0..100 */
    float rh = rh2 * 0.5f;
    if (rh < 0.f)   rh = 0.f;
    if (rh > 100.f) rh = 100.f;

    /* Vraciame zaokrúhlené celé %RH */
    return (int8_t)(rh + 0.5f);
}

float hts221_get_temperature(void)
{
    /* Teplotný výstup */
    uint8_t t_raw[2]; rdn(HTS221_ADDRESS_TEMP_OUT_L, t_raw, 2);
    const int16_t T_OUT = (int16_t)((t_raw[1] << 8) | t_raw[0]);

    /* Kalibračné výstupy */
    uint8_t t0xy[2], t1xy[2];
    rdn(HTS221_ADDRESS_T0_OUT_L, t0xy, 2);
    rdn(HTS221_ADDRESS_T1_OUT_L, t1xy, 2);
    const int16_t T0_OUT = (int16_t)((t0xy[1] << 8) | t0xy[0]);
    const int16_t T1_OUT = (int16_t)((t1xy[1] << 8) | t1xy[0]);

    /* Kalibračné teploty (x8 + MSB v T1T0_msb) */
    const uint8_t T0_x8 = rd8(HTS221_ADDRESS_T0_degC_x8);
    const uint8_t T1_x8 = rd8(HTS221_ADDRESS_T1_degC_x8);
    const uint8_t MSB   = rd8(HTS221_ADDRESS_T1T0_msb);

    const int16_t T0_deg = (int16_t)((MSB & 0x03) << 8 | T0_x8);         /* x8 */
    const int16_t T1_deg = (int16_t)(((MSB >> 2) & 0x03) << 8 | T1_x8);  /* x8 */

    /* Interpolácia v mierke x8; až výsledok prevedieme na °C */
    const float k8 = (float)(T1_deg - T0_deg) / (float)(T1_OUT - T0_OUT);
    const float b8 = (float)T0_deg - k8 * (float)T0_OUT;
    const float T8 = k8 * (float)T_OUT + b8;

    return T8 / 8.0f;
}
