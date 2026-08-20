/**
 ******************************************************************************
 * @file    bmp390.c
 * @brief   Driver I2C del BMP390. Compensación de presión/temperatura según
 *          el modelo de coeficientes NVM de 21 bytes documentado en la hoja
 *          de datos Bosch BMP390 (sección "Output compensation"), reescrito
 *          en punto flotante simple para ejecutarse con la FPU del F411.
 ******************************************************************************
 */
#include "drivers/bmp390.h"
#include <math.h>

#define I2C_TIMEOUT_MS   10u

static bool read_regs(bmp390_handle_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT,
                             buf, len, I2C_TIMEOUT_MS) == HAL_OK;
}

static bool write_reg(bmp390_handle_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

/* Reescala los 21 bytes de calibración NVM a los coeficientes en punto
 * flotante usados por las fórmulas de compensación (potencias de 2 fijas
 * definidas por Bosch para este sensor). */
static void parse_calibration(bmp390_handle_t *dev, const uint8_t *raw)
{
    int16_t nvm_t1 = (int16_t)(raw[0] | (raw[1] << 8));
    int16_t nvm_t2 = (int16_t)(raw[2] | (raw[3] << 8));
    int8_t  nvm_t3 = (int8_t)raw[4];

    int16_t nvm_p1 = (int16_t)(raw[5] | (raw[6] << 8));
    int16_t nvm_p2 = (int16_t)(raw[7] | (raw[8] << 8));
    int8_t  nvm_p3 = (int8_t)raw[9];
    int8_t  nvm_p4 = (int8_t)raw[10];
    uint16_t nvm_p5 = (uint16_t)(raw[11] | (raw[12] << 8));
    uint16_t nvm_p6 = (uint16_t)(raw[13] | (raw[14] << 8));
    int8_t  nvm_p7 = (int8_t)raw[15];
    int8_t  nvm_p8 = (int8_t)raw[16];
    int16_t nvm_p9 = (int16_t)(raw[17] | (raw[18] << 8));
    int8_t  nvm_p10 = (int8_t)raw[19];
    int8_t  nvm_p11 = (int8_t)raw[20];

    bmp390_calib_t *c = &dev->calib;
    c->par_t1 = (float)nvm_t1 / 0.00390625f;      /* / 2^-8   */
    c->par_t2 = (float)nvm_t2 / 1073741824.0f;    /* / 2^30   */
    c->par_t3 = (float)nvm_t3 / 281474976710656.0f; /* / 2^48 */

    c->par_p1  = ((float)nvm_p1 - 16384.0f) / 1048576.0f;      /* / 2^20 */
    c->par_p2  = ((float)nvm_p2 - 16384.0f) / 536870912.0f;    /* / 2^29 */
    c->par_p3  = (float)nvm_p3 / 4294967296.0f;                /* / 2^32 */
    c->par_p4  = (float)nvm_p4 / 137438953472.0f;              /* / 2^37 */
    c->par_p5  = (float)nvm_p5 * 8.0f;                         /* * 2^3  */
    c->par_p6  = (float)nvm_p6 / 64.0f;                        /* / 2^6  */
    c->par_p7  = (float)nvm_p7 / 256.0f;                       /* / 2^8  */
    c->par_p8  = (float)nvm_p8 / 32768.0f;                     /* / 2^15 */
    c->par_p9  = (float)nvm_p9 / 281474976710656.0f;           /* / 2^48 */
    c->par_p10 = (float)nvm_p10 / 281474976710656.0f;          /* / 2^48 */
    c->par_p11 = (float)nvm_p11 / 36893488147419103232.0f;     /* / 2^65 */
}

static float compensate_temperature(bmp390_handle_t *dev, uint32_t uncomp_temp)
{
    bmp390_calib_t *c = &dev->calib;
    float pd1 = (float)uncomp_temp - c->par_t1;
    float pd2 = pd1 * c->par_t2;

    c->t_lin = pd2 + (pd1 * pd1) * c->par_t3;
    return c->t_lin;
}

static float compensate_pressure(bmp390_handle_t *dev, uint32_t uncomp_press)
{
    bmp390_calib_t *c = &dev->calib;
    const float t = c->t_lin;

    float po1 = c->par_p5 + c->par_p6 * t + c->par_p7 * t * t + c->par_p8 * t * t * t;

    float po2 = (float)uncomp_press *
                (c->par_p1 + c->par_p2 * t + c->par_p3 * t * t + c->par_p4 * t * t * t);

    float pd1 = (float)uncomp_press * (float)uncomp_press;
    float pd2 = c->par_p9 + c->par_p10 * t;
    float pd3 = pd1 * pd2 + (float)uncomp_press * pd1 * c->par_p11;

    return po1 + po2 + pd3;
}

bool BMP390_Init(bmp390_handle_t *dev)
{
    uint8_t chip_id = 0;
    if (!read_regs(dev, BMP390_REG_CHIP_ID, &chip_id, 1) || chip_id != BMP390_CHIP_ID_VAL) {
        return false;
    }

    uint8_t calib_raw[21];
    if (!read_regs(dev, BMP390_REG_CALIB_00, calib_raw, sizeof(calib_raw))) {
        return false;
    }
    parse_calibration(dev, calib_raw);

    /* OSR: x8 presión / x1 temperatura -> resolución ~0.16 Pa suficiente
     * para detectar el cruce de apogeo sin saturar el bus a 50 Hz. */
    if (!write_reg(dev, BMP390_REG_OSR, 0x03u)) return false;   /* osr_p=x8, osr_t=x1 */
    if (!write_reg(dev, BMP390_REG_ODR, 0x02u)) return false;   /* ODR ~50 Hz (0x02)  */
    /* PWR_CTRL: press_en=1, temp_en=1, mode=normal(0b11<<4) */
    if (!write_reg(dev, BMP390_REG_PWR_CTRL, 0x33u)) return false;

    return true;
}

bool BMP390_ReadSample(bmp390_handle_t *dev, bmp390_sample_t *out)
{
    uint8_t raw[6];
    if (!read_regs(dev, BMP390_REG_DATA_0, raw, 6)) {
        return false;
    }

    uint32_t uncomp_press = ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | raw[0];
    uint32_t uncomp_temp  = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[4] << 8) | raw[3];

    out->temperature_c = compensate_temperature(dev, uncomp_temp);
    out->pressure_pa    = compensate_pressure(dev, uncomp_press);

    return true;
}

float BMP390_PressureToAltitude(float pressure_pa, float ref_pressure_pa)
{
    /* Fórmula barométrica estándar ISA (troposfera, L=0.0065 K/m, T0=288.15K) */
    const float T0 = 288.15f;
    const float L  = 0.0065f;
    const float exp_term = 0.1902225604f; /* R*L/(g*M) para aire seco */

    return (T0 / L) * (1.0f - powf(pressure_pa / ref_pressure_pa, exp_term));
}
