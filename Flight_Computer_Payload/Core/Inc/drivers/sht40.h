/**
 ******************************************************************************
 * @file    sht40.h
 * @brief   Driver I2C para sensor ambiental Sensirion SHT40 (dirección 0x44).
 *          Uso secundario (housekeeping ambiental), no forma parte del lazo
 *          de control crítico ni del filtro de Kalman.
 ******************************************************************************
 */
#ifndef SHT40_H
#define SHT40_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define SHT40_CMD_MEASURE_HIGH_PRECISION   0xFDu
#define SHT40_MEASURE_DELAY_MS             10u  /* t_meas máx a alta precisión */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr; /* SHT40_I2C_ADDR ya en formato HAL (<<1) */
} sht40_handle_t;

typedef struct {
    float temperature_c;
    float humidity_pct_rh;
} sht40_sample_t;

bool SHT40_ReadSample(sht40_handle_t *dev, sht40_sample_t *out);

#endif /* SHT40_H */
