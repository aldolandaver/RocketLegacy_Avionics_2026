/**
 ******************************************************************************
 * @file    sht40.c
 * @brief   SHT40 no usa registros de memoria direccionable: se envía un
 *          comando de 1 byte y se leen 6 bytes (temp[2]+crc, hum[2]+crc).
 ******************************************************************************
 */
#include "drivers/sht40.h"

#define I2C_TIMEOUT_MS   20u

/* CRC-8/Sensirion: polinomio 0x31, init 0xFF (datasheet SHT4x sección 4.4) */
static uint8_t crc8_sensirion(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFu;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

bool SHT40_ReadSample(sht40_handle_t *dev, sht40_sample_t *out)
{
    uint8_t cmd = SHT40_CMD_MEASURE_HIGH_PRECISION;
    if (HAL_I2C_Master_Transmit(dev->hi2c, dev->dev_addr, &cmd, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    HAL_Delay(SHT40_MEASURE_DELAY_MS);

    uint8_t rx[6];
    if (HAL_I2C_Master_Receive(dev->hi2c, dev->dev_addr, rx, 6, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    if (crc8_sensirion(&rx[0], 2) != rx[2] || crc8_sensirion(&rx[3], 2) != rx[5]) {
        return false; /* integridad de trama comprometida, descartar muestra */
    }

    uint16_t raw_t = (uint16_t)((rx[0] << 8) | rx[1]);
    uint16_t raw_h = (uint16_t)((rx[3] << 8) | rx[4]);

    /* Conversión según fórmulas de la hoja de datos (sección 4.6) */
    out->temperature_c   = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    out->humidity_pct_rh = -6.0f + 125.0f * ((float)raw_h / 65535.0f);

    if (out->humidity_pct_rh > 100.0f) out->humidity_pct_rh = 100.0f;
    if (out->humidity_pct_rh < 0.0f)   out->humidity_pct_rh = 0.0f;

    return true;
}
