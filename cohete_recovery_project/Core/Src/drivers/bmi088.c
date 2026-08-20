/**
 ******************************************************************************
 * @file    bmi088.c
 * @brief   Implementación SPI del BMI088. Ver bmi088.h del proyecto Carga
 *          Útil para el detalle del protocolo (byte dummy en lecturas del
 *          acelerómetro, ausente en el giroscopio).
 ******************************************************************************
 */
#include "drivers/bmi088.h"

#define SPI_TIMEOUT_MS   5u

static inline void cs_low(GPIO_TypeDef *port, uint16_t pin)  { HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); }
static inline void cs_high(GPIO_TypeDef *port, uint16_t pin) { HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); }

static uint8_t acc_read_reg(bmi088_handle_t *dev, uint8_t reg)
{
    uint8_t tx[3] = { (uint8_t)(reg | 0x80u), 0x00u, 0x00u };
    uint8_t rx[3] = { 0 };

    cs_low(dev->acc_cs_port, dev->acc_cs_pin);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 3, SPI_TIMEOUT_MS);
    cs_high(dev->acc_cs_port, dev->acc_cs_pin);

    return rx[2];
}

static void acc_write_reg(bmi088_handle_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };

    cs_low(dev->acc_cs_port, dev->acc_cs_pin);
    HAL_SPI_Transmit(dev->hspi, tx, 2, SPI_TIMEOUT_MS);
    cs_high(dev->acc_cs_port, dev->acc_cs_pin);
}

static bool acc_read_burst(bmi088_handle_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx_header[2] = { (uint8_t)(reg | 0x80u), 0x00u };
    uint8_t dummy_rx[2];

    cs_low(dev->acc_cs_port, dev->acc_cs_pin);
    if (HAL_SPI_TransmitReceive(dev->hspi, tx_header, dummy_rx, 2, SPI_TIMEOUT_MS) != HAL_OK) {
        cs_high(dev->acc_cs_port, dev->acc_cs_pin);
        return false;
    }
    uint8_t tx_dummy[16] = { 0 };
    bool ok = (HAL_SPI_TransmitReceive(dev->hspi, tx_dummy, buf, len, SPI_TIMEOUT_MS) == HAL_OK);
    cs_high(dev->acc_cs_port, dev->acc_cs_pin);
    return ok;
}

static uint8_t gyro_read_reg(bmi088_handle_t *dev, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80u), 0x00u };
    uint8_t rx[2] = { 0 };

    cs_low(dev->gyro_cs_port, dev->gyro_cs_pin);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, SPI_TIMEOUT_MS);
    cs_high(dev->gyro_cs_port, dev->gyro_cs_pin);

    return rx[1];
}

static void gyro_write_reg(bmi088_handle_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };

    cs_low(dev->gyro_cs_port, dev->gyro_cs_pin);
    HAL_SPI_Transmit(dev->hspi, tx, 2, SPI_TIMEOUT_MS);
    cs_high(dev->gyro_cs_port, dev->gyro_cs_pin);
}

static bool gyro_read_burst(bmi088_handle_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx_header = (uint8_t)(reg | 0x80u);
    uint8_t dummy_rx;
    uint8_t tx_dummy[8] = { 0 };

    cs_low(dev->gyro_cs_port, dev->gyro_cs_pin);
    bool ok = (HAL_SPI_TransmitReceive(dev->hspi, &tx_header, &dummy_rx, 1, SPI_TIMEOUT_MS) == HAL_OK);
    ok = ok && (HAL_SPI_TransmitReceive(dev->hspi, tx_dummy, buf, len, SPI_TIMEOUT_MS) == HAL_OK);
    cs_high(dev->gyro_cs_port, dev->gyro_cs_pin);
    return ok;
}

bool BMI088_Init(bmi088_handle_t *dev)
{
    cs_high(dev->acc_cs_port, dev->acc_cs_pin);
    cs_high(dev->gyro_cs_port, dev->gyro_cs_pin);
    HAL_Delay(1);

    /* Lectura "dummy" de CHIP_ID requerida por errata del fabricante antes
     * de que el registro del acelerómetro responda de forma fiable. */
    (void)acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG);
    uint8_t acc_id = acc_read_reg(dev, BMI088_ACC_CHIP_ID_REG);
    if (acc_id != BMI088_ACC_CHIP_ID_VAL) {
        return false;
    }

    uint8_t gyro_id = gyro_read_reg(dev, BMI088_GYRO_CHIP_ID_REG);
    if (gyro_id != BMI088_GYRO_CHIP_ID_VAL) {
        return false;
    }

    acc_write_reg(dev, BMI088_ACC_PWR_CONF_REG, 0x00u);
    HAL_Delay(1);
    acc_write_reg(dev, BMI088_ACC_PWR_CTRL_REG, 0x04u);
    HAL_Delay(5);

    acc_write_reg(dev, BMI088_ACC_CONF_REG, 0xACu);   /* bwp=normal, odr=200Hz */
    acc_write_reg(dev, BMI088_ACC_RANGE_REG, 0x03u);  /* ±24g */

    gyro_write_reg(dev, BMI088_GYRO_RANGE_REG, 0x00u);      /* ±2000 dps */
    gyro_write_reg(dev, BMI088_GYRO_BANDWIDTH_REG, 0x02u);  /* ODR 1000Hz, BW 116Hz */
    gyro_write_reg(dev, BMI088_GYRO_LPM1_REG, 0x00u);       /* modo normal */

    return true;
}

bool BMI088_ReadSample(bmi088_handle_t *dev, bmi088_sample_t *out)
{
    uint8_t acc_raw[6];
    uint8_t gyro_raw[6];

    if (!acc_read_burst(dev, BMI088_ACC_X_LSB_REG, acc_raw, 6)) {
        return false;
    }
    if (!gyro_read_burst(dev, BMI088_GYRO_X_LSB_REG, gyro_raw, 6)) {
        return false;
    }

    int16_t ax = (int16_t)((acc_raw[1] << 8) | acc_raw[0]);
    int16_t ay = (int16_t)((acc_raw[3] << 8) | acc_raw[2]);
    int16_t az = (int16_t)((acc_raw[5] << 8) | acc_raw[4]);

    int16_t gx = (int16_t)((gyro_raw[1] << 8) | gyro_raw[0]);
    int16_t gy = (int16_t)((gyro_raw[3] << 8) | gyro_raw[2]);
    int16_t gz = (int16_t)((gyro_raw[5] << 8) | gyro_raw[4]);

    out->acc_x_g = (float)ax / BMI088_ACC_SENS_LSB_PER_G;
    out->acc_y_g = (float)ay / BMI088_ACC_SENS_LSB_PER_G;
    out->acc_z_g = (float)az / BMI088_ACC_SENS_LSB_PER_G;

    out->gyro_x_dps = (float)gx / BMI088_GYRO_SENS_LSB_PER_DPS;
    out->gyro_y_dps = (float)gy / BMI088_GYRO_SENS_LSB_PER_DPS;
    out->gyro_z_dps = (float)gz / BMI088_GYRO_SENS_LSB_PER_DPS;

    return true;
}
