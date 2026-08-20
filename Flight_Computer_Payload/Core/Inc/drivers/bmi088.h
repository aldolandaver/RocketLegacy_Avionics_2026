/**
 ******************************************************************************
 * @file    bmi088.h
 * @brief   Driver mínimo SPI para IMU Bosch BMI088 (acelerómetro + giroscopio
 *          en sub-buses independientes, cada uno con su propio CS).
 *          Rango configurado: Accel ±24g / Gyro ±2000 dps, ODR 200 Hz / BW 45 Hz.
 ******************************************************************************
 */
#ifndef BMI088_H
#define BMI088_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- Registros ACC (datasheet Bosch BMI088, sección 5.3) ---- */
#define BMI088_ACC_CHIP_ID_REG      0x00u
#define BMI088_ACC_CHIP_ID_VAL      0x1Eu
#define BMI088_ACC_X_LSB_REG        0x12u   /* 6 bytes: X,Y,Z LSB/MSB */
#define BMI088_ACC_PWR_CONF_REG     0x7Cu
#define BMI088_ACC_PWR_CTRL_REG     0x7Du
#define BMI088_ACC_CONF_REG         0x40u
#define BMI088_ACC_RANGE_REG        0x41u

/* ---- Registros GYRO ---- */
#define BMI088_GYRO_CHIP_ID_REG     0x00u
#define BMI088_GYRO_CHIP_ID_VAL     0x0Fu
#define BMI088_GYRO_X_LSB_REG       0x02u   /* 6 bytes: X,Y,Z LSB/MSB */
#define BMI088_GYRO_RANGE_REG       0x0Fu
#define BMI088_GYRO_BANDWIDTH_REG   0x10u
#define BMI088_GYRO_LPM1_REG        0x11u

/* Sensibilidad para ±24g (LSB/g) y ±2000 dps (LSB/(deg/s)), 16-bit signed */
#define BMI088_ACC_SENS_LSB_PER_G   1365.3f
#define BMI088_GYRO_SENS_LSB_PER_DPS 16.384f

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *acc_cs_port;
    uint16_t      acc_cs_pin;
    GPIO_TypeDef *gyro_cs_port;
    uint16_t      gyro_cs_pin;
} bmi088_handle_t;

typedef struct {
    float acc_x_g, acc_y_g, acc_z_g;       /* en g                 */
    float gyro_x_dps, gyro_y_dps, gyro_z_dps; /* en grados/s      */
} bmi088_sample_t;

/**
 * @brief Valida CHIP_ID de ambos sub-sensores y configura ODR/rango.
 * @retval true si accel y gyro responden con su CHIP_ID esperado.
 */
bool BMI088_Init(bmi088_handle_t *dev);

/**
 * @brief Lectura combinada accel+gyro. Se invoca a IMU_SAMPLE_RATE_HZ (200 Hz)
 *        desde la tarea de fusión sensorial (contexto de tarea, no ISR).
 */
bool BMI088_ReadSample(bmi088_handle_t *dev, bmi088_sample_t *out);

#endif /* BMI088_H */
