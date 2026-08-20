/**
 ******************************************************************************
 * @file    bmp390.h
 * @brief   Driver I2C para barómetro Bosch BMP390 (dirección 0x77).
 *          Provee presión/temperatura compensadas y conversión a altitud
 *          mediante el modelo atmosférico estándar (ISA).
 ******************************************************************************
 */
#ifndef BMP390_H
#define BMP390_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BMP390_REG_CHIP_ID      0x00u
#define BMP390_CHIP_ID_VAL      0x60u
#define BMP390_REG_DATA_0       0x04u  /* PRESS_XLSB..TEMP_MSB, 6 bytes  */
#define BMP390_REG_PWR_CTRL     0x1Bu
#define BMP390_REG_OSR          0x1Cu
#define BMP390_REG_ODR          0x1Du
#define BMP390_REG_CALIB_00     0x31u  /* 21 bytes de coeficientes NVM   */

typedef struct {
    /* Coeficientes de compensación, ya escalados a float según Anexo 8.4
     * de la hoja de datos Bosch BMP390 (evita overflow de enteros de 64 bits
     * en un MCU sin FPU doble precisión). */
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin; /* temperatura de referencia usada en la compensación de P */
} bmp390_calib_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr; /* BMP390_I2C_ADDR ya en formato HAL (<<1) */
    bmp390_calib_t calib;
} bmp390_handle_t;

typedef struct {
    float pressure_pa;
    float temperature_c;
} bmp390_sample_t;

bool  BMP390_Init(bmp390_handle_t *dev);
bool  BMP390_ReadSample(bmp390_handle_t *dev, bmp390_sample_t *out);

/**
 * @brief Convierte presión absoluta a altitud relativa usando la fórmula
 *        barométrica ISA respecto a una presión de referencia en rampa.
 */
float BMP390_PressureToAltitude(float pressure_pa, float ref_pressure_pa);

#endif /* BMP390_H */
