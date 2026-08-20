/**
 ******************************************************************************
 * @file    main.h
 * @brief   Externs de periféricos HAL. Solo SPI1 (IMU) e I2C1 (Baro): sin
 *          USART, esta placa no lleva radio ni GNSS (ver Fig. 70 del CDR).
 ******************************************************************************
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1; /* BMI088 (accel + gyro, 2 CS) */
extern I2C_HandleTypeDef hi2c1; /* BMP390                       */

/* Generadas por CubeMX a partir del .ioc — no se reimplementan aquí. */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_I2C1_Init(void);

void Error_Handler(void);

#endif /* MAIN_H */
