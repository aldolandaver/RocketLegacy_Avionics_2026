/**
 ******************************************************************************
 * @file    main.h
 * @brief   Externs de los handles de periféricos HAL y prototipos de las
 *          rutinas MX_*_Init(). Estos cuerpos los genera STM32CubeIDE a
 *          partir del .ioc (ya configurado en el proyecto real); aquí solo
 *          se declaran para que el resto del firmware pueda enlazar contra
 *          ellos sin duplicar configuración gráfica.
 ******************************************************************************
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

/* Handles de periféricos, definidos por el código generado (spi.c, i2c.c,
 * usart.c). SPI1 es compartido por IMU (accel+gyro, 2 CS) y Flash (1 CS).
 * I2C1 es compartido por BMP390 y SHT40 (direcciones distintas). */
extern SPI_HandleTypeDef  hspi1;
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart1; /* LoRa E220 (telemetría + uplink) */
extern UART_HandleTypeDef huart2; /* GNSS M10Q                       */

/* Generadas por CubeMX a partir del .ioc — no se reimplementan aquí,
 * conforme al alcance de este entregable (solo lógica de aplicación). */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_I2C1_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

void Error_Handler(void);

#endif /* MAIN_H */
