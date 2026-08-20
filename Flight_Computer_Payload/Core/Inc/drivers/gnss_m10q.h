/**
 ******************************************************************************
 * @file    gnss_m10q.h
 * @brief   Driver UART para GNSS Mateksys M10Q-5883 (chipset u-blox M10),
 *          consumo de sentencias NMEA-0183 estándar (se usa $GxGGA para
 *          fix/lat/lon/altitud). Recepción por interrupción, parseo por
 *          línea en la tarea de logging (prioridad media).
 ******************************************************************************
 */
#ifndef GNSS_M10Q_H
#define GNSS_M10Q_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define GNSS_LINE_BUF_SIZE   96u

typedef struct {
    UART_HandleTypeDef *huart;

    /* Buffer de recepción por interrupción, un byte a la vez, ensamblado
     * hasta encontrar '\n'. Ver HAL_UART_RxCpltCallback en stm32f4xx_it.c */
    uint8_t  rx_byte;
    char     line_buf[GNSS_LINE_BUF_SIZE];
    uint16_t line_idx;
    volatile bool line_ready;
} gnss_m10q_handle_t;

typedef struct {
    bool  fix_valid;
    float latitude_deg;
    float longitude_deg;
    float altitude_msl_m;
    uint8_t satellites_used;
} gnss_fix_t;

void GNSS_Init(gnss_m10q_handle_t *dev);

/**
 * @brief Llamar desde HAL_UART_RxCpltCallback cuando el byte recibido
 *        corresponde a este periférico. Rearma la recepción por interrupción.
 */
void GNSS_RxByteISR(gnss_m10q_handle_t *dev);

/**
 * @brief Si hay una línea NMEA completa pendiente, la parsea (solo $GxGGA)
 *        y actualiza *fix. Debe llamarse desde contexto de tarea (no ISR).
 * @retval true si se actualizó *fix con una sentencia GGA válida.
 */
bool GNSS_ProcessPendingLine(gnss_m10q_handle_t *dev, gnss_fix_t *fix);

#endif /* GNSS_M10Q_H */
