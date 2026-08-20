/**
 ******************************************************************************
 * @file    lora_e220.h
 * @brief   Driver de enlace para el transceptor Ebyte E220-900T30D operado en
 *          modo transparente (M0=M1=GND, configurado previamente por
 *          herramienta RF Settings de Ebyte: 915 MHz ISM, 30 dBm).
 *          Este driver NO reconfigura el módulo por comandos AT; asume que
 *          los parámetros de RF ya están grabados en su EEPROM interna.
 ******************************************************************************
 */
#ifndef LORA_E220_H
#define LORA_E220_H

#include "stm32f4xx_hal.h"
#include "fsm.h"     /* reutiliza ground_cmd_t: acoplamiento intencional, este
                      * driver es específico del proyecto y no una librería
                      * genérica reutilizable en otras misiones. */
#include "mission_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Trama de telemetría binaria (downlink) — empaquetada para minimizar
 * tiempo de aire dado el límite de 1 paquete/segundo (evitar saturación
 * térmica del PA de 30 dBm, ver síntesis de diseño). */
#pragma pack(push, 1)
typedef struct {
    uint8_t  sync[2];        /* 0xAA 0x55 */
    uint8_t  mission_state;
    uint32_t mission_time_ms;
    float    altitude_agl_m;
    float    vertical_velocity_ms;
    float    gnss_lat_deg;
    float    gnss_lon_deg;
    uint8_t  pyro_fired;
    uint8_t  crc8;
} lora_telemetry_frame_t;
#pragma pack(pop)

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *aux_port;
    uint16_t      aux_pin;

    /* Ensamblado de comandos de uplink por interrupción, un byte a la vez,
     * terminados en '\n' (mismo patrón que el driver GNSS). */
    uint8_t  rx_byte;
    char     cmd_line_buf[CMD_MAX_LEN];
    uint16_t cmd_line_idx;
    volatile bool cmd_line_ready;
} lora_e220_handle_t;

void LoRa_Init(lora_e220_handle_t *dev);

/** Arma la recepción por interrupción para comandos de uplink (USART1 RX). */
void LoRa_InitUplinkRx(lora_e220_handle_t *dev);

/**
 * @brief Llamar desde HAL_UART_RxCpltCallback para el UART del LoRa.
 *        Ensambla bytes en cmd_line_buf y rearma la recepción.
 */
void LoRa_RxByteISR(lora_e220_handle_t *dev);

/**
 * @brief Debe llamarse SOLO desde contexto de ISR (usa xQueueSendFromISR
 *        internamente) cuando cmd_line_ready quedó en true tras
 *        LoRa_RxByteISR. Empuja el comando reconocido a *out_cmd.
 * @retval true si había una línea pendiente y se parseó (aunque el
 *         resultado sea GROUND_CMD_NONE por comando no reconocido).
 */
bool LoRa_ParsePendingCommandFromISR(lora_e220_handle_t *dev, ground_cmd_t *out_cmd);

/**
 * @brief Transmite una trama de telemetría. Espera (con timeout acotado) a
 *        que AUX indique reposo antes de escribir en el UART, según
 *        recomendación del fabricante para no perder bytes durante el
 *        procesamiento interno del módulo.
 */
bool LoRa_SendTelemetryFrame(lora_e220_handle_t *dev, const lora_telemetry_frame_t *frame);

/**
 * @brief Arma y transmite la trama de la baliza de rescate (Estado 4),
 *        de menor tamaño y a 0.2 Hz para conservar batería.
 */
bool LoRa_SendBeacon(lora_e220_handle_t *dev, float lat_deg, float lon_deg, uint32_t mission_time_ms);

uint8_t LoRa_CRC8(const uint8_t *data, uint16_t len);

#endif /* LORA_E220_H */
