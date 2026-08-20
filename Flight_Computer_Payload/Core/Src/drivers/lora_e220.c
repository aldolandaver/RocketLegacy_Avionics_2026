/**
 ******************************************************************************
 * @file    lora_e220.c
 * @brief   En modo transparente el E220 se comporta como un puente UART-RF
 *          puro: todo lo que entra por su pin RX sale por aire, y viceversa.
 *          La única señal de control relevante en este modo es AUX (baja
 *          mientras el módulo procesa/transmite).
 ******************************************************************************
 */
#include "drivers/lora_e220.h"

#define UART_TX_TIMEOUT_MS   100u
#define AUX_WAIT_TIMEOUT_MS  50u

static bool wait_aux_idle(lora_e220_handle_t *dev, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(dev->aux_port, dev->aux_pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return false; /* módulo posiblemente ocupado; el llamador decide reintentar */
        }
    }
    return true;
}

void LoRa_Init(lora_e220_handle_t *dev)
{
    /* Nada que configurar por software: M0/M1 están fijos por hardware en
     * modo transparente y los parámetros RF (915 MHz / 30 dBm) se graban
     * una sola vez en fábrica/banco con la herramienta RF Settings. */
    (void)wait_aux_idle(dev, AUX_WAIT_TIMEOUT_MS);

    dev->cmd_line_idx = 0u;
    dev->cmd_line_ready = false;
    memset(dev->cmd_line_buf, 0, sizeof(dev->cmd_line_buf));
}

void LoRa_InitUplinkRx(lora_e220_handle_t *dev)
{
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

void LoRa_RxByteISR(lora_e220_handle_t *dev)
{
    char c = (char)dev->rx_byte;

    if (!dev->cmd_line_ready) {
        if (c == '\n') {
            dev->cmd_line_buf[dev->cmd_line_idx] = '\0';
            dev->cmd_line_ready = true;
            dev->cmd_line_idx = 0u;
        } else if (c != '\r' && dev->cmd_line_idx < (CMD_MAX_LEN - 1u)) {
            dev->cmd_line_buf[dev->cmd_line_idx++] = c;
        } else if (dev->cmd_line_idx >= (CMD_MAX_LEN - 1u)) {
            dev->cmd_line_idx = 0u; /* comando anómalamente largo, descartar */
        }
    }

    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1); /* rearmar siempre */
}

bool LoRa_ParsePendingCommandFromISR(lora_e220_handle_t *dev, ground_cmd_t *out_cmd)
{
    if (!dev->cmd_line_ready) {
        return false;
    }

    if (strncmp(dev->cmd_line_buf, CMD_SYS_ABORT_STR, sizeof(CMD_SYS_ABORT_STR)) == 0) {
        *out_cmd = GROUND_CMD_SYS_ABORT;
    } else if (strncmp(dev->cmd_line_buf, CMD_FORCE_DEPLOY_STR, sizeof(CMD_FORCE_DEPLOY_STR)) == 0) {
        *out_cmd = GROUND_CMD_FORCE_DEPLOY;
    } else {
        *out_cmd = GROUND_CMD_NONE; /* trama corrupta o comando no reconocido: se ignora */
    }

    dev->cmd_line_ready = false;
    return true;
}

uint8_t LoRa_CRC8(const uint8_t *data, uint16_t len)
{
    /* CRC-8 simple (poly 0x07) suficiente para detectar corrupción de
     * trama por ruido RF en un enlace de baja tasa de paquetes. */
    uint8_t crc = 0x00u;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

bool LoRa_SendTelemetryFrame(lora_e220_handle_t *dev, const lora_telemetry_frame_t *frame)
{
    if (!wait_aux_idle(dev, AUX_WAIT_TIMEOUT_MS)) {
        return false; /* se descarta este ciclo de 1 Hz, no se bloquea la tarea */
    }

    lora_telemetry_frame_t tx_frame = *frame;
    tx_frame.sync[0] = 0xAAu;
    tx_frame.sync[1] = 0x55u;
    tx_frame.crc8 = LoRa_CRC8((uint8_t *)&tx_frame, sizeof(tx_frame) - 1u);

    return HAL_UART_Transmit(dev->huart, (uint8_t *)&tx_frame, sizeof(tx_frame),
                              UART_TX_TIMEOUT_MS) == HAL_OK;
}

bool LoRa_SendBeacon(lora_e220_handle_t *dev, float lat_deg, float lon_deg, uint32_t mission_time_ms)
{
    /* Trama de baliza reducida: solo posición + tiempo, prioriza autonomía
     * de batería en el Estado 4 (Rescate) sobre riqueza de datos. */
    lora_telemetry_frame_t beacon = { 0 };
    beacon.mission_state = 4u; /* MISSION_STATE_RESCUE_BEACON */
    beacon.mission_time_ms = mission_time_ms;
    beacon.gnss_lat_deg = lat_deg;
    beacon.gnss_lon_deg = lon_deg;

    return LoRa_SendTelemetryFrame(dev, &beacon);
}
