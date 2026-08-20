/**
 ******************************************************************************
 * @file    gnss_m10q.c
 * @brief   Ensamblado de líneas NMEA por ISR + parseo mínimo de $GxGGA
 *          (talker ID genérico: GP/GN/GA según constelación combinada).
 *          No se valida checksum NMEA por simplicidad; una trama corrupta
 *          simplemente fallará el parseo de campos y se descartará.
 ******************************************************************************
 */
#include "drivers/gnss_m10q.h"
#include <string.h>
#include <stdlib.h>

void GNSS_Init(gnss_m10q_handle_t *dev)
{
    dev->line_idx = 0u;
    dev->line_ready = false;
    memset(dev->line_buf, 0, sizeof(dev->line_buf));

    /* Arma la primera recepción por interrupción, 1 byte por vez. El M10Q
     * ya sale de fábrica emitiendo NMEA a 9600 bps / 1 Hz (ver USART2). */
    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1);
}

void GNSS_RxByteISR(gnss_m10q_handle_t *dev)
{
    char c = (char)dev->rx_byte;

    if (!dev->line_ready) { /* no pisar un buffer aún no consumido */
        if (c == '\n') {
            dev->line_buf[dev->line_idx] = '\0';
            dev->line_ready = true;
            dev->line_idx = 0u;
        } else if (c != '\r' && dev->line_idx < (GNSS_LINE_BUF_SIZE - 1u)) {
            dev->line_buf[dev->line_idx++] = c;
        } else if (dev->line_idx >= (GNSS_LINE_BUF_SIZE - 1u)) {
            dev->line_idx = 0u; /* línea anómalamente larga, se descarta */
        }
    }

    HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1); /* rearmar siempre */
}

/* Convierte coordenada NMEA ddmm.mmmm / dddmm.mmmm a grados decimales */
static float nmea_coord_to_deg(const char *field, int deg_digits)
{
    if (field[0] == '\0') return 0.0f;

    char deg_str[4] = { 0 };
    memcpy(deg_str, field, (size_t)deg_digits);
    float degrees = (float)atoi(deg_str);
    float minutes = (float)atof(field + deg_digits);

    return degrees + (minutes / 60.0f);
}

bool GNSS_ProcessPendingLine(gnss_m10q_handle_t *dev, gnss_fix_t *fix)
{
    if (!dev->line_ready) {
        return false;
    }

    char local_copy[GNSS_LINE_BUF_SIZE];
    strncpy(local_copy, dev->line_buf, GNSS_LINE_BUF_SIZE);
    local_copy[GNSS_LINE_BUF_SIZE - 1] = '\0';
    dev->line_ready = false; /* liberar el buffer para el siguiente byte-ISR */

    /* Solo procesamos sentencias GGA: $--GGA,hhmmss.ss,lat,N/S,lon,E/W,
     * quality,numSV,HDOP,alt,M,geoidSep,M,dgpsAge,dgpsRef*cs */
    if (strncmp(local_copy + 3, "GGA", 3) != 0) {
        return false;
    }

    char *fields[15] = { 0 };
    uint8_t nf = 0;
    char *tok = strtok(local_copy, ",");
    while (tok != NULL && nf < 15u) {
        fields[nf++] = tok;
        tok = strtok(NULL, ",");
    }
    if (nf < 10u) {
        return false; /* trama truncada */
    }

    uint8_t quality = (uint8_t)atoi(fields[6]);
    fix->fix_valid = (quality > 0u);
    if (!fix->fix_valid) {
        return true; /* trama válida pero sin fix; el llamador decide qué hacer */
    }

    float lat = nmea_coord_to_deg(fields[2], 2);
    if (fields[3][0] == 'S') lat = -lat;

    float lon = nmea_coord_to_deg(fields[4], 3);
    if (fields[5][0] == 'W') lon = -lon;

    fix->latitude_deg     = lat;
    fix->longitude_deg    = lon;
    fix->satellites_used  = (uint8_t)atoi(fields[7]);
    fix->altitude_msl_m   = (float)atof(fields[9]);

    return true;
}
