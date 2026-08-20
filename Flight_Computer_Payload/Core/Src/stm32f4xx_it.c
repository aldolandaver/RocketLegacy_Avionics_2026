/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   SOLO se incluye aquí el callback de aplicación relevante para la
 *          misión (HAL_UART_RxCpltCallback). Los handlers de vector
 *          (USART1_IRQHandler, USART2_IRQHandler, SysTick_Handler, etc.) son
 *          la plantilla estándar generada por CubeMX — invocan
 *          HAL_UART_IRQHandler()/HAL_IncTick() sin lógica adicional y no se
 *          reproducen aquí para no consumir espacio en contenido boilerplate.
 ******************************************************************************
 */
#include "main.h"
#include "freertos_tasks.h"
#include "drivers/lora_e220.h"
#include "drivers/gnss_m10q.h"
#include "FreeRTOS.h"
#include "queue.h"

/**
 * @brief Despachador único de "RX complete" para ambos UART de la misión.
 *        USART1 (LoRa): ensambla comandos de uplink y, si detecta un
 *        comando reconocido, lo empuja a g_ground_cmd_queue para que la
 *        tarea Task_UplinkParser (fuera de contexto de ISR) invoque la FSM.
 *        USART2 (GNSS): solo ensambla la línea NMEA; el parseo ocurre en
 *        contexto de tarea (Task_SensorFusion) vía GNSS_ProcessPendingLine.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t higher_prio_task_woken = pdFALSE;

    if (huart->Instance == USART1) {
        LoRa_RxByteISR(&g_lora);

        ground_cmd_t cmd;
        if (LoRa_ParsePendingCommandFromISR(&g_lora, &cmd)) {
            xQueueSendFromISR(g_ground_cmd_queue, &cmd, &higher_prio_task_woken);
        }
    } else if (huart->Instance == USART2) {
        GNSS_RxByteISR(&g_gnss);
    }

    portYIELD_FROM_ISR(higher_prio_task_woken);
}

/**
 * @brief Manejo de errores de UART (framing/overrun/noise). Se limita a
 *        rearmar la recepción por interrupción para no perder el enlace
 *        completo ante un solo byte corrupto (frecuente en RF a 915 MHz
 *        con interferencia intermitente).
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(huart, &g_lora.rx_byte, 1);
    } else if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(huart, &g_gnss.rx_byte, 1);
    }
}
