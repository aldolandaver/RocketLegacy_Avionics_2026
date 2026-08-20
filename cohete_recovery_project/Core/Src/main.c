/**
 ******************************************************************************
 * @file    main.c
 * @brief   Punto de entrada de la placa de recuperación del cohete.
 ******************************************************************************
 */
#include "main.h"
#include "freertos_tasks.h"
#include "mission_config.h"
#include "cmsis_os2.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();   /* Incluye PYRO_FIRE, STATUS_LED y los CS de SPI       */
    MX_SPI1_Init();   /* IMU (accel + gyro), hasta 10 MHz                    */
    MX_I2C1_Init();   /* BMP390, modo rápido                                  */

    /* Estado seguro por defecto: pin de disparo pirotécnico en bajo */
    HAL_GPIO_WritePin(PYRO_FIRE_PORT, PYRO_FIRE_PIN, GPIO_PIN_RESET);

    osKernelInitialize();
    FreeRTOS_Tasks_CreateAll();
    osKernelStart(); /* no retorna en operación normal */

    while (1) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
        for (volatile uint32_t i = 0; i < 500000u; i++) { }
    }
}
