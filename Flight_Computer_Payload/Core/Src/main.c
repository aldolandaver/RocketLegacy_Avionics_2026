/**
 ******************************************************************************
 * @file    main.c
 * @brief   Punto de entrada. Inicializa HAL y periféricos (buses ya
 *          configurados vía CubeMX/.ioc, no se reimplementa aquí), crea las
 *          tareas de FreeRTOS y arranca el planificador. Toda la lógica de
 *          misión vive en fsm.c / freertos_tasks.c / drivers/.
 ******************************************************************************
 */
#include "main.h"
#include "freertos_tasks.h"
#include "mission_config.h"
#include "cmsis_os2.h"

int main(void)
{
    /* ---- Arranque HAL estándar (generado por CubeMX, no repetido aquí) --- */
    HAL_Init();
    SystemClock_Config();   /* 100 MHz vía PLL desde HSE, según .ioc del proyecto */

    MX_GPIO_Init();         /* Incluye configuración de PYRO_FIRE, STATUS_LED,
                              * CS de SPI y AUX del LoRa como se listó en
                              * mission_config.h */
    MX_SPI1_Init();         /* IMU (accel+gyro) + Flash, hasta 10 MHz         */
    MX_I2C1_Init();         /* BMP390 (0x77) + SHT40 (0x44), modo rápido      */
    MX_USART1_UART_Init();  /* LoRa E220-900T30D                              */
    MX_USART2_UART_Init();  /* GNSS Mateksys M10Q-5883                        */

    /* Estado seguro por defecto: pin de disparo pirotécnico en bajo */
    HAL_GPIO_WritePin(PYRO_FIRE_PORT, PYRO_FIRE_PIN, GPIO_PIN_RESET);

    /* ---- Inicialización del kernel RTOS y creación de tareas ---- */
    osKernelInitialize();
    FreeRTOS_Tasks_CreateAll();
    osKernelStart(); /* no retorna en operación normal */

    while (1) {
        /* Inalcanzable: si el planificador falla al arrancar (memoria
         * insuficiente para los TCB/stacks), caemos aquí. */
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
