/**
 ******************************************************************************
 * @file    mission_config.h
 * @brief   Constantes de misión, mapeo de pines y parámetros de calibración.
 *          Único punto de verdad para direcciones de bus y umbrales de la FSM.
 *          NO contiene inicialización de periféricos (eso lo genera CubeMX/.ioc).
 ******************************************************************************
 */
#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include "stm32f4xx_hal.h"

/* ==================== MCU / CLOCK ==========================================
 * STM32F411CEU6 @ 100 MHz (HSE + PLL), FPU habilitada (usar -mfpu=fpv4-sp-d16
 * -mfloat-abi=hard en el toolchain para aprovechar hardware float en el KF). */

/* ==================== DIRECCIONES I2C (7-bit, formato HAL << 1) ============ */
#define BMP390_I2C_ADDR         (0x77 << 1)   /* SDO a VDDIO -> 0x77 */
#define SHT40_I2C_ADDR          (0x44 << 1)

/* ==================== PINOUT DE CONTROL ===================================
 * Chip-selects, resets y GPIOs de propósito específico. Los handles de
 * hSPIx / hI2Cx / hUARTx se asumen ya generados por CubeMX en main.c.
 * ---------------------------------------------------------------------------
 * Señal                  | Puerto/Pin | Función
 * ---------------------------------------------------------------------------
 * IMU_CS (BMI088 accel)  | PA4        | CS SPI dedicado acelerómetro
 * IMU_CS_GYRO            | PB1        | CS SPI dedicado giroscopio
 * FLASH_CS (W25Q16JV)    | PA1        | CS SPI memoria flash externa
 * PYRO_FIRE              | PA0        | Salida disparo MOSFET IRLZ44N (nicromo)
 * STATUS_LED             | PC13       | LED de estado / error parpadeante
 * LORA_AUX                | PB0       | Línea AUX del E220 (indica ocupado)
 * ---------------------------------------------------------------------------
 * Nota: en el STM32F411 el USART2 solo puede mapearse a PA2(TX)/PA3(RX)
 * (sin PORT D disponible en el encapsulado BlackPill de 48 pines), por lo
 * que el CS del giroscopio se reubicó a PB1 para liberar PA3. */
#define IMU_ACCEL_CS_PORT       GPIOA
#define IMU_ACCEL_CS_PIN        GPIO_PIN_4
#define IMU_GYRO_CS_PORT        GPIOB
#define IMU_GYRO_CS_PIN         GPIO_PIN_1

#define FLASH_CS_PORT           GPIOA
#define FLASH_CS_PIN            GPIO_PIN_1

#define PYRO_FIRE_PORT           GPIOA
#define PYRO_FIRE_PIN            GPIO_PIN_0   /* Disparo aislado hilo nicromo */

#define STATUS_LED_PORT          GPIOC
#define STATUS_LED_PIN           GPIO_PIN_13

#define LORA_AUX_PORT            GPIOB
#define LORA_AUX_PIN             GPIO_PIN_0

/* ==================== FRECUENCIAS DE MUESTREO (Hz) ========================= */
#define IMU_SAMPLE_RATE_HZ       200U
#define BARO_SAMPLE_RATE_HZ      50U
#define TELEMETRY_TX_RATE_HZ     1U
#define BEACON_TX_RATE_HZ_X10    2U   /* 0.2 Hz representado como x10 para evitar float en macro */
#define GNSS_SAMPLE_RATE_HZ      1U

#define IMU_PERIOD_MS            (1000U / IMU_SAMPLE_RATE_HZ)   /* 5 ms  */
#define BARO_PERIOD_MS           (1000U / BARO_SAMPLE_RATE_HZ)  /* 20 ms */
#define TELEMETRY_PERIOD_MS      (1000U / TELEMETRY_TX_RATE_HZ) /* 1000 ms */
#define BEACON_PERIOD_MS         5000U                          /* 0.2 Hz  */

/* ==================== UMBRALES DE LA MÁQUINA DE ESTADOS ==================== */
#define LAUNCH_ACCEL_THRESHOLD_G      3.0f     /* Aceleración vertical de despegue */
#define LAUNCH_ACCEL_HOLD_MS          200U     /* Sostenida >= 0.2 s              */

#define LANDED_ACCEL_G                1.0f     /* Reposo ~1G tras aterrizar        */
#define LANDED_ACCEL_TOLERANCE_G      0.15f
#define LANDED_ALT_DELTA_TOLERANCE_M  1.0f     /* "Delta ~ 0" en altitud            */
#define LANDED_HOLD_MS                5000U    /* Sostenido >= 5 s                  */

#define GRAVITY_MS2                   9.80665f

#define PYRO_FIRE_PULSE_MS            1500U    /* Duración de pulso de disparo PA0 */

/* ==================== COMANDOS DE ENLACE ASCENDENTE (Uplink) =============== */
#define CMD_SYS_CALIBRATE_STR   "SYS_CALIBRATE"
#define CMD_SYS_ARM_STR         "SYS_ARM"
#define CMD_SYS_ABORT_STR       "SYS_ABORT"
#define CMD_FORCE_DEPLOY_STR    "FORCE_DEPLOY"
#define CMD_MAX_LEN              32U

/* ==================== PRIORIDADES DE TAREAS FreeRTOS ========================
 * Convención: mayor número = mayor prioridad (igual que configuración nativa
 * de FreeRTOS). Ver freertos_tasks.h para los tamaños de stack asociados.   */
#define TASK_PRIO_SENSOR_FUSION   (osPriorityRealtime)      /* Crítica, hard real-time */
#define TASK_PRIO_DAQ_LOGGING     (osPriorityAboveNormal)   /* Media                   */
#define TASK_PRIO_TELEMETRY       (osPriorityNormal)        /* Baja                    */
#define TASK_PRIO_UPLINK_PARSER   (osPriorityAboveNormal)   /* Reacciona a comandos     */

#endif /* MISSION_CONFIG_H */