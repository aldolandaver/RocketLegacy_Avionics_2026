/**
 ******************************************************************************
 * @file    mission_config.h
 * @brief   Configuración de la placa de RECUPERACIÓN DEL COHETE (no confundir
 *          con la Carga Útil: esta placa es un sistema físicamente
 *          independiente, ver Fig. 70 del CDR). Su único trabajo es detectar
 *          apogeo y disparar el nicromo — no lleva radio, GNSS ni flash.
 ******************************************************************************
 */
#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

#include "stm32f4xx_hal.h"

/* ==================== MCU / CLOCK ==========================================
 * STM32F411CEU6 @ 100 MHz (HSE + PLL), FPU habilitada. */

/* ==================== DIRECCIÓN I2C (7-bit, formato HAL << 1) ============== */
#define BMP390_I2C_ADDR         (0x77 << 1)   /* Único sensor en el bus I2C1 */

/* ==================== PINOUT ================================================
 * Reducido respecto a la Carga Útil: sin USART (no hay radio ni GNSS en esta
 * placa según Fig. 70 del CDR), por lo que no hay restricción de PA2/PA3.
 * ---------------------------------------------------------------------------
 * Señal                  | Puerto/Pin | Función
 * ---------------------------------------------------------------------------
 * IMU_CS (BMI088 accel)  | PA4        | CS SPI dedicado acelerómetro
 * IMU_CS_GYRO            | PA3        | CS SPI dedicado giroscopio
 * PYRO_FIRE              | PA0        | Salida disparo -> optoacoplador ->
 *                        |            | IRLZ44N (nicromo), ver Fig. 72 CDR
 * STATUS_LED             | PC13       | LED "Power ON" / indicador de estado
 * ---------------------------------------------------------------------------
 * Nota: el BMI088 real requiere dos líneas CS físicas (acelerómetro y
 * giroscopio son dos troqueles SPI independientes dentro del encapsulado),
 * aunque el esquemático simplificado del CDR (Fig. 70) lo etiquete como un
 * único bloque "SPI1 CS1". */
#define IMU_ACCEL_CS_PORT       GPIOA
#define IMU_ACCEL_CS_PIN        GPIO_PIN_4
#define IMU_GYRO_CS_PORT        GPIOA
#define IMU_GYRO_CS_PIN         GPIO_PIN_3

#define PYRO_FIRE_PORT           GPIOA
#define PYRO_FIRE_PIN            GPIO_PIN_0   /* Disparo aislado hilo nicromo */

#define STATUS_LED_PORT          GPIOC
#define STATUS_LED_PIN           GPIO_PIN_13

/* ==================== FRECUENCIAS DE MUESTREO (Hz) ========================= */
#define IMU_SAMPLE_RATE_HZ       200U
#define BARO_SAMPLE_RATE_HZ      50U

#define IMU_PERIOD_MS            (1000U / IMU_SAMPLE_RATE_HZ)   /* 5 ms  */
#define BARO_PERIOD_MS           (1000U / BARO_SAMPLE_RATE_HZ)  /* 20 ms */

/* ==================== UMBRALES DE LA MÁQUINA DE ESTADOS =====================
 * Idénticos a los de la Carga Útil: la lógica de vuelo (detección de
 * despegue/apogeo/aterrizaje) es la misma en ambos sistemas — la Sección
 * 7.4.1-B del CDR confirma que "la lógica eléctrica es idéntica... con
 * diferencias en el diseño de software" (menos tareas, sin radio/log). */
#define LAUNCH_ACCEL_THRESHOLD_G      3.0f
#define LAUNCH_ACCEL_HOLD_MS          200U

#define LANDED_ACCEL_G                1.0f
#define LANDED_ACCEL_TOLERANCE_G      0.15f
#define LANDED_ALT_DELTA_TOLERANCE_M  1.0f
#define LANDED_HOLD_MS                5000U

#define GRAVITY_MS2                   9.80665f

#define PYRO_FIRE_PULSE_MS            1500U

/* ==================== ARRANQUE / ARMADO ======================================
 * Esta placa NO tiene radio (Fig. 70 del CDR no muestra ningún E220), por lo
 * que no puede recibir SYS_ARM/SYS_CALIBRATE/SYS_ABORT por uplink como sí
 * puede la Carga Útil (ver Tabla 18 del CDR). El "armado" de este sistema es
 * puramente físico: el Interruptor General de la Fig. 70 conecta la batería
 * únicamente cuando el equipo de campo lo decide, y la calibración de sesgo/
 * altitud base ocurre automáticamente una vez al energizar (Estado 0->1).
 * Si el diseño final SÍ contempla un receptor de comandos en esta placa,
 * este archivo y fsm.c deben actualizarse para incorporarlo. */

/* ==================== PRIORIDAD DE TAREA ==================================== */
#define TASK_PRIO_RECOVERY_CONTROL   (osPriorityRealtime)

#endif /* MISSION_CONFIG_H */
