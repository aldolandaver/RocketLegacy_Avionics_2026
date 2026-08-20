/**
 ******************************************************************************
 * @file    freertos_tasks.c
 * @brief   Única tarea de la placa de recuperación del cohete.
 ******************************************************************************
 */
#include "freertos_tasks.h"
#include "mission_config.h"
#include "main.h"
#include <math.h>

osThreadId_t g_recovery_task_handle;

bmi088_handle_t g_imu  = { .hspi = &hspi1,
                            .acc_cs_port = IMU_ACCEL_CS_PORT, .acc_cs_pin = IMU_ACCEL_CS_PIN,
                            .gyro_cs_port = IMU_GYRO_CS_PORT, .gyro_cs_pin = IMU_GYRO_CS_PIN };

bmp390_handle_t g_baro = { .hi2c = &hi2c1, .dev_addr = BMP390_I2C_ADDR };

static const osThreadAttr_t recovery_task_attr = {
    .name = "RecoveryControl", .priority = TASK_PRIO_RECOVERY_CONTROL,
    .stack_size = STACK_SIZE_RECOVERY_CONTROL * 4u
};

void FreeRTOS_Tasks_CreateAll(void)
{
    g_recovery_task_handle = osThreadNew(Task_RecoveryControl, NULL, &recovery_task_attr);
}

static void blink_error_led_forever(void)
{
    /* Estado 0 fallido: parpadeo rápido, según Fig. 79 del CDR
     * ("Error: LED PARPADEANDO"). No hay radio para reportar la falla:
     * la única forma de diagnosticar en campo es este patrón visual. */
    while (1) {
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
        osDelay(150);
    }
}

/* Promedia N muestras en reposo para calibrar AGL=0 y el sesgo del IMU.
 * Al no haber comando SYS_CALIBRATE por radio en esta placa, esta rutina
 * corre una única vez al energizar; repetirla requiere apagar/encender
 * el Interruptor General (Fig. 70 del CDR). */
static void calibrate_pad_baseline(float *out_pressure_pa, float *out_accel_bias_ms2)
{
    const uint16_t N = 100u; /* ~2 s a 50 Hz de muestreo barométrico */
    float pressure_sum = 0.0f;
    float accel_sum_g  = 0.0f;
    uint16_t valid = 0u;

    for (uint16_t i = 0; i < N; i++) {
        bmp390_sample_t baro_s;
        bmi088_sample_t imu_s;
        if (BMP390_ReadSample(&g_baro, &baro_s) && BMI088_ReadSample(&g_imu, &imu_s)) {
            pressure_sum += baro_s.pressure_pa;
            accel_sum_g  += imu_s.acc_z_g;
            valid++;
        }
        osDelay(BARO_PERIOD_MS);
    }

    *out_pressure_pa = (valid > 0u) ? (pressure_sum / (float)valid) : 101325.0f;
    float mean_accel_g = (valid > 0u) ? (accel_sum_g / (float)valid) : 1.0f;
    *out_accel_bias_ms2 = (mean_accel_g - 1.0f) * GRAVITY_MS2;
}

void Task_RecoveryControl(void *argument)
{
    (void)argument;

    /* ---------------- Estado 0: Inicialización ---------------- */
    FSM_Init();
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET); /* "Power ON" fijo, ver CDR */

    bool imu_ok  = BMI088_Init(&g_imu);
    bool baro_ok = BMP390_Init(&g_baro);

    if (!imu_ok || !baro_ok) {
        FSM_SetErrorState();
        blink_error_led_forever(); /* no retorna */
    }

    /* ---------------- Estado 1: Espera en rampa ---------------- */
    float baseline_pressure_pa, accel_bias_ms2;
    calibrate_pad_baseline(&baseline_pressure_pa, &accel_bias_ms2);
    FSM_EnterPadIdle(baseline_pressure_pa, accel_bias_ms2);

    /* Control no bloqueante del pulso de disparo (evita retener la tarea
     * crítica durante los 1.5 s del pulso) */
    bool     pyro_active = false;
    uint32_t pyro_start_tick = 0u;

    uint32_t mission_start_tick = HAL_GetTick();
    uint32_t last_baro_read_tick = 0u;
    uint32_t last_recovered_blink_tick = 0u;
    bool     recovered_led_state = false;

    const uint32_t tick_period_ms = IMU_PERIOD_MS; /* 5 ms -> 200 Hz */
    uint32_t next_wake = osKernelGetTickCount();

    while (1) {
        next_wake += tick_period_ms;

        fsm_sensor_input_t in = { 0 };

        bmi088_sample_t imu_sample;
        if (BMI088_ReadSample(&g_imu, &imu_sample)) {
            in.accel_z_g = imu_sample.acc_z_g;
        }

        uint32_t now = HAL_GetTick();
        if ((now - last_baro_read_tick) >= BARO_PERIOD_MS) {
            bmp390_sample_t baro_sample;
            if (BMP390_ReadSample(&g_baro, &baro_sample)) {
                in.new_baro_sample    = true;
                in.baro_pressure_pa   = baro_sample.pressure_pa;
                in.baro_temperature_c = baro_sample.temperature_c;
            }
            last_baro_read_tick = now;
        }

        flight_status_t status;
        FSM_RunStep(&in, (float)tick_period_ms / 1000.0f, &status);
        status.mission_time_ms = now - mission_start_tick;

        /* Gestión no bloqueante del pulso de disparo del nicromo (PA0) */
        if (!pyro_active && FSM_ConsumePyroFireEvent()) {
            HAL_GPIO_WritePin(PYRO_FIRE_PORT, PYRO_FIRE_PIN, GPIO_PIN_SET);
            pyro_active = true;
            pyro_start_tick = now;
        }
        if (pyro_active && (now - pyro_start_tick) >= PYRO_FIRE_PULSE_MS) {
            HAL_GPIO_WritePin(PYRO_FIRE_PORT, PYRO_FIRE_PIN, GPIO_PIN_RESET);
            pyro_active = false;
        }

        /* Indicador visual de recuperación completa: parpadeo lento (no
         * especificado en el CDR como comportamiento del LED "Power ON",
         * pero útil en campo para confirmar de un vistazo que el vuelo
         * terminó y el pyro ya se disparó; no reemplaza el parpadeo de
         * error de Estado 0). */
        if (status.state == MISSION_STATE_RECOVERED) {
            if ((now - last_recovered_blink_tick) >= 500u) {
                recovered_led_state = !recovered_led_state;
                HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN,
                                   recovered_led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                last_recovered_blink_tick = now;
            }
        }

        osDelayUntil(next_wake);
    }
}
