/**
 ******************************************************************************
 * @file    freertos_tasks.c
 * @brief   Definición de las 4 tareas del firmware y de los objetos RTOS /
 *          handles de driver compartidos. Ver mission_config.h para
 *          prioridades y periodos.
 ******************************************************************************
 */
#include "freertos_tasks.h"
#include "mission_config.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* ==================== Objetos RTOS ========================================= */
osThreadId_t   g_sensor_fusion_task_handle;
osThreadId_t   g_daq_logging_task_handle;
osThreadId_t   g_telemetry_task_handle;
osThreadId_t   g_uplink_parser_task_handle;

osMutexId_t    g_telemetry_mutex;
QueueHandle_t  g_ground_cmd_queue;

telemetry_snapshot_t g_telemetry;

/* ==================== Handles de driver ==================================== */
bmi088_handle_t     g_imu   = { .hspi = &hspi1,
                                 .acc_cs_port = IMU_ACCEL_CS_PORT, .acc_cs_pin = IMU_ACCEL_CS_PIN,
                                 .gyro_cs_port = IMU_GYRO_CS_PORT, .gyro_cs_pin = IMU_GYRO_CS_PIN };

bmp390_handle_t     g_baro  = { .hi2c = &hi2c1, .dev_addr = BMP390_I2C_ADDR };
sht40_handle_t      g_env   = { .hi2c = &hi2c1, .dev_addr = SHT40_I2C_ADDR };

w25q16jv_handle_t   g_flash = { .hspi = &hspi1, .cs_port = FLASH_CS_PORT, .cs_pin = FLASH_CS_PIN };

lora_e220_handle_t  g_lora  = { .huart = &huart1, .aux_port = LORA_AUX_PORT, .aux_pin = LORA_AUX_PIN };
gnss_m10q_handle_t  g_gnss  = { .huart = &huart2 };

/* Atributos estáticos de tarea (cmsis_os2), evitan heap dinámico oculto */
static const osThreadAttr_t sensor_fusion_attr = {
    .name = "SensorFusion", .priority = TASK_PRIO_SENSOR_FUSION,
    .stack_size = STACK_SIZE_SENSOR_FUSION * 4u
};
static const osThreadAttr_t daq_logging_attr = {
    .name = "DAQLogging", .priority = TASK_PRIO_DAQ_LOGGING,
    .stack_size = STACK_SIZE_DAQ_LOGGING * 4u
};
static const osThreadAttr_t telemetry_attr = {
    .name = "Telemetry", .priority = TASK_PRIO_TELEMETRY,
    .stack_size = STACK_SIZE_TELEMETRY * 4u
};
static const osThreadAttr_t uplink_attr = {
    .name = "UplinkParser", .priority = TASK_PRIO_UPLINK_PARSER,
    .stack_size = STACK_SIZE_UPLINK_PARSER * 4u
};

void FreeRTOS_Tasks_CreateAll(void)
{
    g_telemetry_mutex = osMutexNew(NULL);
    g_ground_cmd_queue = xQueueCreate(GROUND_CMD_QUEUE_LEN, sizeof(ground_cmd_t));

    memset(&g_telemetry, 0, sizeof(g_telemetry));

    g_sensor_fusion_task_handle  = osThreadNew(Task_SensorFusion, NULL, &sensor_fusion_attr);
    g_daq_logging_task_handle    = osThreadNew(Task_DAQLogging, NULL, &daq_logging_attr);
    g_telemetry_task_handle      = osThreadNew(Task_TelemetryDownlink, NULL, &telemetry_attr);
    g_uplink_parser_task_handle  = osThreadNew(Task_UplinkParser, NULL, &uplink_attr);
}

/* ============================================================================
 * TAREA 1 (prioridad alta) — Sensor Fusion & Control
 * Ejecuta Estado 0 (una vez), luego el bucle de 200 Hz que alimenta la FSM.
 * ==========================================================================*/
static void blink_error_led_forever(void)
{
    while (1) {
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
        osDelay(150); /* parpadeo rápido = fallo de inicialización, Estado 0 */
    }
}

/* Promedia N muestras de presión y sesgo de acelerómetro en reposo para
 * calibrar la referencia AGL=0 y el sesgo inicial del KF (Estado 1). */
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

    *out_pressure_pa   = (valid > 0u) ? (pressure_sum / (float)valid) : 101325.0f;
    /* El sesgo se expresa en m/s^2 sobre el residuo respecto a 1g estático */
    float mean_accel_g = (valid > 0u) ? (accel_sum_g / (float)valid) : 1.0f;
    *out_accel_bias_ms2 = (mean_accel_g - 1.0f) * GRAVITY_MS2;
}

void Task_SensorFusion(void *argument)
{
    (void)argument;

    /* ---------------- Estado 0: Inicialización ---------------- */
    FSM_Init();

    bool imu_ok   = BMI088_Init(&g_imu);
    bool baro_ok  = BMP390_Init(&g_baro);
    bool flash_ok = W25Q16JV_Init(&g_flash);
    GNSS_Init(&g_gnss);          /* el GNSS no tiene un "ready" síncrono fiable;
                                   * se considera disponible si entrega tramas */
    LoRa_Init(&g_lora);
    LoRa_InitUplinkRx(&g_lora);

    if (!imu_ok || !baro_ok || !flash_ok) {
        FSM_SetErrorState();
        blink_error_led_forever(); /* no retorna */
    }

    /* ---------------- Estado 1: Espera en rampa ---------------- */
    float baseline_pressure_pa, accel_bias_ms2;
    calibrate_pad_baseline(&baseline_pressure_pa, &accel_bias_ms2);
    FSM_EnterPadIdle(baseline_pressure_pa, accel_bias_ms2);

    /* Pyro control no bloqueante (evita retener la tarea crítica 1.5 s) */
    bool     pyro_active = false;
    uint32_t pyro_start_tick = 0u;

    uint32_t mission_start_tick = HAL_GetTick();
    uint32_t last_baro_read_tick = 0u;

    const uint32_t tick_period_ms = IMU_PERIOD_MS; /* 5 ms -> 200 Hz */
    uint32_t next_wake = osKernelGetTickCount();

    while (1) {
        next_wake += tick_period_ms;

        fsm_sensor_input_t in = { 0 };

        bmi088_sample_t imu_sample;
        if (BMI088_ReadSample(&g_imu, &imu_sample)) {
            in.accel_z_g = imu_sample.acc_z_g;
        }

        /* El BMP390 se muestrea a 50 Hz: se decima dentro del lazo de 200 Hz */
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

        /* GNSS: se procesa la línea NMEA más reciente si el ISR ya la
         * ensambló (no bloqueante, solo revisa una bandera). */
        gnss_fix_t fix;
        if (GNSS_ProcessPendingLine(&g_gnss, &fix)) {
            in.new_gnss_fix     = true;
            in.gnss_fix_valid   = fix.fix_valid;
            in.gnss_lat_deg     = fix.latitude_deg;
            in.gnss_lon_deg     = fix.longitude_deg;
            in.gnss_alt_msl_m   = fix.altitude_msl_m;
        }

        telemetry_snapshot_t local_snap;
        FSM_RunStep(&in, (float)tick_period_ms / 1000.0f, &local_snap);
        local_snap.mission_time_ms = now - mission_start_tick;

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

        /* Publicación protegida del snapshot compartido */
        if (osMutexAcquire(g_telemetry_mutex, 2u) == osOK) {
            g_telemetry = local_snap;
            osMutexRelease(g_telemetry_mutex);
        }

        osDelayUntil(next_wake); /* mantiene el jitter acotado a 200 Hz */
    }
}

/* ============================================================================
 * TAREA 2 (prioridad media) — DAQ & Logging
 * Empaqueta el snapshot de telemetría y lo escribe secuencialmente en la
 * flash W25Q16JV. Se detiene por completo en RESCUE_BEACON (Estado 4) para
 * no arriesgar corrupción del log una vez aterrizado.
 * ==========================================================================*/
#pragma pack(push, 1)
typedef struct {
    uint32_t mission_time_ms;
    uint8_t  state;
    float    altitude_agl_m;
    float    vertical_velocity_ms;
    float    raw_accel_z_g;
    float    raw_pressure_pa;
    float    raw_temperature_c;
    float    gnss_lat_deg;
    float    gnss_lon_deg;
    uint8_t  gnss_fix_valid;
    uint8_t  pyro_fired;
} log_record_t; /* 35 bytes (empaquetado): caben 7 registros por página de 256B */
#pragma pack(pop)

void Task_DAQLogging(void *argument)
{
    (void)argument;

    /* Se asume flash ya borrada en banco antes del vuelo (ver README,
     * sección de preparación previa al lanzamiento). No se borra en vuelo. */
    uint32_t write_addr = 0u;

    while (1) {
        telemetry_snapshot_t snap;
        if (osMutexAcquire(g_telemetry_mutex, 5u) == osOK) {
            snap = g_telemetry;
            osMutexRelease(g_telemetry_mutex);
        } else {
            osDelay(BARO_PERIOD_MS);
            continue;
        }

        if (snap.state != MISSION_STATE_RESCUE_BEACON) {
            log_record_t rec = {
                .mission_time_ms = snap.mission_time_ms,
                .state = (uint8_t)snap.state,
                .altitude_agl_m = snap.altitude_agl_m,
                .vertical_velocity_ms = snap.vertical_velocity_ms,
                .raw_accel_z_g = snap.raw_accel_z_g,
                .raw_pressure_pa = snap.raw_pressure_pa,
                .raw_temperature_c = snap.raw_temperature_c,
                .gnss_lat_deg = snap.gnss_lat_deg,
                .gnss_lon_deg = snap.gnss_lon_deg,
                .gnss_fix_valid = (uint8_t)snap.gnss_fix_valid,
                .pyro_fired = snap.pyro_fired
            };

            /* El W25Q16JV envuelve la escritura dentro de la misma página si
             * el registro cruza el límite de 256 B (W25Q16JV_WritePage lo
             * rechaza explícitamente, ver driver). En vez de perder la
             * muestra, se salta el remanente de la página actual. */
            uint32_t offset_in_page = write_addr % W25Q_PAGE_SIZE;
            if ((offset_in_page + sizeof(rec)) > W25Q_PAGE_SIZE) {
                write_addr += (W25Q_PAGE_SIZE - offset_in_page);
            }

            if ((write_addr + sizeof(rec)) <= W25Q_TOTAL_SIZE_BYTES) {
                if (W25Q16JV_WritePage(&g_flash, write_addr, (uint8_t *)&rec, sizeof(rec))) {
                    write_addr += sizeof(rec);
                }
                /* Si falla la escritura se reintenta con el mismo puntero
                 * en el siguiente ciclo; se prioriza no perder muestras
                 * frente a avanzar el cursor a ciegas. */
            }
        }

        /* Cadencia de registro alineada al muestreo barométrico (50 Hz) */
        osDelay(BARO_PERIOD_MS);
    }
}

/* ============================================================================
 * TAREA 3 (prioridad baja) — Telemetry Downlink
 * 1 Hz en vuelo normal; 0.2 Hz (cada 5 s) en modo baliza para ahorrar
 * batería del enlace de 30 dBm.
 * ==========================================================================*/
void Task_TelemetryDownlink(void *argument)
{
    (void)argument;

    while (1) {
        telemetry_snapshot_t snap;
        if (osMutexAcquire(g_telemetry_mutex, 10u) == osOK) {
            snap = g_telemetry;
            osMutexRelease(g_telemetry_mutex);
        } else {
            osDelay(TELEMETRY_PERIOD_MS);
            continue;
        }

        if (snap.state == MISSION_STATE_RESCUE_BEACON) {
            LoRa_SendBeacon(&g_lora, snap.gnss_lat_deg, snap.gnss_lon_deg, snap.mission_time_ms);
            osDelay(BEACON_PERIOD_MS); /* 0.2 Hz */
        } else {
            lora_telemetry_frame_t frame = { 0 };
            frame.mission_state = (uint8_t)snap.state;
            frame.mission_time_ms = snap.mission_time_ms;
            frame.altitude_agl_m = snap.altitude_agl_m;
            frame.vertical_velocity_ms = snap.vertical_velocity_ms;
            frame.gnss_lat_deg = snap.gnss_lat_deg;
            frame.gnss_lon_deg = snap.gnss_lon_deg;
            frame.pyro_fired = snap.pyro_fired;

            LoRa_SendTelemetryFrame(&g_lora, &frame);
            osDelay(TELEMETRY_PERIOD_MS); /* 1 Hz */
        }
    }
}

/* ============================================================================
 * TAREA 4 — Uplink command consumer
 * Bloqueada en la cola alimentada desde HAL_UART_RxCpltCallback (USART1).
 * Se mantiene separada de la tarea de telemetría para que un comando de
 * SYS_ABORT se atienda incluso si el downlink está a mitad de un envío.
 * ==========================================================================*/
void Task_UplinkParser(void *argument)
{
    (void)argument;
    ground_cmd_t cmd;

    while (1) {
        if (xQueueReceive(g_ground_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd != GROUND_CMD_NONE) {
                FSM_HandleGroundCommand(cmd);
            }
        }
    }
}
