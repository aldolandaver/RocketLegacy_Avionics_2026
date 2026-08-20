/**
 ******************************************************************************
 * @file    fsm.h
 * @brief   Máquina de estados finitos de la misión (5 estados, transición
 *          unidireccional salvo abort explícito por tierra).
 ******************************************************************************
 */
#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MISSION_STATE_INIT = 0,        /* Estado 0: validación de periféricos     */
    MISSION_STATE_PAD_IDLE,        /* Estado 1: calibración en rampa           */
    MISSION_STATE_POWERED_ASCENT,  /* Estado 2: ascenso + KF + detección apogeo*/
    MISSION_STATE_DESCENT,         /* Estado 3: recuperación / registro        */
    MISSION_STATE_RESCUE_BEACON,   /* Estado 4: baliza de baja frecuencia       */
    MISSION_STATE_ERROR            /* Fallo de inicialización, no recuperable  */
} mission_state_t;

/* Snapshot de telemetría compartido entre tareas. Protegido por mutex
 * (g_telemetry_mutex, ver freertos_tasks.c) porque lo escribe la tarea de
 * fusión sensorial y lo leen las tareas de logging y downlink. */
typedef struct {
    mission_state_t state;

    float altitude_agl_m;      /* Salida del KF                     */
    float vertical_velocity_ms;/* Salida del KF                     */
    float accel_bias_ms2;      /* Sesgo estimado del acelerómetro   */

    float raw_accel_z_g;
    float raw_pressure_pa;
    float raw_temperature_c;
    float humidity_pct;

    float gnss_lat_deg;
    float gnss_lon_deg;
    float gnss_alt_msl_m;
    bool  gnss_fix_valid;

    uint32_t mission_time_ms;
    uint8_t  pyro_fired;
} telemetry_snapshot_t;

/* Flags de comando de tierra recibidos por uplink LoRa (ISR -> tarea) */
typedef enum {
    GROUND_CMD_NONE = 0,
    GROUND_CMD_SYS_ABORT,
    GROUND_CMD_FORCE_DEPLOY
} ground_cmd_t;

/* Entradas crudas de sensores que consume la FSM en cada ciclo. Se separa
 * deliberadamente de telemetry_snapshot_t (que es la SALIDA de estado
 * compartido) para no mezclar semántica de entrada/salida. */
typedef struct {
    float accel_z_g;           /* Eje vertical del IMU, en g                */
    bool  new_baro_sample;     /* true solo en los ciclos de 50 Hz con dato */
    float baro_pressure_pa;
    float baro_temperature_c;
    bool  new_gnss_fix;
    float gnss_lat_deg;
    float gnss_lon_deg;
    float gnss_alt_msl_m;
    bool  gnss_fix_valid;
} fsm_sensor_input_t;

/* --------------------------------------------------------------------------
 * API pública. FSM_RunStep y FSM_EnterPadIdle son llamadas exclusivamente
 * desde la tarea de fusión sensorial (alta prioridad, único escritor de
 * estado). FSM_HandleGroundCommand puede invocarse desde la tarea de
 * uplink (prioridad distinta); internamente usa una sección crítica corta
 * para no correr con FSM_RunStep.
 * ------------------------------------------------------------------------*/
void FSM_Init(void);
mission_state_t FSM_GetState(void);
void FSM_SetErrorState(void);

/** Transición Estado0 -> Estado1 tras validar periféricos y calibrar. */
void FSM_EnterPadIdle(float baseline_pressure_pa, float accel_bias_ms2);

/** Ciclo principal de la FSM. Se invoca a IMU_SAMPLE_RATE_HZ (200 Hz). */
void FSM_RunStep(const fsm_sensor_input_t *in, float dt_s, telemetry_snapshot_t *out_snap);

/** Comando de tierra recibido por uplink (ver stm32f4xx_it.c / lora_e220). */
void FSM_HandleGroundCommand(ground_cmd_t cmd);

/**
 * @brief Consume (lee y limpia atómicamente) el evento de disparo de pyro.
 *        La tarea de fusión sensorial debe sondear esto tras cada RunStep
 *        y, si es true, pulsar PA0 durante PYRO_FIRE_PULSE_MS.
 */
bool FSM_ConsumePyroFireEvent(void);

#endif /* FSM_H */
