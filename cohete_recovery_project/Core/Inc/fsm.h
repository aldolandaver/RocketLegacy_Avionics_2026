/**
 ******************************************************************************
 * @file    fsm.h
 * @brief   Máquina de estados de la placa de RECUPERACIÓN DEL COHETE.
 *          Completamente autónoma (sin uplink): solo detecta despegue,
 *          apogeo y aterrizaje, y dispara el nicromo en PA0.
 ******************************************************************************
 */
#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MISSION_STATE_INIT = 0,        /* Estado 0: validación de IMU/Baro         */
    MISSION_STATE_PAD_IDLE,        /* Estado 1: calibración en rampa            */
    MISSION_STATE_POWERED_ASCENT,  /* Estado 2: ascenso + KF + detección apogeo */
    MISSION_STATE_DESCENT,         /* Estado 3: post-disparo, espera aterrizaje */
    MISSION_STATE_RECOVERED,       /* Estado 4: aterrizado, sistema en reposo   */
    MISSION_STATE_ERROR            /* Fallo de inicialización, no recuperable  */
} mission_state_t;

/* Estado de vuelo expuesto para diagnóstico (LED, o un futuro SWO/printf de
 * depuración). No hay telemetría por radio en esta placa. */
typedef struct {
    mission_state_t state;
    float altitude_agl_m;
    float vertical_velocity_ms;
    float accel_bias_ms2;
    float raw_accel_z_g;
    float raw_pressure_pa;
    uint32_t mission_time_ms;
    uint8_t  pyro_fired;
} flight_status_t;

/* --------------------------------------------------------------------------
 * API pública. Toda la FSM vive en un único contexto (la tarea de control de
 * recuperación); no hay otra tarea ni ISR que la toque, por lo que no se
 * requieren secciones críticas ni mutex (a diferencia de la Carga Útil, que
 * sí comparte estado entre varias tareas y con la ISR de uplink).
 * ------------------------------------------------------------------------*/
typedef struct {
    float accel_z_g;
    bool  new_baro_sample;
    float baro_pressure_pa;
    float baro_temperature_c;
} fsm_sensor_input_t;

void FSM_Init(void);
mission_state_t FSM_GetState(void);
void FSM_SetErrorState(void);

/** Transición Estado0 -> Estado1 tras validar periféricos y calibrar. */
void FSM_EnterPadIdle(float baseline_pressure_pa, float accel_bias_ms2);

/** Ciclo principal de la FSM. Se invoca a IMU_SAMPLE_RATE_HZ (200 Hz). */
void FSM_RunStep(const fsm_sensor_input_t *in, float dt_s, flight_status_t *out_status);

/**
 * @brief Consume (lee y limpia) el evento de disparo de pyro. La tarea de
 *        control debe sondear esto tras cada RunStep y, si es true, pulsar
 *        PA0 durante PYRO_FIRE_PULSE_MS.
 */
bool FSM_ConsumePyroFireEvent(void);

#endif /* FSM_H */
