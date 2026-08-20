/**
 ******************************************************************************
 * @file    fsm.c
 * @brief   Lógica de estados del cohete. Misma matemática de detección que
 *          la Carga Útil (ver Sección 8.7.1 del CDR), sin lógica de radio,
 *          logging ni comandos de tierra: esta placa vuela sola.
 ******************************************************************************
 */
#include "fsm.h"
#include "kalman_filter.h"
#include "drivers/bmp390.h"
#include "mission_config.h"
#include <math.h>

static mission_state_t s_state = MISSION_STATE_INIT;

static kalman_state_t s_kf;
static float s_pad_baseline_pressure_pa = 101325.0f;

static uint32_t s_launch_hold_ms = 0u;
static float    s_prev_vertical_velocity_ms = 0.0f;

static uint32_t s_landed_hold_ms = 0u;
static float    s_prev_altitude_for_landing_m = 0.0f;

static bool s_pyro_fire_event = false;
static bool s_pyro_already_fired = false;

void FSM_Init(void)
{
    s_state = MISSION_STATE_INIT;
    s_launch_hold_ms = 0u;
    s_landed_hold_ms = 0u;
    s_pyro_fire_event = false;
    s_pyro_already_fired = false;
}

mission_state_t FSM_GetState(void)
{
    return s_state;
}

void FSM_SetErrorState(void)
{
    s_state = MISSION_STATE_ERROR;
}

void FSM_EnterPadIdle(float baseline_pressure_pa, float accel_bias_ms2)
{
    s_pad_baseline_pressure_pa = baseline_pressure_pa;
    KF_Init(&s_kf, 0.0f, accel_bias_ms2);
    s_launch_hold_ms = 0u;
    s_state = MISSION_STATE_PAD_IDLE;
}

bool FSM_ConsumePyroFireEvent(void)
{
    /* Un único contexto de ejecución toca esta bandera (ver fsm.h): no se
     * requiere sección crítica como en la Carga Útil. */
    bool evt = s_pyro_fire_event;
    s_pyro_fire_event = false;
    return evt;
}

static void request_pyro_fire(void)
{
    if (!s_pyro_already_fired) {
        s_pyro_already_fired = true;
        s_pyro_fire_event = true;
    }
}

/* Ver nota en fsm.c de la Carga Útil: simplificación que asume alineación
 * del eje Z del IMU con el eje longitudinal del cohete durante el boost. */
static float accel_g_to_linear_ms2(float accel_z_g)
{
    return (accel_z_g - 1.0f) * GRAVITY_MS2;
}

static void run_pad_idle(const fsm_sensor_input_t *in, float dt_s)
{
    if (in->accel_z_g >= LAUNCH_ACCEL_THRESHOLD_G) {
        s_launch_hold_ms += (uint32_t)(dt_s * 1000.0f);
        if (s_launch_hold_ms >= LAUNCH_ACCEL_HOLD_MS) {
            s_state = MISSION_STATE_POWERED_ASCENT;
            s_prev_vertical_velocity_ms = s_kf.x[1];
        }
    } else {
        s_launch_hold_ms = 0u;
    }
}

static void run_powered_ascent(const fsm_sensor_input_t *in, float dt_s)
{
    float accel_ms2 = accel_g_to_linear_ms2(in->accel_z_g);
    KF_Predict(&s_kf, accel_ms2, dt_s);

    if (in->new_baro_sample) {
        float baro_alt = BMP390_PressureToAltitude(in->baro_pressure_pa, s_pad_baseline_pressure_pa);
        KF_UpdateBaro(&s_kf, baro_alt);
    }

    /* Apogeo: cruce de signo positivo -> negativo de la velocidad estimada */
    if (s_prev_vertical_velocity_ms > 0.0f && s_kf.x[1] <= 0.0f) {
        request_pyro_fire();
        s_state = MISSION_STATE_DESCENT;
        s_landed_hold_ms = 0u;
        s_prev_altitude_for_landing_m = s_kf.x[0];
    }
    s_prev_vertical_velocity_ms = s_kf.x[1];
}

static void run_descent(const fsm_sensor_input_t *in, float dt_s)
{
    float accel_ms2 = accel_g_to_linear_ms2(in->accel_z_g);
    KF_Predict(&s_kf, accel_ms2, dt_s);

    if (in->new_baro_sample) {
        float baro_alt = BMP390_PressureToAltitude(in->baro_pressure_pa, s_pad_baseline_pressure_pa);
        KF_UpdateBaro(&s_kf, baro_alt);

        float delta = fabsf(baro_alt - s_prev_altitude_for_landing_m);
        bool alt_stable   = (delta <= LANDED_ALT_DELTA_TOLERANCE_M);
        bool accel_stable = (fabsf(in->accel_z_g - LANDED_ACCEL_G) <= LANDED_ACCEL_TOLERANCE_G);

        if (alt_stable && accel_stable) {
            s_landed_hold_ms += (uint32_t)(1000.0f / BARO_SAMPLE_RATE_HZ);
            if (s_landed_hold_ms >= LANDED_HOLD_MS) {
                /* Estado 3 -> Estado 4: no hay nada más que hacer, el
                 * paracaídas ya se liberó y el vehículo está en tierra. */
                s_state = MISSION_STATE_RECOVERED;
            }
        } else {
            s_landed_hold_ms = 0u;
        }
        s_prev_altitude_for_landing_m = baro_alt;
    }
    (void)dt_s;
}

void FSM_RunStep(const fsm_sensor_input_t *in, float dt_s, flight_status_t *out_status)
{
    switch (s_state) {
        case MISSION_STATE_INIT:
        case MISSION_STATE_ERROR:
        case MISSION_STATE_RECOVERED:
            /* Nada que ejecutar: INIT se resuelve fuera de la FSM (ver
             * freertos_tasks.c), ERROR es terminal, y RECOVERED es el
             * estado de reposo final tras el aterrizaje. */
            break;

        case MISSION_STATE_PAD_IDLE:
            run_pad_idle(in, dt_s);
            break;

        case MISSION_STATE_POWERED_ASCENT:
            run_powered_ascent(in, dt_s);
            break;

        case MISSION_STATE_DESCENT:
            run_descent(in, dt_s);
            break;
    }

    out_status->state = s_state;
    out_status->altitude_agl_m       = s_kf.x[0];
    out_status->vertical_velocity_ms = s_kf.x[1];
    out_status->accel_bias_ms2       = s_kf.x[2];
    out_status->raw_accel_z_g        = in->accel_z_g;
    if (in->new_baro_sample) {
        out_status->raw_pressure_pa = in->baro_pressure_pa;
    }
    out_status->pyro_fired = s_pyro_already_fired ? 1u : 0u;
}
