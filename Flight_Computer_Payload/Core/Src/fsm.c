/**
 ******************************************************************************
 * @file    fsm.c
 * @brief   Lógica de la máquina de estados de misión. Contiene la única
 *          instancia del filtro de Kalman del vuelo y los detectores de
 *          eventos (despegue, apogeo, aterrizaje) descritos en la síntesis
 *          de diseño. Un solo escritor de estado (tarea de fusión sensorial);
 *          los comandos de tierra se sincronizan con sección crítica.
 ******************************************************************************
 */
#include "fsm.h"
#include "kalman_filter.h"
#include "drivers/bmp390.h"
#include "mission_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

static volatile mission_state_t s_state = MISSION_STATE_INIT;

static kalman_state_t s_kf;
static float s_pad_baseline_pressure_pa = 101325.0f; /* valor ISA por defecto hasta calibrar */

/* Detector de despegue (Estado 1 -> 2) */
static uint32_t s_launch_hold_ms = 0u;

/* Detector de apogeo (dentro de Estado 2): flanco de bajada de velocidad */
static float s_prev_vertical_velocity_ms = 0.0f;
static bool  s_ascent_kf_initialized = false;

/* Detector de aterrizaje (Estado 3 -> 4) */
static uint32_t s_landed_hold_ms = 0u;
static float    s_prev_altitude_for_landing_m = 0.0f;

/* Evento de disparo pirotécnico, consumido una única vez por la tarea */
static volatile bool s_pyro_fire_event = false;
static volatile bool s_pyro_already_fired = false;

/* Barrera de seguridad de software (Tabla 18 del CDR):
 *   - s_armed:       arranca en false. Solo SYS_ARM lo pone en true; solo
 *                     entonces PAD_IDLE puede transicionar a ASCENT.
 *   - s_pyro_locked:  arranca en false. SYS_ABORT lo pone en true de forma
 *                     PERMANENTE para la sesión de vuelo actual (no hay
 *                     comando que lo revierta): ni el apogeo automático ni
 *                     FORCE_DEPLOY pueden disparar el pyro mientras esté
 *                     activo. Solo un reinicio físico de la placa lo limpia.
 */
static volatile bool s_armed = false;
static volatile bool s_pyro_locked = false;
static volatile bool s_recalibrate_requested = false;

void FSM_Init(void)
{
    s_state = MISSION_STATE_INIT;
    s_launch_hold_ms = 0u;
    s_landed_hold_ms = 0u;
    s_pyro_fire_event = false;
    s_pyro_already_fired = false;
    s_ascent_kf_initialized = false;
    s_armed = false;
    s_pyro_locked = false;
    s_recalibrate_requested = false;
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
    bool evt;
    taskENTER_CRITICAL();
    evt = s_pyro_fire_event;
    s_pyro_fire_event = false;
    taskEXIT_CRITICAL();
    return evt;
}

static void request_pyro_fire(void)
{
    /* SYS_ABORT tiene prioridad absoluta: si el pyro está bloqueado, ni el
     * apogeo automático ni FORCE_DEPLOY pueden encenderlo. */
    if (s_pyro_locked) {
        return;
    }
    if (!s_pyro_already_fired) {
        s_pyro_already_fired = true;
        s_pyro_fire_event = true;
    }
}

void FSM_HandleGroundCommand(ground_cmd_t cmd)
{
    taskENTER_CRITICAL();
    switch (cmd) {
        case GROUND_CMD_SYS_CALIBRATE:
            /* Solo tiene sentido con el vehículo estático en rampa. Se
             * limita a levantar una bandera; el recálculo real de
             * baseline/sesgo ocurre dentro de run_pad_idle() usando la
             * MISMA muestra que ya llegó ese ciclo por parámetro (in),
             * para no tener que leer los sensores desde este contexto y
             * competir por el bus SPI/I2C con la tarea de fusión sensorial. */
            if (s_state == MISSION_STATE_PAD_IDLE) {
                s_recalibrate_requested = true;
            }
            break;

        case GROUND_CMD_SYS_ARM:
            /* Desbloquea la barrera de software: sin este comando,
             * run_pad_idle() nunca deja pasar a POWERED_ASCENT aunque
             * detecte la aceleración de despegue. */
            if (s_state == MISSION_STATE_PAD_IDLE) {
                s_armed = true;
            }
            break;

        case GROUND_CMD_SYS_ABORT:
            /* Candado de seguridad en tierra: BLOQUEA el disparo, no lo
             * activa. Es permanente para esta sesión de vuelo (no hay
             * comando de "des-abort"; requiere reiniciar la placa). */
            s_pyro_locked = true;
            s_armed = false;
            break;

        case GROUND_CMD_FORCE_DEPLOY:
            /* Único comando que fuerza el disparo, ignorando el criterio
             * del KF. Válido en PAD_IDLE, ASCENT o DESCENT, y sigue sujeto
             * al bloqueo de SYS_ABORT (ver request_pyro_fire). */
            if (s_state == MISSION_STATE_PAD_IDLE ||
                s_state == MISSION_STATE_POWERED_ASCENT ||
                s_state == MISSION_STATE_DESCENT) {
                request_pyro_fire();
                if (s_state != MISSION_STATE_DESCENT) {
                    s_state = MISSION_STATE_DESCENT;
                    s_landed_hold_ms = 0u;
                }
            }
            break;

        default:
            break;
    }
    taskEXIT_CRITICAL();
}

/* Convierte lectura cruda del acelerómetro (g, eje vertical) a aceleración
 * lineal en el marco inercial, restando la componente estática de gravedad.
 * Simplificación de diseño: asume alineación del eje Z del IMU con el eje
 * longitudinal del cohete durante el boost (ángulo de ataque bajo, válido
 * para cohetes estabilizados por aletas). Una versión con AHRS completo
 * usaría el giroscopio para proyectar el vector gravedad en cada instante. */
static float accel_g_to_linear_ms2(float accel_z_g)
{
    return (accel_z_g - 1.0f) * GRAVITY_MS2;
}

static void run_pad_idle(const fsm_sensor_input_t *in, float dt_s)
{
    /* SYS_CALIBRATE: recalibración instantánea con la muestra de este
     * ciclo (no repite el promediado de ~2 s de la calibración inicial de
     * Estado 0->1; se asume que el equipo de campo ya confirmó visualmente
     * que el vehículo está estático antes de mandar el comando). */
    if (s_recalibrate_requested) {
        if (in->new_baro_sample) {
            float new_bias_ms2 = (in->accel_z_g - 1.0f) * GRAVITY_MS2;
            KF_Init(&s_kf, 0.0f, new_bias_ms2);
            s_pad_baseline_pressure_pa = in->baro_pressure_pa;
            s_recalibrate_requested = false;
        }
        /* Si aún no hay muestra barométrica nueva este ciclo, se reintenta
         * en el siguiente (50 Hz) — no bloquea el resto de la FSM. */
    }

    if (!s_armed) {
        /* Sin SYS_ARM, ni siquiera se acumula el temporizador de despegue:
         * el sistema se queda en PAD_IDLE indefinidamente por diseño. */
        s_launch_hold_ms = 0u;
        return;
    }

    if (in->accel_z_g >= LAUNCH_ACCEL_THRESHOLD_G) {
        s_launch_hold_ms += (uint32_t)(dt_s * 1000.0f);
        if (s_launch_hold_ms >= LAUNCH_ACCEL_HOLD_MS) {
            /* Estado 1 -> Estado 2: despegue confirmado y sistema armado */
            s_state = MISSION_STATE_POWERED_ASCENT;
            s_prev_vertical_velocity_ms = s_kf.x[1];
            s_ascent_kf_initialized = true;
        }
    } else {
        s_launch_hold_ms = 0u; /* la condición debe ser continua, no acumulativa */
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

    /* Apogeo: la velocidad vertical estimada cruza de positivo a negativo.
     * Se usa el signo estricto (no un umbral) tal como especifica el
     * documento de diseño: "cruza por cero hacia valores negativos". */
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
    /* El KF se mantiene activo para disponer de altitud/velocidad
     * estimadas también durante el descenso (registro + criterio de
     * aterrizaje), aunque el disparo de pyro ya ocurrió al entrar aquí. */
    float accel_ms2 = accel_g_to_linear_ms2(in->accel_z_g);
    KF_Predict(&s_kf, accel_ms2, dt_s);

    if (in->new_baro_sample) {
        float baro_alt = BMP390_PressureToAltitude(in->baro_pressure_pa, s_pad_baseline_pressure_pa);
        KF_UpdateBaro(&s_kf, baro_alt);

        float delta = fabsf(baro_alt - s_prev_altitude_for_landing_m);
        bool alt_stable  = (delta <= LANDED_ALT_DELTA_TOLERANCE_M);
        bool accel_stable = (fabsf(in->accel_z_g - LANDED_ACCEL_G) <= LANDED_ACCEL_TOLERANCE_G);

        if (alt_stable && accel_stable) {
            /* dt aproximado del muestreo barométrico (50 Hz nominal) */
            s_landed_hold_ms += (uint32_t)((1000.0f / BARO_SAMPLE_RATE_HZ));
            if (s_landed_hold_ms >= LANDED_HOLD_MS) {
                /* Estado 3 -> Estado 4 */
                s_state = MISSION_STATE_RESCUE_BEACON;
            }
        } else {
            s_landed_hold_ms = 0u;
        }
        s_prev_altitude_for_landing_m = baro_alt;
    }
    (void)dt_s;
}

void FSM_RunStep(const fsm_sensor_input_t *in, float dt_s, telemetry_snapshot_t *out_snap)
{
    switch (s_state) {
        case MISSION_STATE_INIT:
        case MISSION_STATE_ERROR:
            /* No hay lógica de vuelo activa; el bucle de inicialización en
             * main()/freertos_tasks.c es responsable de sacar de este
             * estado mediante FSM_EnterPadIdle() o quedar en error. */
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

        case MISSION_STATE_RESCUE_BEACON:
            /* Sin lógica de estimación adicional: la tarea de telemetría
             * conmuta a cadencia de baliza (0.2 Hz) leyendo este estado. */
            break;
    }

    /* Publicación del snapshot de telemetría, independiente del estado */
    out_snap->state = s_state;
    out_snap->altitude_agl_m       = s_kf.x[0];
    out_snap->vertical_velocity_ms = s_kf.x[1];
    out_snap->accel_bias_ms2       = s_kf.x[2];
    out_snap->raw_accel_z_g        = in->accel_z_g;
    if (in->new_baro_sample) {
        out_snap->raw_pressure_pa    = in->baro_pressure_pa;
        out_snap->raw_temperature_c  = in->baro_temperature_c;
    }
    if (in->new_gnss_fix) {
        out_snap->gnss_lat_deg     = in->gnss_lat_deg;
        out_snap->gnss_lon_deg     = in->gnss_lon_deg;
        out_snap->gnss_alt_msl_m   = in->gnss_alt_msl_m;
        out_snap->gnss_fix_valid   = in->gnss_fix_valid;
    }
    out_snap->pyro_fired   = s_pyro_already_fired ? 1u : 0u;
    out_snap->armed        = s_armed ? 1u : 0u;
    out_snap->pyro_locked  = s_pyro_locked ? 1u : 0u;
}