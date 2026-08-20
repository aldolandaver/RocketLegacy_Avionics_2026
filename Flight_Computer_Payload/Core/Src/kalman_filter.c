/**
 ******************************************************************************
 * @file    kalman_filter.c
 * @brief   Implementación del KF lineal 1D de 3 estados. Se evita el uso de
 *          malloc/librerías de álgebra pesadas: todo es aritmética 3x3 fija,
 *          apta para ejecutarse dentro de una ISR-callable task a 200 Hz con
 *          FPU hardware (single-precision).
 ******************************************************************************
 */
#include "kalman_filter.h"
#include <string.h>

/* Modelo:
 *   a_true(k) = a_medido(k) - bias(k)
 *   alt(k)  = alt(k-1) + vel(k-1)*dt + 0.5*a_true*dt^2
 *   vel(k)  = vel(k-1) + a_true*dt
 *   bias(k) = bias(k-1)                      (paseo aleatorio, ruido q_bias)
 *
 * Jacobiano de transición F (linealizado, aquí ya es lineal exacto):
 *   F = [ 1   dt   -0.5*dt^2 ]
 *       [ 0   1    -dt       ]
 *       [ 0   0     1        ]
 *
 * Medición (BMP390 -> altitud ISA):  z = H x + r,  H = [1 0 0]
 */

void KF_Init(kalman_state_t *kf, float initial_alt_m, float initial_bias_ms2)
{
    memset(kf, 0, sizeof(kalman_state_t));

    kf->x[0] = initial_alt_m;   /* altitud relativa AGL, 0 en rampa */
    kf->x[1] = 0.0f;            /* velocidad vertical inicial       */
    kf->x[2] = initial_bias_ms2;/* sesgo del acelerómetro en reposo */

    /* Covarianza inicial: confianza moderada en altitud/velocidad,
     * mayor incertidumbre en el sesgo hasta converger. */
    kf->P[0][0] = 1.0f;   kf->P[1][1] = 1.0f;   kf->P[2][2] = 4.0f;

    /* Ruido de proceso: la velocidad y el sesgo son los términos más
     * sensibles a ajuste fino en banco de pruebas (vibración del motor). */
    kf->q_alt  = 0.001f;
    kf->q_vel  = 0.05f;
    kf->q_bias = 0.0005f;

    /* Ruido de medición barométrica: derivado del ruido RMS del BMP390
     * (~0.03 hPa en modo alta resolución) propagado al modelo ISA local. */
    kf->r_baro = 0.6f;
}

void KF_Predict(kalman_state_t *kf, float accel_z_ms2, float dt_s)
{
    const float dt  = dt_s;
    const float dt2 = dt * dt;

    const float alt  = kf->x[0];
    const float vel   = kf->x[1];
    const float bias  = kf->x[2];

    const float a_true = accel_z_ms2 - bias;

    /* ---- Propagación de estado ---- */
    kf->x[0] = alt + vel * dt + 0.5f * a_true * dt2;
    kf->x[1] = vel + a_true * dt;
    kf->x[2] = bias; /* sesgo modelado como constante entre pasos */

    /* ---- Propagación de covarianza: P = F P F^T + Q ----
     * Se expande manualmente (F es dispersa) en lugar de usar producto
     * matricial genérico, para ahorrar ciclos dentro de la tarea de 200 Hz. */
    const float f02 = -0.5f * dt2;
    const float f12 = -dt;

    float P00 = kf->P[0][0], P01 = kf->P[0][1], P02 = kf->P[0][2];
    float P10 = kf->P[1][0], P11 = kf->P[1][1], P12 = kf->P[1][2];
    float P20 = kf->P[2][0], P21 = kf->P[2][1], P22 = kf->P[2][2];

    /* FP = F * P */
    float FP00 = P00 + dt * P10 + f02 * P20;
    float FP01 = P01 + dt * P11 + f02 * P21;
    float FP02 = P02 + dt * P12 + f02 * P22;

    float FP10 = P10 + f12 * P20;
    float FP11 = P11 + f12 * P21;
    float FP12 = P12 + f12 * P22;

    float FP20 = P20;
    float FP21 = P21;
    float FP22 = P22;

    /* (FP) * F^T */
    kf->P[0][0] = FP00 + FP01 * dt + FP02 * f02 + kf->q_alt;
    kf->P[0][1] = FP01 + FP02 * f12;
    kf->P[0][2] = FP02;

    kf->P[1][0] = FP10 + FP11 * dt + FP12 * f02;
    kf->P[1][1] = FP11 + FP12 * f12 + kf->q_vel;
    kf->P[1][2] = FP12;

    kf->P[2][0] = FP20 + FP21 * dt + FP22 * f02;
    kf->P[2][1] = FP21 + FP22 * f12;
    kf->P[2][2] = FP22 + kf->q_bias;
}

void KF_UpdateBaro(kalman_state_t *kf, float baro_alt_m)
{
    /* Innovación: y = z - H x, con H = [1 0 0] -> solo compara altitud */
    const float y = baro_alt_m - kf->x[0];

    /* Covarianza de innovación: S = H P H^T + R = P00 + R */
    const float S = kf->P[0][0] + kf->r_baro;
    if (S <= 1e-6f) {
        return; /* evita división por cero ante covarianza degenerada */
    }

    /* Ganancia de Kalman: K = P H^T / S -> primera columna de P sobre S */
    const float K0 = kf->P[0][0] / S;
    const float K1 = kf->P[1][0] / S;
    const float K2 = kf->P[2][0] / S;

    /* Corrección de estado */
    kf->x[0] += K0 * y;
    kf->x[1] += K1 * y;
    kf->x[2] += K2 * y;

    /* Corrección de covarianza: P = (I - K H) P */
    const float P00 = kf->P[0][0], P01 = kf->P[0][1], P02 = kf->P[0][2];
    const float P10 = kf->P[1][0], P11 = kf->P[1][1], P12 = kf->P[1][2];
    const float P20 = kf->P[2][0], P21 = kf->P[2][1], P22 = kf->P[2][2];

    kf->P[0][0] = P00 - K0 * P00;
    kf->P[0][1] = P01 - K0 * P01;
    kf->P[0][2] = P02 - K0 * P02;

    kf->P[1][0] = P10 - K1 * P00;
    kf->P[1][1] = P11 - K1 * P01;
    kf->P[1][2] = P12 - K1 * P02;

    kf->P[2][0] = P20 - K2 * P00;
    kf->P[2][1] = P21 - K2 * P01;
    kf->P[2][2] = P22 - K2 * P02;
}
