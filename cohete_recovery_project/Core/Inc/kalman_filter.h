/**
 ******************************************************************************
 * @file    kalman_filter.h
 * @brief   Filtro de Kalman lineal 1D, vector de estado x = [alt, vel, bias]^T.
 *          Idéntico al de la Carga Útil (misma matemática, Sección 8.7.1 CDR).
 ******************************************************************************
 */
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

typedef struct {
    float x[3];        /* alt [m], vel [m/s], bias [m/s^2] */
    float P[3][3];      /* Covarianza de estado 3x3          */
    float q_alt;
    float q_vel;
    float q_bias;
    float r_baro;
} kalman_state_t;

void KF_Init(kalman_state_t *kf, float initial_alt_m, float initial_bias_ms2);
void KF_Predict(kalman_state_t *kf, float accel_z_ms2, float dt_s);
void KF_UpdateBaro(kalman_state_t *kf, float baro_alt_m);

#endif /* KALMAN_FILTER_H */
