/**
 ******************************************************************************
 * @file    kalman_filter.h
 * @brief   Filtro de Kalman lineal 1D, vector de estado x = [alt, vel, bias]^T.
 *          Predicción a 200 Hz con IMU (BMI088), corrección a 50 Hz con
 *          altitud barométrica (BMP390 vía modelo ISA).
 ******************************************************************************
 */
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

typedef struct {
    /* Estado: alt [m], vel [m/s], bias del acelerómetro [m/s^2] */
    float x[3];

    /* Covarianza de estado 3x3, almacenada como arreglo plano fila-mayor */
    float P[3][3];

    /* Ruido de proceso (diagonal) y de medición (escalar, solo mide altitud) */
    float q_alt;
    float q_vel;
    float q_bias;
    float r_baro;
} kalman_state_t;

/**
 * @brief Inicializa el filtro con altitud/velocidad/sesgo iniciales y
 *        covarianza de partida. Llamar una vez al pasar a PAD_IDLE tras
 *        promediar la presión base.
 */
void KF_Init(kalman_state_t *kf, float initial_alt_m, float initial_bias_ms2);

/**
 * @brief Paso de predicción, se ejecuta a IMU_SAMPLE_RATE_HZ (200 Hz).
 * @param accel_z_ms2 Aceleración vertical medida, YA compensada de gravedad
 *                     estática en el eje de referencia del cohete.
 * @param dt_s         Paso de integración (nominal 1/200 s).
 */
void KF_Predict(kalman_state_t *kf, float accel_z_ms2, float dt_s);

/**
 * @brief Paso de corrección, se ejecuta a BARO_SAMPLE_RATE_HZ (50 Hz) cuando
 *        hay una lectura barométrica nueva disponible.
 * @param baro_alt_m Altitud derivada del modelo ISA a partir de presión BMP390.
 */
void KF_UpdateBaro(kalman_state_t *kf, float baro_alt_m);

#endif /* KALMAN_FILTER_H */
