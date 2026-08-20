/**
 ******************************************************************************
 * @file    freertos_tasks.h
 * @brief   Esta placa solo tiene UN trabajo (detectar apogeo y disparar el
 *          nicromo), por lo que basta con una sola tarea de máxima
 *          prioridad. Se conserva FreeRTOS (en vez de un super-loop a mano)
 *          por el temporizado determinista de osDelayUntil y porque
 *          mantiene la misma toolchain/depuración que la Carga Útil.
 ******************************************************************************
 */
#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include "cmsis_os2.h"
#include "fsm.h"
#include "drivers/bmi088.h"
#include "drivers/bmp390.h"

#define STACK_SIZE_RECOVERY_CONTROL   (512u) /* palabras de 32 bits */

extern osThreadId_t g_recovery_task_handle;

extern bmi088_handle_t  g_imu;
extern bmp390_handle_t  g_baro;

/** Crea la tarea única. Llamar una vez desde main() antes de osKernelStart(). */
void FreeRTOS_Tasks_CreateAll(void);

void Task_RecoveryControl(void *argument);

#endif /* FREERTOS_TASKS_H */
