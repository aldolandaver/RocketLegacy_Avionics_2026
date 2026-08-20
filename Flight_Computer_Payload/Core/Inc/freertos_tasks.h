/**
 ******************************************************************************
 * @file    freertos_tasks.h
 * @brief   Declaración de las 3 tareas de misión + tarea de parseo de
 *          comandos de uplink, y de los objetos RTOS/handles de driver
 *          compartidos entre ellas.
 ******************************************************************************
 */
#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "fsm.h"
#include "drivers/bmi088.h"
#include "drivers/bmp390.h"
#include "drivers/sht40.h"
#include "drivers/w25q16jv.h"
#include "drivers/lora_e220.h"
#include "drivers/gnss_m10q.h"

/* Tamaños de stack en palabras de 32 bits (no bytes), convención CMSIS-RTOS2 */
#define STACK_SIZE_SENSOR_FUSION   (512u)
#define STACK_SIZE_DAQ_LOGGING     (384u)
#define STACK_SIZE_TELEMETRY       (384u)
#define STACK_SIZE_UPLINK_PARSER   (256u)

/* Profundidad de la cola de comandos de tierra: 1 comando SYS_ABORT o
 * FORCE_DEPLOY es suficiente por diseño (son comandos de estado, no
 * eventos que deban encolarse en ráfaga). */
#define GROUND_CMD_QUEUE_LEN       4u

/* --------------------------------------------------------------------------
 * Objetos RTOS compartidos (definidos en freertos_tasks.c)
 * ------------------------------------------------------------------------*/
extern osThreadId_t   g_sensor_fusion_task_handle;
extern osThreadId_t   g_daq_logging_task_handle;
extern osThreadId_t   g_telemetry_task_handle;
extern osThreadId_t   g_uplink_parser_task_handle;

extern osMutexId_t    g_telemetry_mutex;      /* Protege telemetry_snapshot_t compartido */
extern QueueHandle_t  g_ground_cmd_queue;     /* ISR UART1 -> tarea de uplink             */

extern telemetry_snapshot_t g_telemetry; /* Última foto de estado, mutex-protegida */

/* Handles de drivers, instanciados con los periféricos ya configurados por
 * CubeMX en main.c (hspi1, hi2c1, huart1, huart2, etc. son externs de HAL). */
extern bmi088_handle_t     g_imu;
extern bmp390_handle_t     g_baro;
extern sht40_handle_t      g_env;
extern w25q16jv_handle_t   g_flash;
extern lora_e220_handle_t  g_lora;
extern gnss_m10q_handle_t  g_gnss;

/**
 * @brief Crea las 4 tareas y objetos RTOS. Llamar una vez desde main() antes
 *        de osKernelStart(). No inicializa periféricos HAL (eso ya se hizo
 *        en la etapa MX_*_Init generada) ni valida sensores (eso ocurre
 *        dentro de la tarea de fusión sensorial, Estado 0 de la FSM).
 */
void FreeRTOS_Tasks_CreateAll(void);

/* Cuerpos de tarea (ver freertos_tasks.c para el detalle de cada bucle) */
void Task_SensorFusion(void *argument);
void Task_DAQLogging(void *argument);
void Task_TelemetryDownlink(void *argument);
void Task_UplinkParser(void *argument);

#endif /* FREERTOS_TASKS_H */
