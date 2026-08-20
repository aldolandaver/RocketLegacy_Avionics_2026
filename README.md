# Sistema de Aviónica de Recuperación — STM32F411CEU6 (BlackPill)

Firmware de vuelo para cohetería experimental: fusión sensorial inercial/barométrica en tiempo real, detección determinística de apogeo mediante Filtro de Kalman, disparo pirotécnico aislado del sistema de recuperación y enlace de telemetría bidireccional LoRa a 915 MHz.

## 1. Arquitectura de software

El firmware corre sobre **FreeRTOS** (vía wrapper CMSIS-RTOS2) con cuatro tareas de prioridad estricta, diseñadas para garantizar que la detección de apogeo y el disparo de recuperación nunca sean bloqueados por operaciones de I/O de menor criticidad (escritura en flash, radio):

| Tarea | Prioridad | Frecuencia | Responsabilidad |
|---|---|---|---|
| `Task_SensorFusion` | **Alta (Realtime)** | 200 Hz | Lectura IMU BMI088, decimación de BMP390 a 50 Hz, Filtro de Kalman 1D, máquina de estados de misión, disparo de PA0 |
| `Task_DAQLogging` | Media | 50 Hz | Empaquetado y escritura secuencial del log de vuelo en la flash W25Q16JV |
| `Task_UplinkParser` | Media | Por evento | Consume comandos de tierra (`SYS_ABORT`, `FORCE_DEPLOY`) recibidos por interrupción en USART1 |
| `Task_TelemetryDownlink` | Baja | 1 Hz (0.2 Hz en baliza) | Empaquetado y envío de telemetría por el módulo LoRa E220 |

La sincronización entre tareas usa un único **mutex** (`g_telemetry_mutex`) que protege el snapshot de telemetría compartido, y una **cola** (`g_ground_cmd_queue`) alimentada directamente desde la ISR de USART1 para los comandos de uplink.

### Máquina de estados de misión (`fsm.c`)

```
Estado 0 (INIT) ──valida IMU/Baro/Flash──▶ Estado 1 (PAD_IDLE)
Estado 1 (PAD_IDLE) ──accel_z > 3G por ≥0.2s──▶ Estado 2 (POWERED_ASCENT)
Estado 2 (POWERED_ASCENT) ──vel. vertical cruza 0 (+→-)──▶ dispara PA0 ──▶ Estado 3 (DESCENT)
Estado 3 (DESCENT) ──Δalt≈0 y accel≈1G por ≥5s──▶ Estado 4 (RESCUE_BEACON)

Cualquier estado activo ──SYS_ABORT (uplink)──▶ dispara PA0 (si no se ha disparado) + Estado 4
PAD_IDLE / ASCENT / DESCENT ──FORCE_DEPLOY (uplink)──▶ dispara PA0 + Estado 3 (o se mantiene)
```

### Filtro de Kalman (`kalman_filter.c`)

Vector de estado `x = [altitud, velocidad_vertical, sesgo_acelerómetro]ᵀ`. Predicción a 200 Hz con la aceleración vertical del BMI088 (compensada de gravedad estática); corrección a 50 Hz con la altitud derivada de la presión del BMP390 vía modelo atmosférico ISA. El apogeo se declara quirúrgicamente en el instante en que la componente de velocidad del vector de estado cambia de signo (positivo → negativo), sin umbrales heurísticos adicionales.

## 2. Lista de hardware

| Componente | Modelo | Bus | Función |
|---|---|---|---|
| Microcontrolador | STM32F411CEU6 (BlackPill) | — | Cómputo principal, 100 MHz, FPU HW |
| IMU | Bosch BMI088 | SPI (≤10 MHz) | Aceleración/giro, 200 Hz |
| Barómetro | Bosch BMP390 (0x77) | I2C Fast-mode | Altitud (corrección KF), 50 Hz |
| Sensor ambiental | Sensirion SHT40 (0x44) | I2C Fast-mode | Temperatura/humedad (housekeeping) |
| Memoria de vuelo | Winbond W25Q16JV (2 MB) | SPI (≤10 MHz) | Bitácora de datos (DAQ log) |
| Transceptor RF | Ebyte E220-900T30D | USART1 (915 MHz, 30 dBm) | Telemetría downlink + comandos uplink |
| GNSS | Mateksys M10Q-5883 (u-blox M10) | USART2 | Posición, 1 Hz |
| Disparo de recuperación | Optoacoplador + MOSFET IRLZ44N | GPIO (PA0) | Quema de hilo de nicromo |

## 3. Pinout

| Pin | Función | Notas |
|---|---|---|
| **PA0** | **⚡ DISPARO PIROTÉCNICO (nicromo)** | **Salida digital → optoacoplador → gate IRLZ44N. Pulso HIGH de 1.5 s al detectar apogeo o comando `FORCE_DEPLOY`/`SYS_ABORT`. Verificar polaridad y aislamiento galvánico antes de cada vuelo.** |
| PA1 | CS Flash W25Q16JV | SPI1 |
| PA2/PA3 | TX/RX | USART2 (GNSS M10Q) — únicas opciones de USART2 en el BlackPill (sin Puerto D) |
| PA4 | CS Acelerómetro BMI088 | SPI1 |
| PA5/PA6/PA7 | SCK/MISO/MOSI | SPI1 (compartido IMU + Flash) |
| PA9/PA10 | TX/RX | USART1 (LoRa E220) |
| PB0 | AUX LoRa E220 | Indica módulo ocupado/transmitiendo |
| PB1 | CS Giroscopio BMI088 | SPI1 — reubicado desde PA3 para no chocar con USART2 |
| PB6/PB7 | SCL/SDA | I2C1 (BMP390 + SHT40) |
| PC13 | LED de estado | Parpadeo rápido = error de inicialización (Estado 0) |

> ⚠️ **Nota de seguridad:** PA0 controla directamente el circuito de ignición del sistema de recuperación. Todo cambio en `mission_config.h` (umbral de lanzamiento, duración de pulso) debe validarse en banco con el hilo de nicromo desconectado del paracaídas antes de cualquier prueba en campo.

## 4. Estructura del repositorio

```
Core/
├── Inc/
│   ├── main.h                  Externs de periféricos HAL (bodies generados por CubeMX)
│   ├── mission_config.h        Pinout, direcciones de bus, umbrales de misión
│   ├── fsm.h                   Máquina de estados + tipos de telemetría
│   ├── kalman_filter.h         Filtro de Kalman 1D (3 estados)
│   ├── freertos_tasks.h        Declaración de tareas y objetos RTOS compartidos
│   └── drivers/
│       ├── bmi088.h / bmp390.h / sht40.h        Sensores
│       ├── w25q16jv.h                            Flash DAQ
│       └── lora_e220.h / gnss_m10q.h            Enlaces de comunicación
└── Src/
    ├── main.c                  Bring-up de HAL/RTOS
    ├── stm32f4xx_it.c          Callback de RX UART (uplink + GNSS)
    ├── fsm.c                   Lógica de transición de estados + detectores
    ├── kalman_filter.c         Predicción/corrección del KF
    ├── freertos_tasks.c        Cuerpos de las 4 tareas
    └── drivers/                Implementación de cada driver
```

## 5. Compilación

1. Crear un proyecto STM32CubeIDE para **STM32F411CEU6** y, desde el editor de pines (.ioc), configurar: SPI1 (modo full-duplex, hasta 10 MHz), I2C1 (Fast-mode 400 kHz), USART1 y USART2 (según pinout de la sección 3), y los GPIO listados (incluyendo PA0 como salida push-pull).
2. Habilitar **FreeRTOS** (middleware CMSIS-RTOS2) desde el mismo editor; esto genera automáticamente `FreeRTOSConfig.h` y los archivos `freertos.c`/`cmsis_os2.c` base.
3. Copiar el contenido de `Core/Inc` y `Core/Src` de este repositorio dentro del proyecto generado, **sobrescribiendo** `main.c`, `main.h` y `stm32f4xx_it.c` (conservando los handlers de vector de interrupción ya generados, ver comentario al inicio de `stm32f4xx_it.c`).
4. Verificar en Project Properties → C/C++ Build que el FPU esté habilitado: `-mfpu=fpv4-sp-d16 -mfloat-abi=hard -mcpu=cortex-m4`.
5. Compilar (`Project → Build All`) y flashear vía ST-Link o el bootloader USB DFU del BlackPill.

## 6. Comandos de uplink soportados

Enviados como línea de texto terminada en `\n` por el enlace LoRa (recepción por interrupción en USART1):

- `SYS_ABORT` — Aborta la misión: dispara el pyro si no se ha disparado y fuerza el sistema a modo baliza (Estado 4) de inmediato.
- `FORCE_DEPLOY` — Disparo manual de recuperación, útil si la detección automática de apogeo no se activó.