# Rocket Legacy — Aviónica de Vuelo (STM32F411CEU6)

Repositorio único de firmware para las dos placas de aviónica de la misión, cada una en su propia carpeta porque son sistemas física y eléctricamente independientes (baterías, PCB y microcontrolador propios — ver Sección 7.2 y Figs. 69-70 del CDR):

```
repo/
├── payload/     Carga Útil — rastreable, comandable, con radio
└── cohete/      Cuerpo del cohete — autónomo, sin radio
```

**División de responsabilidades:** el payload viaja *dentro* del cohete. La única misión del cohete es detectar su propio apogeo y, con un solo evento de disparo, liberar su paracaídas **y** eyectar al payload. A partir de ahí, el payload cae por su cuenta y ejecuta su propia secuencia de recuperación (con su propio KF, su propio disparo de paracaídas y, a diferencia del cohete, telemetría y comandos por radio porque es la pieza que hay que rastrear y recuperar en campo). El cohete no necesita ser comandado ni rastreado individualmente, por eso no lleva radio.

---

## 1. Placa `payload/` — Carga Útil

Fusión sensorial inercial/barométrica en tiempo real, detección determinística de apogeo mediante Filtro de Kalman, disparo pirotécnico aislado, y enlace de telemetría bidireccional LoRa a 915 MHz para rastreo y comandos de tierra.

### 1.1 Arquitectura de software

FreeRTOS (vía CMSIS-RTOS2) con cuatro tareas de prioridad estricta:

| Tarea | Prioridad | Frecuencia | Responsabilidad |
|---|---|---|---|
| `Task_SensorFusion` | **Alta (Realtime)** | 200 Hz | Lectura IMU BMI088, decimación de BMP390 a 50 Hz, Filtro de Kalman 1D, máquina de estados de misión, disparo de PA0 |
| `Task_DAQLogging` | Media | 50 Hz | Empaquetado y escritura secuencial del log de vuelo en la flash W25Q16JV |
| `Task_UplinkParser` | Media | Por evento | Consume comandos de tierra recibidos por interrupción en USART1 |
| `Task_TelemetryDownlink` | Baja | 1 Hz (0.2 Hz en baliza) | Empaquetado y envío de telemetría por el módulo LoRa E220 |

Sincronización: un **mutex** (`g_telemetry_mutex`) protege el snapshot de telemetría compartido, y una **cola** (`g_ground_cmd_queue`) alimentada desde la ISR de USART1 entrega los comandos de uplink a `Task_UplinkParser`.

### 1.2 Máquina de estados de misión (`fsm.c`)

```
Estado 0 (INIT) ──valida IMU/Baro/Flash──▶ Estado 1 (PAD_IDLE)
Estado 1 (PAD_IDLE) ──accel_z > 3G por ≥0.2s Y sistema ARMADO──▶ Estado 2 (POWERED_ASCENT)
Estado 2 (POWERED_ASCENT) ──vel. vertical cruza 0 (+→-)──▶ dispara PA0 ──▶ Estado 3 (DESCENT)
Estado 3 (DESCENT) ──Δalt≈0 y accel≈1G por ≥5s──▶ Estado 4 (RESCUE_BEACON)
```

El sistema **arranca desarmado** (`armed = false`): aunque se detecte la aceleración de despegue, la transición a Estado 2 no ocurre hasta recibir `SYS_ARM` desde tierra. Esto es intencional — es la barrera de seguridad para evitar una ignición accidental en rampa.

### 1.3 Filtro de Kalman (`kalman_filter.c`)

Vector de estado `x = [altitud, velocidad_vertical, sesgo_acelerómetro]ᵀ`. Predicción a 200 Hz con la aceleración vertical del BMI088 (compensada de gravedad estática); corrección a 50 Hz con la altitud derivada de la presión del BMP390 vía modelo atmosférico ISA. El apogeo se declara en el instante en que la componente de velocidad del vector de estado cambia de signo (positivo → negativo).

### 1.4 Hardware

| Componente | Modelo | Bus | Función |
|---|---|---|---|
| Microcontrolador | STM32F411CEU6 (BlackPill) | — | Cómputo principal, 100 MHz, FPU HW |
| IMU | Bosch BMI088 | SPI (≤10 MHz) | Aceleración/giro, 200 Hz |
| Barómetro | Bosch BMP390 (0x77) | I2C Fast-mode | Altitud (corrección KF), 50 Hz |
| Sensor ambiental | Sensirion SHT40 (0x44) | I2C Fast-mode | Temperatura/humedad (housekeeping) |
| Memoria de vuelo | Winbond W25Q16JV (2 MB) | SPI (≤10 MHz) | Bitácora de datos (DAQ log) |
| Transceptor RF | Ebyte E220-900T30D | USART1 (915 MHz, 30 dBm) | Telemetría downlink + comandos uplink |
| GNSS | Mateksys M10Q-5883 (u-blox M10) | USART2 | Posición, 1 Hz |
| Disparo de recuperación | Optoacoplador + MOSFET IRLZ44N + diodo Schottky flyback | GPIO (PA0) | Quema de hilo de nicromo (paracaídas del payload) |

### 1.5 Pinout

| Pin | Función | Notas |
|---|---|---|
| **PA0** | **⚡ DISPARO PIROTÉCNICO (nicromo del payload)** | **Salida digital → optoacoplador → gate IRLZ44N. Pulso HIGH de 1.5 s al detectar apogeo (con el sistema ARMADO) o comando `FORCE_DEPLOY`. Verificar polaridad y aislamiento galvánico antes de cada vuelo.** |
| PA1 | CS Flash W25Q16JV | SPI1 |
| PA2/PA3 | TX/RX | USART2 (GNSS M10Q) — únicas opciones de USART2 en el BlackPill (sin Puerto D) |
| PA4 | CS Acelerómetro BMI088 | SPI1 |
| PA5/PA6/PA7 | SCK/MISO/MOSI | SPI1 (compartido IMU + Flash) |
| PA9/PA10 | TX/RX | USART1 (LoRa E220) |
| PB0 | AUX LoRa E220 | Indica módulo ocupado/transmitiendo |
| PB1 | CS Giroscopio BMI088 | SPI1 — reubicado desde PA3 para no chocar con USART2 |
| PB6/PB7 | SCL/SDA | I2C1 (BMP390 + SHT40) |
| PC13 | LED de estado | Parpadeo rápido = error de inicialización (Estado 0) |

> ⚠️ **Nota de seguridad:** PA0 controla directamente el circuito de ignición del sistema de recuperación del payload. Todo cambio en `mission_config.h` (umbral de lanzamiento, duración de pulso) debe validarse en banco con el hilo de nicromo desconectado del paracaídas antes de cualquier prueba en campo.

### 1.6 Comandos de uplink (Tabla 18 del CDR)

Enviados como línea de texto terminada en `\n` por el enlace LoRa (recepción por interrupción en USART1). **Estos comandos solo existen en el payload** — el cohete no tiene radio y no puede recibir nada de tierra (ver Sección 2).

| Comando | Acción | Momento permitido |
|---|---|---|
| `SYS_CALIBRATE` | Repite la calibración de sesgo IMU y recalcula el Ground Level barométrico. | Únicamente en **Pad Idle**, vehículo estático. |
| `SYS_ARM` | Desbloquea la barrera de software del sistema de recuperación: solo después de este comando el KF puede evaluar apogeo y disparar el nicromo. Antes de `SYS_ARM`, el sistema permanece en Pad Idle aunque detecte una aceleración de "despegue". | Pad Idle, tras confirmación de zona segura (*Range Clear*). |
| `SYS_ABORT` | **Bloquea permanentemente** los pines de disparo (el pyro queda inhabilitado por software durante el resto de la sesión de vuelo, no dispara) y detiene la escritura en Flash. Es un candado de seguridad, **no** un disparo forzado. | Cualquier fase previa al despegue, ante anomalías de hardware o cancelación del director de vuelo. |
| `FORCE_DEPLOY` | Ignora las condiciones del KF y fuerza el disparo inmediato del nicromo. | Solo en fase de vuelo (Ascenso/Descenso), si el operador detecta que el cohete cruzó apogeo sin despliegue autónomo. |

> **Corrección respecto a versiones previas de este README:** antes documentaba `SYS_ABORT` como un disparo forzado seguido de paso a modo baliza. Es al revés: `SYS_ABORT` **inhabilita** el disparo (candado de seguridad en tierra), no lo activa. El único comando que dispara el pyro por fuera del KF es `FORCE_DEPLOY`. El código (`fsm.c` / `freertos_tasks.c`) todavía implementa la semántica antigua e incorrecta — pendiente de corregir.

### 1.7 Estructura de la carpeta `payload/`

```
payload/
├── Core/
│   ├── Inc/
│   │   ├── main.h                  Externs de periféricos HAL (bodies generados por CubeMX)
│   │   ├── mission_config.h        Pinout, direcciones de bus, umbrales de misión
│   │   ├── fsm.h                   Máquina de estados + tipos de telemetría
│   │   ├── kalman_filter.h         Filtro de Kalman 1D (3 estados)
│   │   ├── freertos_tasks.h        Declaración de tareas y objetos RTOS compartidos
│   │   └── drivers/
│   │       ├── bmi088.h / bmp390.h / sht40.h        Sensores
│   │       ├── w25q16jv.h                            Flash DAQ
│   │       └── lora_e220.h / gnss_m10q.h            Enlaces de comunicación
│   └── Src/
│       ├── main.c                  Bring-up de HAL/RTOS
│       ├── stm32f4xx_it.c          Callback de RX UART (uplink + GNSS)
│       ├── fsm.c                   Lógica de transición de estados + detectores
│       ├── kalman_filter.c         Predicción/corrección del KF
│       ├── freertos_tasks.c        Cuerpos de las 4 tareas
│       └── drivers/                Implementación de cada driver
```

---

## 2. Placa `cohete/` — Sistema de Recuperación del Cuerpo

Sistema autónomo, sin radio ni comandos de tierra. Su única misión: detectar apogeo y disparar el mecanismo que libera su paracaídas y eyecta al payload — un solo evento mecánico, un solo pulso en PA0.

### 2.1 Arquitectura de software

Una **única tarea** de FreeRTOS a máxima prioridad ejecuta todo el ciclo (lectura de sensores, KF, FSM, disparo). No hay mutex ni colas porque no hay concurrencia que proteger: nada más comparte estado con esta tarea.

### 2.2 Máquina de estados (idéntica en umbrales a la del payload, sin radio ni logging)

```
Estado 0 (INIT) ──valida IMU/Baro──▶ Estado 1 (PAD_IDLE)
Estado 1 (PAD_IDLE) ──accel_z > 3G por ≥0.2s──▶ Estado 2 (POWERED_ASCENT)
Estado 2 (POWERED_ASCENT) ──vel. vertical cruza 0 (+→-)──▶ dispara PA0 ──▶ Estado 3 (DESCENT)
Estado 3 (DESCENT) ──Δalt≈0 y accel≈1G por ≥5s──▶ Estado 4 (RECOVERED)
```

Sin `SYS_ARM`/`SYS_ABORT`/`FORCE_DEPLOY`: no hay receptor a bordo. El "armado" es físico (el Interruptor General de la Fig. 70 del CDR conecta la batería solo cuando el equipo de campo lo decide), y la calibración de sesgo/altitud corre una única vez al energizar.

### 2.3 Hardware

| Componente | Bus | Función |
|---|---|---|
| STM32F411CEU6 (BlackPill) | — | Cómputo, 100 MHz, FPU HW |
| BMI088 | SPI (≤10 MHz) | Aceleración/giro, 200 Hz |
| BMP390 (0x77) | I2C Fast-mode | Altitud (corrección KF), 50 Hz |
| Optoacoplador + IRLZ44N + diodo Schottky flyback | GPIO (PA0) | Disparo aislado: libera paracaídas del cohete y eyecta el payload |
| LED | GPIO | Indicador "Power ON" / estado |
| Batería Tattu 1050 mAh 7.4V 75C 2S LiPo | — | Fuente dedicada, independiente de la del payload |

**No lleva:** SHT40, memoria Flash, LoRa, GNSS ni buzzer — todo eso es exclusivo del payload.

### 2.4 Pinout

| Pin | Función |
|---|---|
| **PA0** | **⚡ Disparo pirotécnico** → optoacoplador → gate IRLZ44N → libera paracaídas del cohete y eyecta payload |
| PA3 | CS Giroscopio BMI088 (SPI1) |
| PA4 | CS Acelerómetro BMI088 (SPI1) |
| PA5/PA6/PA7 | SCK/MISO/MOSI (SPI1) |
| PB6/PB7 | SCL/SDA (I2C1, BMP390) |
| PC13 | LED de estado |

Sin restricciones de USART: esta placa no usa ningún UART, así que no hay conflicto de pines con PA2/PA3 como en el payload.

### 2.5 Estructura de la carpeta `cohete/`

```
cohete/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── mission_config.h
│   │   ├── fsm.h
│   │   ├── kalman_filter.h
│   │   ├── freertos_tasks.h
│   │   └── drivers/
│   │       └── bmi088.h / bmp390.h
│   └── Src/
│       ├── main.c
│       ├── fsm.c
│       ├── kalman_filter.c
│       ├── freertos_tasks.c
│       └── drivers/
│           └── bmi088.c / bmp390.c
```

`kalman_filter.c/h` y `drivers/bmi088.c/h` + `drivers/bmp390.c/h` son código idéntico al del payload (misma matemática, mismos sensores) — se mantienen como copias independientes en cada carpeta en vez de una librería compartida, para que cada placa sea un proyecto STM32CubeIDE autocontenido.

---

## 3. Compilación (ambas placas)

1. Crear un proyecto STM32CubeIDE por placa (`payload/` y `cohete/` son proyectos separados) para **STM32F411CEU6**.
2. Desde el editor de pines (.ioc) de cada proyecto, configurar los periféricos según la sección de pinout correspondiente:
   - `payload/`: SPI1, I2C1, USART1, USART2, y los GPIO listados (PA0 como salida push-pull).
   - `cohete/`: SPI1, I2C1, y los GPIO listados (PA0 como salida push-pull). Sin USART.
3. Habilitar **FreeRTOS** (middleware CMSIS-RTOS2) desde el mismo editor en ambos proyectos.
4. Copiar el contenido de `Core/Inc` y `Core/Src` de la carpeta correspondiente dentro del proyecto generado, sobrescribiendo `main.c`/`main.h` (y `stm32f4xx_it.c` solo en el payload).
5. Verificar que el FPU esté habilitado: `-mfpu=fpv4-sp-d16 -mfloat-abi=hard -mcpu=cortex-m4`.
6. Compilar y flashear vía ST-Link.

> **Pendiente:** ni `payload/` ni `cohete/` usan todavía los bloques `/* USER CODE BEGIN X */ ... /* USER CODE END X */` que CubeMX necesita para no borrar el código al regenerar desde el `.ioc`. Hay que resolver eso antes de tocar el editor gráfico de nuevo (ver conversación previa).

## 4. Pendientes conocidos

- [ ] Corregir la semántica de `SYS_ABORT`/`SYS_ARM`/`SYS_CALIBRATE` en `fsm.c` y `freertos_tasks.c` del payload para que coincida con la Tabla 18 del CDR (documentado arriba en la Sección 1.6, todavía no implementado en código).
- [ ] Reestructurar `main.c`/`stm32f4xx_it.c` de ambos proyectos con los marcadores `USER CODE BEGIN/END` de CubeMX.
- [ ] Confirmar con el equipo si el evento de disparo del cohete es mecánicamente único (un pulso libera ambas cosas) o si en algún momento se necesitará un segundo canal de disparo.