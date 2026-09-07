# ELYOS_MotorDriver

Firmware FOC (Field-Oriented Control) para el controlador de motor BLDC del vehículo del equipo **Silca Elyos** (Shell Eco-marathon). Diseñado para exprimir al máximo el microcontrolador **NXP i.MX RT1062 (ARM Cortex-M7 @ 600 MHz)** en la placa **Teensy 4.1** mediante una arquitectura de tiempo real preemptiva con **FreeRTOS** y **cero dependencias de Arduino en el código de aplicación**.

> Fork de [Polo280/INDY_2026](https://github.com/Polo280/INDY_2026).

---

## Arquitectura de Tiempo Real (FreeRTOS + Hardware ISR)

Para evitar que las operaciones lentas de I/O (escritura en tarjeta SD que tarda de 5 a 20 ms, o la comunicación serie) congelen el lazo FOC y dañen la etapa de potencia (48 V / 45 A), el sistema desacopla el control en tareas independientes:

```
[ Hardware PIT Timer @ 10 kHz (NVIC Interrupt) ]
                         │
                         ▼
┌─────────────────────────────────────────────────────────┐
│ Lazo FOC (ISR de Tiempo Real Crítico - 100 µs)          │
│ • loopFOC() + move(g_iq_target)                         │
│ • Determinismo absoluto (<4 µs de cómputo en Cortex-M7) │
│ • Jitter: 0 µs                                          │
└────────────────────────┬────────────────────────────────┘
                         │ Lee objetivo de corriente (atómico)
                         ▼
┌─────────────────────────────────────────────────────────┐
│ Tarea Throttle & Control (Prioridad 3 - 250 Hz / 4 ms)  │
│ • Lectura del pedal vía ADC (12 bits)                   │
│ • Zona muerta, filtro paso bajo y slew-rate asimétrico  │
│ • Actualiza g_iq_target suavemente                      │
└────────────────────────┬────────────────────────────────┘
                         │ Encola snapshot
                         ▼
┌─────────────────────────────────────────────────────────┐
│ Tarea Telemetría y Consola (Prioridad 2 - 50 Hz / 20 ms)│
│ • VBUS, corriente de bus estimada e Idc                 │
│ • Protocolo binario UART hacia companion ESP            │
│ • Consola Commander (sintonización PID en vivo)         │
│ • Parpadeo de LED de estado (1 Hz)                      │
└────────────────────────┬────────────────────────────────┘
                         │ xQueueSend(&record, 0)
                         ▼
┌─────────────────────────────────────────────────────────┐
│ Tarea SD Logger (Prioridad 1 - Fondo / Event-driven)    │
│ • Consume de la cola FreeRTOS (g_sd_queue)              │
│ • Escribe filas CSV en la tarjeta SD                    │
│ • Sus bloqueos de 15 ms NO afectan al motor             │
└─────────────────────────────────────────────────────────┘
```

---

## Cero Arduino en Aplicación (Capa HAL `elyos_hal`)

El código de aplicación (`main.cpp`, `ThrottleFOC.h`, `Telemetry_Manager.h`, `BLDC_Logger.h`, `Pinout.h`) no incluye `<Arduino.h>`. En su lugar, utiliza tipos estándar de C++ y una capa de abstracción de hardware tipada:

- `elyos::Time`: `millis()`, `micros()`, `delayMs()`.
- `elyos::Gpio`: `setMode()`, `write()`, `read()`, `toggle()`.
- `elyos::Adc`: `init()`, `setResolution()`, `read()`.
- `elyos::Uart`: Interfaz de puerto serie tipada para el protocolo de telemetría.

---

## Estructura del Proyecto

```
ELYOS_MotorDriver/
├── include/
│   ├── elyos_hal.h             # Abstracción de hardware (Time, Gpio, Adc, Uart)
│   ├── FOC_Parameters.h        # Parámetros del motor (HUB_MOTOR / KOFORD_MOTOR)
│   └── Pinout.h                # Asignación numérica de pines Teensy 4.1
├── lib/
│   ├── BLDC_Logger/            # Registro CSV en SD con desacoplamiento de colas
│   ├── ELYOS_DRIVER/           # Orquestador FOC, ThrottleFOC y SmoothingSensor
│   └── Telemetry Handler/      # Protocolo binario UART (SOF 0xAA, CRC-8)
├── src/
│   ├── elyos_hal.cpp           # Implementación del HAL para Teensy 4.1
│   └── main.cpp                # Punto de entrada, tareas FreeRTOS y scheduler
└── platformio.ini              # Configuración PlatformIO (target: teensy41)
```

---

## Asignación de Pines (Teensy 4.1)

| Función | Pin | Descripción |
|---|---|---|
| **Fase A (High / Low)** | 4 / 33 | FlexPWM sub-módulo de potencia |
| **Fase B (High / Low)** | 6 / 9 | FlexPWM sub-módulo de potencia |
| **Fase C (High / Low)** | 36 / 37 | FlexPWM sub-módulo de potencia |
| **Sensores Hall** | 10, 12, 21 | Entradas de interrupción Hall A, B, C |
| **Sensado de Corriente**| 26, 27, 38 | Shunt inline fases A, B, C (5 mΩ, ganancia 20) |
| **Voltaje de Batería** | 41 (A17) | Divisor resistivo VBUS |
| **Acelerador (Pedal)** | 24 (A10) | Entrada analógica ADC (12 bits) |
| **LED de Estado** | 13 | LED integrado (heartbeat a 1 Hz) |
| **Telemetría UART** | Serial1 (0/1) | Conexión con ESP companion @ 115200 bps |
| **Commander / USB** | Serial (USB) | Consola de sintonización SimpleFOC @ 115200 bps |

---

## Detalle Técnico: Corrección del NVIC Cortex-M7 para FreeRTOS

En el microcontrolador NXP i.MX RT1062, el gestor de arranque (*bootloader* HalfKay) y el core de Teensyduino copian la tabla de vectores a la memoria RAM dinámica (`_VectorsRam`) y apuntan el registro `SCB_VTOR` a dicha dirección. Durante el arranque de Teensyduino, todas las entradas de `_VectorsRam` se inicializan con la dirección de la función `unused_interrupt_vector` (ubicada en memoria Flash).

Los puertos estándar de FreeRTOS para ARM Cortex-M7 asumen que `_VectorsRam[0]` contiene la dirección del puntero inicial de pila (`MSP` / `_estack`). Al ejecutar `prvPortStartFirstTask()`, FreeRTOS intentaba cargar el MSP leyendo `_VectorsRam[0]`. Debido a que esta posición apuntaba a Flash de solo lectura, la posterior llamada `svc 0` disparaba un fallo de hardware crítico (**HardFault**), dejando al microcontrolador en un bucle infinito de reinicios.

Para solucionar esto de manera definitiva y permitir la ejecución estable de FreeRTOS sin modificar el core de Teensyduino, `src/main.cpp` enlaza explícitamente el puntero inicial del stack antes de activar el planificador:

```cpp
extern "C" {
    extern unsigned long _estack;
    extern void (* volatile _VectorsRam[])(void);
}

// Restaura el puntero de pila inicial del sistema en la posición 0 de los vectores
_VectorsRam[0] = (void(*)())&_estack;

// Inicia el scheduler de FreeRTOS con MSP válido
vTaskStartScheduler();
```

---

## Consola de Comandos (SimpleFOC Commander)

A través del puerto serie USB (`COM9` a 115200 baudios), es posible interactuar en tiempo real con el inversor:

| Comando | Acción | Ejemplo |
|---|---|---|
| `?` | Listar todos los comandos disponibles | `?` |
| `a<val>` | Consultar o ajustar ganancia P de corriente Id | `a1.2` |
| `b<val>` | Consultar o ajustar ganancia I de corriente Id | `b5.0` |
| `c<val>` | Consultar o ajustar ganancia P de corriente Iq | `c1.6` |
| `d<val>` | Consultar o ajustar ganancia I de corriente Iq | `d7.0` |
| `t<val>` | Consultar o enviar meta manual de corriente Iq | `t2.5` |

---

## Calibración antes de operar

1. **Perfil del motor**: Seleccionar `#define HUB_MOTOR` o `KOFORD_MOTOR` en [`include/FOC_Parameters.h`](include/FOC_Parameters.h).
2. **Calibrar Pedal**: Ajustar `adcMin` y `adcMax` en `ThrottleFOC::Config` (en [`include/FOC_Parameters.h`](include/FOC_Parameters.h)).
3. **Ángulo Eléctrico Cero**: Ajustar `ZERO_ALIGN_VALUE` según la alineación mecánica del sensor Hall.
4. **Límites de Seguridad**: Confirmar `VOLTAGE_LIMIT` y `CURRENT_LIMIT` antes de energizar la etapa de alta tensión (48 V).
