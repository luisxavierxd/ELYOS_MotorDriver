# ELYOS_MotorDriver

Firmware FOC (Field-Oriented Control) para el driver de motor del vehículo del equipo **Silca Elyos** (Shell Eco-marathon). Corre sobre un **Teensy 4.1** usando la librería [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) y controla un motor BLDC trifásico (hub motor o Koford motor) vía un puente de 6-PWM con sensado de corriente inline.

> Fork de [Polo280/INDY_2026](https://github.com/Polo280/INDY_2026).

## Qué hace

- Control FOC de un motor BLDC con `BLDCDriver6PWM` + `InlineCurrentSense` (shunt de 5 mΩ, ganancia 20).
- Sensado por Hall (`HallSensor`) con una capa de suavizado (`SmoothingSensor`) sobre la posición/velocidad.
- **ThrottleFOC**: mapea el pedal (ADC de 12 bits) a corriente `Iq` de referencia, con:
  - deadband y filtro paso-bajo en el pedal,
  - curva de mapeo lineal/cuadrática ajustable (`THROTTLE_BLEND_LINEAR`),
  - límite de corriente dependiente de velocidad (cap de arranque + rampa hasta `iqMax`),
  - slew-rate asimétrico (sube más lento que baja) por eficiencia y seguridad,
  - detección de falla del pedal (ADC fuera de rango) con corte a cero opcional.
- **Telemetry_Manager**: protocolo binario simple por UART (SOF `0xAA`, CRC-8) para leer/escribir voltaje de bus, corriente, RPM, Iq/Id y ganancias PI en caliente.
- **BLDC_Logger**: log a SD (CSV) de timestamp, throttle crudo, VBat, corrientes de fase y RPM, con flush cada 200 muestras.
- **Commander** (SimpleFOC) sobre `Serial` para tuning manual de ganancias PID de Iq/Id y target en vivo.

## Hardware objetivo

| Parámetro | HUB_MOTOR (activo) | KOFORD_MOTOR |
|---|---|---|
| Voltaje de alimentación | 48 V | 24 V |
| Pares de polos | 15 | 15 |
| Límite de corriente | 45 A | 2 A |
| Límite de voltaje | 35 V | 24 V |

El perfil activo se selecciona con `#define HUB_MOTOR` / `KOFORD_MOTOR` en `include/FOC_Parameters.h`. **`PHASE_RESISTANCE` y `KV_RATING` están marcados como pendientes de calibrar** ("Missing adjust") — no asumas que los valores actuales corresponden a tu motor real.

Pines (Teensy 4.1, ver `include/Pinout.h`): 6 PWM de fase, 3 Hall, 3 sensado de corriente, VBUS en `A17`, throttle en `A10`.

## Requisitos

- [PlatformIO](https://platformio.org/) (VS Code o CLI).
- Placa: Teensy 4.1, framework Arduino.
- Librería: `askuric/Simple FOC@^2.4.0` (se resuelve sola vía `platformio.ini`).

## Build / flash

```bash
pio run -e teensy41           # compilar
pio run -e teensy41 -t upload # flashear
pio device monitor            # consola serie (Commander + logs)
```

## Calibración antes de operar

1. Confirmar `PHASE_RESISTANCE` y `KV_RATING` reales del motor (no dejar los placeholders).
2. Calibrar `adcMin`/`adcMax` del pedal en `ThrottleFOC::Config` (marcados `// calibrate!`).
3. Verificar `ZERO_ALIGN_VALUE` del sensor Hall para tu montaje mecánico.
4. Ajustar `CURRENT_LIMIT` / `VOLTAGE_LIMIT` según el pack de batería y el motor reales — son los límites de seguridad duros.

## Estructura

```
src/main.cpp              # loop principal: driver_Init() + runFOC()
lib/ELYOS_DRIVER/          # clase ELYOS_DRIVER (orquesta FOC, throttle, telemetría)
lib/ELYOS_DRIVER/ThrottleFOC.h   # mapeo pedal -> Iq de referencia
lib/BLDC_Logger/           # logging a SD en CSV
lib/Telemetry Handler/     # protocolo UART binario (status, gains, streaming)
include/FOC_Parameters.h   # parámetros por tipo de motor
include/Pinout.h           # asignación de pines Teensy 4.1
```

## Créditos / dependencias

- [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) (askuric) — librería base de control FOC, licencia MIT.
- Basado en el trabajo original de [Polo280/INDY_2026](https://github.com/Polo280/INDY_2026).

## Licencia

Ver `LICENSE`.
