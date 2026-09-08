# ELYOS Motor Driver - Zephyr RTOS Port

## Descripción
Este directorio contiene la implementación "Bare-Metal" del controlador de motores ELYOS (FOC) sobre el sistema operativo en tiempo real Zephyr (RTOS) para el microcontrolador NXP i.MX RT1062 (Teensy 4.1).

El objetivo de esta migración fue reemplazar el framework de Arduino (Teensyduino) y `SimpleFOC` por un stack más determinista, eficiente y completamente nativo de Zephyr.

## Arquitectura

El sistema utiliza la API de hilos de Zephyr para dividir las cargas de trabajo cooperativamente:
- **`telemetry_task` (Hilo de Baja Prioridad)**: Envía datos de estado y depuración (voltajes, fase, acelerador) a la PC a través de USB CDC ACM a 50Hz.
- **`throttle_task` (Hilo de Baja Prioridad)**: Lee continuamente el voltaje del pedal/acelerador analógico (ADC) y mapea los valores para controlar la demanda de voltaje `v_q` en el lazo FOC.
- **`foc_loop_task` (Hilo de Máxima Prioridad, Cooperativo)**: Este es el "cerebro" matemático del motor. Es liberado por un temporizador por hardware a 20kHz (cada 50us). Calcula las transformadas de Clarke/Park Inversa y ejecuta el algoritmo Space Vector PWM (SVPWM).

## Logros Técnicos y Solución de Problemas

1. **Bug del PWM Hardware (NXP FlexPWM)**:
   - Zephyr implementa un driver PWM genérico (`pwm_mcux.c`) diseñado para emitir pulsos simples (Edge-Aligned). Sin embargo, el control FOC requiere pares de señales **Complementarias con Alineación Central (Center-Aligned)** e inserción de **Tiempos Muertos (Deadtime)** en hardware para evitar cortocircuitos en el puente H.
   - **Solución**: Se omitió la API genérica de Zephyr para los pines PWM. En su lugar, el código en `init_foc_pwm()` se comunica directamente con las bibliotecas de bajo nivel HAL de NXP (`fsl_pwm.h`).

2. **Problema del Pin Enable y Fallas de PWM (`DISMAP`)**:
   - Inicialmente, el motor no recibía voltaje (giraba libremente) a pesar de que los temporizadores y el IOMUXC estaban configurados.
   - **Causa**: El módulo FlexPWM de NXP tiene pines de "Falla" activados por hardware por defecto. Si detectan ruido o estados flotantes, un circuito de protección interno bloquea la salida de los transistores instantáneamente. El núcleo de Arduino desactivaba estas fallas automáticamente, pero Zephyr no lo hacía para los submódulos adicionales.
   - **Solución**: Se inyectó código en los registros `DISMAP` para anular el ruteo de estas fallas y se forzó en ON el registro `OUTEN` de las 6 salidas PWM, permitiendo que las compuertas PWM pasaran hacia el driver externo.

3. **Inclusión y Pinmux**:
   - Se configuraron los 6 pines físicamente utilizando las macros `IOMUXC_SetPinMux` y `IOMUXC_SetPinConfig` del SDK de NXP (e.g. `IOMUXC_GPIO_B0_10_FLEXPWM2_PWMA02`) con una configuración de `0x10B0U` (slew rate rápido, drive strength alto) en lugar de depender de la abstracción de devicetree, garantizando el enrutamiento perfecto.

4. **Matemática FOC Customizada**:
   - En lugar de compilar la enorme librería de SimpleFOC, el port implementa un namespace limpio `elyos_foc` con las funciones trigonométricas e inversores espaciales nativos (`inv_park`, `svpwm`), compilados usando el FPU de hardware (Unidad de Punto Flotante) del Cortex-M7 para máxima velocidad.

## Estado Actual

- [x] Consola Serial USB Nativa en Zephyr
- [x] Multi-threading RTOS configurado
- [x] Hardware PWM Center-Aligned operando a 20kHz
- [x] Mapeo de pines de IOMUXC resuelto y protegido contra fallos (`DISMAP`)
- [x] Matemática FOC Open-Loop (inyectando campos magnéticos giratorios directamente)

## Próximos Pasos Recomendados

- Completar la implementación del Lazo Cerrado (Closed-Loop) leyendo el encoder magnético por SPI dentro de `foc_loop_task`.
- Reconectar las lecturas de corriente ADC para implementar control de corriente PI.
