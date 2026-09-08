# ELYOS Motor Driver

Este repositorio contiene las distintas iteraciones del firmware para el controlador de motores ELYOS. El proyecto ha evolucionado a lo largo del tiempo, y se mantienen tres versiones separadas en directorios independientes para preservar el historial, permitir pruebas comparativas y facilitar el desarrollo.

## Estructura del Repositorio

El código está organizado en tres ramas arquitectónicas principales:

*   📂 **[`normal_arduino/`](normal_arduino/)**
    *   **Descripción**: La versión original y más básica del código.
    *   **Arquitectura**: Bucle de control simple (`setup`/`loop`) bajo el framework clásico de Arduino (Teensyduino).
    *   **Uso**: Funciona como un "Hello World" del motor. Ideal para pruebas rápidas de hardware sin la complejidad de sistemas operativos en tiempo real. Utiliza la librería `SimpleFOC`.

*   📂 **[`teensyduino_freertos/`](teensyduino_freertos/)**
    *   **Descripción**: La versión estable, testeada y funcional con capacidades multitarea.
    *   **Arquitectura**: Combina el framework de Arduino (Teensyduino) con el planificador de tareas de **FreeRTOS**.
    *   **Uso**: Implementa hilos separados para la telemetría, el procesamiento del acelerador y el bucle principal de control FOC (usando `SimpleFOC`). Es la versión de producción actual construida bajo PlatformIO.

*   📂 **[`zephyr_rtos/`](zephyr_rtos/)**
    *   **Descripción**: La versión de grado industrial hiper-optimizada (Bare-Metal).
    *   **Arquitectura**: Un port completamente nativo corriendo sobre el sistema operativo **Zephyr RTOS**. Elimina el overhead de Arduino y SimpleFOC.
    *   **Uso**: Se comunica directamente con los registros de hardware del i.MX RT1062 (Teensy 4.1) mediante la HAL de NXP (`fsl_pwm.h`). Contiene un motor matemático FOC customizado para máxima velocidad usando la FPU de hardware. 

## Compilación y Despliegue

Todos los entornos están configurados para compilarse mediante **PlatformIO**.

Para trabajar con cualquier versión:
1. Abre la carpeta deseada (ej. `zephyr_rtos`) en VS Code.
2. Asegúrate de tener instalada la extensión de **PlatformIO**.
3. Utiliza los botones de PlatformIO (`Build` / `Upload`) o la terminal: `pio run -t upload`

*Nota: Revisa el `README.md` dentro de cada carpeta para detalles técnicos específicos de cada arquitectura.*
