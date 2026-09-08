# ELYOS Motor Driver - Normal Arduino (Legacy)

## Descripción
Este directorio contiene la primera iteración funcional del controlador de motores ELYOS. Fue extraída del historial temprano del repositorio para preservar una versión puramente basada en la filosofía de "bucle único" (Single-loop).

## Arquitectura
A diferencia de las arquitecturas posteriores, esta versión **NO utiliza un Sistema Operativo en Tiempo Real (RTOS)**.
- Se basa 100% en el framework estándar de Arduino (Teensyduino).
- Toda la ejecución ocurre secuencialmente dentro de las funciones estándar `setup()` y `loop()`.
- Utiliza la biblioteca open-source `SimpleFOC` para abstraer la generación de señales PWM y las matemáticas del Field Oriented Control.

## ¿Cuándo usar esta versión?
Esta versión es ideal para:
1. **Pruebas de concepto rápidas (PoC)**: Cuando se necesita diagnosticar un fallo de hardware sin preocuparse de si un RTOS está bloqueando hilos o interrupciones.
2. **Depuración de hardware**: Si el motor no gira, esta base de código es la más simple posible para verificar si los sensores Hall/Encoder o los MOSFETs del puente H están funcionando físicamente.

## Requisitos de Software
- PlatformIO (VS Code)
- Librería `SimpleFOC` (Se descarga automáticamente vía el archivo `platformio.ini`)
