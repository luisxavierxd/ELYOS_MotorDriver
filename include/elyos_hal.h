#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @file elyos_hal.h
 * @brief Capa de Abstracción de Hardware (HAL) para el firmware ELYOS Motor Driver.
 * 
 * Desacopla por completo la capa de aplicación (FOC, Throttle, Telemetría, Logger)
 * de las APIs de Arduino, exponiendo interfaces fuertemente tipadas en C++ para:
 * - Temporización de alta resolución (milisegundos, microsegundos).
 * - Control de pines digitales (GPIO rápido con atajos de silicio).
 * - Convertidor Analógico a Digital (ADC de 12 bits con secciones críticas atómicas).
 * - Comunicación UART para telemetría binaria hacia el microcontrolador companion.
 */

namespace elyos {

/**
 * @brief Modos de configuración para pines GPIO.
 */
enum class GpioMode : uint8_t {
    Input,          ///< Entrada de alta impedancia
    Output,         ///< Salida digital en contrafase (push-pull)
    InputPullup,    ///< Entrada con resistencia pull-up interna
    InputPulldown,  ///< Entrada con resistencia pull-down interna
};

/**
 * @brief Niveles lógicos de voltaje digital en pines GPIO.
 */
enum class GpioLevel : uint8_t {
    Low = 0,        ///< Nivel lógico bajo (0 V / GND)
    High = 1,       ///< Nivel lógico alto (3.3 V)
};

/**
 * @brief Utilidades de tiempo del sistema de alta resolución.
 */
class Time {
public:
    /// Retorna los milisegundos transcurridos desde el arranque
    static uint32_t millis();

    /// Retorna los microsegundos transcurridos desde el arranque
    static uint32_t micros();

    /// Pausa bloqueante en milisegundos
    static void delayMs(uint32_t ms);

    /// Atiende eventos de fondo y buffers de transmisión USB Serial
    static void yield();
};

/**
 * @brief Operaciones digitales sobre pines GPIO (optimizadas para Cortex-M7).
 */
class Gpio {
public:
    /// Configura el modo de dirección de un pin (Input, Output, Pullup)
    static void setMode(int pin, GpioMode mode);

    /// Escribe un nivel lógico en un pin (utiliza registros directos digitalWriteFast)
    static void write(int pin, GpioLevel level);

    /// Lee el nivel lógico de un pin (digitalReadFast)
    static GpioLevel read(int pin);

    /// Invierte el estado actual de un pin de salida digital
    static void toggle(int pin);
};

/**
 * @brief Controlador del Convertidor Analógico a Digital (ADC).
 * 
 * Protegido con secciones críticas atómicas (cpsid i / msr primask) para evitar
 * que el muestreo de corriente en la interrupción FOC colisione con lecturas
 * de pedal o voltaje de batería en el mismo periférico analógico (ADC1).
 */
class Adc {
public:
    /// Inicializa la resolución por defecto (12 bits)
    static void init();

    /// Configura la resolución global de muestreo (10 o 12 bits)
    static void setResolution(int bits = 12);

    /// Realiza una conversión atómica sobre el canal especificado
    static uint16_t read(int pin, int bits = 12);
};

/**
 * @brief Interfaz abstracta para comunicación serie asíncrona (UART).
 */
class Uart {
public:
    virtual ~Uart() = default;

    /// Inicializa el puerto con la velocidad en baudios indicada
    virtual void begin(uint32_t baud_rate) = 0;

    /// Retorna el número de bytes disponibles para lectura en el buffer FIFO
    virtual int available() = 0;

    /// Lee un byte del buffer de recepción (-1 si no hay datos)
    virtual int readByte() = 0;

    /// Envía un arreglo de bytes por la línea de transmisión TX
    virtual size_t write(const uint8_t *data, size_t len) = 0;

    /// Envía un byte individual por la línea de transmisión TX
    virtual size_t writeByte(uint8_t byte) = 0;
};

/**
 * @brief Obtiene la instancia del puerto UART dedicado a telemetría (Serial1).
 * @return Puntero a la interfaz Uart implementada por el HAL
 */
Uart* getTelemetryUart();

}  // namespace elyos

