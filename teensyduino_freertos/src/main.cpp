/**
 * @file main.cpp
 * @brief Firmware ELYOS Motor Driver para Teensy 4.1 con arquitectura FreeRTOS (0 Arduino en aplicación)
 * 
 * Arquitectura de tiempo real para NXP i.MX RT1062 (ARM Cortex-M7 @ 600 MHz):
 * - Bucle FOC: Ejecución determinista por hardware timer (IntervalTimer @ 10 kHz, 100 us)
 * - Tarea Throttle: Lectura y filtrado de pedal a 250 Hz (slew-rate, deadband)
 * - Tarea Telemetría: Protocolo binario UART hacia companion ESP y Commander a 50 Hz
 * - Tarea SD Logger: Escritura no bloqueante a SD vía cola FreeRTOS (xQueue)
 * - Corrección de vector NVIC (_VectorsRam[0]) para arranque estable del scheduler en Cortex-M7
 */

#include <FreeRTOS_TEENSY4.h>
#include <IntervalTimer.h>
#include "Pinout.h"
#include "FOC_Parameters.h"
#include "elyos_hal.h"
#include "ELYOS_DRIVER.h"
#include "BLDC_Logger.h"

// ============================================================================
// REPARACIÓN DE ARQUITECTURA TEENSY 4.1 / CORTEX-M7 PARA FREERTOS
// ============================================================================
extern "C" {
    extern unsigned long _estack;
    extern void (* volatile _VectorsRam[])(void);

    // Gancho de tarea inactiva (Idle Hook) de FreeRTOS para mantener el bus USB vivo
    void loop() {
        elyos::Time::yield();
    }
}

// ============================================================================
// CONFIGURACIÓN DE FOC Y COLA
// ============================================================================
#define FOC_TIMER_PERIOD_US     100   // 100 us = 10 kHz

static ELYOS_DRIVER driver;
static BLDC_Logger logger;
static IntervalTimer foc_timer;

// Objetivo de corriente Iq compartido entre la tarea de pedal y el lazo FOC
static volatile float g_iq_target = 0.0f;

// Cola FreeRTOS para pasar registros a la tarea de tarjeta SD
static QueueHandle_t g_sd_queue = nullptr;
constexpr size_t SD_QUEUE_LENGTH = 32;

// ============================================================================
// INTERRUPCIÓN FOC (TEMPORIZADOR POR HARDWARE A 10 kHz)
// ============================================================================
static void foc_timer_isr() {
    driver.stepFOC(g_iq_target);
}

// ============================================================================
// TAREA THROTTLE (PEDAL Y RAMPAS DE CORRIENTE @ 250 Hz)
// ============================================================================
static void task_throttle(void *arg) {
    (void)arg;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 250 Hz (cada 4 ms)

    for (;;) {
        float new_iq = driver.updateThrottle();
        g_iq_target = new_iq;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================================================
// TAREA TELEMETRÍA Y COMUNICACIONES (@ 50 Hz)
// ============================================================================
static void task_telemetry(void *arg) {
    (void)arg;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz (cada 20 ms)

    uint32_t last_blink = elyos::Time::millis();

    for (;;) {
        // 1. Métricas (VBUS, corriente estimada, RPM) y UART con companion ESP
        driver.processTelemetry();

        // 2. Procesamiento de consola serie (Commander y SimpleFOC monitor)
        driver.processCommander();

        // 3. Encolar snapshot para registro en tarjeta SD (no bloqueante, timeout 0)
        if (g_sd_queue != nullptr && logger.isReady()) {
            BLDC_Logger_Data record;
            driver.populateLoggerData(record);
            xQueueSend(g_sd_queue, &record, 0);
        }

        // 4. Parpadeo de LED de estado (heartbeat a 1 Hz en pin 13)
        if (elyos::Time::millis() - last_blink >= 500) {
            elyos::Gpio::toggle(LED_STATUS_PIN);
            last_blink = elyos::Time::millis();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================================================
// TAREA SD LOGGER (ESCRITURA CSV EN SEGUNDO PLANO)
// ============================================================================
static void task_logger(void *arg) {
    (void)arg;
    BLDC_Logger_Data record;

    for (;;) {
        // Espera bloqueada hasta que haya un nuevo dato disponible en la cola
        if (xQueueReceive(g_sd_queue, &record, portMAX_DELAY) == pdTRUE) {
            logger.logMotorDataSD(record);
        }
    }
}

// ============================================================================
// MAIN / SETUP
// ============================================================================
int main() {
    // 1. Configuración del LED de estado via HAL
    elyos::Gpio::setMode(LED_STATUS_PIN, elyos::GpioMode::Output);
    elyos::Gpio::write(LED_STATUS_PIN, elyos::GpioLevel::High);

    // 2. Inicialización del hardware del inversor FOC
    driver.driver_Init();

    // 3. Inicialización del registrador en tarjeta SD
    logger.init();

    // 4. Creación de cola FreeRTOS para SD
    g_sd_queue = xQueueCreate(SD_QUEUE_LENGTH, sizeof(BLDC_Logger_Data));

    // 5. Creación de tareas FreeRTOS
    // Prioridad 3: Throttle (Controlador de pedal y rampas a 250 Hz)
    xTaskCreate(task_throttle, "throttle", 512, nullptr, 3, nullptr);

    // Prioridad 2: Telemetría y Commander (50 Hz)
    xTaskCreate(task_telemetry, "telemetry", 1024, nullptr, 2, nullptr);

    // Prioridad 1: SD Logger (Fondo / segundo plano)
    xTaskCreate(task_logger, "logger", 1024, nullptr, 1, nullptr);

    // 6. Temporizador por hardware a 10 kHz con prioridad alta
    foc_timer.priority(32);
    foc_timer.begin(foc_timer_isr, FOC_TIMER_PERIOD_US);

    // 7. REPARACIÓN CRÍTICA TEENSY 4.1 NVIC (CORTEX-M7):
    // Asigna el puntero inicial del stack (MSP) en la posición 0 de la tabla de vectores en RAM.
    // Esto evita que FreeRTOS prvPortStartFirstTask cargue una dirección Flash y dispare un HardFault.
    _VectorsRam[0] = (void(*)())&_estack;

    // 8. Iniciar planificador FreeRTOS
    vTaskStartScheduler();

    // En caso de que se quede sin memoria RAM
    for (;;) {
        elyos::Gpio::toggle(LED_STATUS_PIN);
        elyos::Time::delayMs(100);
    }

    return 0;
}
