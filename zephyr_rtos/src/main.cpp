#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(elyos_driver, LOG_LEVEL_INF);

// ============================================================================
// DEFINICIÓN DE PINES (Próximamente definidos en app.overlay)
// ============================================================================
// Para pruebas sin overlay configurado, asumimos que el LED es el de la placa
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

// ============================================================================
// VARIABLES COMPARTIDAS ENTRE HILOS
// ============================================================================
static volatile float g_iq_target = 0.0f;

// Cola de mensajes para emular xQueue de FreeRTOS
struct sd_log_data {
    uint32_t timestamp;
    float current_iq;
    float throttle_val;
};
K_MSGQ_DEFINE(sd_log_queue, sizeof(struct sd_log_data), 32, 4);

// ============================================================================
// VARIABLES FOC
// ============================================================================
#include "foc_math.h"

// Voltaje de bus (batería) y ángulo eléctrico (simulado o del encoder)
static volatile float g_vbus = 24.0f;
static volatile float g_elec_angle = 0.0f; 

// Controladores PI (Proporcional-Integral)
// Ajuste de ganancias (placeholder)
static elyos_foc::PIController pi_d(0.5f, 0.01f, 24.0f);
static elyos_foc::PIController pi_q(0.5f, 0.01f, 24.0f);

// ============================================================================
// HILO DEL LAZO FOC (Reemplazo del IntervalTimer a 10kHz)
// ============================================================================
void foc_timer_handler(struct k_timer *timer_id)
{
    // 1. Lectura de corrientes de fase (simulación ADC)
    elyos_foc::PhaseCurrents i_abc = {0.0f, 0.0f, 0.0f};

    // 2. Transformada de Clarke (3 fases -> Alpha/Beta)
    elyos_foc::AlphaBeta i_ab = elyos_foc::clarke(i_abc);

    // 3. Transformada de Park (Alpha/Beta -> D/Q)
    elyos_foc::DQCurrents i_dq = elyos_foc::park(i_ab, g_elec_angle);

    // 4. Controladores PI de Corriente
    // dt = 100us (0.0001 s)
    float dt = 0.0001f;
    
    // Id objetivo siempre es 0 para un BLDC de imanes superficiales
    float error_d = 0.0f - i_dq.d;
    float v_d = pi_d(error_d, dt);

    // Iq objetivo viene del acelerador (g_iq_target)
    float error_q = g_iq_target - i_dq.q;
    float v_q = pi_q(error_q, dt);

    // 5. Transformada Inversa de Park (D/Q voltajes -> Alpha/Beta voltajes)
    elyos_foc::DQVoltages v_dq_target = {v_d, v_q};
    elyos_foc::AlphaBeta v_ab_target = elyos_foc::inv_park(v_dq_target, g_elec_angle);

    // 6. SVPWM (Alpha/Beta voltajes -> Ciclos de trabajo)
    elyos_foc::PhaseVoltages duties = elyos_foc::svpwm(v_ab_target, g_vbus);

    // 7. Aquí iría la escritura de PWM a los registros del i.MX RT1062
    // pwm_set_cycles(pwm_dev, channel, period, duties.a * period, 0);
}
K_TIMER_DEFINE(foc_timer, foc_timer_handler, NULL);

// ============================================================================
// HILO THROTTLE (250 Hz)
// ============================================================================
#define THROTTLE_STACK_SIZE 1024
#define THROTTLE_PRIORITY 3

void task_throttle_entry(void *, void *, void *)
{
    LOG_INF("Iniciando tarea Throttle a 250 Hz...");
    while (1) {
        // Simulación de lectura de pedal (Aquí iría ADC con Zephyr)
        g_iq_target = 5.0f; // placeholder

        // vTaskDelayUntil equivalente en Zephyr (espera exacta de 4ms)
        k_msleep(4); 
    }
}
K_THREAD_DEFINE(throttle_tid, THROTTLE_STACK_SIZE,
                task_throttle_entry, NULL, NULL, NULL,
                THROTTLE_PRIORITY, 0, 0);

// ============================================================================
// HILO TELEMETRÍA (50 Hz)
// ============================================================================
#define TELEMETRY_STACK_SIZE 2048
#define TELEMETRY_PRIORITY 4

void task_telemetry_entry(void *, void *, void *)
{
    LOG_INF("Iniciando tarea Telemetría a 50 Hz...");
    struct sd_log_data record;
    
    while (1) {
        // Encolar datos (no bloqueante - K_NO_WAIT) equivalente a xQueueSend con timeout 0
        record.timestamp = k_uptime_get_32();
        record.current_iq = g_iq_target;
        record.throttle_val = 1.0f;
        k_msgq_put(&sd_log_queue, &record, K_NO_WAIT);

        // Parpadeo del LED
        if (led.port != NULL) {
            gpio_pin_toggle_dt(&led);
        }

        k_msleep(20);
    }
}
K_THREAD_DEFINE(telemetry_tid, TELEMETRY_STACK_SIZE,
                task_telemetry_entry, NULL, NULL, NULL,
                TELEMETRY_PRIORITY, 0, 0);

// ============================================================================
// HILO SD LOGGER (Fondo)
// ============================================================================
#define LOGGER_STACK_SIZE 2048
#define LOGGER_PRIORITY 5 // Menor prioridad (mayor número en Zephyr)

void task_logger_entry(void *, void *, void *)
{
    LOG_INF("Iniciando tarea SD Logger (esperando datos)...");
    struct sd_log_data record;
    
    while (1) {
        // Esperar bloqueante a que lleguen datos (equivalente a portMAX_DELAY)
        if (k_msgq_get(&sd_log_queue, &record, K_FOREVER) == 0) {
            // Aquí iría la escritura a la tarjeta SD
            // LOG_DBG("Log guardado: %d ms, Iq: %f", record.timestamp, record.current_iq);
        }
    }
}
K_THREAD_DEFINE(logger_tid, LOGGER_STACK_SIZE,
                task_logger_entry, NULL, NULL, NULL,
                LOGGER_PRIORITY, 0, 0);

// ============================================================================
// MAIN / SETUP
// ============================================================================
int main(void)
{
    LOG_INF("Arrancando ELYOS Motor Driver (Zephyr RTOS Port)");

    if (led.port != NULL) {
        if (!gpio_is_ready_dt(&led)) {
            LOG_ERR("LED GPIO no está listo");
            return 0;
        }
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    } else {
        LOG_WRN("LED alias (led0) no definido en DeviceTree");
    }

    // Iniciar el temporizador FOC a 10 kHz (100 us periódicos)
    k_timer_start(&foc_timer, K_USEC(100), K_USEC(100));

    // En Zephyr, no es necesario llamar a vTaskStartScheduler(), 
    // el scheduler de Zephyr ya está corriendo al entrar a main(),
    // y los hilos definidos con K_THREAD_DEFINE ya se han iniciado automáticamente.

    return 0;
}
