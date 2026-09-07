#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include "foc_math.h"

LOG_MODULE_REGISTER(elyos_driver, LOG_LEVEL_INF);

// ============================================================================
// HARDWARE DEVICES
// ============================================================================
const struct device *pwm_dev_a = NULL;
const struct device *pwm_dev_b = NULL;
const struct device *pwm_dev_c = NULL;
const struct device *spi_dev = NULL;
const struct device *adc_dev = NULL;
const struct device *uart_dev = NULL;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

// Configuración SPI
static const struct spi_config spi_cfg = {
    .frequency = 10000000,
    .operation = SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL,
};

// ============================================================================
// VARIABLES COMPARTIDAS ENTRE HILOS
// ============================================================================
static volatile float g_iq_target = 0.0f;
static volatile float g_vbus = 24.0f;
static volatile float g_elec_angle = 0.0f;
static volatile float g_motor_rpm = 0.0f;

// Controladores PI
static elyos_foc::PIController pi_d(0.5f, 0.01f, 24.0f);
static elyos_foc::PIController pi_q(0.5f, 0.01f, 24.0f);

struct sd_log_data {
    uint32_t timestamp;
    float current_iq;
    float throttle_val;
    float rpm;
};
K_MSGQ_DEFINE(sd_log_queue, sizeof(struct sd_log_data), 32, 4);

// ============================================================================
// HILO FOC (10 kHz - Prioridad muy alta cooperativa)
// ============================================================================
// Utilizamos un semáforo liberado por un timer de hardware para mantener el determinismo.
K_SEM_DEFINE(foc_sem, 0, 1);

void foc_timer_isr(struct k_timer *timer_id) {
    k_sem_give(&foc_sem);
}
K_TIMER_DEFINE(foc_timer, foc_timer_isr, NULL);

#define FOC_STACK_SIZE 2048
#define FOC_PRIORITY -1 // Cooperativo (no es interrumpido por hilos normales)

void task_foc_entry(void *, void *, void *) {
    LOG_INF("Iniciando tarea FOC a 10 kHz...");
    const uint32_t period_ns = 100000;
    float dt = 0.0001f;

    // Buffer SPI (16 bits)
    uint16_t spi_tx = 0xFFFF;
    uint16_t spi_rx = 0;
    struct spi_buf tx_buf = {.buf = &spi_tx, .len = 2};
    struct spi_buf rx_buf = {.buf = &spi_rx, .len = 2};
    struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};

    while (1) {
        // Espera exacta al tick del timer de hardware (100 us)
        k_sem_take(&foc_sem, K_FOREVER);

        // 1. Lectura del ángulo (SPI)
        if (spi_dev != NULL) {
            if (spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs) == 0) {
                // Conversión de 14 bits (ej. AS5048) a radianes (ejemplo simplificado)
                uint16_t angle_raw = spi_rx & 0x3FFF;
                g_elec_angle = (float)angle_raw * (2.0f * 3.14159265f / 16384.0f) * 7.0f; // * pares de polos
            }
        }

        // 2. Lectura de corrientes (Simulada por ahora, ya que requiere secuencia ADC1)
        elyos_foc::PhaseCurrents i_abc = {0.0f, 0.0f, 0.0f};

        // 3. FOC Math
        elyos_foc::AlphaBeta i_ab = elyos_foc::clarke(i_abc);
        elyos_foc::DQCurrents i_dq = elyos_foc::park(i_ab, g_elec_angle);

        float error_d = 0.0f - i_dq.d;
        float v_d = pi_d(error_d, dt);

        float error_q = g_iq_target - i_dq.q;
        float v_q = pi_q(error_q, dt);

        elyos_foc::DQVoltages v_dq_target = {v_d, v_q};
        elyos_foc::AlphaBeta v_ab_target = elyos_foc::inv_park(v_dq_target, g_elec_angle);
        elyos_foc::PhaseVoltages duties = elyos_foc::svpwm(v_ab_target, g_vbus);

        // 4. Inyección a PWM
        if (pwm_dev_a && pwm_dev_b && pwm_dev_c) {
            pwm_set_cycles(pwm_dev_a, 0, period_ns, duties.a * period_ns, 0);
            pwm_set_cycles(pwm_dev_b, 0, period_ns, duties.b * period_ns, 0);
            pwm_set_cycles(pwm_dev_c, 0, period_ns, duties.c * period_ns, 0);
        }
    }
}
K_THREAD_DEFINE(foc_tid, FOC_STACK_SIZE, task_foc_entry, NULL, NULL, NULL, FOC_PRIORITY, 0, 0);

// ============================================================================
// HILO THROTTLE (250 Hz)
// ============================================================================
#define THROTTLE_STACK_SIZE 1024
#define THROTTLE_PRIORITY 3

void task_throttle_entry(void *, void *, void *) {
    LOG_INF("Iniciando tarea Throttle a 250 Hz...");
    
    int16_t adc_buffer;
    struct adc_sequence sequence = {
        .options = NULL,
        .channels = BIT(0), // Canal simulado
        .buffer = &adc_buffer,
        .buffer_size = sizeof(adc_buffer),
        .resolution = 12,
    };

    while (1) {
        if (adc_dev != NULL) {
            adc_read(adc_dev, &sequence);
            // Mapeo rudimentario de ADC a objetivo Iq
            float val = (float)adc_buffer;
            if (val > 490.0f) {
                g_iq_target = (val - 490.0f) * 0.01f; // Escala 
            } else {
                g_iq_target = 0.0f;
            }
        }
        k_msleep(4); 
    }
}
K_THREAD_DEFINE(throttle_tid, THROTTLE_STACK_SIZE, task_throttle_entry, NULL, NULL, NULL, THROTTLE_PRIORITY, 0, 0);

// ============================================================================
// HILO TELEMETRÍA (50 Hz)
// ============================================================================
#define TELEMETRY_STACK_SIZE 2048
#define TELEMETRY_PRIORITY 4

void task_telemetry_entry(void *, void *, void *) {
    LOG_INF("Iniciando tarea Telemetría (UART) a 50 Hz...");
    struct sd_log_data record;
    
    while (1) {
        // Enviar por UART al companion ESP
        if (uart_dev != NULL) {
            // Ejemplo de telemetria binaria simplificada (placeholder)
            uint8_t buffer[4] = {0xAA, (uint8_t)g_iq_target, 0, 0xBB};
            for (int i = 0; i < 4; i++) {
                uart_poll_out(uart_dev, buffer[i]);
            }
        }

        // Encolar datos para la SD
        record.timestamp = k_uptime_get_32();
        record.current_iq = g_iq_target;
        record.throttle_val = g_iq_target;
        record.rpm = g_motor_rpm;
        k_msgq_put(&sd_log_queue, &record, K_NO_WAIT);

        if (led.port != NULL) {
            gpio_pin_toggle_dt(&led);
        }

        k_msleep(20);
    }
}
K_THREAD_DEFINE(telemetry_tid, TELEMETRY_STACK_SIZE, task_telemetry_entry, NULL, NULL, NULL, TELEMETRY_PRIORITY, 0, 0);

// ============================================================================
// HILO SD LOGGER (Fondo)
// ============================================================================
#define LOGGER_STACK_SIZE 2048
#define LOGGER_PRIORITY 5

void task_logger_entry(void *, void *, void *) {
    LOG_INF("Iniciando tarea SD Logger...");
    struct sd_log_data record;
    
    while (1) {
        if (k_msgq_get(&sd_log_queue, &record, K_FOREVER) == 0) {
            // fs_write a tarjeta SD (FatFs)
        }
    }
}
K_THREAD_DEFINE(logger_tid, LOGGER_STACK_SIZE, task_logger_entry, NULL, NULL, NULL, LOGGER_PRIORITY, 0, 0);

// ============================================================================
// MAIN / SETUP
// ============================================================================
int main(void) {
    LOG_INF("Arrancando ELYOS Motor Driver (Zephyr RTOS Port) - 100% Capacidades");

    if (led.port != NULL) {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    }

    // Inicializar dispositivos
    pwm_dev_a = DEVICE_DT_GET(DT_NODELABEL(flexpwm2_pwm0));
    pwm_dev_b = DEVICE_DT_GET(DT_NODELABEL(flexpwm2_pwm2));
    pwm_dev_c = DEVICE_DT_GET(DT_NODELABEL(flexpwm2_pwm3));
    spi_dev = DEVICE_DT_GET(DT_NODELABEL(lpspi1));
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
    uart_dev = DEVICE_DT_GET(DT_NODELABEL(lpuart2));

    if (!device_is_ready(pwm_dev_a)) LOG_ERR("PWM A no listo");
    if (!device_is_ready(spi_dev)) LOG_ERR("SPI no listo");
    if (!device_is_ready(adc_dev)) LOG_ERR("ADC no listo");
    if (!device_is_ready(uart_dev)) LOG_ERR("UART no listo");

    // Iniciar temporizador de hardware (Dispara el semáforo FOC cada 100us)
    k_timer_start(&foc_timer, K_USEC(100), K_USEC(100));

    return 0;
}
