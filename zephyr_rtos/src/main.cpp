// Workaround para bug de CMSIS con C++ en GCC
#define __sxtb16(x) (x)
#define __sxtab16(x, y) (x)

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <fsl_pwm.h>
#include <fsl_clock.h>
#include "foc_math.h"

LOG_MODULE_REGISTER(elyos_driver, LOG_LEVEL_INF);

// ============================================================================
// HARDWARE DEVICES
// ============================================================================
const struct device *spi_dev = NULL;
const struct device *adc_dev = NULL;
const struct device *uart_dev = NULL;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

static const struct spi_config spi_cfg = {
    .frequency = 10000000,
    .operation = SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0
};

// ============================================================================
// VARIABLES GLOBALES (FOC)
// ============================================================================
float current_throttle = 0.0f;
float motor_angle = 0.0f;
float motor_velocity = 0.0f;
float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f;

#define _2PI 6.28318530718f
#define DEADTIME_VAL 50

// ============================================================================
// RTOS PRIMITIVES
// ============================================================================
K_SEM_DEFINE(foc_timer_sem, 0, 1);
K_MSGQ_DEFINE(telemetry_queue, sizeof(float) * 3, 10, 4);

struct k_thread foc_thread_data;
struct k_thread throttle_thread_data;
struct k_thread telemetry_thread_data;
K_THREAD_STACK_DEFINE(foc_stack, 2048);
K_THREAD_STACK_DEFINE(throttle_stack, 1024);
K_THREAD_STACK_DEFINE(telemetry_stack, 1024);

// ============================================================================
// HILOS
// ============================================================================

#include <fsl_iomuxc.h>

void init_foc_pwm() {
    CLOCK_EnableClock(kCLOCK_Iomuxc);
    CLOCK_EnableClock(kCLOCK_Pwm2);
    
    // Configurar explAcitamente los pines para FlexPWM2 (Fase A, B, C)
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_06_FLEXPWM2_PWMA00, 0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_06_FLEXPWM2_PWMA00, 0x10B0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_07_FLEXPWM2_PWMB00, 0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_07_FLEXPWM2_PWMB00, 0x10B0U);
    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_10_FLEXPWM2_PWMA02, 0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_10_FLEXPWM2_PWMA02, 0x10B0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_11_FLEXPWM2_PWMB02, 0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_11_FLEXPWM2_PWMB02, 0x10B0U);
    
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_02_FLEXPWM2_PWMA03, 0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_02_FLEXPWM2_PWMA03, 0x10B0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_FLEXPWM2_PWMB03, 0U); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_FLEXPWM2_PWMB03, 0x10B0U);
    
    uint32_t pwmSourceClockInHz = CLOCK_GetFreq(kCLOCK_IpgClk);
    if (pwmSourceClockInHz == 0) pwmSourceClockInHz = 150000000; // Fallback seguro
    uint32_t pwmFreq = 20000; // 20kHz FOC
    
    pwm_config_t pwmConfig;
    PWM_GetDefaultConfig(&pwmConfig);
    pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
    pwmConfig.pairOperation = kPWM_ComplementaryPwmA;
    pwmConfig.enableDebugMode = true;
    
    PWM_Init(PWM2, kPWM_Module_0, &pwmConfig);
    PWM_Init(PWM2, kPWM_Module_2, &pwmConfig);
    PWM_Init(PWM2, kPWM_Module_3, &pwmConfig);
    
    // Deshabilitar fuentes de falla que podrian estar bloqueando las salidas
    for (int i = 0; i < FSL_FEATURE_PWM_FAULT_CH_COUNT; i++) {
        PWM2->SM[kPWM_Module_0].DISMAP[i] = 0x0000;
        PWM2->SM[kPWM_Module_2].DISMAP[i] = 0x0000;
        PWM2->SM[kPWM_Module_3].DISMAP[i] = 0x0000;
    }
    
    pwm_signal_param_t pwmSignal[2];
    
    pwmSignal[0].pwmChannel = kPWM_PwmA;
    pwmSignal[0].level = kPWM_HighTrue;
    pwmSignal[0].dutyCyclePercent = 0;
    pwmSignal[0].deadtimeValue = DEADTIME_VAL;
    pwmSignal[0].faultState = kPWM_PwmFaultState0;
    pwmSignal[0].pwmchannelenable = true;
    
    pwmSignal[1].pwmChannel = kPWM_PwmB;
    pwmSignal[1].level = kPWM_HighTrue;
    pwmSignal[1].dutyCyclePercent = 0;
    pwmSignal[1].deadtimeValue = DEADTIME_VAL;
    pwmSignal[1].faultState = kPWM_PwmFaultState0;
    pwmSignal[1].pwmchannelenable = true;
    
    PWM_SetupPwm(PWM2, kPWM_Module_0, pwmSignal, 2, kPWM_CenterAligned, pwmFreq, pwmSourceClockInHz);
    PWM_SetupPwm(PWM2, kPWM_Module_2, pwmSignal, 2, kPWM_CenterAligned, pwmFreq, pwmSourceClockInHz);
    PWM_SetupPwm(PWM2, kPWM_Module_3, pwmSignal, 2, kPWM_CenterAligned, pwmFreq, pwmSourceClockInHz);
    
    PWM_SetPwmLdok(PWM2, kPWM_Control_Module_0 | kPWM_Control_Module_2 | kPWM_Control_Module_3, true);
    
    // Forzar el encendido de las salidas fisicas A y B (High/Low) para los tres modulos
    PWM2->OUTEN |= ((1U << kPWM_Module_0) << PWM_OUTEN_PWMA_EN_SHIFT);
    PWM2->OUTEN |= ((1U << kPWM_Module_0) << PWM_OUTEN_PWMB_EN_SHIFT);
    PWM2->OUTEN |= ((1U << kPWM_Module_2) << PWM_OUTEN_PWMA_EN_SHIFT);
    PWM2->OUTEN |= ((1U << kPWM_Module_2) << PWM_OUTEN_PWMB_EN_SHIFT);
    PWM2->OUTEN |= ((1U << kPWM_Module_3) << PWM_OUTEN_PWMA_EN_SHIFT);
    PWM2->OUTEN |= ((1U << kPWM_Module_3) << PWM_OUTEN_PWMB_EN_SHIFT);
    
    PWM_StartTimer(PWM2, kPWM_Control_Module_0 | kPWM_Control_Module_2 | kPWM_Control_Module_3);
}

void set_pwm_duty(uint8_t duty_a_pct, uint8_t duty_b_pct, uint8_t duty_c_pct) {
    PWM_UpdatePwmDutycycle(PWM2, kPWM_Module_0, kPWM_PwmA, kPWM_CenterAligned, duty_a_pct);
    PWM_UpdatePwmDutycycle(PWM2, kPWM_Module_2, kPWM_PwmA, kPWM_CenterAligned, duty_b_pct);
    PWM_UpdatePwmDutycycle(PWM2, kPWM_Module_3, kPWM_PwmA, kPWM_CenterAligned, duty_c_pct);
    PWM_SetPwmLdok(PWM2, kPWM_Control_Module_0 | kPWM_Control_Module_2 | kPWM_Control_Module_3, true);
}

void foc_loop_task(void *a, void *b, void *c) {
    float v_bus = 12.0f;
    
    while (1) {
        k_sem_take(&foc_timer_sem, K_FOREVER);
        
        motor_angle += 0.001f; // 10 rad/s open loop
        if (motor_angle > _2PI) motor_angle -= _2PI;
        
        // Open loop voltage control para pruebas
        float v_d = 0.0f;
        float v_q = current_throttle * v_bus; 
        
        elyos_foc::DQVoltages dq = {v_d, v_q};
        elyos_foc::AlphaBeta ab = elyos_foc::inv_park(dq, motor_angle);
        elyos_foc::PhaseVoltages phases = elyos_foc::svpwm(ab, v_bus);
        
        Ua = phases.a;
        Ub = phases.b;
        Uc = phases.c;
        
        // Convertir fraccion de SVPWM (0.0 - 1.0) a porcentaje (0-100)
        uint8_t duty_a = (uint8_t)(phases.a * 100.0f);
        uint8_t duty_b = (uint8_t)(phases.b * 100.0f);
        uint8_t duty_c = (uint8_t)(phases.c * 100.0f);
        
        // Limitar duty
        if(duty_a > 100) duty_a = 100;
        if(duty_b > 100) duty_b = 100;
        if(duty_c > 100) duty_c = 100;
        
        set_pwm_duty(duty_a, duty_b, duty_c);
    }
}

void throttle_task(void *a, void *b, void *c) {
    while (1) {
        k_msleep(4); // 250 Hz
        if (adc_dev) {
            current_throttle = 0.4f; // 40% throttle fijo simulado
        }
    }
}

void telemetry_task(void *a, void *b, void *c) {
    while (1) {
        k_msleep(20); // 50 Hz
        
        float data[3] = {motor_angle, current_throttle, Ua};
        k_msgq_put(&telemetry_queue, &data, K_NO_WAIT);
        
        if (uart_dev) {
            uint8_t ch = 'T';
            uart_poll_out(uart_dev, ch);
        }
        
        if (led.port != NULL) {
            gpio_pin_toggle_dt(&led);
        }
    }
}

// ISR Timer Simulado
void simulated_timer_isr(struct k_timer *dummy) {
    k_sem_give(&foc_timer_sem);
}
K_TIMER_DEFINE(foc_timer, simulated_timer_isr, NULL);

// ============================================================================
// MAIN / SETUP
// ============================================================================
int main(void) {
    // El puerto USB ya se inicializa solo
    k_msleep(1500); 

    LOG_INF("================================================");
    LOG_INF("Arrancando ELYOS Motor Driver (Zephyr RTOS Port)");
    LOG_INF("================================================");

    if (led.port != NULL) {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    }

    spi_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(spi_mag));
    adc_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(adc_motor));
    uart_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(uart_telem));

    if (!spi_dev) LOG_WRN("SPI no configurado (standalone)");
    
    // Inicializar FOC PWM Hardware a bajo nivel (HAL)
    LOG_INF("Inicializando hardware puente H trifásico...");
    init_foc_pwm();

    LOG_INF("Iniciando hilos del RTOS...");
    
    k_thread_create(&foc_thread_data, foc_stack, K_THREAD_STACK_SIZEOF(foc_stack),
                    foc_loop_task, NULL, NULL, NULL,
                    K_PRIO_COOP(1), 0, K_NO_WAIT);

    k_thread_create(&throttle_thread_data, throttle_stack, K_THREAD_STACK_SIZEOF(throttle_stack),
                    throttle_task, NULL, NULL, NULL,
                    K_PRIO_COOP(5), 0, K_NO_WAIT);

    k_thread_create(&telemetry_thread_data, telemetry_stack, K_THREAD_STACK_SIZEOF(telemetry_stack),
                    telemetry_task, NULL, NULL, NULL,
                    K_PRIO_COOP(7), 0, K_NO_WAIT);

    // Arrancar timer a 10kHz (100us)
    k_timer_start(&foc_timer, K_USEC(100), K_USEC(100));

    LOG_INF("Sistema listo y operando a 10kHz.");

    return 0;
}
