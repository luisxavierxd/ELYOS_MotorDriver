#pragma once 
#include <cstdint>

/**
 * @file Pinout.h
 * @brief Mapeo físico de pines y periféricos para Teensy 4.1 (NXP i.MX RT1062 @ 600 MHz).
 * 
 * Configuración de hardware para inversor BLDC trifásico con modulación 6PWM complementaria,
 * sensado de corriente inline por shunt, sensores de efecto Hall y lecturas analógicas.
 */

// ============================================================================
// ETAPA DE POTENCIA: SALIDAS 6PWM (FLEXPWM COMPLEMENTARIO CON DEAD-TIME HARDWARE)
// ============================================================================
// Fase A: Módulo FLEXPWM2, Submódulo 0 (Canales A y B)
constexpr int A_PHASE_HIGH_PIN  {4};   ///< Fase A High-Side (FLEXPWM2_MOD0_A)
constexpr int A_PHASE_LOW_PIN   {33};  ///< Fase A Low-Side  (FLEXPWM2_MOD0_B)

// Fase B: Módulo FLEXPWM2, Submódulo 2 (Canales A y B)
constexpr int B_PHASE_HIGH_PIN  {6};   ///< Fase B High-Side (FLEXPWM2_MOD2_A)
constexpr int B_PHASE_LOW_PIN   {9};   ///< Fase B Low-Side  (FLEXPWM2_MOD2_B)

// Fase C: Módulo FLEXPWM2, Submódulo 3 (Canales A y B)
constexpr int C_PHASE_HIGH_PIN  {36};  ///< Fase C High-Side (FLEXPWM2_MOD3_A)
constexpr int C_PHASE_LOW_PIN   {37};  ///< Fase C Low-Side  (FLEXPWM2_MOD3_B)

// ============================================================================
// SENSORES DE POSICIÓN / EFECTO HALL (INTERRUPCIONES GPIO EXTI)
// ============================================================================
constexpr int HALL_A_PIN {10};  ///< Sensor Hall Fase A (Pin 10 / GPIO_B0_00)
constexpr int HALL_B_PIN {12};  ///< Sensor Hall Fase B (Pin 12 / GPIO_B0_01)
constexpr int HALL_C_PIN {21};  ///< Sensor Hall Fase C (Pin 21 / GPIO_AD_B1_11)

// ============================================================================
// SENSADO DE CORRIENTE INLINE (SHUNTS 5 mΩ + AMPLIFICADORES INA/OPAMP GAIN 20)
// ============================================================================
constexpr int CURRENT_SENSE_A_PIN {26};  ///< Muestreo corriente Fase A (Canal ADC1 / Pin 26)
constexpr int CURRENT_SENSE_B_PIN {27};  ///< Muestreo corriente Fase B (Canal ADC1 / Pin 27)
constexpr int CURRENT_SENSE_C_PIN {38};  ///< Muestreo corriente Fase C (Canal ADC1 / A14)

// ============================================================================
// ENTRADAS ANALÓGICAS AUXILIARES
// ============================================================================
/// Monitoreo de voltaje de batería (Divisor resistivo VBUS hacia canal analógico A17 / Pin 41)
constexpr int VBUS_SENSE_PIN {41};  

/// Entrada del pedal acelerador (Sensor Hall analógico de 0.8V a 4.2V en A10 / Pin 24)
constexpr int THROTTLE_PIN {24};

// ============================================================================
// INDICADORES Y DIAGNÓSTICO
// ============================================================================
/// LED de estado integrado en placa Teensy 4.1 (señal de pulso/heartbeat a 1 Hz)
constexpr int LED_STATUS_PIN {13};