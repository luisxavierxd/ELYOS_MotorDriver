#pragma once

// Define the type of motor in the car 
#define HUB_MOTOR
// #define KOFORD_MOTOR

#ifdef HUB_MOTOR
    constexpr float SUPPLY_VOLTAGE {48.0f};
    constexpr int POLE_PAIRS {15};
    constexpr float ZERO_ALIGN_VALUE {5.38f};
    // Missing adjust 
    constexpr float PHASE_RESISTANCE {0.1f};
    constexpr float KV_RATING        {50.0f};
    // Safety values
    constexpr float VOLTAGE_LIMIT {35.0f};
    constexpr float CURRENT_LIMIT {45.0f};
    // Controller Iq gains
    constexpr float IQ_KP {1.6f};
    constexpr float IQ_KI {7.0f};  // 10
    constexpr float IQ_KD {0.0f};
    // Controller Id gains
    constexpr float ID_KP {0.8f};
    constexpr float ID_KI {5.0f};
    constexpr float ID_KD {0.0f};
    // Filter time constants (TF)
    constexpr float IQ_TF {0.06f};
    constexpr float ID_TF {0.06f};

    // Throttle FOC parameters
    constexpr float THROTTLE_FILTER_TAU {0.06f};    // Secs
    constexpr float THROTTLE_DEADBAND {0.05f};      // Pedal input below this is ignored
    constexpr float FAULT_LOW_MARGIN {20.0f};       // ADC counts below adcMin tolerated
    constexpr float FAULT_HIGH_MARGIN {20.0f};      // ADC counts above adcMax tolerated
    // Curve shaping
    constexpr float THROTTLE_BLEND_LINEAR {0.4f};  // 0 = fully quadratic, 1 = fully linear
    // Current limits 
    constexpr float THROTTLE_IQ_MAX {CURRENT_LIMIT};              // Max current at full pedal
    constexpr float THROTTLE_IQ_LAUNCH_MAX {40.0f};               // Speed-based derating
    constexpr float THROTTLE_LAUNCH_SPEED_RAD_PER_SEC {5.0f};     // Below this speed, use launch current limit
    constexpr float THROTTLE_SPEED_FOR_FULL_IQ {30.0f};          // Above this speed, full throttle current is available
    // Rate limiting
    constexpr float THROTTLE_IQ_RAMP_UP {100.0f};                  // A/s, how fast the current reference can rise, slower for smoother throttle response and efficiency
    constexpr float THROTTLE_IQ_RAMP_DOWN {200.0f};               // Optional creep / stiction handling
    constexpr bool  THROTTLE_ENABLE_MIN_START_CURRENT = true;
    constexpr float THROTTLE_MIN_START_CURRENT = 6.0f;
    constexpr float THROTTLE_MIN_START_PEDAL = 0.08f;             // Pedal threshold to trigger minimum start current
    // Safety behavior    
    constexpr bool THROTTLE_CUT_TO_ZERO_ON_FAULT = true;          // If true, will immediately cut current to zero if a fault is detected on the pedal input (e.g. out of range ADC reading)

#endif /* HUB_MOTOR */

#ifdef KOFORD_MOTOR
    constexpr int SUPPLY_VOLTAGE {24};
    constexpr int POLE_PAIRS {15};
    constexpr float ZERO_ALIGN_VALUE {5.24};
    // Missing adjust 
    constexpr float PHASE_RESISTANCE {0.124f};
    constexpr float KV_RATING        {70.0f};
    // Safety values
    constexpr float VOLTAGE_LIMIT {24.0f};
    constexpr float CURRENT_LIMIT {2.0f};   // Iq is higher than the DC current flowing from battery approx Idc = 0.8Iq
    // Controller Iq gains
    constexpr float IQ_KP {3.0f};
    constexpr float IQ_KI {500.0f};
    constexpr float IQ_KD {0.0f};
    // Controller Id gains
    constexpr float ID_KP {0.1f};
    constexpr float ID_KI {100.0f};
    constexpr float ID_KD {0.0f};
    // Filter time constants (TF)
    constexpr float IQ_TF {0.01f};
    constexpr float ID_TF {0.01f};

    // Throttle FOC parameters
    constexpr float THROTTLE_FILTER_TAU {0.03f};    // Secs
    constexpr float THROTTLE_DEADBAND {0.02f};      // Pedal input below this is ignored
    constexpr float FAULT_LOW_MARGIN {20.0f};       // ADC counts below adcMin tolerated
    constexpr float FAULT_HIGH_MARGIN {20.0f};      // ADC counts above adcMax tolerated
    // Curve shaping
    constexpr float THROTTLE_BLEND_LINEAR {0.30f};  // 0 = fully quadratic, 1 = fully linear
    // Current limits 
    constexpr float THROTTLE_IQ_MAX {CURRENT_LIMIT / 2.0f};     // Max current at full pedal
    constexpr float THROTTLE_IQ_LAUNCH_MAX {8.0f};              // Speed-based derating
    constexpr float THROTTLE_LAUNCH_SPEED_RAD_PER_SEC {8.0f};   // Below this speed, use launch current limit
    constexpr float THROTTLE_SPEED_FOR_FULL_IQ {40.0f};         // Above this speed, full throttle current is available
    // Rate limiting
    constexpr float THROTTLE_IQ_RAMP_UP {80.0f};                // A/s, how fast the current reference can rise, slower for smoother throttle response and efficiency
    constexpr float THROTTLE_IQ_RAMP_DOWN {180.0f};             // Optional creep / stiction handling
    constexpr bool  THROTTLE_ENABLE_MIN_START_CURRENT = false;  // If true, will inject a small minimum current when pedal is pressed beyond min start pedal to help overcome stiction at very low speeds
    constexpr float THROTTLE_MIN_START_PEDAL = 0.08f;           // Pedal threshold to trigger minimum start current
    constexpr float THROTTLE_MIN_START_CURRENT = 2.0f;          // Minimum current to inject when pedal exceeds min start pedal, adjust based on your motor and load to overcome stiction without causing excessive creep
    // Safety behavior    
    constexpr bool THROTTLE_CUT_TO_ZERO_ON_FAULT = true;        // If true, will immediately cut current to zero if a fault is detected on the pedal input (e.g out of range ADC reading)
#endif /* KOFORD_MOTOR */


// Current sensing (inline) parameters 
constexpr float SHUNT_VALUE       {0.005f};
constexpr int CURRENT_SENSOR_GAIN {20};

// PWM parameters
constexpr long PWM_FREQUENCY {20000};

// Throttle common values
constexpr int THROTTLE_RESOLUTION_BITS {12};