#pragma once 

// Phase PWM signal pins 
constexpr int A_PHASE_HIGH_PIN  {4};
constexpr int A_PHASE_LOW_PIN   {33};
constexpr int B_PHASE_HIGH_PIN  {6};
constexpr int B_PHASE_LOW_PIN   {9};
constexpr int C_PHASE_HIGH_PIN  {36};
constexpr int C_PHASE_LOW_PIN   {37};

// Hall sense pins 
constexpr int HALL_A_PIN {10};
constexpr int HALL_B_PIN {12};
constexpr int HALL_C_PIN {21};

// Current sense pins 
constexpr int CURRENT_SENSE_A_PIN {26};
constexpr int CURRENT_SENSE_B_PIN {27};
constexpr int CURRENT_SENSE_C_PIN {38};

// Throttle pin
constexpr int THROTTLE_PIN {24};