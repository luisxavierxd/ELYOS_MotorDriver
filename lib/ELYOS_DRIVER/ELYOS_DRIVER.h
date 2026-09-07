#pragma once 
#include "Pinout.h"
#include "FOC_Parameters.h"
#include <SimpleFOC.h>
#include "SmoothingSensor.h"
#include "ThrottleFOC.h"

// Telemetry and logging
#include "Telemetry_Manager.h"
#include "BLDC_Logger.h"

#define ELYOS_DRIVER_OK      0
#define ELYOS_DRIVER_ERROR  -1

typedef struct {
    int code;
    const char *message;
} ELYOS_DRIVER_STATUS;

class ELYOS_DRIVER {
    protected:
        // Simple FOC handlers 
        BLDCMotor motor{POLE_PAIRS, PHASE_RESISTANCE, KV_RATING};

        BLDCDriver6PWM driver{A_PHASE_HIGH_PIN, A_PHASE_LOW_PIN,
                             B_PHASE_HIGH_PIN, B_PHASE_LOW_PIN,
                             C_PHASE_HIGH_PIN, C_PHASE_LOW_PIN};

        InlineCurrentSense current_sense{SHUNT_VALUE, CURRENT_SENSOR_GAIN, CURRENT_SENSE_A_PIN,
                                          CURRENT_SENSE_B_PIN, CURRENT_SENSE_C_PIN};

        // Change motor parameters during tunning easily 
        Commander commander{Serial};
        TelemetryManager telemetry;
        uint16_t telemetry_vbus_mV;
        int32_t telemetry_ibus_mA;
        float telemetry_ibus_mA_filtered;
        int32_t telemetry_rpm;
        int16_t telemetry_iq_mA;
        int16_t telemetry_id_mA;

        TelemetryManager::foc_all_fast fast_data;

        // Low pass filter for throttle
        // LowPassFilter throttle_lpf = LowPassFilter(0.02f);

        // Sensors (must match constructor initializer order)
    public:
        HallSensor *hall_sensor = nullptr;
        SmoothingSensor *smooth_sensor = nullptr;

    protected:
        // Throttle FOC approach
        ThrottleFOC throttle;

    public:
        ELYOS_DRIVER();
        int driver_Init();
        int control_Init();
        void ThrottleFOC_Init();
        void runFOC();

        // Modular execution steps for FreeRTOS / RTOS tasks
        void stepFOC(float iq_cmd);
        float updateThrottle();
        void processTelemetry();
        void processCommander();
        void populateLoggerData(BLDC_Logger_Data &log_data);

        // State accessors
        float getShaftVelocity();
        float getMotorTarget() const { return motor.target; }
        ThrottleFOC& getThrottle() { return throttle; }

        // Telemetry calculation
        void calculateTelemetry();

        // Callbacks
        void do_Motor(char *cmd);
        // Id PID
        void cmd_set_Id_P(char* cmd);
        void cmd_set_Id_I(char* cmd);
        void cmd_set_Id_D(char* cmd);
        // Iq PID
        void cmd_set_Iq_P(char* cmd);
        void cmd_set_Iq_I(char* cmd);
        void cmd_set_Iq_D(char* cmd);

        // Target command
        void cmd_set_target(char* cmd);
};