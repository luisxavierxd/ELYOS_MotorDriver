#pragma once 
#include "Pinout.h"
#include "FOC_Parameters.h"
#include <SimpleFOC.h>
#include "Telemetry_Manager.h"
#include "BLDC_Logger.h"

#define ELYOS_DRIVER_OK      0
#define ELYOS_DRIVER_ERROR  -1

typedef struct {
    int code;
    String message;
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
        // Low pass filter for throttle
        LowPassFilter throttle_lpf = LowPassFilter(0.02f);

    public:
        // For ISR
        HallSensor *hall_sensor;

        ELYOS_DRIVER();
        int driver_Init();
        int control_Init();
        void runFOC();

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
};