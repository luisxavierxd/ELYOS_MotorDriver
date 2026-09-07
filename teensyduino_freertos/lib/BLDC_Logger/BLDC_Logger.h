#pragma once 

#include <cstdint>

// Flush to SD every N samples 
constexpr uint8_t kSamplesToFlushSD{200};

typedef struct {
    uint32_t timestamp;  // Milliseconds
    uint16_t raw_throttle;
    uint16_t VBat;       // mV
    float currentA;      // A
    float currentB;      // A
    float currentC;      // A
    uint16_t rpm;
} BLDC_Logger_Data;


class BLDC_Logger {
private:
    const char* logger_file_name_ = "testing1.csv";
    void* file_handle_ = nullptr;
    bool sd_detected_ = false;
    uint8_t sample_id_ = 0;

public:
    BLDC_Logger_Data data{};

    BLDC_Logger();
    ~BLDC_Logger();

    void init();

    bool isReady() const { return sd_detected_; }
    void logMotorDataSD();
    void logMotorDataSD(const BLDC_Logger_Data &record);
};