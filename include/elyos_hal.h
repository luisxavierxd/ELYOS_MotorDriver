#pragma once

#include <cstdint>
#include <cstddef>

namespace elyos {

enum class GpioMode : uint8_t {
    Input,
    Output,
    InputPullup,
    InputPulldown,
};

enum class GpioLevel : uint8_t {
    Low = 0,
    High = 1,
};

class Time {
public:
    static uint32_t millis();
    static uint32_t micros();
    static void delayMs(uint32_t ms);
    static void yield();
};

class Gpio {
public:
    static void setMode(int pin, GpioMode mode);
    static void write(int pin, GpioLevel level);
    static GpioLevel read(int pin);
    static void toggle(int pin);
};

class Adc {
public:
    static void init();
    static void setResolution(int bits = 12);
    static uint16_t read(int pin, int bits = 12);
};

class Uart {
public:
    virtual ~Uart() = default;
    virtual void begin(uint32_t baud_rate) = 0;
    virtual int available() = 0;
    virtual int readByte() = 0;
    virtual size_t write(const uint8_t *data, size_t len) = 0;
    virtual size_t writeByte(uint8_t byte) = 0;
};

// Obtains the hardware UART interface (e.g. Serial1 on Teensy 4.1)
Uart* getTelemetryUart();

}  // namespace elyos
