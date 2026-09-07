#include "elyos_hal.h"
#include <core_pins.h>
#include <HardwareSerial.h>

namespace elyos {

uint32_t Time::millis() {
    return ::millis();
}

uint32_t Time::micros() {
    return ::micros();
}

void Time::delayMs(uint32_t ms) {
    ::delay(ms);
}

void Time::yield() {
    ::yield();
}

void Gpio::setMode(int pin, GpioMode mode) {
    uint8_t m = INPUT;
    switch (mode) {
        case GpioMode::Output:
            m = OUTPUT;
            break;
        case GpioMode::InputPullup:
            m = INPUT_PULLUP;
            break;
        case GpioMode::InputPulldown:
            m = INPUT_PULLDOWN;
            break;
        default:
            m = INPUT;
            break;
    }
    ::pinMode(pin, m);
}

void Gpio::write(int pin, GpioLevel level) {
    ::digitalWriteFast(pin, (level == GpioLevel::High) ? HIGH : LOW);
}

GpioLevel Gpio::read(int pin) {
    return ::digitalReadFast(pin) ? GpioLevel::High : GpioLevel::Low;
}

void Gpio::toggle(int pin) {
    int current = ::digitalReadFast(pin);
    ::digitalWriteFast(pin, current ? LOW : HIGH);
}

static inline uint32_t get_primask() {
    uint32_t result;
    __asm__ volatile ("mrs %0, primask" : "=r" (result) :: "memory");
    return result;
}

static inline void set_primask(uint32_t pri) {
    __asm__ volatile ("msr primask, %0" :: "r" (pri) : "memory");
}

void Adc::init() {
    setResolution(12);
}

void Adc::setResolution(int bits) {
    uint32_t primask = get_primask();
    __asm__ volatile ("cpsid i" : : : "memory");
    ::analogReadResolution(bits);
    set_primask(primask);
}

uint16_t Adc::read(int pin, int bits) {
    (void)bits;
    // Sección crítica atómica para evitar que una interrupción de FOC colisione
    // con lecturas de ADC en segundo plano (pedal o VBUS) sobre el mismo módulo de silicio (ADC1)
    uint32_t primask = get_primask();
    __asm__ volatile ("cpsid i" : : : "memory");
    uint16_t val = static_cast<uint16_t>(::analogRead(pin));
    set_primask(primask);
    return val;
}

class TeensyHardwareUart : public Uart {
public:
    explicit TeensyHardwareUart(HardwareSerial &serial) : serial_(serial) {}

    void begin(uint32_t baud_rate) override {
        serial_.begin(baud_rate);
    }

    int available() override {
        return serial_.available();
    }

    int readByte() override {
        return serial_.read();
    }

    size_t write(const uint8_t *data, size_t len) override {
        return serial_.write(data, len);
    }

    size_t writeByte(uint8_t byte) override {
        return serial_.write(byte);
    }

private:
    HardwareSerial &serial_;
};

static TeensyHardwareUart s_telemetry_uart(Serial1);

Uart* getTelemetryUart() {
    return &s_telemetry_uart;
}

}  // namespace elyos
