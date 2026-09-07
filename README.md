# ELYOS Motor Driver

This repository contains the firmware for the ELYOS motor driver.
It has been split into two parts to maintain a stable, working version while exploring a new industrial-grade architecture.

## Directory Structure

*   [`teensyduino_freertos/`](teensyduino_freertos/): The current, stable working version using **Teensyduino** and **FreeRTOS** on the Teensy 4.1. This uses the `SimpleFOC` library and implements a multi-tasking architecture for telemetry, FOC control, and pedal processing. This project is built using PlatformIO.
*   [`zephyr_rtos/`](zephyr_rtos/): An experimental port of the motor driver to the **Zephyr RTOS**. This aims to evaluate Zephyr as a more robust, industrial-grade RTOS alternative to the Teensyduino framework.

### Building

**Teensyduino + FreeRTOS**
1. Open the `teensyduino_freertos` directory in VS Code with the PlatformIO extension installed.
2. Build and upload using PlatformIO.

**Zephyr RTOS**
1. Ensure you have the Zephyr SDK and west workspace set up.
2. From the `zephyr_rtos` directory, build using: `west build -b teensy41`
3. Flash using: `west flash`
