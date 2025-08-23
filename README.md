# Autonomous Floor Cleaning Robot - ESP32 FreeRTOS

An advanced autonomous floor cleaning robot implementation using ESP32, FreeRTOS, and Cytron MD10C motor drivers.

## Features

- Real-time obstacle avoidance with ultrasonic sensors
- Efficient path planning algorithms
- Bluetooth connectivity for monitoring and control
- Battery monitoring with low-voltage protection
- PID-controlled motor movement with encoder feedback
- Emergency stop functionality
- FreeRTOS-based multitasking architecture

## Hardware Requirements

- ESP32 DOIT DevKit V1
- Cytron MD10C Motor Drivers (x2)
- HC-SR04 Ultrasonic Sensors (x3)
- DC Motors with Encoders (x2)
- Cleaning mechanism motor
- 12V Li-ion Battery Pack
- Voltage divider circuit for battery monitoring

## Software Requirements

- PlatformIO IDE or Arduino IDE
- ESP32 Board Support
- FreeRTOS library

## Installation

1. Clone this repository
2. Open in PlatformIO
3. Connect your ESP32 board
4. Build and upload

## Wiring Guide

See `docs/wiring_guide.md` for detailed connection instructions.

## Usage

1. Power on the robot
2. Connect via Bluetooth using a serial terminal app
3. Send commands:
   - `START`: Begin cleaning
   - `STOP`: Stop cleaning
   - `DOCK`: Return to charging dock
   - `EMERGENCY_STOP`: Immediate stop
   - `STATUS`: Request status update

## Project Structure

- `src/`: Main source code
- `include/`: Configuration files
- `test/`: Unit tests
- `hardware/`: Schematics and PCB designs
- `docs/`: Documentation

## Contributing

Please read CONTRIBUTING.md for details on our code of conduct and the process for submitting pull requests.

## License

This project is licensed under the MIT License - see the LICENSE.md file for details.

## Acknowledgments

- Espressif for the ESP32 platform
- Cytron Technologies for the MD10C motor driver
- FreeRTOS for the real-time operating system