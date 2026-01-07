# OmniBot Code

This directory contains the firmware for the Raspberry Pi Pico W.

## Main Firmware
The active and most recent firmware version is located in the **`picoOmniCAR_latest_noMPU_oled_SavedWifi_P1`** directory.

### Features
*   **Omnidirectional Control**: Implements vector-based kinematics for holonomic movement.
*   **Wi-Fi Control**: Hosts a web server (or connects to one) to receive commands from the Web UI.
*   **PID Control**: (If applicable in this version) uses encoder feedback to maintain precise motor speeds.
*   **OLED Feedback**: Displays connection status, IP address, and current state.

## Legacy / Reference
*   `legacy_working_cursor.ino`: An older, known-working version of the code that supports cursor control and MPU6050 integration.

## Dependencies
This project is built using the Arduino IDE (with RP2040 core) or PlatformIO.
Required Libraries (install via Library Manager):
*   **Adafruit SSD1306** (for OLED)
*   **Adafruit GFX** (for OLED)
*   **Adafruit MPU6050** (if using MPU version)
*   **Adafruit Unified Sensor**
*   **WiFi** (Standard Pico W library)
