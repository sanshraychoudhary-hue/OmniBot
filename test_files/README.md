# Test Files

This directory contains individual unit test sketches used to debug and verify specific hardware components before full integration.

## Files
*   **picoEncoderTest**: Verifies that the motor encoders are counting ticks correctly.
*   **picoMotorTest**: Tests basic motor rotation (Forward/Backward) to check driver wiring.
*   **picoDisplayTest**: Tests the OLED screen connection and library.
*   **picoMPU_test**: Reads raw data from the MPU6050 accelerometer/gyroscope.
*   **picoServoTest / picoServoTestWifi**: Tests servo motor control for the arm.
*   **picoWifiTester**: Verifies Wi-Fi connectivity on the Pico W.
*   **picoLed**: Simple blink test.

## Usage
Upload these sketches individually to isolate issues with specific components.
