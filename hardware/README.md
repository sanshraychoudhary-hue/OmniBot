# OmniBot Hardware Documentation

## Overview
The OmniBot is an autonomous holonomic mobile robot designed for precision warehouse operations. It uses a 4-wheel omnidirectional drive system (X-drive configuration) powered by a Raspberry Pi Pico W.

## Components List

### Microcontroller
*   **Raspberry Pi Pico W**: Dual-core ARM Cortex M0+ processor with built-in Wi-Fi. Handles all motor control, kinematics, sensor fusion, and web server for UI.

### Actuation & Drive
*   **Motors**: 4x DC Gear Motors with Magnetic Encoders (High precision for odometry).
*   **Motor Drivers**: 2x TB6612FNG Dual Motor Driver Carriers.
    *   *Note*: L298N was considered but TB6612FNG was chosen for better efficiency and compact size.
*   **Wheels**: 4x Omni-directional wheels (38mm diameter). The rollers allow sideways movement.

### Sensors
*   **Encoders**: 4x Incremental Quadrature Encoders (attached to motors) - 520 ticks/revolution.
*   **IMU**: MPU6050 (Accelerometer + Gyroscope) for orientation tracking (optional/integrated in some code versions).

### Interface
*   **Display**: OLED Display (0.96" 128x64 I2C) for status messages, IP address, and mode display.
*   **Joystick**: Analog Joystick (for manual control in some test setups).

## Pinout Configuration (Raspberry Pi Pico W)

| Component | Pin Function | Pico GP Pin |
| :--- | :--- | :--- |
| **Motor A (Front Left)** | IN1 | GP2 |
| | IN2 | GP3 |
| | PWM | GP4 |
| | Enc A | GP16 |
| | Enc B | GP17 |
| **Motor B (Front Right)** | IN1 | GP5 |
| | IN2 | GP6 |
| | PWM | GP7 |
| | Enc A | GP18 |
| | Enc B | GP19 |
| **Motor C (Rear Left)** | IN1 | GP10 |
| | IN2 | GP11 |
| | PWM | GP12 |
| | Enc A | GP20 |
| | Enc B | GP21 |
| **Motor D (Rear Right)** | IN1 | GP13 |
| | IN2 | GP14 |
| | PWM | GP15 |
| | Enc A | GP22 |
| | Enc B | GP26 |
| **Drivers** | STBY | GP8 |
| **I2C (Display/MPU)** | SDA | (Check Code) |
| | SCL | (Check Code) |

## Kinematics
The robot uses a 4-wheel X-configuration. The wheels are mounted at 45, 135, 225, and 315 degrees relative to the chassis frame.
*   **Wheel Angles**: [45°, 135°, 225°, 315°]
*   **Kinematic Model**: Allows calculation of individual wheel speeds based on desired Forward (Vx), Strafe (Vy), and Rotational (ω) velocities.
