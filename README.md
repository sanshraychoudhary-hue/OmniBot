# OmniServe

**Project Title**: Design and Implementation of an Autonomous Holonomic Mobile Robot for Precision Warehouse Operations

## Overview
**OmniServe** is a Wi-Fi-controlled omni-directional robotic platform designed for enhanced mobility and object manipulation in constrained environments. It features precise movement using encoder feedback, a 3-DOF robotic arm for pick-and-place operations, and a web-based dashboard for real-time control, system monitoring, PID tuning, and vector-based motion commands. The robot can move in all eight directions and rotate in place, making it ideal for agile navigation. Using a PID algorithm, it achieves accurate linear movements with control over distance and direction. OmniServe is suitable for applications such as warehouse automation, smart delivery systems, and educational robotics.

This project was developed as part of a summer internship at **AICTE IDEA Lab, Guru Gobind Singh Indraprastha University**, from June 23, 2024, to August 1, 2024.

## Repository Structure
This repository contains all the design and source files for OmniServe, organized as follows:

*   **[`hardware/`](hardware/README.md)**: Specifications for motor drivers, encoders, RP2040, and wiring.
*   **[`code/`](code/README.md)**: Firmware for the Raspberry Pi Pico W (Arduino C++).
*   **[`design/`](design/README.md)**: Laser-cut chassis files (`.dxf`).
*   **[`UI/`](UI/README.md)**: Web Dashboard HTML/JS files.
*   **[`simulator_scripts/`](simulator_scripts/README.md)**: Python kinetics simulators.
*   **[`test_files/`](test_files/README.md)**: Unit tests for individual components.
*   **[`report/`](report/README.md)**: Full project report, presentation, and research.
*   **[`assets/`](assets/README.md)**: Images and demonstration videos.

---

## Features
*   **Omni-Directional Movement**: 360-degree movement in any direction, including rotation in place, enabled by omni-wheels.
*   **Encoder Feedback**: Real-time position and speed monitoring using quadrature encoders for precise control.
*   **PID Control**: Ensures accurate distance and direction travel, with tunable parameters via the dashboard.
*   **Web Dashboard**: Responsive interface for real-time control, telemetry display, and PID tuning. Supports joystick (using NippleJS) and keyboard (WASD) input.
*   **3-DOF Robotic Arm**: Servo-based arm for basic pick-and-place tasks, controlled via PWM signals.
*   **Emergency Stop**: Safety feature for immediate halt, implemented in both firmware and dashboard.
*   **Telemetry**: Live display of encoder values, robot state, and diagnostics on the dashboard.
*   **Vector-Based Commands**: Allows precise control over direction and distance.
*   **Expandable Design**: Ready for additional sensors like IMU or cameras.

### Feature Summary Table
| Feature | Description |
| :--- | :--- |
| **Omni-directional drive** | Move in any direction, rotate in place |
| **Encoder feedback** | Accurate distance and speed measurement |
| **PID control** | Precise movement and stopping |
| **Web dashboard** | Real-time control and monitoring |
| **Robotic arm** | 3-DOF, pick-and-place tasks |
| **Emergency stop** | Safety for users and environment |
| **Telemetry** | Live data on dashboard |
| **Expandable** | Add sensors and modules |

## Applications
*   **Warehouse Automation**: Material movement and inventory management in narrow aisles.
*   **Smart Delivery**: Last-mile delivery in controlled environments.
*   **Factory Logistics**: Transporting parts and tools.
*   **Educational Robotics**: Platform for teaching robotics, control systems, and programming.
*   **Prototyping**: Base for research and development in automation.

### Example Use Case: OmniServe in a Smart Warehouse
OmniServe can autonomously navigate narrow aisles, retrieve packages using its robotic arm, and deliver them to specified locations. Operators monitor via the dashboard and adjust PID parameters dynamically for varying payloads. It supports obstacle avoidance, route re-planning, and multi-robot collaboration.

## Hardware Architecture

### Chassis and Mechanical Design
*   **Four-wheel omni-wheel chassis** for 360-degree movement.
*   **Frame**: Lightweight aluminum for strength and portability.
*   **Omni-wheels**: 1.5-inch diameter for smooth lateral and diagonal movement.

### Motors and Drivers
*   **Motors**: Four DC motors, each with a magnetic quadrature encoder.
*   **Drivers**: Dual **TB6612FNG** motor drivers are used. Each driver controls two motors.
    *   **Driver 1 (Front)**: Controls Motor A (Front Left) and Motor B (Front Right).
    *   **Driver 2 (Rear)**: Controls Motor C (Rear Left) and Motor D (Rear Right).

### Encoders
*   **Quadrature encoders** (520 ticks/rev) on each wheel for closed-loop PID control and accurate distance estimation.

### Robotic Arm
*   **3-DOF servo-based arm** mounted on the chassis.
*   **Servos**: Standard hobby servos for base rotation, shoulder, and gripper movement.
*   **Capabilities**: Pick-and-place operations for objects up to 200g.

### Power Supply
*   Rechargeable **Li-ion battery pack** with voltage regulation for stable operation of logic and motors.

### Detailed Connection Diagram (GPIO Mapping)

#### Motor Driver 1 (Front Motors)
Controls the Front Left and Front Right wheels.
*   **Standby (STBY)**: GP8 (Shared)
*   **Channel A (Front Left - Motor A)**
    *   IN1: GP2
    *   IN2: GP3
    *   PWM: GP4
    *   *Encoder*: A (GP16), B (GP17)
*   **Channel B (Front Right - Motor B)**
    *   IN1: GP5
    *   IN2: GP6
    *   PWM: GP7
    *   *Encoder*: A (GP18), B (GP19)

#### Motor Driver 2 (Rear Motors)
Controls the Rear Left and Rear Right wheels.
*   **Standby (STBY)**: GP8 (Shared)
*   **Channel A (Rear Left - Motor C)**
    *   IN1: GP10
    *   IN2: GP11
    *   PWM: GP12
    *   *Encoder*: A (GP20), B (GP21)
*   **Channel B (Rear Right - Motor D)**
    *   IN1: GP13
    *   IN2: GP14
    *   PWM: GP15
    *   *Encoder*: A (GP22), B (GP26)

#### Other Connections
| Function | Pico W Pin(s) |
| :--- | :--- |
| **Standby** | GP8 |
| **Robotic Arm PWM** | GP23, GP24, GP25 (suggested) |
| **I2C (OLED/MPU)** | GP0 (SDA), GP1 (SCL) |

### Main Components Summary
| Module | Description |
| :--- | :--- |
| **Controller** | Raspberry Pi Pico W, runs main logic and hosts web dashboard |
| **Motor Drivers** | TB6612FNG, controls four DC motors |
| **Encoders** | Quadrature, provides feedback for PID control |
| **Robotic Arm** | 3-DOF, servo-based, for object manipulation |
| **Web Dashboard** | HTML/JS interface for control and monitoring |
| **Power Supply** | Li-ion battery, voltage regulation for stable operation |

### Design Choices and Trade-offs
*   **Omni-wheels** provide maximum maneuverability but require precise control logic.
*   **Raspberry Pi Pico W** selected for Wi-Fi capability and high GPIO count.
*   **Power management** ensures long operational time and safe charging.

## Software Architecture

### Embedded Firmware
*   Implemented in **Arduino C++** on Raspberry Pi Pico W.
*   Handles motor control, encoder feedback, PID computation, and dashboard communication.

### Web Dashboard
*   Responsive interface using **HTML, CSS, and JavaScript**.
*   Served directly from the Pico W.
*   **Features**: Joystick input (NippleJS), keyboard control (WASD), real-time telemetry, PID tuning.

### Communication Protocols
*   **WebSocket**: For low-latency, real-time control and telemetry.
*   **HTTP/REST**: For configuration and status queries.

### Control Logic
*   Supports direct (WASD) and vector-based movement.
*   Encoder feedback closes the loop for precise distance and direction.
*   PID controller tunable in real-time from the dashboard.

#### PID Implementation Example
```cpp
float error = target - measured;
integral += error * dt;
derivative = (error - prev_error) / dt;
output = Kp * error + Ki * integral + Kd * derivative;
prev_error = error;
```

### Encoder Handling
Interrupts are used for accurate tick counting at high speeds.

### Safety Features
Emergency stop validation for all commands.

## Methodology and Development Process
The project followed an iterative approach:
1.  **Design Phase**: Requirements analysis, block diagrams, and schematics.
2.  **Prototyping**: Breadboard testing of motors, encoders, and arm; 3D-printed parts.
3.  **Integration**: Full chassis assembly, wiring, and subsystem connection.
4.  **Testing and Validation**: Scenarios for movement, rotation, pick-and-place; accuracy checks.
5.  **Iterative Refinement**: Regular logging and addressing of issues.

### Project Timeline (Gantt Chart Overview)
Tasks were prioritized as Critical (Red), High (Orange), Medium (Yellow), Low (Blue):
*   Component Selection & BOM (Low)
*   Chassis Assembly & Wiring (Critical)
*   Motor & Power Integration (Critical)
*   Wi-Fi Setup & Web UI (Medium)
*   Encoder Calibration (High)
*   Motion Control (Critical)
*   PID Tuning & Distance Control (High)
*   Robotic Arm Mounting (High)
*   Pick-and-Place Logic (High)
*   Final Testing (Critical)
*   Documentation & Demo Video (Low)

Core functionality was built first, followed by precision control and arm integration.

## Results and Performance
*   **Movement Accuracy**: Average positional error <1 cm over 2 meters.
*   **Dashboard Latency**: <100 ms for responsive operation.
*   **Limitations**: Battery life under heavy load; occasional Wi-Fi dropouts in congested environments.

### Accuracy Table
| Target Distance (cm) | Measured (cm) | Error (cm) |
| :--- | :--- | :--- |
| 50 | 49.5 | 0.5 |
| 100 | 99.2 | 0.8 |
| 200 | 199.1 | 0.9 |

### Robotic Arm Performance
Successful pick-and-place for up to **200g** objects; repeatability within **2 mm**.

## Challenges and Solutions
| Challenge | Solution |
| :--- | :--- |
| **Omni-wheel control** | PID tuning, speed calibration |
| **Encoder noise** | Software debouncing, filtering |
| **Power management** | Separate power rails, voltage regulation |
| **Chassis alignment** | Precision assembly, iterative testing |
| **Servo mounting** | Custom 3D-printed brackets |
| **Real-time communication** | WebSocket optimization, error handling |
| **Dashboard responsiveness** | Efficient JS, minimal data transfer |
| **Subsystem integration** | Modular design, staged integration |
| **Testing in real environments** | Adaptive code, robust error handling |

## Future Work
*   **IMU Integration**: For angle feedback and stabilization.
*   **Camera and AI**: Object detection and visual navigation.
*   **Path Planning**: Autonomous navigation using map grids.
*   **Gripper Sensing**: Force or proximity sensors for delicate handling.
*   **Autonomous Object Sorting**: Algorithms for logistics applications.
*   **Research and Commercial Potential**: Platform for further development in warehouse automation, smart delivery, and education.

## Installation and Setup

### Hardware Setup
1.  Assemble the chassis with omni-wheels and mount the motors.
2.  Connect motor drivers (TB6612FNG) to the Raspberry Pi Pico W per GPIO mapping.
3.  Attach encoders to each wheel.
4.  Mount the 3-DOF robotic arm and connect servos.
5.  Wire the power supply with voltage regulation.
6.  Ensure all connections are secure and test for shorts.

### Software Setup
1.  Install Arduino IDE and add support for Raspberry Pi Pico.
2.  Upload the firmware (Arduino C++) from the `code/` directory to the Pico W.
3.  Host the web dashboard files on the Pico W (embedded in the code).
4.  Connect to the Pico W's Wi-Fi network.
5.  Access the dashboard via a web browser (e.g., `http://192.168.4.1` or the IP displayed on OLED).

### Dependencies
*   **Arduino C++** for firmware.
*   **HTML/CSS/JavaScript** for dashboard (NippleJS for joystick).
*   No additional package installations required beyond standard Arduino setup.

## Usage
1.  Power on the robot and connect to its Wi-Fi network.
2.  Open the web dashboard in a browser.
3.  Use **joystick** or **WASD keys** for movement.
4.  Tune **PID parameters** (Kp, Ki, Kd) via sliders.
5.  Control the **robotic arm** with dedicated buttons/sliders.
6.  Monitor **telemetry** (encoder values, speed, position).
7.  Activate **emergency stop** if needed.
8.  **Vector-based control**: Input direction (angle) and distance; the robot will execute precisely using PID.

## Contributors
*   **Alay Singh Negi**
*   **Kaustubh Singh Rautela**
*   **Shiven Pundir**
*   **Sanshray Choudhary**

*University School of Information, Communication & Technology (USIC&T), GGSIPU.*

<img width="800" height="460" alt="image" src="https://github.com/user-attachments/assets/779b9bac-039e-4d94-93d3-cb716c0ca839" />

