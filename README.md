🤖 Dental Assistant Robotic Arm
A 3-DOF robotic manipulator designed to assist dentists by delivering instruments in clinical settings in Masaya, Nicaragua.

🎯 Project Overview
This project addresses challenges in dental practices by providing automated instrument delivery, improving precision and workflow efficiency in space-constrained environments.

Key Features
3-DOF Robotic Arm with Rz-Ry-Ry configuration

Arduino Uno R3 based control system

Low-cost design using accessible materials

Clinical safety protocols with collision avoidance

Sterilizable surfaces for medical environments

🔧 Technical Specifications
Mechanical Design
Configuration: Rz-Ry-Ry (3 Degrees of Freedom)

Workspace: 350-650mm radial coverage

Payload Capacity: 50-200 grams

Position Accuracy: ≤5mm

Materials: Aluminum, Stainless Steel

Electronic System
Controller: Arduino Uno R3

Actuators: 3x Servo Motors (0.5-3 kg·cm)

Sensors: Force, Proximity, Encoders

Power: 5V DC, 2-3A

Communication: UART, I2C

🚀 Quick Start
Hardware Setup
Assemble mechanical structure

Install servos on base, shoulder, and elbow joints

Connect electronics to Arduino Uno R3

Mount parallel gripper end-effector

Software Installation
cpp
// Required Arduino Libraries
#include <Servo.h>
#include <Wire.h>
Clone repository

Open main_control_sketch.ino in Arduino IDE

Select board: Arduino Uno

Upload to microcontroller

Pin Configuration
cpp
const int BASE_SERVO = 9;
const int SHOULDER_SERVO = 10; 
const int ELBOW_SERVO = 11;
const int GRIPPER_SERVO = 6;
const int FORCE_SENSOR = A0;
💡 Usage
Basic Operation
Power on the system

Run homing sequence

Position arm using control interface

Grasp and deliver instruments

Safety Features
Emergency stop button

Collision detection sensors

Force limiting on gripper

Mechanical end-stops

📁 Project Structure
text
robotic-arm-prototype/
├── firmware/           # Arduino control code
├── mechanical/         # CAD designs and schematics
├── documentation/      # Project documentation
└── simulations/        # Kinematic simulations
👥 Team
Br. Wilberth Alejandro Pérez Loredo

Br. Shalom Jireh Rayo Blandon

Br. Anthony Isai Aráuz Galán

Br. Engel Mohamed Urbina Rivas

Br. Luis Miguel Garcia Mondragon

<div align="center">
"2025: Efficiency and Quality to Continue in Victories"
Robotics Engineering - National University of Engineering

</div>
