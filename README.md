🤖 Dental Assistant Robotic Arm Prototype
📋 Project Overview
The Dental Assistant Robotic Arm Prototype is an innovative 3-DOF (Degree of Freedom) robotic manipulator designed to assist dentists in clinical settings in Masaya, Nicaragua. This project addresses local challenges in dental practice by providing automated instrument delivery, improving precision, and enhancing workflow efficiency in space-constrained environments.

🎯 Key Features
3-DOF Robotic Manipulator with Rz-Ry-Ry configuration

Low-Cost Design using accessible local materials

Arduino Uno R3-based Control System for reliable operation

Clinical Environment Adaptation for dental instrument handling

Safety-First Approach with collision avoidance and force limiting

Sterilizable Surfaces compatible with medical protocols

🏗️ System Architecture
Mechanical Design
text
┌─────────────────────────────────────────────────────────────┐
│                    ROBOTIC ARM STRUCTURE                    │
├─────────────────────────────────────────────────────────────┤
│  • Configuration: Rz-Ry-Ry (3-DOF)                         │
│  • Reachable Workspace: 350-650mm radial coverage          │
│  • Payload Capacity: 50-200 grams                          │
│  • Positional Accuracy: ≤5mm                               │
│  • Repeatability: ≤5mm                                     │
└─────────────────────────────────────────────────────────────┘
Joint Configuration
Joint	Type	Range	Actuator	Purpose
Base (Rz)	Rotational	±180°	Servo Motor	Circumferential access around dental chair
Shoulder (Ry)	Rotational	0-90°	Servo Motor	Height and radial reach adjustment
Elbow (Ry)	Rotational	0-135°	Servo Motor	Fine positioning and orientation
Electronic System
text
┌─────────────────────────────────────────────────────────────┐
│                   ELECTRONIC ARCHITECTURE                   │
├─────────────────────────────────────────────────────────────┤
│  Primary Controller: Arduino Uno R3                        │
│  Power Supply: 5V DC, 2-3A                                 │
│  Communication: UART, I2C, SPI                             │
│  Safety Systems: E-stop, torque limiting, collision detection │
└─────────────────────────────────────────────────────────────┘
Component Specifications
Component	Model/Specs	Quantity	Purpose
Microcontroller	Arduino Uno R3	1	Main control unit
Servo Motors	DC Servo (0.5-3 kg·cm)	3	Joint actuation
Force Sensors	±1-5N range	2	Grasp force monitoring
Proximity Sensors	IR/Ultrasonic	2	Collision avoidance
Encoders	≥10 bits resolution	3	Position feedback
End-Effector	Parallel Gripper	1	Instrument handling
🔧 Technical Specifications
Kinematic Configuration
Type: Serial manipulator with three rotational joints

Configuration: Rz–Ry–Ry (Base–Shoulder–Elbow)

Workspace: Spherical volume with 350-650mm radius

Maximum Speed: 0.05–0.3 m/s

Control Frequency: 100–500 Hz

Mathematical Modeling
cpp
// Forward Kinematics Example
// Rz-Ry-Ry configuration transformation matrices

// Base rotation around Z-axis
R_z(θ₁) = [cosθ₁  -sinθ₁  0;
           sinθ₁   cosθ₁  0;
           0       0      1]

// Shoulder rotation around Y-axis  
R_y(θ₂) = [cosθ₂   0   sinθ₂;
           0       1   0;
           -sinθ₂  0   cosθ₂]

// Elbow rotation around Y-axis
R_y(θ₃) = [cosθ₃   0   sinθ₃;
           0       1   0;
           -sinθ₃  0   cosθ₃]
Physical Dimensions
Component	Length (mm)	Material	Weight (g)
Base	80-150	Aluminum	200-400
Upper Arm	120-250	Aluminum 6061	150-300
Forearm	150-300	Aluminum 6061	100-250
End-Effector	50-120	Stainless Steel	50-100
🚀 Installation & Setup
Hardware Requirements
Microcontroller: Arduino Uno R3

Power Supply: 5V DC, 3A minimum

Tools: Basic mechanical workshop tools

Materials: Aluminum profiles, servo brackets, fasteners

Software Dependencies
cpp
// Required Libraries for Arduino IDE
#include <Servo.h>          // Servo motor control
#include <Wire.h>           // I2C communication
#include <Math.h>           // Mathematical functions
#include <EEPROM.h>         // Parameter storage
Installation Steps
1. Mechanical Assembly
Base Construction

Mount base servo on aluminum plate

Ensure smooth 360° rotation capability

Secure wiring through cable management channels

Arm Assembly

Attach upper arm to base joint

Install shoulder servo with proper alignment

Mount forearm to shoulder joint

Install elbow servo and end-effector

End-Effector Installation

Mount parallel gripper mechanism

Connect gripper servo and force sensors

Calibrate grip force (2-3N maximum)

2. Electronic Integration
cpp
// Pin Configuration for Arduino Uno R3
const int BASE_SERVO_PIN = 9;      // PWM pin for base servo
const int SHOULDER_SERVO_PIN = 10; // PWM pin for shoulder servo  
const int ELBOW_SERVO_PIN = 11;    // PWM pin for elbow servo
const int GRIPPER_SERVO_PIN = 6;   // PWM pin for gripper servo
const int FORCE_SENSOR_PIN = A0;   // Analog pin for force sensor
const int PROXIMITY_SENSOR_PIN = 2;// Digital pin for proximity sensor
3. Software Installation
Arduino IDE Setup

Install Arduino IDE 1.8.x or later

Configure board: Tools → Board → Arduino Uno

Set correct COM port

Upload Control Software

Clone repository: git clone https://github.com/your-repo/robotic-arm-prototype

Open main_control_sketch.ino

Upload to Arduino Uno R3

💻 Usage Instructions
Basic Operation
Power On Sequence

Connect 5V power supply

Wait for initialization complete (LED indicator)

Perform homing sequence automatically

Instrument Delivery Workflow

cpp
// Example movement sequence
void deliverInstrument(float targetX, float targetY, float targetZ) {
  // Calculate inverse kinematics
  JointAngles angles = calculateInverseKinematics(targetX, targetY, targetZ);
  
  // Move to instrument pickup position
  moveToPosition(angles);
  
  // Grasp instrument
  gripperGrasp();
  
  // Deliver to dentist
  moveToDeliveryPosition();
  
  // Release instrument
  gripperRelease();
  
  // Return to home position
  returnToHome();
}
Safety Protocols
Emergency Stop: Physical button interrupts all power

Collision Detection: Proximity sensors halt movement

Force Limiting: Software limits maximum grip force

Joint Limits: Mechanical and software end-stops

🔬 Technical Details
Kinematic Analysis
The manipulator uses Denavit-Hartenberg parameters for forward and inverse kinematics:

Joint	θ	d	a	α
1	θ₁	0	0	-90°
2	θ₂	0	L₁	0
3	θ₃	0	L₂	0
Control Algorithm
cpp
// PID Control Implementation
class JointController {
private:
  float Kp, Ki, Kd;
  float prevError, integral;
  
public:
  JointController(float p, float i, float d) : Kp(p), Ki(i), Kd(d) {}
  
  float compute(float setpoint, float current) {
    float error = setpoint - current;
    integral += error;
    float derivative = error - prevError;
    prevError = error;
    
    return Kp*error + Ki*integral + Kd*derivative;
  }
};
Power Management
Total Power Consumption: ~40W maximum

Servo Motors: 6V, 2A each (12W peak)

Control Electronics: 5V, 500mA

Safety Margin: 15% additional capacity

🛠️ Maintenance
Daily Procedures
Surface cleaning with 70% alcohol

Visual inspection of mechanical components

Verify smooth joint movement

Check electrical connections

Weekly Maintenance
Calibrate joint encoders

Verify force sensor accuracy

Update control parameters if needed

Backup configuration to EEPROM

Troubleshooting
Symptom	Possible Cause	Solution
Jerky movement	Low power supply	Check voltage and current capacity
Position drift	Encoder calibration needed	Perform homing sequence
Overheating motors	Excessive load	Reduce payload or check alignment
📊 Performance Metrics
Metric	Target Value	Actual Performance
Position Accuracy	≤5mm	Under verification
Repeatability	≤5mm	Under verification
Instrument Transfer Time	≤3 seconds	Under verification
Payload Capacity	200g	Under verification
Workspace Coverage	350-650mm radius	Under verification
👥 Team & Contribution
Project Team
Br. Wilberth Alejandro Pérez Loredo (24-40923-3)

Br. Shalom Jireh Rayo Blandon (24-18100-5)

Br. Anthony Isai Aráuz Galán (25-40195-1)

Br. Engel Mohamed Urbina Rivas (25-41338-0)

Br. Luis Miguel Garcia Mondragon (25-41224-7)

Contributing
We welcome contributions! Please see our Contributing Guidelines for details.

<div align="center">
"2025: Efficiency and Quality to Continue in Victories"
Robotics Engineering - National University of Engineering

</div>
