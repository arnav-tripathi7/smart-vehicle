# 🚗 Smart Vehicle

### Webots R2025a • Python • IIT Guwahati

A perception-driven autonomous vehicle controller featuring computer-vision lane following, LiDAR-based collision avoidance, Time-To-Collision (TTC) emergency braking, autonomous curve handling, manual drive modes, and live dashboard visualization in a simulated urban traffic environment.

---

## Overview

Smart Vehicle is a complete autonomous driving controller developed in Python for the Webots robotics simulator.

The controller integrates:

- Camera-based lane perception
- PID-controlled steering
- LiDAR-based obstacle detection
- TTC emergency braking
- Curve-aware speed regulation
- Autonomous and manual driving modes
- Multi-gear operation (Autonomous / Drive / Reverse)
- Real-time dashboard visualization
- GPS telemetry monitoring

The objective is to create a robust urban driving system capable of maintaining lane discipline, navigating intersections, adapting to curves, and safely reacting to dynamic obstacles.

---

## 📽️ Demo

### Autodrive — Lane Following
https://github.com/arnav-tripathi7/smart-vehicle/blob/main/media/demo_autodrive1.mp4
### Emergency Braking
media/demo_emergency_brake.mp4

### Manual Drive — Gear Switching
media/demo_manual.mp4

### Live Speedometer
Speedometer

### Onboard Camera Feed — Lane Detection
Camera Feed

---

## 🧠 Features

### Autonomous Driving (Gear A)

- Vision-based lane following using onboard camera data
- PID controller (Kp = 0.25, Ki = 0.006, Kd = 2) for steering control
- 3-sample moving average filtering for noise suppression
- Speed-adaptive steering damping at high speeds
- Automatic speed reduction on sharp curves
- Smooth recovery to cruising speed after curve exit
- Robust lane-loss recovery at intersections
- Steering-hold behaviour during temporary lane disappearance
- LiDAR-assisted obstacle-aware steering corrections
- Automatic restoration of user-defined cruising speed

### Collision Avoidance & Emergency Braking

- LiDAR-based front obstacle detection
- Time-To-Collision (TTC) based braking logic
- Progressive speed reduction in caution zones
- Full emergency braking with immediate cruise override
- Automatic speed restoration after obstacle clearance
- Dynamic braking distance scaling with vehicle speed

| Speed | Caution Zone | Emergency Brake |
|--------|-------------|----------------|
| 20 km/h | ~33 m | ~17 m |
| 50 km/h | ~83 m | ~42 m |
| 80 km/h | ~133 m | ~67 m |

> Note: Actual intervention distance depends on the configured LiDAR maximum range.

### Manual Driving (Gear D / R)

- Smooth acceleration and deceleration
- Progressive steering control
- Self-centering steering wheel behaviour
- Dedicated reverse gear support
- Manual brake override
- Seamless switching between autonomous and manual modes

### Live Dashboard

- Real-time speedometer rendering
- GPS coordinate display
- Speed telemetry overlay
- Reverse-speed support on dashboard gauge

---

## 🔍 Perception System

The controller fuses information from multiple sensors:

| Sensor | Purpose |
|----------|----------|
| Camera | Lane detection and tracking |
| LiDAR | Obstacle detection and collision avoidance |
| GPS | Speed and position telemetry |
| Display | Dashboard visualization |

This enables simultaneous lane tracking, obstacle detection, emergency braking, and autonomous navigation.

---

## 🏗️ System Architecture

System Architecture

---

## ⚙️ Control Strategy

### Perception Layer

- Camera lane extraction
- LiDAR obstacle detection
- GPS telemetry acquisition

### Decision Layer

- Lane tracking
- Curve detection
- TTC evaluation
- Gear state management

### Control Layer

- PID steering
- Speed ramping
- Brake control
- Steering smoothing

### Vehicle Interface Layer

- Webots Driver API

---

## 🗂️ Repository Structure
smart-vehicle/
│
├── assets/
│   └── speedometer.png
│
├── controllers/
│   ├── autonomous_vehicle/
│   │   ├── Makefile
│   │   ├── autonomous_vehicle
│   │   └── autonomous_vehicle.c
│   │
│   └── smart_vehicle_controller/
│       └── smart_vehicle_controller.py
│
├── docs/
│   ├── emergency_braking_pipeline.png
│   ├── lane_detection_pipeline.png
│   └── system_architecture.png
│
├── media/
│   ├── demo_autodrive1.mp4
│   ├── demo_camera_view.gif
│   ├── demo_emergency_brake.mp4
│   ├── demo_manual.mp4
│   └── demo_speedometer.gif
│
├── worlds/
│   └── city_traffic.wbt
│
├── .gitignore
├── LICENSE
├── README.md
└── requirements.txt

---

## ⚙️ Tunable Parameters

| Parameter | Default | Description |
|---|---|---|
| KP / KI / KD | 0.25 / 0.006 / 2 | PID gains |
| SPEED_KEY_STEP | 1.0 km/h | Manual speed increment |
| SPEED_RAMP | 1.0 km/h/tick | Speed smoothing |
| STEER_KEY_STEP | 0.005 rad | Steering increment |
| STEER_RAMP | 0.005 rad/tick | Steering smoothing |
| CURVE_THRESHOLD | 0.06 rad | Curve detection threshold |
| CURVE_SPEED_MAX | 25 km/h | Maximum curve speed |
| CURVE_STEER_RAMP | 0.02 rad/tick | Curve steering response |
| SPEED_STEER_DAMP | 40 km/h | Steering damping reference |
| TTC_EMERGENCY | 3.0 s | Emergency braking threshold |
| TTC_CAUTION | 6.0 s | Caution threshold |
| FRONT_HALF_AREA | 5 beams | LiDAR scan half-width |

---

## 🎮 Controls

| Key | Action |
|------|---------|
| A | Enable autonomous driving |
| D | Drive gear |
| R | Reverse gear |
| ↑ | Accelerate |
| ↓ | Decelerate |
| ← / → | Steering |
| SPACE | Brake |

---

## 📊 Performance Characteristics

| Metric | Value |
|----------|----------|
| Control Frequency | 20 Hz |
| Simulation Timestep | 50 ms |
| Camera Processing | Every cycle |
| LiDAR Scan Rate | Every cycle |
| Steering Update Rate | 20 Hz |
| Filter Window Size | 3 samples |
| Maximum Forward Speed | 120 km/h |
| Maximum Reverse Speed | 30 km/h |

---

## 🔧 Setup & Requirements

### Software

- Webots R2025a
- Python 3.10+
- Python standard library
- Webots vehicle and controller APIs

### Running the Controller

1. Open worlds/city_traffic.wbt
2. Select the vehicle node
3. Set the controller to:

text smart_vehicle_controller 

4. Start the simulation
5. Click inside the Webots window
6. Use the keyboard controls listed above

---

## 🚨 LiDAR Configuration

Recommended configuration:

Lidar {
  translation           2.0 0.5 0
  maxRange              50
  numberOfLayers        1
  horizontalResolution  512
  fieldOfView           1.57
  near                  0.01
}

> The LiDAR should be positioned outside the vehicle chassis to prevent self-occlusion.

---

## 🔄 Lane Following Pipeline

Lane Detection Pipeline

### Processing Flow
Camera Frame
      │
      ▼
Yellow Pixel Detection
      │
      ▼
Centroid Computation
      │
      ▼
Moving Average Filter
      │
      ▼
PID Controller
      │
      ▼
Speed-Adaptive Damping
      │
      ▼
Steering Ramp
      │
      ▼
Vehicle Steering Command
---

## 🛑 Emergency Braking Pipeline

Emergency Braking Pipeline

### Processing Flow
LiDAR Scan
      │
      ▼
Closest Obstacle Distance
      │
      ▼
TTC Computation
      │
      ▼
Emergency?
 ┌────┴────┐
 │         │
Yes       No
 │         │
 ▼         ▼
Full      Caution
Brake     Slowdown
 │         │
 ▼         ▼
Vehicle Safety Response

---

## 🚀 Future Improvements

- Traffic sign recognition
- Traffic light detection
- Adaptive Cruise Control (ACC)
- Multi-lane navigation
- Dynamic obstacle tracking
- ROS 2 integration
- Global path planning
- Model Predictive Control (MPC)

---

## 🛠️ Technical Highlights

- Python
- Computer Vision
- LiDAR Processing
- PID Control
- Autonomous Navigation
- Real-Time Control Systems
- Vehicle Dynamics & Control
- Collision Avoidance
- Sensor Fusion
- Robotics Simulation
- Webots

---

## 👤 Author

Arnav Tripathi  
B.Tech, Civil Engineering  
Indian Institute of Technology Guwahati

📧 arnavt@iitg.ac.in

---

## 📄 License

This project is released under the MIT License.

The Webots simulator is developed by Cyberbotics and distributed under the Apache 2.0 Lic
