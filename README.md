# Interceptor Drone — Autonomous Moving Target Detection & Interception

An autonomous drone simulation that detects, tracks, and physically intercepts moving aerial targets using ArduPilot SITL, Python, MAVLink, and OpenCV.

## Demo Features

- Virtual Camera View with bounding box, corner targeting markers, dotted tracking line
- Phase System: SCANNING → TARGET LOCKED → CHASING → CLOSING → INTERCEPT ZONE → INTERCEPTED
- RE-SCANNING: Auto hover and re-acquire if target moves out of frame
- Mission Planner Integration: Live distance, speed, status messages every second
- Manual Target Control: WASD keys to move target in real-time

## Tech Stack

- Flight Simulation: ArduPilot SITL
- Ground Station: Mission Planner
- Drone Control: Python + pymavlink
- Camera View: OpenCV
- Object Detection (planned): YOLOv8-nano
- Companion Computer (planned): Jetson Nano
- Flight Controller (planned): SpeedyBee F405

## Hardware In Progress

- Custom X-frame quadcopter designed in SolidWorks
- Motors: 2205 1900KV
- Battery: 4S LiPo
- FC: SpeedyBee F405
- Companion: Jetson Nano

## Run Simulation

Terminal 1 - Start SITL:
sim_vehicle.py -v ArduCopter --console --map --out=127.0.0.1:14551

Terminal 2 - MAVProxy:
mode guided
arm throttle

Terminal 3 - Run:
python3 interceptor_camera.py

## Files

- interceptor_camera.py: Main script with camera view and full intercept logic
- interceptor.py: Basic GPS-based target chase
- drone_control.py: Basic drone arm/takeoff/land control

## Future Work

- Real hardware assembly
- YOLOv8-nano on Jetson Nano
- Real camera feed
- Proportional Navigation guidance
- Multi-target tracking

## Author

Durgesh - Aerospace and Drone Enthusiast
GitHub: https://github.com/durgesh2302
