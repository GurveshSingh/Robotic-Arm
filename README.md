# Robotic Arm with Visual Servoing

A ROS 2-based robotic arm system featuring real-time visual servoing using YOLO object detection and a custom cycloidal drive gearbox. Built on Ubuntu 22.04 with ROS Humble.

NOTE- This project is still in progress YOLO node has been Integrated but is highly unoptimised





---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Waveshare ESP32 |
| Actuator | Waveshare ST3215 Serial Bus Servo |
| Gearbox | Custom 3D-printed Cycloidal Drive (20:1) |
| Sensor | Intel RealSense D455 RGB-D Camera |

---

## Repository Structure
```
├── my_robot_description       # URDF, RViz profile, ros2_control joint interface
├── my_robot_bringup           # Launch files, controller configuration
├── my_robot_hardware          # C++ hardware interface (.cpp / .hpp)
├── my_robot_moveit_config     # MoveIt2 config (SRDF, kinematics plugin, etc.)
└── my_robot_commander         # Visual servoing commander node
```
---

## System Architecture
D455 Camera
│
▼
YOLO Node
(detects object, computes pixel error)
│
▼
Commander Node
(joint-space visual servoing)
│
▼
Joint Trajectory Controller (ros2_control)
│
▼
Hardware Interface (C++)
│
▼
ESP32 Firmware (Serial)
│
▼
ST3215 Servo → Cycloidal Gearbox → Arm Output
│
▼ (state feedback)
Joint State Publisher → Robot State Publisher

### Pipeline Description

1. **YOLO Node** — subscribes to the D455 color image stream, detects the target object, and computes the bounding box offset from the camera center. Publishes normalized pixel error to `/visual_error`.
2. **Commander Node** — subscribes to `/visual_error` and performs joint-space visual servoing, directly incrementing yaw and pitch joints proportional to pixel error. Publishes to the joint trajectory controller.
3. **Joint Trajectory Controller** — receives joint goals via `ros2_control` and forwards to the hardware interface.
4. **Hardware Interface** — translates joint positions into the ST3215 protocol and communicates with the ESP32 over serial.
5. **ESP32 Firmware** — drives the servo to the commanded position and streams state feedback back to the hardware interface.
6. **State Publisher** — hardware interface publishes joint states → `joint_state_publisher` → `robot_state_publisher`.

---

## Firmware

The `Firmware/` directory contains the following:

| File | Purpose |
|---|---|
| `main` | Primary firmware — load this onto the ESP32 for full motor control and feedback |
| Others | Troubleshooting utilities: set motor IDs, configure motor mode |

### Motor Mode Configuration

The ST3215 supports four operating modes. **Mode 3 must be set before use.**

| Mode | Description |
|---|---|
| 0 | Servo mode — absolute feedback, limited to 360° rotation |
| 1 | Wheel mode (velocity control) |
| 2 | Wheel mode variant |
| **3** | **Stepper mode — continuous rotation with relative feedback ✓** |

Mode 3 is required for continuous rotation and correct relative position feedback used by the control pipeline.

---

## Software Dependencies

- [ROS 2 Humble](https://docs.ros.org/en/humble/) on Ubuntu 22.04
- [ros2_control](https://control.ros.org/)
- [MoveIt 2](https://moveit.ros.org/)
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [Intel RealSense ROS 2](https://github.com/IntelRealSense/realsense-ros)

---

## Build & Run

```bash
# Clone and build
cd ~/ros2_ws/src
git clone git@github.com:GurveshSingh/Robotic-Arm.git
cd ~/ros2_ws
colcon build 
source install/setup.bash

# Launch
robot startup: ros2 launch my_robot_bringup my_robot.launch
commander startup: ros2 launch my_robot_commander commander_launch.py
Yolo startup: ros2 run yolo_tracker yolo_node
Camera startup: ros2 launch realsense2_camera rs_launch.py

```

---

## Future Work

- [ ] Additional DOF for independent camera pitch control
- [ ] Low-latency visual servoing with YOLO optimization (TensorRT / INT8)
- [ ] Full Gazebo simulation with ros2_control integration
- [ ] IMU / flex sensor based human arm motion tracking input
- [ ] Reinforcement learning based pose goal generation
- [ ] Gripper end effector integration
- [ ] Robotic hand controlled by human hand motion capture
