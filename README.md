# Collision-Aware Perception and Braking System

**Prototype_V1**

A real-time multi-sensor collision-awareness prototype built on a Raspberry Pi 5 using camera-based object detection, LiDAR ranging, ROS 2 sensor fusion, closing-speed estimation, time-to-collision (TTC), and a micro-ROS-connected Arduino Nano ESP32 for physical hazard indication.

The current prototype has been physically validated through the hazard-indication stage. Vehicle actuation and controlled RC-car braking are planned future work.

> **Prototype notice:** This project is an experimental engineering demonstrator. It is not a safety-certified automotive braking system and should not be used for safety-critical vehicle control.

---

## System Overview

Prototype_V1 combines camera detections with LiDAR measurements to identify relevant targets, estimate their range and relative motion, calculate TTC when appropriate, and assign a discrete collision-risk state.

```text
Logitech C922 Camera
        │
        ▼
Hailo-accelerated YOLOv8 detection
        │
        ▼
ROS 2 camera/detection bridge
        │
        ├───────────────┐
        │               │
        ▼               ▼
Camera detection     RPLIDAR C1
bearing              /scan
        │               │
        └───────┬───────┘
                ▼
       Target Association
                │
        ┌───────┴────────┐
        ▼                ▼
 Target distance     Target bearing
        │
        ▼
 Closing-speed estimation
        │
        ▼
 Time-to-collision
        │
        ▼
 Fused Hazard Controller
        │
        ▼
     /hazard_level
        │
        ▼
    micro-ROS Agent
        │
        ▼
 Arduino Nano ESP32
        │
        ▼
 GREEN / BLUE / YELLOW / RED
```

---

## Hardware

Prototype_V1 currently uses:

- Raspberry Pi 5
- Hailo-8 AI accelerator
- Logitech C922 USB camera
- SLAMTEC RPLIDAR C1
- Arduino Nano ESP32
- Built-in Nano ESP32 RGB LED for hazard-state feedback

The C922 is operated at:

```text
1280 × 720
30 FPS
MJPEG
```

---

## Software

Core technologies used in Prototype_V1 include:

- Python
- C++
- ROS 2
- micro-ROS
- YOLOv8
- Hailo inference
- OpenCV
- NumPy
- GStreamer
- ZeroMQ
- TF2
- Foxglove
- Arduino

Development and validation were performed with ROS 2 Jazzy on Ubuntu Server 24.04.

---

## Repository Structure

```text
EBD/
├── README.md
├── .gitignore
│
├── firmware/
│   └── nano_esp32_hazard_led/
│       └── hazard_led_subscriber.ino
│
└── ros2_ws/
    └── src/
        ├── camera_detection_bridge/
        │   ├── camera_detection_bridge/
        │   │   ├── __init__.py
        │   │   ├── detection_annotation_bridge.py
        │   │   ├── tcp_image_bridge.py
        │   │   └── zmq_detection_bridge.py
        │   ├── config/
        │   │   └── c922_prototype_v1.yaml
        │   ├── launch/
        │   │   └── prototype_camera.launch.py
        │   ├── resource/
        │   ├── package.xml
        │   ├── setup.cfg
        │   └── setup.py
        │
        └── hazard_controller/
            ├── hazard_controller/
            │   ├── __init__.py
            │   ├── target_association_node.py
            │   └── fusion_hazard_node.py
            ├── resource/
            ├── package.xml
            ├── setup.cfg
            └── setup.py
```

Third-party projects such as the SLAMTEC ROS driver, Hailo runtime/software stack, micro-ROS Agent, and `micro_ros_arduino` are external dependencies and are not vendored into this repository.

---

## Camera Calibration

The Logitech C922 was calibrated at 1280×720 resolution.

Final calibration quality:

```text
RMS reprojection error: 0.251393 px
```

The validated calibration is stored at:

```text
ros2_ws/src/camera_detection_bridge/config/c922_prototype_v1.yaml
```

The ROS package installs this file into its package share directory so the public source does not depend on a machine-specific home-directory path.

---

## Camera-to-LiDAR Association

A fixed camera-to-LiDAR transform was physically validated using left, center, and right target placements.

The current transform from:

```text
laser
→ c922_camera_optical_frame
```

uses:

```text
translation:
x =  0.100 m
y = -0.003 m
z = -0.100 m

rotation:
roll  = -90°
pitch =   0°
yaw   = -90°
```

The target-association node projects camera detections into bearing space and associates them with nearby valid LiDAR measurements.

Current association window:

```text
±2.0°
```

---

## ROS 2 Outputs

### Target Association

`target_association_node` publishes:

```text
/target_valid
    std_msgs/msg/Bool

/target_class
    std_msgs/msg/String

/target_distance
    std_msgs/msg/Float32

/target_bearing
    std_msgs/msg/Float32

/closing_speed
    std_msgs/msg/Float32

/time_to_collision
    std_msgs/msg/Float32
```

### Hazard Fusion

`fusion_hazard_node` publishes:

```text
/hazard_level
    std_msgs/msg/UInt8
```

---

## Closing-Speed Estimation

Closing speed is estimated from the associated target's LiDAR distance history using a rolling linear regression.

The estimator uses the `LaserScan` timestamp as its primary time source.

Sign convention:

```text
positive  = approaching
near zero = stationary
negative  = receding
```

Current parameters:

```text
closing_window_sec   = 0.80
closing_min_samples  = 5
closing_min_span_sec = 0.35
```

Physical stationary testing produced approximately:

```text
-0.003 to +0.003 m/s
```

Approach and receding tests correctly produced positive and negative closing speeds respectively.

---

## Target-Continuity Protection

The current detection pipeline does not provide persistent physical-object tracking IDs.

To prevent the closing-speed estimator from calculating velocity across an obvious same-class target switch, Prototype_V1 includes a lightweight continuity guard.

Current limits:

```text
maximum distance jump = 0.20 m
maximum bearing jump  = 10°
```

When an obvious target switch is detected, the velocity history is reset and closing speed temporarily returns `NaN` while the estimator rebuilds its rolling window.

This behavior was physically tested using two different printed dog targets at different ranges and bearings.

---

## Time-to-Collision

Prototype_V1 uses a simple constant-relative-velocity TTC estimate.

TTC is only produced when:

```text
closing_speed > 0.05 m/s
```

Otherwise TTC is published as:

```text
NaN
```

This intentionally covers:

- stationary targets
- receding targets
- invalid targets
- estimator warm-up periods

Approaching-target tests produced finite positive TTC values as expected.

---

## Hazard States

The current Prototype_V1 hazard interface is:

| Value | State | Meaning |
|---:|---|---|
| `0` | GREEN | No valid relevant target |
| `1` | BLUE | Valid target, no higher hazard condition |
| `2` | YELLOW | Warning condition |
| `3` | RED | Critical condition |

Current provisional indoor thresholds:

```text
YELLOW distance:        <= 1.00 m
RED distance:           <= 0.60 m

meaningful closing:      > 0.05 m/s

YELLOW TTC:             <= 6.0 s
RED TTC:                <= 3.0 s
```

The controller evaluates RED conditions before YELLOW conditions.

Conceptually:

```text
if no valid target:
    GREEN

elif distance <= 0.60 m:
    RED

elif meaningful closing and TTC <= 3.0 s:
    RED

elif distance <= 1.00 m:
    YELLOW

elif meaningful closing and TTC <= 6.0 s:
    YELLOW

else:
    BLUE
```

These are experimental Prototype_V1 thresholds, not automotive safety standards.

---

## Physical Validation

The current normal fused hazard paths have been physically tested.

```text
No valid target
→ GREEN

Valid distant stationary target
→ BLUE

Close stationary target
→ YELLOW from distance

Approaching target above the distance threshold
→ YELLOW from TTC

Very-close stationary target
→ RED from distance

Approaching target above the RED distance threshold
→ RED from TTC
```

The TTC-only validation tests deliberately kept target distance outside the corresponding distance-triggered region so that the TTC decision path could be isolated.

The Nano ESP32 LED endpoint has also been tested with the full:

```text
0 → GREEN
1 → BLUE
2 → YELLOW
3 → RED
```

mapping.

---

## Nano ESP32 Hazard Endpoint

The included Arduino sketch creates the micro-ROS node:

```text
hazard_led_node
```

and subscribes to:

```text
/hazard_level
std_msgs/msg/UInt8
```

The built-in RGB LED maps hazard values to:

```text
0 → GREEN
1 → BLUE
2 → YELLOW
3 → RED
```

The firmware also implements local fail-safe behavior.

It forces RED when:

- the device starts before receiving ROS hazard data
- micro-ROS initialization fails
- an unknown hazard value is received
- hazard updates disappear for more than 1000 ms

The included sketch was physically tested on an Arduino Nano ESP32 using `micro_ros_arduino`.

---

## Controlled Prototype_V1 Validation Configuration

Some current physical tests use a printed dog image as a repeatable controlled target.

During those tests, the detection bridge is temporarily configured as:

```text
allowed_classes = ['dog']
min_confidence  = 0.25
```

and target association is run with:

```text
min_confidence        = 0.25
detection_timeout_sec = 0.35
```

The dog-only filter is a **testing configuration**, not the intended final scope of the perception system.

The detector threshold and broader semantic class configuration will be revisited after the current Prototype_V1 validation sequence is complete.

---

## Building the ROS 2 Packages

From the repository:

```bash
cd ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build   --packages-select   camera_detection_bridge   hazard_controller

source install/setup.bash
```

The sanitized public copy has been independently built successfully with both packages completing.

Expected executables include:

```text
camera_detection_bridge detection_annotation_bridge
camera_detection_bridge tcp_image_bridge
camera_detection_bridge zmq_detection_bridge

hazard_controller target_association_node
hazard_controller fusion_hazard_node
```

---

## Running the First-Party ROS Nodes

The external Hailo/GStreamer inference pipeline must first provide:

```text
C922 MJPEG stream:
tcp://127.0.0.1:5556

Hailo detection metadata:
tcp://127.0.0.1:5555
```

The exact Hailo model and local runtime paths are installation-dependent and are intentionally not hard-coded into this repository.

Start the external RPLIDAR C1 ROS driver:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch   sllidar_ros2   sllidar_c1_launch.py   serial_port:=/dev/rplidar   frame_id:=laser
```

Start the camera/detection subsystem:

```bash
ros2 launch   camera_detection_bridge   prototype_camera.launch.py
```

For the current controlled dog-only validation configuration:

```bash
ros2 param set   /zmq_detection_bridge   allowed_classes   "['dog']"

ros2 param set   /zmq_detection_bridge   min_confidence   0.25
```

Start target association:

```bash
ros2 run   hazard_controller   target_association_node   --ros-args   -p min_confidence:=0.25   -p detection_timeout_sec:=0.35
```

Start hazard fusion:

```bash
ros2 run   hazard_controller   fusion_hazard_node
```

---

## micro-ROS Connection

The Arduino Nano ESP32 communicates with the Raspberry Pi through a micro-ROS Agent over serial.

The tested agent configuration uses:

```text
device: /dev/ttyACM0
baud:   115200
```

Example:

```bash
ros2 run   micro_ros_agent   micro_ros_agent   serial   -D /dev/ttyACM0   -b 115200
```

After starting the Agent, resetting the Nano allows the `hazard_led_node` endpoint to join the ROS graph.

---

## Visualization

Foxglove has been used during development to monitor the camera feed, detections, LiDAR, and hazard pipeline.

Useful Prototype_V1 values include:

```text
/target_distance.data
/closing_speed.data
/time_to_collision.data
/hazard_level.data
/target_valid.data
```

---

## Current Limitations

Prototype_V1 currently has several intentional limitations:

- It is an experimental demonstrator, not a production braking controller.
- The current detector does not provide persistent physical-object tracking IDs.
- TTC assumes approximately constant relative velocity.
- The current hazard thresholds are provisional indoor test values.
- The controlled validation configuration currently uses a dog-only detector filter.
- Full semantic-class validation has not yet been completed.
- Hazard-state hysteresis has not yet been added.
- Vehicle motor/brake actuation has not yet been implemented.
- A camera-independent very-close LiDAR emergency override has not yet been implemented.

---

## Roadmap

Planned next steps include:

1. Add and physically validate a narrow, very-close LiDAR-only emergency RED override.
2. Restore and validate the broader relevant object-class configuration.
3. Re-evaluate the detector confidence threshold using multiple real target classes.
4. Monitor for hazard-state chatter and add hysteresis only if testing shows it is necessary.
5. Move the validated hazard pipeline onto an RC-car platform.
6. Add controlled slowdown and braking actuation.
7. Perform low-speed controlled collision-avoidance experiments.

---

## License

This repository is publicly viewable for portfolio and demonstration purposes.

**No open-source license is currently granted.**

The first-party ROS package manifests identify the software as proprietary. Third-party dependencies retain their respective licenses.
