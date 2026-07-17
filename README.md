# UAV Failure Detection & Autonomous Response System

Real-time anomaly detection and autonomous failsafe response for a simulated UAV, built on ROS2, PX4 SITL, and Gazebo. Combines rule-based detection for fast, explainable checks with a trained ML model for subtler failure patterns that rules can't express.

## Overview

Extends a static UAV failure-prognosis project into a live control loop: telemetry is streamed in real time from a PX4-controlled quadcopter, evaluated by two parallel anomaly detectors, and fed into a response node that autonomously takes over via offboard control when a fault is detected.



## Architecture

```
PX4 SITL (Gazebo) --> Fault Injector --> Rule-based Detector --> Response Node
                                      --> ML Detector (IsolationForest) --^
```

## Components

- **`fault_injector.py`** — Custom ROS2-level fault injection framework. Built after discovering that PX4's built-in failure injection (`failure gps off`) doesn't actually affect sensor topics on Gazebo Harmonic (a known upstream issue). Supports `low_sats`, `frozen`, and `dropout` fault modes, switchable live via ROS2 parameters.
- **`anomaly_detector.py`** — Rule-based detector: satellite count thresholds, GPS/topic staleness, low battery, and implausible position jumps.
- **`ml_anomaly_detector.py`** — Isolation Forest model trained on log-scaled GPS position variance, specifically catching "frozen" GPS sensors (stuck repeating stale values) — a failure mode the rule-based detector cannot express.
- **`response_node.py`** — Subscribes to detector output; on a detected anomaly, autonomously engages PX4 offboard control mode and holds the vehicle's last known position.
- **`telemetry_logger.py` / `data_collector.py`** — Real-time telemetry logging for monitoring and ML training data collection.
- **`train_model.py`** — Offline training/evaluation script for the Isolation Forest model.

## Key Result

The rule-based detector fails to catch a "frozen" GPS sensor (satellite count and topic rate look normal, but the reported position silently stops updating). After identifying this gap through testing, an Isolation Forest model was trained on log-scaled rolling-window GPS variance:

- **Before log-transform:** model failed to separate frozen vs. normal (near-identical anomaly scores)
- **After log-transform:** 100% detection during the sustained frozen-fault window, 0% false positives on clean baseline flight

## Setup

Requires ROS2 Humble, PX4-Autopilot (SITL), Gazebo Harmonic, and QGroundControl. See [PX4 ROS2 setup guide](https://docs.px4.io/main/en/ros2/user_guide.html) for base environment setup.

```bash
cd ~/px4_ros_ws
colcon build --packages-select uav_monitor
source install/setup.bash

# Terminal 1: fault injector
ros2 run uav_monitor fault_injector

# Terminal 2: rule-based detector
ros2 run uav_monitor anomaly_detector

# Terminal 3: ML detector
ros2 run uav_monitor ml_anomaly_detector

# Terminal 4: response node
ros2 run uav_monitor response_node
```

Trigger a fault:
```bash
ros2 param set /fault_injector fault_mode frozen
```
