# Visual SLAM hardware-ready plan

## Target architecture

```text
RGB/RGB-D + CameraInfo + optional IMU + wheel odometry
    -> C++ Visual SLAM adapter
    -> LocalizationEstimate v1
    -> TF/localization quality bridge
    -> Python shoe perception and task planner
    -> versioned base/arm intent
    -> C++ safety controller
    -> TurtleBot3 and dual SO-101 hardware
```

Visual SLAM estimates where the robot is. RGB-D shoe perception estimates where
the shoe is. They are separate modules and are combined through calibrated frame
transforms.

## Replay-first requirements

Before opening a camera, motor bus or live ROS graph, the following scenarios
must run from deterministic fake or recorded input:

- normal tracking and loop traversal;
- initialization and map maturity;
- low texture and insufficient features;
- motion blur and rapid rotation;
- temporary camera occlusion by the arms or a person;
- dropped, duplicated and out-of-order frames;
- RGB/depth/IMU timestamp skew;
- missing or wrong TF/calibration identity;
- recently-lost and lost tracking;
- relocalization pose jump;
- map/session restart and reset generation change;
- laptop process or network loss.

Each failure must propagate to a bounded base/arm hold or stop without using
simulator truth as a fallback.

## Hardware connection should only add measured profiles

The implementation is considered hardware-ready when core behavior has no
hardware-only TODO and the remaining values are external profiles:

- camera device path and serial selection;
- RGB/depth intrinsics and alignment;
- camera-to-base and IMU-to-camera extrinsics;
- sensor timestamp offset and IMU noise parameters;
- wheel radius, track width and odometry scale;
- Raspberry Pi/host IP and ROS domain configuration;
- E-stop confirmation and low-speed commissioning limits.

The ORB-SLAM3 paper is a system architecture and algorithm reference. Its
reported timing on an Intel Core i7-7700 does not establish Raspberry Pi
performance, and its RGB-D support does not by itself establish a tightly
coupled RGB-D+IMU configuration for the selected hardware.
