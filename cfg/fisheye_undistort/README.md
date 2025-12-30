# fisheye_undistort

This package provides a ROS node for fisheye image undistortion using
pre-calibrated camera intrinsic parameters.

The package is designed to be used within the CARLA–ROS bridge workspace
and serves as a preprocessing module for downstream perception tasks
(e.g., BEV projection, object detection).

---

## Package Location

This package should be placed at the following path:

carla-ros-bridge/catkin_ws/src/fisheye_undistort/

Make sure it is built within the same catkin workspace as the
CARLA ROS bridge.

---

## Directory Structure

fisheye_undistort/
├── CMakeLists.txt                 # Catkin build configuration
├── package.xml                    # ROS package manifest
├── scripts/                       # Optional helper scripts
├── src/
│   └── fisheye_undistort_node.cpp # Main undistortion ROS node
├── launch/
│   └── undistort.launch           # Launch file for the undistortion node
└── config/
    └── params.yaml                # Camera intrinsic parameters

---

## Role in the System

This package is responsible for undistorting raw fisheye camera images
using calibrated intrinsic parameters.

It is typically executed as an early-stage preprocessing node in the
perception pipeline:

Raw Fisheye Image
        ↓
Fisheye Undistortion (this package)
        ↓
BEV Projection / Object Detection / Tracking

---

## Configuration

Camera intrinsic parameters are defined in:

config/params.yaml

The configuration file should include the following fields:

- Camera intrinsic matrix K
- Distortion coefficients D
- Image width
- Image height

Example parameters:

K: [fx, 0, cx,
    0, fy, cy,
    0,  0,  1]

D: [k1, k2, k3, k4]

width: 1920
height: 1280

---

## Build Instructions

Make sure the node source file is executable:

chmod +x src/fisheye_undistort_node.cpp

Then build the package inside the catkin workspace:

cd ~/carla-ros-bridge/catkin_ws
catkin build fisheye_undistort
source devel/setup.bash

---

## Usage

Launch the fisheye undistortion node using the provided launch file:

roslaunch fisheye_undistort undistort.launch

The launch file loads the camera parameters from config/params.yaml
and starts the undistortion node accordingly.

---

## Input and Output Topics

Input:
- Raw fisheye image topic (sensor_msgs/Image)

Output:
- Undistorted image topic (sensor_msgs/Image)

Exact topic names can be configured in the launch file or source code.

---

## Notes

- The undistortion assumes a calibrated fisheye camera model.
- Image quality and geometric accuracy depend on the calibration results.
- This node does not perform image rectification for stereo setups.

---

## Author

Eric Wu
