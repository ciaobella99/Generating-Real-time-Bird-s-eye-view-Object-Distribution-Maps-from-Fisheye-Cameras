# Generating-Real-time-Bird-s-eye-view-Object-Distribution-Maps-from-Fisheye-Cameras
This repository implements an omnidirectional perception system using **three fisheye cameras**. By performing real-time distortion correction, object detection (YOLOv7), and geometric distance estimation, the system generates a unified 360-degree top-down occupancy map. This research provides a high-precision, low-cost sensing alternative for autonomous mobile platforms in indoor environments.

<p align="center">
  <img width="1964" height="1296" alt="Screenshot from 2025-06-30 16-24-42" src="https://github.com/user-attachments/assets/9279e49f-aa7a-4886-a650-28f43cd52902" />

  <br>
  <em>Figure 1: 360° Object Detection and Grid Map Visualization in CARLA 0.9.13</em>
</p>

---

## I.Key Contributions

* **Cost-Effective Sensing Solution**: Achieves an average localization error of **~11.5 cm** (within a 20m radius). Sensor costs are reduced by **70%–90%** compared to traditional LiDAR or Depth Camera solutions.
* **360° Seamless Perception**: Synchronizes three fisheye streams into a unified coordinate system, eliminating blind spots and providing continuous spatial awareness.
* **Robust High-Fidelity Validation**: Developed and validated within **CARLA 0.9.13**, offering a reproducible framework for testing fisheye camera parameters and localization stability.

---

## II. System Architecture & Modules

The system is integrated using **ROS1 (Robot Operating System)** to manage real-time data streaming and inter-process communication.

### 1. Simulation & Image Processing
* **Simulation Environment**: Built using **CARLA 0.9.13**. We utilize the **Kannala–Brandt polynomial model** to simulate virtual fisheye optics and perform undistortion.
* **Real-world Adaptation**: Uses **IMX219 fisheye lenses (190° FOV)**. Camera intrinsics/extrinsics are calibrated using the **Kalibr** toolbox for precise model alignment.

### 2. Object Detection & Tracking
* **Detection Core**: Leverages **YOLOv7** for robust, real-time multi-class object detection.
* **Temporal Tracking**: Incorporates **DeepSORT** to maintain object identities across frames, providing stable spatial trajectory data.

### 3. Distance Estimation & Mapping
* Implements **geometric projection models** to estimate distances from 2D bounding boxes.
* Generates a real-time **Grid Map** (top-down view) to visualize object distribution and facilitate error analysis.

---

## III. Performance Metrics

| Metric | Performance |
| :--- | :--- |
| **Max Effective Range** | 20 Meters |
| **Avg. Distance Error** | 11.5 cm |
| **Cost Reduction** | 70% - 90% (vs. LiDAR) |
| **Field of View** | 360° Omnidirectional |

* **P.S.**:Avg. Distance Error refers to the average error calculated based on the target object’s distance from the camera at different angles and distances.

---

## IV Getting Started

### Prerequisites
* **GPU**: NVIDIA GeForce **GTX 4090** (24GB VRAM)
* Ubuntu 20.04
* ROS Noetic
* **CARLA Simulator 0.9.13**
* Python 3.8+ & PyTorch (CUDA supported)

### Third-Party References

This project builds upon and references the following open-source resources:

- **YOLOv7 + DeepSORT Integration**  
  Repository: https://github.com/MuhammadMoinFaisal/YOLOv7-DeepSORT-Object-Tracking  
  Description: An open-source implementation integrating YOLOv7 for object detection
  and DeepSORT for multi-object tracking.

- **CARLA Fisheye Camera Support (Official Patch)**  
  Repository: https://github.com/carla-simulator/carla/pull/3755  
  Description: Official CARLA simulator pull request enabling fisheye camera sensor
  support.

The tracking and perception pipeline in this work is adapted and extended for
fisheye-based BEV projection and distance estimation.

### Installation

### 1. Clone the repository
```bash
git clone [https://github.com/ciaobella99/Generating-Real-time-Bird-s-eye-view-Object-Distribution-Maps-from-Fisheye-Cameras.git](https://github.com/ciaobella99/Generating-Real-time-Bird-s-eye-view-Object-Distribution-Maps-from-Fisheye-Cameras.git)
```
### 2. Install Python dependencies (ensure CARLA version matches)
```bash
pip install carla==0.9.13
pip install -r requirements.txt
```

### 3. Build ROS workspace
```bash
catkin_make
source devel/setup.bash
```

### 4. Apply third-party modules

This project relies on the following third-party components, which must be set up separately:

**(1) YOLOv7 + DeepSORT Integration**

Download the repository from:
https://github.com/MuhammadMoinFaisal/YOLOv7-DeepSORT-Object-Tracking

Follow the instructions provided in the official repository to configure the
detection and tracking pipeline.

**(2) CARLA Fisheye Camera Support**

Apply the official CARLA fisheye camera patch according to the following pull request:
https://github.com/carla-simulator/carla/pull/3755

Please follow the modification details and build instructions described in the
pull request to enable fisheye camera sensors in CARLA.


