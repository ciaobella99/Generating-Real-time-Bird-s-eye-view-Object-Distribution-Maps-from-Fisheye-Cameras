# Generating-Real-time-Bird-s-eye-view-Object-Distribution-Maps-from-Fisheye-Cameras
This repository implements an omnidirectional perception system using **three fisheye cameras**. By performing real-time distortion correction, object detection (YOLOv7), and geometric distance estimation, the system generates a unified 360-degree top-down occupancy map. This research provides a high-precision, low-cost sensing alternative for autonomous mobile platforms in indoor environments.

<p align="center">
  <img src="https://via.placeholder.com/800x400.png?text=Place+Your+CARLA+Demo+GIF+Here" width="800">
  <br>
  <em>Figure 1: 360° Object Detection and Grid Map Visualization in CARLA 0.9.13</em>
</p>

---

## 🌟 Key Contributions

* **Cost-Effective Sensing Solution**: Achieves an average localization error of **~11.5 cm** (within a 20m radius). Sensor costs are reduced by **70%–90%** compared to traditional LiDAR or Depth Camera solutions.
* **360° Seamless Perception**: Synchronizes three fisheye streams into a unified coordinate system, eliminating blind spots and providing continuous spatial awareness.
* **Robust High-Fidelity Validation**: Developed and validated within **CARLA 0.9.13**, offering a reproducible framework for testing fisheye camera parameters and localization stability.

---

## 🛠 System Architecture & Modules

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

## 📊 Performance Metrics

| Metric | Performance |
| :--- | :--- |
| **Max Effective Range** | 20 Meters |
| **Avg. Distance Error** | 11.5 cm |
| **Cost Reduction** | 70% - 90% (vs. LiDAR) |
| **Field of View** | 360° Omnidirectional |

---

## 🚀 Getting Started

### Prerequisites
* Ubuntu 20.04
* ROS Noetic
* **CARLA Simulator 0.9.13**
* Python 3.8+ & PyTorch (CUDA supported)
