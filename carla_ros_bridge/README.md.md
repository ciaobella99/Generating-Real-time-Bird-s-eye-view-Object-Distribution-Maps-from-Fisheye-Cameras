# CARLA ROS Bridge – Custom Scripts Usage Guide

本文件說明如何使用本專案中的 **`carla_ros_bridge`** 資料夾，以及如何將其正確整合至 **CARLA Simulator** 與 **ROS carla-ros-bridge** 的工作環境中。

---

## 📦 資料夾放置方式

請將本專案中的 **`carla_ros_bridge`** 資料夾 **整個複製** 到你本機的 `carla-ros-bridge` ROS workspace 之中，放置路徑如下：

```text
carla-ros-bridge/
└── catkin_ws/
    └── src/
        └── carla_ros_bridge/   ← 請將整個資料夾複製到此處
```

完成後，請回到 `catkin_ws` 目錄並重新編譯 ROS workspace：

```bash
cd carla-ros-bridge/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 📂 carla_ros_bridge/scripts 腳本說明

`carla_ros_bridge/scripts` 目錄中包含多個 Python 腳本，主要用於 **Bird’s‑Eye View (BEV) 建構、物件生成、距離估測，以及 Semantic LiDAR Ground Truth 取得**。

```text
carla_ros_bridge/
└── scripts/
    ├── bev.py
    ├── distance_via_service.py
    ├── Get Actor Dimensions.py
    ├── ground_truth_bev.py
    ├── semantic_lidar_gt_listener.py
    ├── semantic_lidar_horizon_gt_listener.py
    ├── spawn_object.py
    ├── spawn_object_with_tf.py
    └── track_object_distance.py
```

### 各腳本功能簡介

- **bev.py**  
  將 CARLA 模擬環境中的感測資料或物件資訊轉換為 Bird’s‑Eye View（BEV）表示，供後續視覺化或空間分析使用。

- **distance_via_service.py**  
  透過 ROS Service 的方式計算目標物件與 ego vehicle 或感測器之間的距離，方便在不同節點中呼叫距離估測功能。

- **Get Actor Dimensions.py**  
  讀取 CARLA Actor（例如車輛、行人）的 bounding box 與實際物理尺寸資訊。

- **ground_truth_bev.py**  
  使用 CARLA 提供的 Ground Truth 資訊生成對應的 BEV Ground Truth 地圖，常用於距離估測或位置誤差評估。

- **semantic_lidar_gt_listener.py**  
  監聽 CARLA Semantic LiDAR topic，擷取並整理語意 LiDAR 的 Ground Truth 資料。

- **semantic_lidar_horizon_gt_listener.py**  
  Semantic LiDAR Ground Truth 的水平（horizon）投影版本，適用於特定 BEV 或感測幾何設定。

- **spawn_object.py**  
  在 CARLA 模擬場景中生成指定的物件（如車輛、行人），用於場景配置與實驗測試。

- **spawn_object_with_tf.py**  
  在生成物件的同時發布對應的 TF 座標轉換，方便與 ROS TF tree 整合使用。

- **track_object_distance.py**  
  持續追蹤目標物件，並即時計算其與 ego vehicle 或感測器之間的距離。

---

## ⚠️ 注意事項

- 本資料夾需搭配 **carla-ros-bridge** 使用，請確認 CARLA 與 ROS 版本相容。
- 部分腳本需先正確啟動 CARLA world、sensor 與 actor，否則可能無法正常運作。
- 若有自訂 topic 或 service 名稱，請依照實際 CARLA / ROS 設定進行修改。

---

## 📌 建議用途

本模組適合用於：
- 多感測器（Camera / LiDAR）BEV 建構實驗
- 物件距離估測與追蹤研究
- Semantic LiDAR Ground Truth 分析
- CARLA + ROS 自動駕駛感知系統原型開發

