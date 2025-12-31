#!/usr/bin/env python  
import rospy, math
import numpy as np
from sensor_msgs.msg import PointCloud2  
import sensor_msgs.point_cloud2 as pc2  
from carla_msgs.msg import CarlaActorList

# tf2 工具
import tf2_ros
from tf.transformations import quaternion_matrix

#-------------------------------------------------------------------
# 參數：  
# FRONT_BUMPER_OFFSET = 半車長 (m)，Tesla Model 3 約 2.35 m  
# BUMPER_HEIGHT 等、高度濾波仍保留  
#-------------------------------------------------------------------
FRONT_BUMPER_OFFSET = 2.35   # m
BUMPER_HEIGHT       = 0.50
HEIGHT_TOLERANCE    = 0.10
VEHICLE_FRAME       = 'ego_vehicle'

actor_map = {}

def actor_list_cb(msg: CarlaActorList):
    global actor_map
    # 只更新 ID→Actor map，用來在日誌印出 type/rolename
    actor_map = { a.id: a for a in msg.actors }

def lidar_cb(pc2_msg: PointCloud2):
    # （不再用 ego_half_length，直接用常數 FRONT_BUMPER_OFFSET）

    # 取得 LiDAR frame → 車體 frame 的變換
    try:
        trans = tf_buffer.lookup_transform(
            VEHICLE_FRAME,
            pc2_msg.header.frame_id,
            rospy.Time(0),
            rospy.Duration(1.0)
        )
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"TF 查詢失敗: {e}")
        return

    # build 4×4 變換矩陣
    quat = [
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w
    ]
    mat = quaternion_matrix(quat)
    mat[0:3,3] = [
        trans.transform.translation.x,
        trans.transform.translation.y,
        trans.transform.translation.z
    ]

    dists_sensor = {}
    dists_bumper = {}

    # 逐點處理
    for x, y, z, ObjIdx, ObjTag in pc2.read_points(
            pc2_msg,
            field_names=("x","y","z","ObjIdx","ObjTag"),
            skip_nans=True):

        # (1) 原始 LiDAR → 點 的 3D 距離
        dist_sensor = math.sqrt(x*x + y*y + z*z)
        dists_sensor.setdefault(ObjIdx, []).append(dist_sensor)

        # (2) 轉到車體座標系
        p_vehicle = mat.dot(np.array([x, y, z, 1.0]))
        x_v, y_v, z_v = p_vehicle[:3]

        # --- 高度濾波：只保留接近保險桿水平面的點 ---
        if abs(z_v - BUMPER_HEIGHT) > HEIGHT_TOLERANCE:
            continue

        # (3) 計算水平 plane 上「保桿中心 → 回波點」距離
        #    車體中心到保桿的半車長 = ego_half_length
        dx = x_v - FRONT_BUMPER_OFFSET
        dy = y_v
        dist = math.hypot(dx, dy)
        dists_bumper.setdefault(ObjIdx, []).append(dist)

    # (4) 輸出：只對那些有保桿回波的物件計算最小值
    for obj_id, arr in dists_bumper.items():
        min_sensor = min(dists_sensor.get(obj_id, [float('nan')]))
        min_bumper = min(arr)

        actor = actor_map.get(obj_id)
        if actor:
            rospy.loginfo(
                f"[GT] ID={obj_id:3d} "
                f"型號={actor.type:30s} "
                f"角色={actor.rolename:10s} "
                f"LiDAR距離={min_sensor:.2f}m  "
                f"保桿距離={min_bumper:.2f}m"
            )
        else:
            rospy.loginfo(
                f"[GT] ID={obj_id:3d} 物件=未知 "
                f"LiDAR距離={min_sensor:.2f}m  "
                f"保桿距離={min_bumper:.2f}m"
            )

if __name__ == '__main__':
    rospy.init_node('semantic_lidar_horizon_gt_listener')

    # 啟動 tf2 listener
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 訂閱演員列表 & Semantic LiDAR
    rospy.Subscriber('/carla/actor_list', CarlaActorList, actor_list_cb)
    rospy.Subscriber('/carla/ego_vehicle/semantic_lidar', PointCloud2, lidar_cb)

    rospy.spin()
