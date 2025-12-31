#!/usr/bin/env python3
"""
track_object_distance.py
ROS 節點：訂閱 /carla/ego_vehicle/odometry 以獲取控制車輛的位置與朝向，
並使用 CARLA API 查詢目標 Actor 座標，計算並持續輸出：
  - 兩者之間的歐氏距離
  - 相對於 ego_vehicle 車頭方向的水平面角度（0°=車頭前方，90°右側，180°後方，270°左側）
"""
import rospy
from nav_msgs.msg import Odometry
import carla
import math
import re
import sys
from tf.transformations import euler_from_quaternion

# 全域變數，用於儲存最新的 ego_vehicle 位置與 yaw
ego_position = None
ego_yaw = 0.0

def odometry_callback(msg):
    """Odometry 訂閱回調: 更新 ego_vehicle 的位置與車頭朝向 (yaw)。"""
    global ego_position, ego_yaw
    ego_position = msg.pose.pose.position

    # 解析四元數到 Euler，取 yaw
    q = msg.pose.pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    _, _, ego_yaw = euler_from_quaternion(quat)

def main():
    rospy.init_node('track_object_distance', anonymous=True)
    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, odometry_callback)

    rate = rospy.Rate(10)  # 10 Hz
    rospy.loginfo("等待 /carla/ego_vehicle/odometry 中的第一筆資料…")
    while ego_position is None and not rospy.is_shutdown():
        rate.sleep()
    if rospy.is_shutdown():
        return
    rospy.loginfo(f"已接收 Ego 初始位置：x={ego_position.x:.2f}, y={ego_position.y:.2f}, z={ego_position.z:.2f}")

    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # 1. 一次輸入多個 Actor ID（逗號或空格分隔）
    try:
        ids_str    = input("請輸入目標 Actor ID (可用逗號或空格分隔多個ID): ").strip()
        target_ids = [int(x) for x in re.split(r'[,\\s]+', ids_str) if x]
    except ValueError:
        rospy.logerr("輸入錯誤：請輸入整數 ID，並以逗號或空格分隔。")
        sys.exit(1)

    # 2. 預先抓好所有 actor，找不到的以 None 代替
    target_actors = {}
    for tid in target_ids:
        actor = world.get_actor(tid)
        if actor:
            rospy.loginfo(f"加入追蹤目標 Actor {tid}")
        else:
            rospy.logwarn(f"找不到 Actor ID={tid}, 將使用 (0,0,0)")
        target_actors[tid] = actor
    

    try:
        while not rospy.is_shutdown():
            ex, ey, ez = ego_position.x, ego_position.y, ego_position.z

            # 3. 對每個目標一起計算並輸出
            infos = []
            for tid in target_ids:
                actor = target_actors[tid]
                if actor:
                    loc = actor.get_transform().location
                    tx, ty, tz = loc.x, loc.y, loc.z
                else:
                    tx, ty, tz = 0.0, 0.0, 0.0
                dx, dy, dz = tx - ex, ty - (-ey), tz - ez   #這裡的y座標怪怪的
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                bearing = math.atan2(dy, dx)
                rel_ang = bearing - ego_yaw
                angle_deg = (math.degrees(rel_ang) + 360) % 360
                
                # 依 blueprint id 決定類別文字
                if actor:
                    bp = actor.type_id  # e.g. "walker.pedestrian.0019"
                    if "pedestrian" in bp:
                        cls = "person"
                    elif "crossbike" in bp or "bicycle" in bp:
                        cls = "bicycle"
                    elif "motorcycle" in bp or "vespa" in bp:
                        cls = "motorcycle"
                    else:
                        cls = "vehicle"
                else:
                    cls = "unknown"
                infos.append(f"{cls} Actor({tid})={dist:.2f}m,{angle_deg:.1f}°")

            rospy.loginfo("[距離追蹤] " + "  ".join(infos))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

