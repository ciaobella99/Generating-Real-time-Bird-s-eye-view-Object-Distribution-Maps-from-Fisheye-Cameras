#!/usr/bin/env python
import rospy
import numpy as np
from tf.transformations import quaternion_matrix
from carla_msgs.srv import GetActorTransform

# 半車長 (m)，愛用 Tesla Model 3 這裡設 2.35
HALF_LENGTH = 2.35

def get_front_bumper_point(transform):
    """輸入：GetActorTransform.response.transform，
       回傳：該 Transform 下前保桿的世界座標點 [x,y,z]"""
    # 1. 取四元數轉旋轉矩陣
    q = transform.rotation
    R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    # 2. 取中心點向量
    t = np.array([transform.translation.x,
                  transform.translation.y,
                  transform.translation.z])
    # 3. 半車長向量在車體本地座標是 [HALF_LENGTH,0,0]
    #    乘上 R → 世界方向，再加上 t → 世界座標
    return t + R.dot(np.array([HALF_LENGTH, 0.0, 0.0]))

if __name__ == '__main__':
    rospy.init_node('distance_via_service')

    # 等到 /carla/get_actor_transform 服務出現
    rospy.wait_for_service('/carla/get_actor_transform')
    get_tf = rospy.ServiceProxy('/carla/get_actor_transform', GetActorTransform)

    # 要追蹤的物件 ID（碰完 box 後，輸入 166）
    target_id = int(input("請輸入要追蹤的 Actor ID："))

    rate = rospy.Rate(10)  # 每秒 10 次
    while not rospy.is_shutdown():
        try:
            # 第一次呼叫 0 → ego_vehicle 的 transform
            ego_resp  = get_tf(0)
            ego_tf    = ego_resp.transform
            # 第二次呼叫 target_id → box 的 transform
            obj_resp  = get_tf(target_id)
            obj_tf    = obj_resp.transform

            # 算前保桿點
            P_ego = get_front_bumper_point(ego_tf)
            P_obj = get_front_bumper_point(obj_tf)

            # 歐式距離
            dist  = np.linalg.norm(P_obj - P_ego)
            rospy.loginfo(f"[距離] 保桿對保桿 = {dist:.2f} m")

        except Exception as e:
            rospy.logwarn(f"呼叫 /carla/get_actor_transform 失敗：{e}")

        rate.sleep()

