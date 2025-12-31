#!/usr/bin/env python  
import rospy, math  
from sensor_msgs.msg import PointCloud2  
import sensor_msgs.point_cloud2 as pc2  
from carla_msgs.msg import CarlaActorList  

# 存 ID→ActorInfo 映射
actor_map = {}  

def actor_list_cb(msg: CarlaActorList):
    global actor_map
    actor_map = { actor.id: actor for actor in msg.actors }

def lidar_cb(pc2_msg: PointCloud2):
    # 1) 分組累積每點距離
    dists = {}
    for x, y, z, ObjIdx, ObjTag in pc2.read_points(
            pc2_msg,
            field_names=("x","y","z","ObjIdx","ObjTag"),
            skip_nans=True):
        dist = math.sqrt(x*x + y*y + z*z)
        dists.setdefault(ObjIdx, []).append(dist)

    # 2) 取每個物件最小距離，並從 actor_map 取出 type/rolename
    for obj_id, dist_list in dists.items():
        gt = min(dist_list)
        actor = actor_map.get(obj_id)
        if actor:
            # 改用 actor.type 跟 actor.rolename
            model = actor.type
            role  = actor.rolename
            rospy.loginfo(
                f"[GT] ID={obj_id:3d} "
                f"型號={model:30s} "
                f"角色={role:10s} "
                f"距離={gt:.2f} m"
            )
        else:
            rospy.loginfo(
                f"[GT] ID={obj_id:3d} 物件=未知 距離={gt:.2f} m"
            )

if __name__ == '__main__':
    rospy.init_node('semantic_lidar_gt_listener')

    # 訂閱 ActorList，保持最新的 ID→ActorInfo
    rospy.Subscriber('/carla/actor_list',
                     CarlaActorList,
                     actor_list_cb)

    # 訂閱 Semantic Lidar（PointCloud2）
    rospy.Subscriber('/carla/ego_vehicle/semantic_lidar',
                     PointCloud2,
                     lidar_cb)

    rospy.spin()
