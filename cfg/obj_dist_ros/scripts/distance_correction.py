#!/usr/bin/env python3
"""
distance_correction.py

自動化測試：
  - 在 CARLA 中於指定座標 spawn actor
  - 訂閱 dis_test.py 發布之距離與角度估測 (/bev_measure/front)
  - 訂閱 ego vehicle odometry (/carla/ego_vehicle/odometry)
  - 計算 ground truth 距離與相對角度
  - 寫入 CSV，包括測量值、真值與誤差
  - 刪除 actor

使用方式：
  chmod +x distance_correction.py
  rosrun obj_dist_ros distance_correction.py
"""
import carla
import rospy
import csv
import time
import math
import json
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# 固定 EGO VEHICLE 位置與朝向
EGO_X, EGO_Y, EGO_YAW = -126.7, 8.5, 180.0
# Spawn 位置列表 [(x, y), ...]
SPAWN_POS = [(-126.7, -10.5)]
# CSV 輸出檔名
CSV_FILE = 'calibration.csv'
# 等待測量 timeout (秒)
TIMEOUT = 10.0
# 標高
SPAWN_Z = 1.0
SPAWN_YAW   = 0.0
SPAWN_PITCH = 0.0
SPAWN_ROLL  = 0.0
# Actor blueprint ID
ACTOR_BP = 'walker.pedestrian.0009'

# 全域暫存
meas_dist = None
meas_angle = None
ego_x = EGO_X
ego_y = EGO_Y
ego_yaw = EGO_YAW


def meas_cb(msg):
    global meas_dist, meas_angle
    try:
        m = json.loads(msg.data)
        meas_dist  = float(m['distances'][0]) / 100.0  # cm -> m
        meas_angle = float(m['angles'][0])
    except Exception as e:
        rospy.logwarn(f"解析估測值失敗: {e}")


def odo_cb(msg):
    global ego_x, ego_y, ego_yaw
    p = msg.pose.pose.position
    ego_x, ego_y = p.x, p.y
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    ego_yaw = math.degrees(yaw)


def spawn_actor(world, blueprint, x, y, z=SPAWN_Z, yaw=EGO_YAW):
    tf = carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0)
    )
    actor = world.try_spawn_actor(blueprint, tf)
    return actor


def get_ground_truth(world, actor_id):
    actor = world.get_actor(actor_id)
    loc = actor.get_transform().location
    dx, dy = loc.x - ego_x, loc.y - ego_y
    dist = math.hypot(dx, dy)
    angle = (math.degrees(math.atan2(dy, dx)) - ego_yaw + 360) % 360
    return dist, angle


def measure_once(world, blueprint, writer):
    global meas_dist, meas_angle
    x, y = SPAWN_POS[0]
    z = SPAWN_Z
    actor = spawn_actor(
        world, blueprint,
        x, y,
        z=z,
        yaw=SPAWN_YAW,
        pitch=SPAWN_PITCH,
        roll=SPAWN_ROLL
    )
    if actor is None:
        rospy.logerr(f"Spawn actor failed at ({x},{y})")
        return
    actor_id = actor.id
    rospy.loginfo(f"Spawn actor ID={actor_id} @ ({x},{y})")

    # 等待測量
    t0 = time.time()
    meas_dist = meas_angle = None
    while meas_dist is None and (time.time() - t0) < TIMEOUT and not rospy.is_shutdown():
        rospy.sleep(0.1)

    md, ma = (meas_dist, meas_angle) if meas_dist is not None else ('NA','NA')
    # Ground truth
    try:
        gt_dist, gt_angle = get_ground_truth(world, actor_id)
    except Exception as e:
        rospy.logerr(f"取得 Ground Truth 失敗: {e}")
        gt_dist, gt_angle = 'NA','NA'

    # 誤差
    err_dist  = (gt_dist - md) if isinstance(md, float) else 'NA'
    err_angle = (gt_angle - ma) if isinstance(ma, float) else 'NA'

    # 寫入 CSV
    writer.writerow([
        actor_id,
        x, y, z,
        md, ma,
        gt_dist, gt_angle,
        err_dist, err_angle,
        time.strftime("%Y-%m-%d %H:%M:%S")
    ])

    # 刪除 actor
    actor.destroy()
    rospy.loginfo(f"Destroyed actor ID={actor_id}")


def main():
    rospy.init_node('calibration_recorder', anonymous=True)
    # 訂閱估測與 odom
    rospy.Subscriber('/bev_measure/front', String, meas_cb)
    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, odo_cb)

    # CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    blueprint = world.get_blueprint_library().find(ACTOR_BP)

    # CSV
    with open(CSV_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'actor_id','spawn_x','spawn_y',
            'meas_dist_m','meas_angle_deg',
            'gt_dist_m','gt_angle_deg',
            'err_dist_m','err_angle_deg',
            'timestamp'
        ])

        rospy.loginfo("Starting calibration recording...")
        try:
            # 單筆量測
            measure_once(world, blueprint, writer)
        except rospy.ROSInterruptException:
            pass
        finally:
            rospy.loginfo(f"Calibration data saved to {CSV_FILE}")

if __name__=='__main__':
    main()
