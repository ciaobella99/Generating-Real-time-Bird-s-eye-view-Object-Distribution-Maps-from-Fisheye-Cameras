#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import math

def callback(data):
    # 解析 PointCloud2 資料為每個 (x, y, z)
    min_dist = float('inf')
    for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
        dist = math.sqrt(x**2 + y**2 + z**2)
        if dist < min_dist:
            min_dist = dist
    rospy.loginfo("最近點距離：%.2f m" % min_dist)

def listener():
    rospy.init_node('lidar_distance_listener', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/lidar", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
