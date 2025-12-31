#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import carla, math
import numpy as np

# Helper drawing functions using pure NumPy
def draw_line(img, p1, p2, color):
    x1, y1 = p1; x2, y2 = p2
    length = int(max(abs(x2 - x1), abs(y2 - y1)))
    if length == 0:
        if 0 <= y1 < img.shape[0] and 0 <= x1 < img.shape[1]:
            img[y1, x1] = color
        return
    xs = np.linspace(x1, x2, length, dtype=np.int32)
    ys = np.linspace(y1, y2, length, dtype=np.int32)
    mask = (ys >= 0) & (ys < img.shape[0]) & (xs >= 0) & (xs < img.shape[1])
    img[ys[mask], xs[mask]] = color

def draw_circle(img, center, radius, color):
    px, py = center
    ys, xs = np.ogrid[-radius:radius+1, -radius:radius+1]
    mask = xs**2 + ys**2 <= radius**2
    h, w = img.shape[:2]
    for dy in range(mask.shape[0]):
        yy = py - radius + dy
        if 0 <= yy < h:
            xs_idx = np.nonzero(mask[dy])[0]
            xs_img = px - radius + xs_idx
            valid = (xs_img >= 0) & (xs_img < w)
            img[yy, xs_img[valid]] = color

def draw_triangle(img, center, size, color):
    px, py = center
    s = size
    # 從 apex (-s) 到 base (+s) 橫掃每一列，畫水平線填滿
    for dy in range(-s, s+1):
        y = py + dy
        if y < 0 or y >= img.shape[0]:
            continue
        # dy = -s → width s at apex; dy = +s → width 0 at tip
        ratio = (dy + s) / (2.0 * s)
        half_w = int((1.0 - ratio) * s)
        x_left  = max(0, px - half_w)
        x_right = min(img.shape[1]-1, px + half_w)
        img[y, x_left:x_right+1] = color

def draw_cross(img, center, size, color):
    px, py = center
    s = size
    # 三條線疊起來，讓叉叉更粗
    for ofs in (-1, 0, +1):
        draw_line(img,
                  (px-s, py-s+ofs),
                  (px+s, py+s+ofs),
                  color)
        draw_line(img,
                  (px-s, py+s+ofs),
                  (px+s, py-s+ofs),
                  color)


def draw_square(img, center, size, color):
    px, py = center; s = size
    draw_line(img, (px-s, py-s), (px+s, py-s), color)
    draw_line(img, (px+s, py-s), (px+s, py+s), color)
    draw_line(img, (px+s, py+s), (px-s, py+s), color)
    draw_line(img, (px-s, py+s), (px-s, py-s), color)

class BEVVisualizerNumPy:
    def __init__(self):
        rospy.init_node('ground_truth_bev_np', anonymous=True)
        ids = rospy.get_param('~target_ids', '')
        self.target_ids = [int(x) for x in ids.split() if x]
        self.rate_hz = rospy.get_param('~rate', 10)
        self.scale   = rospy.get_param('~scale', 50)
        self.window  = rospy.get_param('~window_size', 10)
        self.img_size = int(self.window * 2 * self.scale)
        self.center   = self.img_size // 2

        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher('bev_image', Image, queue_size=1)

        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        self.world = client.get_world()

        self.ego_pos = None; self.ego_yaw = 0.0
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odom_cb)
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and self.ego_pos is None:
            rate.sleep()

        self.target_actors = {}
        for tid in self.target_ids:
            actor = self.world.get_actor(tid)
            if actor:
                rospy.loginfo(f"[BEV_NP] 加入 actor id={tid}")
                self.target_actors[tid] = actor
            else:
                rospy.logwarn(f"[BEV_NP] 無法找到 actor id={tid}")
        self.rate = rospy.Rate(self.rate_hz)

    def odom_cb(self, msg):
        p = msg.pose.pose.position; self.ego_pos = p
        q = msg.pose.pose.orientation
        self.ego_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def run(self):
        while not rospy.is_shutdown():
            img = np.full((self.img_size, self.img_size, 3), 255, dtype=np.uint8)
            # 繪製以 ego 為中心、間隔 0.5m 的網格背景
            step = int(self.scale * 0.5)  # 每 0.5 公尺對應的像素
            grid_color = (200, 200, 200)
            # offset 確保中心有一條格線
            offset = self.center % step
            # 垂直線
            for x in range(offset, self.img_size, step):
                img[:, x] = grid_color
            # 水平線
            for y in range(offset, self.img_size, step):
                img[y, :] = grid_color
            # draw grid every 0.5m
            step = int(self.scale * 0.5)
            grid_color=(200,200,200)
            
            offset = self.center % step
            for x in range(offset, self.img_size, step): img[:,x]=grid_color
            for y in range(offset, self.img_size, step): img[y,:]=grid_color
            # —— 用較深且稍粗的線畫出 X、Y 軸 —— 
            axis_color = (50, 50, 50)   # 深灰色
            # 模擬粗線：在中心線上下各畫一條
            for ofs in (-1, 0, +1):
                # X 軸（橫向）
                draw_line(img,
                        (0, self.center + ofs),
                        (self.img_size - 1, self.center + ofs),
                        axis_color)
                # Y 軸（縱向）
                draw_line(img,
                        (self.center + ofs, 0),
                        (self.center + ofs, self.img_size - 1),
                        axis_color)

            # —— 在軸上畫刻度，每隔兩格網格（1 m）一格 —— 
            tick_len   = int(self.scale * 0.2)   # 刻度長度約 0.2 m
            tick_step  = int(self.scale * 0.5) * 2  # 兩格（0.5 m ×2 = 1 m）
            offset     = self.center % int(self.scale * 0.5)

            # X 軸刻度：在中心橫線上畫小豎線
            for x in range(offset, self.img_size, tick_step):
                draw_line(img,
                        (x, self.center - tick_len),
                        (x, self.center + tick_len),
                        axis_color)

            # Y 軸刻度：在中心縱線上畫小橫線
            for y in range(offset, self.img_size, tick_step):
                draw_line(img,
                        (self.center - tick_len, y),
                        (self.center + tick_len, y),
                        axis_color)
            half = int(0.5 * self.scale)
            draw_line(img, (self.center, self.center-half), (self.center, self.center+half), (0,0,255))
            draw_line(img, (self.center-half, self.center), (self.center+half, self.center), (0,0,255))
            # ——— 在這裡畫出 Ego (vehicle.micro.microlino) 的車體邊界 ———
            # vehicle.micro.microlino 半長度與半寬度 (公尺)
            half_length_m = 1.10
            half_width_m  = 0.74 
            # 換算成像素
            hl = int(half_length_m * self.scale)
            hw = int(half_width_m  * self.scale)

            # 四個角點（車頭朝上）
            corners = [
                (self.center - hw, self.center - hl),  # 左上
                (self.center + hw, self.center - hl),  # 右上
                (self.center + hw, self.center + hl),  # 右下
                (self.center - hw, self.center + hl),  # 左下
            ]
            # 用 draw_line 連成矩形
            box_color = (0, 0, 255)  # 紅色
            for i in range(4):
                draw_line(img, corners[i], corners[(i+1)%4], box_color)
            # ———————————————————————————————


            for tid, actor in self.target_actors.items():
                loc = actor.get_transform().location
                dx = loc.x - self.ego_pos.x
                dy = loc.y - ( - self.ego_pos.y)
                # 跑 track_object_distance 的算法：
                dist = math.hypot(dx, dy)
                bearing = math.atan2(dy, dx)
                rel_ang = bearing - self.ego_yaw
                deg = (math.degrees(rel_ang) + 360) % 360
                # 以距離與相對角度計算像素偏移
                rad = math.radians(270 - deg)
                x_rel = dist * math.cos(rad)
                y_rel = dist * math.sin(rad)
                px = int(self.center + x_rel * self.scale)
                py = int(self.center - y_rel * self.scale)

                t = actor.type_id
                in_bounds = (0 <= px < self.img_size) and (0 <= py < self.img_size)
                rospy.loginfo(f"[DEBUG] id={tid} dist={dist:.2f}m ang={deg:.1f}° px_py=({px},{py}) in_bounds={in_bounds}")
                if not in_bounds: continue

                if 'pedestrian' in t:
                    draw_circle(img, (px, py), radius=5, color=(255, 0, 0))
                elif 'vespa' in t or 'motorcycle' in t:
                    draw_triangle(img, (px, py), size=6, color=(0, 255, 0))
                elif 'bicycle' in t or 'crossbike' in t:
                    draw_cross(img, (px, py), size=6, color=(255, 0, 255))
                else:
                    draw_square(img, (px, py), size=6, color=(128, 128, 128))

            ros_img = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            self.img_pub.publish(ros_img)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        BEVVisualizerNumPy().run()
    except rospy.ROSInterruptException:
        pass
