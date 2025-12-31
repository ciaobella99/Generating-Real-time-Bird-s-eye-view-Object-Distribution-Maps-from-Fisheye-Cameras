#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
import json

# Drawing helpers using NumPy
def draw_line(img, p1, p2, color):
    x1, y1 = p1; x2, y2 = p2
    length = int(max(abs(x2-x1), abs(y2-y1)))
    if length == 0:
        if 0 <= y1 < img.shape[0] and 0 <= x1 < img.shape[1]:
            img[y1, x1] = color
        return
    xs = np.linspace(x1, x2, length, dtype=np.int32)
    ys = np.linspace(y1, y2, length, dtype=np.int32)
    mask = (ys>=0)&(ys<img.shape[0])&(xs>=0)&(xs<img.shape[1])
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
            valid = (xs_img>=0)&(xs_img<w)
            img[yy, xs_img[valid]] = color

def draw_triangle(img, center, size, color):
    px, py = center; s = size
    for dy in range(-s, s+1):
        y = py + dy
        if 0 <= y < img.shape[0]:
            ratio = (dy + s)/(2.0*s)
            half_w = int((1.0-ratio)*s)
            x_left = max(0, px-half_w)
            x_right = min(img.shape[1]-1, px+half_w)
            img[y, x_left:x_right+1] = color

def draw_cross(img, center, size, color):
    px, py = center; s = size
    for ofs in (-1,0,1):
        draw_line(img, (px-s, py-s+ofs), (px+s, py+s+ofs), color)
        draw_line(img, (px-s, py+s+ofs), (px+s, py-s+ofs), color)

def draw_square(img, center, size, color):
    px, py = center; s = size
    draw_line(img, (px-s, py-s), (px+s, py-s), color)
    draw_line(img, (px+s, py-s), (px+s, py+s), color)
    draw_line(img, (px+s, py+s), (px-s, py+s), color)
    draw_line(img, (px-s, py+s), (px-s, py-s), color)

class BEVNode:
    def __init__(self):
        rospy.init_node('bev_node', anonymous=True)
        rospy.loginfo('[BEV] node started')
        # parameters
        self.rate_hz  = rospy.get_param('~rate', 10)
        self.scale    = rospy.get_param('~scale', 50)
        self.window   = rospy.get_param('~window_size', 10)
        self.half_fov = rospy.get_param('~half_fov', 60.0)  # degrees
        # image size and center
        self.img_size = int(self.window * 2 * self.scale)
        self.center   = self.img_size // 2

        # setup measurement storage and subscribers
        self.measurements = {'front':[], 'right':[], 'left':[]}
        # camera boresights (global angles)
        offsets = {'front':180, 'right':300, 'left':60}
        for cam, bore in offsets.items():
            topic = f'/bev_measure/{cam}'
            rospy.loginfo(f'[BEV] subscribing to {topic} with FOV±{self.half_fov}°')
            rospy.Subscriber(topic, String, self.measure_cb, callback_args=(cam, bore))

        # publisher for BEV image
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher('bev_estimate', Image, queue_size=1)
        self.rate = rospy.Rate(self.rate_hz)

    def measure_cb(self, msg, args):
        cam, bore = args
        try:
            data = json.loads(msg.data)
            dists_cm = data.get('distances', [])
            angles   = data.get('angles', [])
            classes  = data.get('classes', [])
            mlist = []
            for d_str, a_str, c in zip(dists_cm, angles, classes):
                d = float(d_str)/100.0
                raw_a = float(a_str)
                global_a = (raw_a + bore) % 360
                # compute angular deviation from camera boresight
                dev = abs((global_a - bore + 180)%360 - 180)
                # filter out-of-FOV points
                if dev > self.half_fov:
                    continue
                mlist.append((d, global_a, c))
            self.measurements[cam] = mlist
            rospy.loginfo(f'[BEV] {cam} got {len(mlist)} objs: {mlist}')
        except Exception as e:
            rospy.logwarn(f'[BEV] parse error {cam}: {e}')

    def run(self):
        while not rospy.is_shutdown():
            # blank canvas
            img = np.full((self.img_size, self.img_size, 3), 255, dtype=np.uint8)
            # grid lines
            step = int(self.scale * 0.5)
            grid_color = (200,200,200)
            offs = self.center % step
            for x in range(offs, self.img_size, step): img[:,x] = grid_color
            for y in range(offs, self.img_size, step): img[y,:] = grid_color
            # thick axes
            axis_color = (50,50,50)
            for ofs in (-1,0,1):
                draw_line(img,(0,self.center+ofs),(self.img_size-1,self.center+ofs),axis_color)
                draw_line(img,(self.center+ofs,0),(self.center+ofs,self.img_size-1),axis_color)
            # ticks every 1m
            tick_len = int(self.scale*0.2)
            tick_step = step*2
            for x in range(offs, self.img_size, tick_step):
                draw_line(img,(x,self.center-tick_len),(x,self.center+tick_len),axis_color)
            for y in range(offs, self.img_size, tick_step):
                draw_line(img,(self.center-tick_len,y),(self.center+tick_len,y),axis_color)
            # center cross at 0.5m
            half = int(0.5*self.scale)
            draw_line(img,(self.center,self.center-half),(self.center,self.center+half),(0,0,255))
            draw_line(img,(self.center-half,self.center),(self.center+half,self.center),(0,0,255))
            # ego vehicle box (2.4m x 1.08m)
            hl = int(1.10 * self.scale); hw = int(0.74 * self.scale)
            corners=[(self.center-hw,self.center-hl),(self.center+hw,self.center-hl),
                     (self.center+hw,self.center+hl),(self.center-hw,self.center+hl)]
            for i in range(4): draw_line(img, corners[i], corners[(i+1)%4], (0,0,255))
            # plot measurements
            for cam, mlist in self.measurements.items():
                for dist, ang, cls in mlist:
                    rad = math.radians(270-ang)
                    x = dist*math.cos(rad); y = dist*math.sin(rad)
                    px = int(self.center + x*self.scale)
                    py = int(self.center - y*self.scale)
                    if not (0<=px<self.img_size and 0<=py<self.img_size): continue
                    if 'person' in cls:      draw_circle(img,(px,py),5,(255,0,0))
                    elif 'motorcycle' in cls: draw_triangle(img,(px,py),6,(0,255,0))
                    elif 'bicycle' in cls:    draw_cross(img,(px,py),6,(255,0,255))
                    else:                     draw_square(img,(px,py),6,(128,128,128))
            rospy.loginfo_throttle(5,f'[BEV] buffers={self.measurements}')
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img,'bgr8'))
            self.rate.sleep()

if __name__=='__main__':
    try: node=BEVNode(); node.run()
    except rospy.ROSInterruptException: pass
