#!/home/ciao/anaconda3/envs/carlayolo/bin/python
import sys, os
yolov7_root = os.path.expanduser('~/yolov7')
if os.path.isdir(yolov7_root):
    sys.path.insert(0, yolov7_root)
else:
    # 如果找不到路徑可以印個 warn
    import rospy
    rospy.logwarn(f"找不到 YOLOv7 根目錄：{yolov7_root}")

deep_sort_root = os.path.expanduser('~/YOLOv7-DeepSORT-Object-Tracking')
if os.path.isdir(deep_sort_root):
    sys.path.insert(0, deep_sort_root)
else:
    import rospy
    rospy.logwarn(f"找不到 DeepSort 根目錄：{deep_sort_root}")
import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from distance_utils import calculate_x_distance
import argparse
import time
from pathlib import Path
import os
import matplotlib.pyplot as plt
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np

# 強制把 attempt_download() 改成空函式，跳過所有下載動作
try:
    import utils.google_utils as google_utils
    google_utils.attempt_download = lambda *args, **kwargs: None
    rospy.loginfo("已跳過 YOLOv7 權重下載，直接載入本地檔案")
except ImportError:
    rospy.logwarn("無法 import utils.google_utils，未套用跳過下載的設定")


from PIL import Image

from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages, letterbox
from utils.general import (
    check_img_size,
    check_requirements,
    check_imshow,
    non_max_suppression,
    apply_classifier,
    scale_coords,
    xyxy2xywh,
    strip_optimizer,
    increment_path,
    set_logging,
)
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel
from deep_sort_pytorch.utils.parser import get_config
from deep_sort_pytorch.deep_sort import DeepSort

# Import functions from b.py
from b_t import xyxy_to_xywh, compute_color_for_labels, UI_box, update_movement_status, load_classes, draw_boxes, calculate_distance
# Import vanishing point and grid generation functions
#from vanishing_point import GetLines, GetVanishingPoint
from grid import generate_grid, GetLines, GetVanishingPoint


class RosDetectNode:
    def __init__(self):
        rospy.init_node('ros_detect_node')
        import sys, os
        rospy.loginfo(f"[ros_detect_node] Python Exec: {sys.executable}")
        rospy.loginfo(f"[ros_detect_node] LD_LIBRARY_PATH: {os.environ.get('LD_LIBRARY_PATH')}")
        rospy.loginfo(f"[ros_detect_node] CUDA_VISIBLE_DEVICES: {os.environ.get('CUDA_VISIBLE_DEVICES')}")
        rospy.loginfo(f"[ros_detect_node] torch.cuda.is_available(): {torch.cuda.is_available()}")

        # ——— 誤差補償模型（單位 cm） ———
        # 1) 量測值及地面真實值都用 cm
        measured_cm    = np.array([215, 197.9, 166, 141.6, 114.6, 75.88, 45])
        truth_cm       = np.array([340, 290, 240, 190, 140,  90, 40])
        errors_cm      = truth_cm - measured_cm     # e = 真實 – 測量

        # 2) 擬合二次多項式 e(z) ≈ a·z² + b·z + c
        #    你也可以改成 degree=1 (線性) 或 3
        self.err_coeffs = np.polyfit(measured_cm, errors_cm, 2)
        rospy.loginfo(f"[ros_detect_node] 補償係數 (cm): {self.err_coeffs}")


        # —— 1. 參數 —— #
        # 把 dis_test.py 裡 argparse 定的都搬成 rosparam
        class P: pass
        self.opt = P()
        self.opt.weights       = rospy.get_param('~weights', '/home/ciao/YOLOv7-DeepSORT-Object-Tracking/yolov7.pt')
        self.opt.img_size      = rospy.get_param('~img_size',      640)
        self.opt.conf_thres    = rospy.get_param('~conf_thres',    0.45)
        self.opt.iou_thres     = rospy.get_param('~iou_thres',     0.45)
        self.opt.device        = rospy.get_param('~device',        '')
        self.opt.names         = rospy.get_param('~names', '/home/ciao/YOLOv7-DeepSORT-Object-Tracking/data/coco.names')
        self.opt.project       = rospy.get_param('~project',       'runs/detect')
        self.opt.name          = rospy.get_param('~name',          'exp')
        self.opt.exist_ok      = rospy.get_param('~exist_ok',      False)
        self.opt.save_img      = rospy.get_param('~save_img',      False)
        self.opt.camera_height = rospy.get_param('~camera_height', 298)
        self.opt.tile_size     = rospy.get_param('~tile_size',     100)#39.5
        self.opt.num_tiles     = rospy.get_param('~num_tiles',     50)
        self.opt.trailslen     = rospy.get_param('~trailslen',     64)
        self.cx = 1.0694310681346919e+03
        self.cy = 6.4810816967583071e+02
        self.fx = 3.7188644469223198e+02   # 你的 fx
        self.fy = 3.9512475008928237e+02
        

        # 設定存檔資料夾（跟 dis_test.py 一樣）
        self.save_dir = Path(increment_path(
            Path(self.opt.project) / self.opt.name,
            exist_ok=self.opt.exist_ok
        ))

        (self.save_dir / 'labels').mkdir(parents=True, exist_ok=True)


        # —— 2. 初始化模型 —— #
        set_logging()
        self.device = select_device(self.opt.device)
        rospy.loginfo(f"[ros_detect_node] 使用裝置：{self.device}")

        self.model  = attempt_load(self.opt.weights, map_location=self.device)
        self.model.float()
        self.stride = int(self.model.stride.max())
        self.opt.img_size = check_img_size(self.opt.img_size, s=self.stride)
        if self.device.type != 'cpu':
            cudnn.benchmark = True
            # warmup
            dummy = torch.zeros(1, 3, self.opt.img_size, self.opt.img_size).to(self.device)
            self.model(dummy)

        # —— 3. 初始化 DeepSort —— #
        cfg = get_config()
        cfg.merge_from_file("/home/ciao/YOLOv7-DeepSORT-Object-Tracking/deep_sort_pytorch/configs/deep_sort.yaml")

        use_cuda = (self.device.type != 'cpu')
        rospy.loginfo(f"[ros_detect_node] DeepSort use_cuda：{use_cuda}")
        self.deepsort = DeepSort(
            cfg.DEEPSORT.REID_CKPT,
            max_dist=cfg.DEEPSORT.MAX_DIST,
            min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
            nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP,
            max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
            max_age=cfg.DEEPSORT.MAX_AGE,
            n_init=cfg.DEEPSORT.N_INIT,
            nn_budget=cfg.DEEPSORT.NN_BUDGET,
            use_cuda=use_cuda
        )

        # 其他狀態
        self.names = load_classes(self.opt.names)
        self.vp    = None
        self.grid  = None
        self.prev_dist = {}
        self.fps = 30  # 固定值或從 param 拿

        
        # ROS pub/sub: 由 private ROS 參數決定 topic
        self.bridge = CvBridge()
        # 從 rosparam 拿輸入／輸出 topic 名
        input_topic  = rospy.get_param('~input_topic',  '/fisheye_front/rectified_image')
        output_topic = rospy.get_param('~output_topic', '/detect/image')
        # 加這一行，預設就用 topic 當 camera_name
        self.camera_name = rospy.get_param('~camera_name', input_topic)

        rospy.loginfo(f"[ros_detect_node] camera_name: {self.camera_name}")
        rospy.loginfo(f"[ros_detect_node] input_topic: {input_topic}, output_topic: {output_topic}")
        self.pub = rospy.Publisher(output_topic, ROSImage, queue_size=1)
        rospy.Subscriber(input_topic, ROSImage, self.callback, queue_size=1)
        #rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', ROSImage, self.callback, queue_size=1)
        rospy.loginfo("ros_detect_node ready…")

        from std_msgs.msg import String
        self.measure_pub = rospy.Publisher(f'/bev_measure/{self.camera_name}', String, queue_size=1)

    def callback(self, msg: ROSImage):
        # —— 1) 為每個物件收集的清單 —— #
        dists_cm    = []
        speeds_cm_s = []
        x_offs      = []
        angles      = []

        # —— 2) 影像轉 CV2 + 推論、NMS、scale_coords、格網 —— #
        t_frame = time.time()
        frame   = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img0    = frame.copy()
        img     = letterbox(img0, new_shape=self.opt.img_size, stride=self.stride)[0]
        img     = img[:, :, ::-1].transpose(2,0,1)
        img     = np.ascontiguousarray(img)
        img     = torch.from_numpy(img).to(self.device).float() / 255.0
        img     = img.unsqueeze(0) if img.ndimension()==3 else img

        with torch.no_grad():
            pred = self.model(img, augment=False)[0]
        pred = non_max_suppression(pred, self.opt.conf_thres, self.opt.iou_thres)

        if not (pred and len(pred[0])):
            # 沒偵測到任何物件
            if self.opt.save_img:
                cv2.imwrite(str(self.frames_dir/f"frame_{int(t_frame*1000)}.jpg"), frame)
            out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            out_msg.header = msg.header
            return self.pub.publish(out_msg)

        det = pred[0]
        # scale 回原圖
        h0, w0 = img0.shape[:2]
        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], (h0, w0)).round()

        # 格網
        if self.vp is None:
            self.vp = (self.cx, self.cy)

        frame, self.grid = generate_grid(           # 這裡得frame指的是 輸入原始圖片，輸出有grid的圖片
            frame, self.vp,
            camera_height_cm=self.opt.camera_height,
            tile_size_cm=self.opt.tile_size,
            num_tiles=self.opt.num_tiles
        )

        # DeepSort 更新
        xywhs, confs, oids = [], [], []
        for *xyxy, conf, cls in reversed(det):
            x1,y1,x2,y2 = map(int, xyxy)
            if x2-x1<2 or y2-y1<2: continue
            xywhs.append(xyxy_to_xywh(x1,y1,x2,y2))
            confs.append([conf.item()])
            oids.append(int(cls))
        outputs = (self.deepsort.update(
            torch.Tensor(xywhs), torch.Tensor(confs), oids, frame
        ) if xywhs else [])

        # —— 3) 一次迴圈：計算縱向距離、補償、速度、水平偏移、角度 —— #
        if len(outputs)>0:
            boxes      = outputs
            # DeepSort 回傳格式： [x1,y1,x2,y2, track_id, class_id]
            track_ids  = outputs[:, 4].astype(int)   # 第五欄是 track ID
            cls_ids    = outputs[:, 5].astype(int)   # 第六欄是 class ID
            identities = track_ids
        else:
            boxes      = det
            # det 只有偵測，沒有 track ID
            track_ids  = None
            cls_ids    = det[:, -1].astype(int)
            identities = None
            

        for out in boxes:
            xyxy   = out[:4]
            obj_id = int(out[-1]) if identities is not None else None
            bottom_y = xyxy[3]

            # 縱向距離（cm）＋誤差補償
            raw_cm    = calculate_distance(
                self.grid, bottom_y,
                self.opt.tile_size,
                self.opt.camera_height,
                self.opt.num_tiles
            )
            corr_err  = np.polyval(self.err_coeffs, raw_cm)
            corr_cm   = raw_cm + corr_err

            # 速度 (cm/s)
            prev_cm   = self.prev_dist.get(obj_id, raw_cm)
            speed_cm  = (corr_cm - prev_cm) / (1.0/self.fps)
            self.prev_dist[obj_id] = corr_cm

            # 水平偏移 & 偏航角
            x_cm, diag_cm, angle_deg = calculate_x_distance(
                box_xyxy       = xyxy,
                optical_center = (self.cx, self.cy),
                Zc_cm          = corr_cm,
                focal_length_x = self.fx,
                return_degree  = True
            )

            # 收集
            
            dists_cm.append(diag_cm)  
            speeds_cm_s.append(speed_cm)
            x_offs.append(x_cm)
            angles.append(angle_deg)

            # —— 在這裡插入 log 輸出 —— #
        
        rospy.loginfo(f"[{self.camera_name}] [x_offsets] {[f'{x:.1f}' for x in x_offs]} cm")
        rospy.loginfo(f"[{self.camera_name}] [angles]    {[f'{a:.1f}' for a in angles]} °")
        # —— 為每個偵測到的物件輸出獨立一行 Log —— #
        for idx, cls_id in enumerate(cls_ids):
            # 如果有 track id 就用它，沒有就用陣列索引
            obj_id = (track_ids[idx] if track_ids is not None else idx)
            name   = self.names[cls_id]
            dist   = dists_cm[idx]
            angle  = angles[idx]
            rospy.loginfo(
                f"[{self.camera_name}] {name}{obj_id} {dist:.2f} cm {angle:.1f}°"
            )

            # —— 新增：將所有距離／角度／類別打包成 JSON，發到 /bev_measure/… topic —— #
        # 1) 用 cls_ids 建立類別名稱列表
        classes = [ self.names[cid] for cid in cls_ids ]
        # 2) 打包 JSON（distance 用 dists_cm，angle 用 angles）
        import json
        from std_msgs.msg import String
        measurement = {
            'distances': [ f"{d:.1f}" for d in dists_cm ],  # cm 字串
            'angles':    [ f"{a:.1f}" for a in angles ],    # 度 字串
            'classes':    classes                            # 類別名稱字串
        }
        self.measure_pub.publish( String(data=json.dumps(measurement)) )
        rospy.loginfo(f"[{self.camera_name}] published "
                    f"{len(classes)} objs → /bev_measure/{self.camera_name}")





        # —— 4) 一次呼叫 draw_boxes，把所有補償後數據傳入 —— #
        frame = draw_boxes(
            img           = frame,
            bbox          = boxes[:, :4],
            names         = self.names,
            object_id     = cls_ids,
            grid_y_values = self.grid,
            tile_size_cm  = self.opt.tile_size,
            camera_height_cm = self.opt.camera_height,
            num_tiles     = self.opt.num_tiles,
            fps           = self.fps,
            identities    = track_ids,
            frame_count   = 0,
            trailslen     = self.opt.trailslen,
            distances     = dists_cm,
            speeds        = speeds_cm_s,
            x_offsets     = x_offs,
            angles        = angles
        )

        # —— 5) （可選）儲存圖檔 & 發佈 ROS topic —— #
        if self.opt.save_img:
            fname = self.frames_dir / f"frame_{int(t_frame*1000)}.jpg"
            cv2.imwrite(str(fname), frame)

        out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = RosDetectNode()
    node.spin()