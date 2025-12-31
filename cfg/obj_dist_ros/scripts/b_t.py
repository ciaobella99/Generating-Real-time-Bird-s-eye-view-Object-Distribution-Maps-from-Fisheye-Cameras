import argparse
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized
from collections import defaultdict
import math

from deep_sort_pytorch.utils.parser import get_config
from deep_sort_pytorch.deep_sort import DeepSort
from collections import deque
import numpy as np
palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)
data_deque = {}


##########################################################################################
def xyxy_to_xywh(x1, y1, x2, y2):
    """
    將絕對像素座標 xyxy 轉成 (x_center, y_center, width, height)。
    這裡同時支援傳入 torch.Tensor 或純 Python int/float。
    """
    # 如果是 0-d Tensor，就用 .item() 拿數值；否則保留原值
    if hasattr(x1, 'item'): x1 = x1.item()
    if hasattr(y1, 'item'): y1 = y1.item()
    if hasattr(x2, 'item'): x2 = x2.item()
    if hasattr(y2, 'item'): y2 = y2.item()

    # 左上角、寬高計算
    bbox_left = min(x1, x2)
    bbox_top  = min(y1, y2)
    bbox_w    = abs(x2 - x1)
    bbox_h    = abs(y2 - y1)

    # 中心點
    x_c = bbox_left + bbox_w / 2.0
    y_c = bbox_top  + bbox_h / 2.0

    return x_c, y_c, bbox_w, bbox_h


def xyxy_to_tlwh(bbox_xyxy):
    tlwh_bboxs = []
    for i, box in enumerate(bbox_xyxy):
        x1, y1, x2, y2 = [int(i) for i in box]
        top = x1
        left = y1
        w = int(x2 - x1)
        h = int(y2 - y1)
        tlwh_obj = [top, left, w, h]
        tlwh_bboxs.append(tlwh_obj)
    return tlwh_bboxs

def compute_color_for_labels(label):
    """
    Simple function that adds fixed color depending on the class
    """
    if label == 0: #person
        color = (85,45,255)
    elif label == 2: # Car
        color = (222,82,175)
    elif label == 3:  # Motobike
        color = (0, 204, 255)
    elif label == 5:  # Bus
        color = (0, 149, 255)
    else:
        color = [int((p * (label ** 2 - label + 1)) % 255) for p in palette]
    return tuple(color)

def draw_border(img, pt1, pt2, color, thickness, r, d):
    x1,y1 = pt1
    x2,y2 = pt2
    # Top left
    cv2.line(img, (x1 + r, y1), (x1 + r + d, y1), color, thickness)
    cv2.line(img, (x1, y1 + r), (x1, y1 + r + d), color, thickness)
    cv2.ellipse(img, (x1 + r, y1 + r), (r, r), 180, 0, 90, color, thickness)
    # Top right
    cv2.line(img, (x2 - r, y1), (x2 - r - d, y1), color, thickness)
    cv2.line(img, (x2, y1 + r), (x2, y1 + r + d), color, thickness)
    cv2.ellipse(img, (x2 - r, y1 + r), (r, r), 270, 0, 90, color, thickness)
    # Bottom left
    cv2.line(img, (x1 + r, y2), (x1 + r + d, y2), color, thickness)
    cv2.line(img, (x1, y2 - r), (x1, y2 - r - d), color, thickness)
    cv2.ellipse(img, (x1 + r, y2 - r), (r, r), 90, 0, 90, color, thickness)
    # Bottom right
    cv2.line(img, (x2 - r, y2), (x2 - r - d, y2), color, thickness)
    cv2.line(img, (x2, y2 - r), (x2, y2 - r - d), color, thickness)
    cv2.ellipse(img, (x2 - r, y2 - r), (r, r), 0, 0, 90, color, thickness)

    cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), color, -1, cv2.LINE_AA)
    cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r - d), color, -1, cv2.LINE_AA)
    
    cv2.circle(img, (x1 +r, y1+r), 2, color, 12)
    cv2.circle(img, (x2 -r, y1+r), 2, color, 12)
    cv2.circle(img, (x1 +r, y2-r), 2, color, 12)
    cv2.circle(img, (x2 -r, y2-r), 2, color, 12)
    
    return img

def UI_box(x, img, color=None, label=None, line_thickness=None):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]

        img = draw_border(img, (c1[0], c1[1] - t_size[1] -3), (c1[0] + t_size[0], c1[1]+3), color, 1, 8, 2)

        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

status_history = {}
object_diagonal_history = {}
object_coordinate_history = {}

def compute_diagonal(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
#物件邊界框 (bounding box) 的座標 計算 對角線長度。

def update_movement_status(id, x1, y1, x2, y2, frame_count):
    
    current_diagonal = compute_diagonal(x1, y1, x2, y2)
    current_coords = ((x1, y1), (x2, y2))
    #print("current_coords:",current_coords)
    
    last_diagonal = object_diagonal_history.get(id, None)
    last_coords = object_coordinate_history.get(id, ((None, None), (None, None)))
    #print("last coords:",last_coords)
    last_status = status_history.get(id, "Initializing")
    prev_x1 = last_coords[0][0]
    prev_y1 = last_coords[0][1]
    prev_x2 = last_coords[1][0]
    prev_y2 = last_coords[1][1]
    #print("last status:",last_status )
    
    if frame_count % 3 == 0:  # Only update every 'update_interval' frames
            if last_diagonal is not None:
                if current_diagonal > last_diagonal:
                   if (x1 >= prev_x1 and y1 >= prev_y1 and x2 >= prev_x2 and y2>= prev_y2 ) or (x1 <= prev_x1 and y1 >= prev_y1 and x2 <= prev_x2 and y2>= prev_y2):
                       status ="Approaching"  
                                 
                   elif (x1 - prev_x1) + (x2 - prev_x2) + (y1 - prev_y1) + (y2 - prev_y2) > 0: #for case 2
                       status ="Approaching"  
                       
                   elif (prev_x1 - x1) + (prev_x2 - x2 ) + (y1 - prev_y1) + (y2 - prev_y2) > 0: #for case 1
                       status ="Approaching"  
                                  
                   else:
                       status = "Moving Away"

                elif current_diagonal < last_diagonal:
                    if (x1 >= prev_x1 and y1 <= prev_y1 and x2 >= prev_x2 and y2<= prev_y2 ) or (x1 <= prev_x1 and y1 <= prev_y1 and x2 <= prev_x2 and y2<= prev_y2): 
                       status = "Moving Away"
                       
                    elif (x1 - prev_x1) + (x2 - prev_x2) + (prev_y1 -y1) + (prev_y2 - y2) > 0: #for case 3
                       status = "Moving Away"  
                       
                    elif (prev_x1 - x1) + (prev_x2 - x2 ) + (prev_y1 -y1) + (prev_y2 - y2) > 0: #for case 4
                       status = "Moving Away"
                       
                    else:   
                       status ="Approaching" 
                else:
                    status = "Stationary"
            else:
                status = "Initializing"
    
            # Update the history to the current diagonal and coordinates
            object_diagonal_history[id] = current_diagonal   # 記錄對角線長度
            object_coordinate_history[id] = ((x1, y1), (x2, y2))
            status_history[id] = status
            #print("s1 :",status)
    else:
            # Use the last known status if not updating this frame
            #status = last_status.get(id, "Data Insufficient")
            status = last_status
            #print("s3 :",status)
            
    
    #print("frame:", frame_count)
    #print("current_diagonal:", current_diagonal)
    #print("last_diagonal:", last_diagonal)
    #print("status:",status )
    #print("last status:",last_status )
    
    return status, last_status, current_diagonal, last_diagonal, current_coords, last_coords, frame_count

def calculate_distance(grid_y_values, object_bottom_y, tile_size_cm, camera_height_cm, num_tiles):
    i = 0  # 物體後方的完整網格數量
    
    # 找出物體底部後面的完整網格數
    for y in grid_y_values:
        if object_bottom_y > y:
            i += 1

    grid_num_behind = i-1
    # 計算物體位於網格之間的相對距離
    a = abs(grid_y_values[-1-grid_num_behind] - object_bottom_y )  # 物體底部和最近網格線之間的距離
    b = abs(object_bottom_y - grid_y_values[-1-(grid_num_behind+1)] )  # 修正為絕對距離，防止負數

    # 計算物體距離
    if grid_num_behind >= 0:
       distance = (num_tiles - grid_num_behind) * tile_size_cm - (a / (a + b)) * tile_size_cm
    else:
       distance = 0 
    '''
    print(f"grid_y_values: {grid_y_values}")
    print(f"object_bottom_y: {object_bottom_y}")
    print("grid num :",grid_num_behind)
    print("y1=",grid_y_values[-1-grid_num_behind])
    print("y2=",grid_y_values[-1-(grid_num_behind+1)])
    print("a =",a)
    print("b =",b)
    print("distance: ",distance)
    '''
    return distance

previous_distance = {}

def calculate_speed(obj_id, current_distance, fps):
    """
    計算物體速度 (cm/s).
    obj_id: 物體的 ID.
    current_distance: 當前物體到攝像頭的距離 (cm).
    fps: 每秒幀數.
    """
    if fps == 0:
        return 0  # 如果 FPS 是 0，則無法計算速度

    frame_time_interval = 1.0 / fps  # 每幀的時間間隔 (秒)
    
    # 如果該物體已經有前幀的距離紀錄
    if obj_id in previous_distance:
        distance_diff = current_distance - previous_distance[obj_id]  # 距離差
        speed = distance_diff / frame_time_interval  # cm/s
    else:
        speed = 0  # 第一次看到該物體時，速度設置為 0

    # 更新上一幀的距離
    previous_distance[obj_id] = current_distance

    return speed

def draw_boxes(img, bbox, names, object_id, grid_y_values, tile_size_cm, camera_height_cm, num_tiles, fps, identities=None, frame_count=0, offset=(0, 0), trailslen=64, distances=None, speeds=None, x_offsets=None,
               angles=None):
    height, width, _ = img.shape

    # 移除未追踪物體的緩衝區
    if identities is not None:
        identity_set = set(identities)
        for key in list(data_deque.keys()):
            if key not in identity_set:
                data_deque.pop(key)

    # 當 identities 為 None 時，使用 bbox 的索引
    identities = identities if identities is not None else range(len(bbox))

    for i, box in enumerate(bbox):
        x1, y1, x2, y2 = [int(j) for j in box]
        x1 += offset[0]
        x2 += offset[0]
        y1 += offset[1]
        y2 += offset[1]

        # 計算物體的距離
        if distances is not None:
            distance = distances[i]
        else:
            bottom_edge = y2
            distance = calculate_distance(grid_y_values, bottom_edge, tile_size_cm, camera_height_cm, num_tiles)

        # 計算物體的速度
        if speeds is not None:
            speed = speeds[i]
        else:
            obj_id = int(identities[i])
            speed = calculate_speed(obj_id, distance, fps)

            # 計算物體的水平偏移
        if x_offsets is not None:
            x_offset = x_offsets[i]
        else:
        # fallback：若沒給，你可以自己算或設定 0
            x_offset = 0

            # 計算物體的偏航角
        if angles is not None:
            yaw_angle = angles[i]
        else:
            # fallback：若沒給，就設定 0
            yaw_angle = 0


        # 繪製邊界框
        color = compute_color_for_labels(object_id[i])
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        id = int(identities[i])  # 確保物件 ID 是唯一的
        data_deque.setdefault(id, deque(maxlen=trailslen))  # 建立緩衝區追蹤物件
        
        status = update_movement_status(id, x1, y1, x2, y2, frame_count)[1]

        

        if int(object_id[i]) < len(names):
            # 只顯示「類別+ID  距離」，不畫速度
            label = (
                f"{names[int(object_id[i])]}{id} "
                f"{distance:.2f} cm "
                f"{yaw_angle:.1f}Deg"
                if distance > 0 else "Object out of grid range"
            )
        else:
            label = (
                f"Unknown {id} 距離: {distance:.2f} cm "
                f"{yaw_angle:.1f}Deg"
                if distance > 0 else "Object out of grid range"
            )
        # 添加中心點到緩衝區
        center = (int((x2 + x1) / 2), int((y2 + y1) / 2))
        data_deque[id].appendleft(center)

        # 在圖像上畫出邊框和標籤
        UI_box(box, img, label=label, color=color, line_thickness=2)

        # 繪製物件的移動軌跡
        for j in range(1, len(data_deque[id])):
            if data_deque[id][j - 1] is None or data_deque[id][j] is None:
                continue
            thickness = int(np.sqrt(trailslen / float(j + j)) * 1.5)
            cv2.line(img, data_deque[id][j - 1], data_deque[id][j], color, thickness)

    return img



def load_classes(path):
    # Loads *.names file at 'path'
    with open(path, 'r') as f:
        names = f.read().split('\n')
    return list(filter(None, names))


def detect(save_img=False):
    frame_count = 0  # Initialize frame counter
    #last_status = {}  # 跟踪上一幀的運動狀態
    names, source, weights, view_img, save_txt, imgsz, trace = opt.names, opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size, not opt.no_trace 
    save_img = not opt.nosave and not source.endswith('.txt')  # save inference images
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))

    # Directories
    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir
    # initialize deepsort
    cfg_deep = get_config()
    cfg_deep.merge_from_file("deep_sort_pytorch/configs/deep_sort.yaml")
    deepsort = DeepSort(cfg_deep.DEEPSORT.REID_CKPT,
                        max_dist=cfg_deep.DEEPSORT.MAX_DIST, min_confidence=cfg_deep.DEEPSORT.MIN_CONFIDENCE,
                        nms_max_overlap=cfg_deep.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=cfg_deep.DEEPSORT.MAX_IOU_DISTANCE,
                        max_age=cfg_deep.DEEPSORT.MAX_AGE, n_init=cfg_deep.DEEPSORT.N_INIT, nn_budget=cfg_deep.DEEPSORT.NN_BUDGET,
                        use_cuda=True)

    # Initialize
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # 調整 imgsz 使其為 stride 的倍數
#YOLOv5 的 stride 通常是 32，這意味著影像大小應該是 32 的倍數，才能適配 YOLO 的卷積層。
#imgsz 是影像的目標大小
    if trace:
        model = TracedModel(model, device, opt.img_size)

    if half:
        model.half()  # to FP16

    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model']).to(device).eval()

    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride)

    # Get names and colors
    names = load_classes(names)
    #colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    old_img_w = old_img_h = imgsz
    old_img_b = 1

    t0 = time.time()
    for path, img, im0s, vid_cap in dataset:
        frame_count += 1
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                model(img, augment=opt.augment)[0]

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = model(img, augment=opt.augment)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t3 = time_synchronized()

        # Apply Classifier
        if classify:
            pred = apply_classifier(pred, modelc, img, im0s)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if webcam:  # batch_size >= 1
                p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), dataset.count
            else:
                p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # img.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, names[int(c)])  # add to string
                xywh_bboxs = []
                confs = []
                oids = []
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    x_c, y_c, bbox_w, bbox_h = xyxy_to_xywh(*xyxy)
                    xywh_obj = [x_c, y_c, bbox_w, bbox_h]
                    xywh_bboxs.append(xywh_obj)
                    confs.append([conf.item()])
                    oids.append(int(cls))
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    #if save_img or view_img:  # Add bbox to image
                        #label = f'{names[int(cls)]} {conf:.2f}'
                        #plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=1)
                xywhs = torch.Tensor(xywh_bboxs)
                confss = torch.Tensor(confs)
                
                outputs = deepsort.update(xywhs, confss, oids, im0)
                if len(outputs) > 0:
                    bbox_xyxy = outputs[:, :4]
                    identities = outputs[:, -2]
                    object_id = outputs[:, -1]
                    #print("ob:",object_id)
                    
                    # 只在每3幀時更新運動狀態
                    if frame_count % 3 == 0: 
                        for idx, (id, x1, y1, x2, y2) in enumerate(zip(identities, bbox_xyxy[:, 0], bbox_xyxy[:, 1], bbox_xyxy[:, 2], bbox_xyxy[:, 3])):
                               #print("frame_b:",frame_count)
                               status,last_status, current_diagonal, last_diagonal, current_coords, last_coords, frame_count = update_movement_status(int(id), x1, y1, x2, y2,frame_count)
                               label = f"{names[object_id[idx]]}{int(id)} {status}"
                               #print("i:",idx)
                               #print("name:",names[object_id[i]])
                               #print("label 1=",label)
                               #print("s1:",status)
                               
                               
                               center = (int((x2+x1)/ 2), int((y2+y2)/2))
                               
                               # Draw bounding box
                               color = compute_color_for_labels(object_id[idx])
                               cv2.rectangle(im0, (x1, y1), (x2, y2), color, 2)
                               box =(x1,y1,x2,y2)
                               UI_box(box, im0, label=label, color=color, line_thickness=2)
                               #draw_boxes(im0, bbox_xyxy, names, object_id,identities=identities,frame_count=frame_count)
                               
                                                            
            
                    else:
                        for idx, (id, x1, y1, x2, y2) in enumerate(zip(identities, bbox_xyxy[:, 0], bbox_xyxy[:, 1], bbox_xyxy[:, 2], bbox_xyxy[:, 3])):
                            #if int(id) in last_status:
                               #status = last_status[int(id)]
                               status, last_status, current_diagonal, last_diagonal, current_coords, last_coords, frame_count = update_movement_status(int(id), x1, y1, x2, y2,frame_count)
                            #movement_status = f"{status} (prev)"
                               label = f"{names[object_id[idx]]}{int(id)} {status}"
                               #print("name:",names)
                               #print("i:",idx)
                               #print("label 2=:",label)
                               #print("s2:",status)
                               color = compute_color_for_labels(object_id[idx])
                               cv2.rectangle(im0, (x1, y1), (x2, y2), color, 2)
                               box =(x1,y1,x2,y2)
                               UI_box(box, im0, label=label, color=color, line_thickness=2)
                               
                               #draw_boxes(im0, bbox_xyxy, names, object_id,identities=identities,frame_count=frame_count)
                             
                               
                                      
                    #color = compute_color_for_labels(object_id[i])
                    #cv2.rectangle(im0, (x1, y1), (x2, y2), color, 2)
                    #box =(x1,y1,x2,y2)
                    #UI_box(box, im0, label=label, color=color, line_thickness=2)          
                    #draw_boxes(im0, bbox_xyxy, names, object_id,identities,frame_count=frame_count)
                              
                               
                               

            # Print time (inference + NMS)
            print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')

            # Stream results
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                    print(f" The image with the result is saved in: {save_path}")
                else:  # 'video' or 'stream'
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                            save_path += '.mp4'
                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer.write(im0)

    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        #print(f"Results saved to {save_dir}{s}")

    print(f'Done. ({time.time() - t0:.3f}s)')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='yolov7.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--names', type=str, default='data/coco.names', help='*.cfg path')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--no-trace', action='store_true', help='don`t trace model')
    parser.add_argument('--trailslen', type=int, default=64, help='trails size (new parameter)')
    opt = parser.parse_args()
    print(opt)
    #check_requirements(exclude=('pycocotools', 'thop'))

    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['yolov7.pt']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()
