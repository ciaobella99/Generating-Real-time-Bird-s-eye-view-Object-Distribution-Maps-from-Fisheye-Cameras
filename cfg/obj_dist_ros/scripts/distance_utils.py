import numpy as np
import math

def calculate_x_distance(box_xyxy,
                         optical_center,
                         Zc_cm,
                         focal_length_x,
                         return_degree=True):
    """
    計算物體到影像光學中心中線的實際橫向距離（cm）、斜對角距離（cm）和偏航角。

    參數:
      box_xyxy        -- [x1, y1, x2, y2] 邊界框（像素）
      optical_center  -- (cx, cy) 校正後的光學中心（像素）
      Zc_cm           -- 物體到相機的縱向距離（cm）
      focal_length_x  -- 校正後的水平焦距 fx（像素）
      return_degree   -- 是否將角度以度為單位輸出，否則回傳弧度

    回傳:
      x_distance_cm        -- 水平偏移距離（cm），左側負，右側正
      diagonal_distance_cm -- 對角線距離（cm）
      angle                -- 偏航角（° 或 rad）
    """
    # 1. 邊界框中心 x 座標（像素）
    x1, _, x2, _ = box_xyxy[:4]
    x_c = (float(x1) + float(x2)) / 2.0

    # 2. 光學中心
    cx, _ = optical_center

    # 3. 轉純 Python 數值
    if hasattr(Zc_cm, 'item'):
        Zc_cm = Zc_cm.item()

    # 4. 計算水平偏移距離 X
    #    X = (x_c - cx) * Zc / fx
    x_distance_cm = (x_c - cx) * Zc_cm / focal_length_x

    # 5. 計算斜對角距離
    diagonal_distance_cm = np.hypot(x_distance_cm, Zc_cm)

    # 6. 計算偏航角
    #    θ = arctan2(X, Z)
    angle_rad = math.atan2(x_distance_cm, Zc_cm)
    if return_degree:
        angle = math.degrees(angle_rad)
    else:
        angle = angle_rad

    return x_distance_cm, diagonal_distance_cm, angle