import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

# 定義計算 k 值的函數
def calculate_k(h):
    # 已知的數據點
    h1, k1 = 50, 1220
    h2, k2 = 80, 3000

    # 計算斜率 a 和截距 b
    a = (k2 - k1) / (h2 - h1)
    b = k1 - a * h1

    return a * h + b

# 定義角度閾值，用於過濾接近水平或垂直的線條
REJECT_DEGREE_TH = 4.0

def ReadImage(InputImagePath):
    InputImage = cv2.imread(InputImagePath)  # 讀取圖像
    if InputImage is None:
        print("Image not read. Provide a correct path")
        exit()
    return InputImage

def FilterLines(Lines):
    FinalLines = []
    for Line in Lines:
        [[x1, y1, x2, y2]] = Line
        if x1 != x2:
            m = (y2 - y1) / (x2 - x1)
        else:
            m = 100000000
        c = y2 - m * x2
        theta = np.degrees(np.arctan(m))
        if REJECT_DEGREE_TH <= abs(theta) <= (90 - REJECT_DEGREE_TH):
            l = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            FinalLines.append([x1, y1, x2, y2, m, c, l])
    if len(FinalLines) > 15:
        FinalLines = sorted(FinalLines, key=lambda x: x[-1], reverse=True)
        FinalLines = FinalLines[:15]
    return FinalLines

def GetLines(Image):
    GrayImage = cv2.cvtColor(Image, cv2.COLOR_BGR2GRAY)
    BlurGrayImage = cv2.GaussianBlur(GrayImage, (5, 5), 1)
    EdgeImage = cv2.Canny(BlurGrayImage, 40, 255)
    Lines = cv2.HoughLinesP(EdgeImage, 1, np.pi / 180, 50, 10, 15)
    if Lines is None:
        print("Not enough lines found in the image for Vanishing Point detection.")
        exit(0)
    FilteredLines = FilterLines(Lines)
    return FilteredLines

def GetVanishingPoint(Lines):
    VanishingPoint = None
    MinError = float('inf')
    for i in range(len(Lines)):
        for j in range(i + 1, len(Lines)):
            m1, c1 = Lines[i][4], Lines[i][5]
            m2, c2 = Lines[j][4], Lines[j][5]
            if m1 != m2:
                x0 = (c1 - c2) / (m2 - m1)
                y0 = m1 * x0 + c1
                err = 0
                for k in range(len(Lines)):
                    m, c = Lines[k][4], Lines[k][5]
                    m_ = -1 / m
                    c_ = y0 - m_ * x0
                    x_ = (c - c_) / (m_ - m)
                    y_ = m_ * x_ + c_
                    l = np.sqrt((y_ - y0)**2 + (x_ - x0)**2)
                    err += l**2
                err = np.sqrt(err)
                if MinError > err:
                    MinError = err
                    VanishingPoint = [x0, y0]
    return VanishingPoint

def world_to_image(X, Z, h, optical_center, focal_length_x, focal_length_y):
    if Z == 0:
        Z = 1e-5
    cx, cy = optical_center
    x = cx + (X / Z) * focal_length_x
    y = cy + (h / Z) * focal_length_y
    return x, y

def generate_grid(image,
                  optical_center,
                  camera_height_cm=298,
                  tile_size_cm=13.5,
                  num_tiles=15,#15
                  focal_length_x=3.7188644469223198e+02,   # 你的 fx
                  focal_length_y=3.9512475008928237e+02):  # 你的 fy

    road_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    width, height = road_image.size
    grid_image = road_image.copy()
    draw = ImageDraw.Draw(grid_image)
    red = (255, 0, 0)

    grid_y_values = []

    # 橫向
    for i in range(num_tiles + 1):
        y_world = i * tile_size_cm  #   i 從 0 到 num_tiles，計算每條橫線在地面上的 y（與相機平行方向）座標 y_world
        start_x, start_y = world_to_image(-2 * width, y_world, camera_height_cm,    #   用 world_to_image 將世界座標 (x, y, z) 投影回影像座標
                                          optical_center,
                                          focal_length_x, focal_length_y)
        end_x, end_y     = world_to_image( 2 * width, y_world, camera_height_cm,
                                          optical_center,
                                          focal_length_x, focal_length_y)
        draw.line((start_x, start_y, end_x, end_y), fill=red, width=2)
        grid_y_values.append(start_y)

    # 縱向
    num_columns = int(2 * width / tile_size_cm)
    for i in range(-num_columns, num_columns + 1):
        x_world = i * tile_size_cm
        sx, sy = world_to_image(x_world,     0.1, camera_height_cm,
                                optical_center,
                                focal_length_x, focal_length_y)
        ex, ey = world_to_image(x_world, num_tiles * tile_size_cm, camera_height_cm,
                                optical_center,
                                focal_length_x, focal_length_y)
        draw.line((sx, sy, ex, ey), fill=red, width=2)

    return cv2.cvtColor(np.array(grid_image), cv2.COLOR_RGB2BGR), grid_y_values





if __name__ == "__main__":
    road_image_path = '50cm.jpg'
    image = ReadImage(road_image_path)
    
    camera_height_cm = 50
    tile_size_cm = 39.5
    
    Lines = GetLines(image)
    VanishingPoint = GetVanishingPoint(Lines)
    vanishing_point_x, vanishing_point_y = int(round(VanishingPoint[0])), int(round(VanishingPoint[1]))
    
    k = calculate_k(camera_height_cm)

    road_image = Image.open(road_image_path)
    width, height = road_image.size
    
    grid_image = road_image.copy()
    draw = ImageDraw.Draw(grid_image)
    
    blank_image = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    draw_w = ImageDraw.Draw(blank_image)
    
    num_tiles = 6
    for i in range(num_tiles + 1):
        y_world = i * tile_size_cm
        start_x, start_y = world_to_image(-2 * width, y_world, camera_height_cm, (vanishing_point_x, vanishing_point_y), k)
        end_x, end_y = world_to_image(2 * width, y_world, camera_height_cm, (vanishing_point_x, vanishing_point_y), k)
        draw.line((start_x, start_y, end_x, end_y), fill=(255, 0, 0), width=2)
        draw_w.line((start_x, start_y, end_x, end_y), fill=(255, 0, 0), width=2)

    num_columns = int(2 * width / tile_size_cm)
    for i in range(-num_columns, num_columns + 1):
        x_world = i * tile_size_cm
        start_x, start_y = world_to_image(x_world, 0.1, camera_height_cm, (vanishing_point_x, vanishing_point_y), k)
        end_x, end_y = world_to_image(x_world, num_tiles * tile_size_cm, camera_height_cm, (vanishing_point_x, vanishing_point_y), k)
        draw.line((start_x, start_y, end_x, end_y), fill=(255, 0, 0), width=2)
        draw_w.line((start_x, start_y, end_x, end_y), fill=(255, 0, 0), width=2)

    grid_image_np = np.array(grid_image)
    grid_image_w = np.array(blank_image)

    plt.figure(figsize=(width / 100, height / 100))
    plt.imshow(grid_image_np)
    plt.title(f'Grid with k = {k}')
    plt.axis('off')
    plt.show()

    blank_image.save('grid_overlay.png')
    plt.show()

    print(f"Vanishing Point Coordinates (rounded): ({vanishing_point_x}, {vanishing_point_y})")
    print(f"k value: {k}")
