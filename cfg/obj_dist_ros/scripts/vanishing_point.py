import os
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
from collections import defaultdict

REJECT_HORIZONTAL_DEGREE_TH = 15.0  # 水平角度閾值
DISTANCE_THRESHOLD = 50.0  # 距離閾值，用於統計交點數量
MIN_LINE_LENGTH = 100  # 最小線段長度

def ReadImage(InputImagePath):
    if os.path.isfile(InputImagePath):
        InputImage = cv2.imread(InputImagePath)
        if InputImage is None:
            print("Image not read. Provide a correct path.")
            exit()
    else:
        print("\nEnter valid Image Path.\n")
        exit()
    return InputImage

def FilterLines(Lines, img_height, img_width):
    FinalLines = []
    for Line in Lines:
        [[x1, y1, x2, y2]] = Line
        if x1 != x2:
            m = (y2 - y1) / (x2 - x1)
        else:
            m = 1e6
        c = y2 - m * x2
        theta = math.degrees(math.atan(m))
        l = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        # Filter lines that are above the middle of the image and not close to horizontal
        if REJECT_HORIZONTAL_DEGREE_TH <= abs(theta) <= 90 and l > MIN_LINE_LENGTH and min(y1, y2) < img_height * 0.75:
            FinalLines.append([x1, y1, x2, y2, m, c, l])
    if len(FinalLines) > 15:
        FinalLines = sorted(FinalLines, key=lambda x: x[-1], reverse=True)
        FinalLines = FinalLines[:15]
    return FinalLines, len(FinalLines)

def GetLines(Image):
    GrayImage = cv2.cvtColor(Image, cv2.COLOR_BGR2GRAY)
    BlurGrayImage = cv2.GaussianBlur(GrayImage, (5, 5), 1)
    EdgeImage = cv2.Canny(BlurGrayImage, 50, 150)
    '''
    plt.imshow(EdgeImage, cmap='gray')
    plt.title('Edge Image')
    plt.axis('off')
    plt.show()
    '''
    
    Lines = cv2.HoughLinesP(EdgeImage, 1, np.pi / 180, threshold=30, minLineLength=50, maxLineGap=5)
    if Lines is None:
        print("Not enough lines found in the image for Vanishing Point detection.")
        exit(0)
    img_height, img_width = Image.shape[:2]
    FilteredLines, num_lines = FilterLines(Lines, img_height, img_width)
    print(f"Number of lines used for vanishing point detection: {num_lines}")
    return FilteredLines

def GetVanishingPoint(Lines, img_width):
    intersection_points = []
    for i in range(len(Lines)):
        for j in range(i + 1, len(Lines)):
            m1, c1 = Lines[i][4], Lines[i][5]
            m2, c2 = Lines[j][4], Lines[j][5]
            if m1 != m2:
                x0 = (c1 - c2) / (m2 - m1)
                y0 = m1 * x0 + c1
                intersection_points.append((x0, y0))
    
    if len(intersection_points) == 0:
        return None
    
    # 統計每個交點附近的交點數量
    point_counts = defaultdict(int)
    for p1 in intersection_points:
        for p2 in intersection_points:
            if np.linalg.norm(np.array(p1) - np.array(p2)) < DISTANCE_THRESHOLD:
                point_counts[p1] += 1
    
    # 選取交點數量最多的那個交點作為消失點
    vanishing_point = max(point_counts, key=point_counts.get)
    
    # 強制將消失點的 x 座標設置為圖像寬度的一半
    vanishing_point = (img_width / 2, vanishing_point[1])
    
    return vanishing_point

if __name__ == "__main__":
    InputImagePath = "1.jpg"
    Image = ReadImage(InputImagePath)
    Lines = GetLines(Image)
    img_height, img_width = Image.shape[:2]
    VanishingPoint = GetVanishingPoint(Lines, img_width)
    if VanishingPoint is None:
        print("Vanishing Point not found.")
    else:
        print(f"Vanishing Point Coordinates: {VanishingPoint}")
        ImageRGB = cv2.cvtColor(Image, cv2.COLOR_BGR2RGB)
        height, width, _ = Image.shape
        fig, ax = plt.subplots(figsize=(width / 100, height / 100), dpi=100)
        ax.imshow(ImageRGB)
        for Line in Lines:
            ax.plot([Line[0], Line[2]], [Line[1], Line[3]], color='yellow', linewidth=2)
            ax.scatter([Line[0], Line[2]], [Line[1], Line[3]], color='green', s=50)  # 標記線段的兩端
            
            # Extend the lines to see their intersection
            if Line[0] != Line[2]:  # avoid division by zero
                m = (Line[3] - Line[1]) / (Line[2] - Line[0])
                c = Line[1] - m * Line[0]
                x_ext = np.array([0, width])
                y_ext = m * x_ext + c
                ax.plot(x_ext, y_ext, color='blue', linestyle='dashed')
        
        ax.scatter(int(VanishingPoint[0]), int(VanishingPoint[1]), color='red', s=100)
        ax.set_xlim([0, width])
        ax.set_ylim([height, 0])
        ax.axis('off')
        plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
        plt.savefig('vanishing_point_result.jpg', bbox_inches='tight', pad_inches=0, dpi=100)
        plt.show()
