#!/usr/bin/env python3
"""
get_actor_dimensions.py
輸入 CARLA 中已 spawn Actor 的 ID，輸出其 bounding box:
  - 長度 (length)
  - 寬度 (width)
  - 高度 (height)
  - 半長度 (half_length)
  - 半寬度 (half_width)
  - 半高度 (half_height)
以及物件中心在世界座標系的座標 (world_center)
"""
import carla
import sys


def main():
    # 1. 連線 CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    # 2. 讀取 Actor ID
    actor_input = input("請輸入 Actor ID: ").strip()
    try:
        actor_id = int(actor_input)
    except ValueError:
        print("輸入錯誤：請輸入數字 Actor ID。")
        sys.exit(1)

    # 3. 取得 Actor
    actor = world.get_actor(actor_id)
    if not actor:
        print(f"找不到 Actor {actor_id}")
        sys.exit(1)

    # 4. 計算尺寸
    ext = actor.bounding_box.extent
    half_length = ext.x  # 從中心到前緣
    half_width  = ext.y  # 從中心到側邊
    half_height = ext.z  # 從中心到頂部
    length = half_length * 2
    width  = half_width  * 2
    height = half_height * 2

    # 5. 計算世界座標系下的 bouding box 中心
    local_center = actor.bounding_box.location
    world_center = actor.get_transform().transform(local_center)

    # 6. 輸出結果
    print(f"Actor {actor_id} ({actor.type_id})")
    print(f"長度 (length)       : {length:.2f} m")
    print(f"寬度 (width)        : {width:.2f} m")
    print(f"高度 (height)       : {height:.2f} m")
    print(f"半長度 (half_length): {half_length:.2f} m")
    print(f"半寬度 (half_width) : {half_width:.2f} m")
    print(f"半高度 (half_height): {half_height:.2f} m")
    print("-----")
    print(f"BoundingBox 中心 世界座標 → x={world_center.x:.2f}, y={world_center.y:.2f}, z={world_center.z:.2f}")

if __name__ == '__main__':
    main()

