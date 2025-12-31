#!/usr/bin/env python3
import carla
import time
import json
import os

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'spawned_objects.json')

# 列出 Blueprint 分類
def list_blueprint_categories(bp_lib):
    cats = {}
    for bp in bp_lib:
        cat = bp.id.split('.')[0]
        cats.setdefault(cat, []).append(bp.id)
    return cats

# 互動式選擇
def choose_from_list(items, prompt):
    for i, it in enumerate(items): print(f"  [{i}] {it}")
    while True:
        sel = input(prompt).strip()
        if sel.isdigit() and int(sel) in range(len(items)):
            return items[int(sel)]
        print("請輸入有效編號。")

# Spawn 並回傳 actor
def spawn_actor(world, bp_id, transform):
    bp = world.get_blueprint_library().find(bp_id)
    actor = world.try_spawn_actor(bp, transform)
    return actor

# 儲存設定到檔案
def save_config(config):
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"已儲存 {len(config)} 筆配置到 {CONFIG_FILE}")

# 讀取設定
def load_config():
    if not os.path.exists(CONFIG_FILE): return []
    with open(CONFIG_FILE) as f:
        return json.load(f)

# 顯示說明
def show_help():
    print("可用指令:")
    print("  help          顯示本說明")
    print("  list cats     列出物件類別")
    print("  list bps      列出某類別底下的 Blueprint")
    print("  spawn         互動式 spawn 一個物件")
    print("  load          載入並 spawn 已保存的配置")
    print("  show          顯示已保存的配置")
    print("  delete        刪除指定配置並銷毀物件")
    print("  save          將當前配置寫入檔案")
    print("  exit          離開程式")

# 主程式
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    categories = list_blueprint_categories(bp_lib)

    # 載入並 spawn 歷史配置
    config = load_config()
    actor_list = []
    if config:
        print(f"載入 {len(config)} 筆歷史配置，開始 spawn…")
        for item in config:
            tf = carla.Transform(
                carla.Location(**item['location']),
                carla.Rotation(**item['rotation'])
            )
            actor = spawn_actor(world, item['bp_id'], tf)
            actor_list.append(actor)
            if actor:
                # 顯示 Blueprint、座標、以及 Actor 實際的 ID
                print(f"Spawn {item['bp_id']} @ {item['location']} @ID: {actor.id} → 成功")
            else:
                print(f"Spawn {item['bp_id']} @ {item['location']} → 失敗")
    # 互動式迴圈
    print("輸入 'help' 查看可用指令。")
    while True:
        cmd = input("CMD> ").strip().lower()
        if cmd in ('help','h'):
            show_help()

        elif cmd == 'list cats':
            print("物件類別:")
            for c in categories: print(f"  - {c}")

        elif cmd.startswith('list bps'):
            cat = input("請輸入類別名稱: ").strip()
            if cat in categories:
                print(f"{cat} 底下的 Blueprint:")
                for bp in categories[cat]: print(f"  - {bp}")
            else:
                print("無效的類別名稱。")

        elif cmd == 'spawn':
            print("選擇類別:")
            cat = choose_from_list(list(categories.keys()), "類別編號: ")
            print(f"已選: {cat}")
            print("選擇 Blueprint:")
            bp_id = choose_from_list(categories[cat], "Blueprint 編號: ")
            print(f"已選: {bp_id}")
            print("輸入位置與方向 (公尺/度):")
            x = float(input("  x: "))
            y = float(input("  y: "))
            z = float(input("  z: "))
            yaw   = float(input("  yaw: "))
            pitch = float(input("  pitch (default 0): ") or 0)
            roll  = float(input("  roll (default 0): ") or 0)
            tf = carla.Transform(
                carla.Location(x=x,y=y,z=z),
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
            )
            actor = spawn_actor(world, bp_id, tf)
            if actor:
                print(f"Spawn 成功 ID={actor.id} @ ({x},{y},{z})")
                config.append({
                    'bp_id': bp_id,
                    'location': {'x':x,'y':y,'z':z},
                    'rotation': {'pitch':pitch,'yaw':yaw,'roll':roll}
                })
                actor_list.append(actor)
            else:
                print("Spawn 失敗，可能該位置已被佔用。")

        elif cmd == 'show':
            if not config:
                print("目前無任何儲存的配置。")
            else:
                print("已保存的配置:")
                for i,item in enumerate(config):
                    loc=item['location']; rot=item['rotation']; bp=item['bp_id']
                    print(f"  [{i}] {bp} @ loc({loc['x']},{loc['y']},{loc['z']}) rot(yaw={rot['yaw']})")

        elif cmd == 'delete':
            if not config:
                print("目前無任何可刪除的配置。")
            else:
                print("選擇要刪除的配置編號:")
                for i,item in enumerate(config):
                    loc=item['location']; bp=item['bp_id']
                    print(f"  [{i}] {bp} @ loc({loc['x']},{loc['y']},{loc['z']})")
                idx = input("輸入配置編號: ").strip()
                if idx.isdigit() and int(idx) in range(len(config)):
                    i = int(idx)
                    actor = actor_list[i]
                    if actor:
                        actor.destroy()
                        print(f"已刪除 Actor ID={actor.id}")
                    else:
                        print("Actor 已不存在，或刪除失敗。")
                    # 從列表移除
                    config.pop(i)
                    actor_list.pop(i)
                    print("請使用 'save' 指令將更改寫入檔案。")
                else:
                    print("無效的編號。")

        elif cmd == 'save':
            save_config(config)

        elif cmd == 'load':
            config = load_config()
            actor_list.clear()
            print(f"重新載入 {len(config)} 筆配置。請重啟程式以 spawn 這些物件。")

        elif cmd in ('exit','quit'):
            print("離開程式。再見！")
            break

        else:
            print("未知指令，輸入 'help' 查詢。")

if __name__ == '__main__':
    main()
