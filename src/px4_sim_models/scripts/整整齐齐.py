#!/usr/bin/env python3
import os

# 路径设置
current_dir = os.path.dirname(os.path.abspath(__file__))
world_path = os.path.join(current_dir, "../worlds/test.world")

def generate():
    # 墙体定义：后墙在 x=-1，前墙在 x=29，左右墙在 y=+/-2.5
    # (0,0) 就在进入房间后的起始位置
    header = """<?xml version="1.0" ?><sdf version="1.6"><world name="default">
    <include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>
    <model name="room_frame"><static>true</static>
    <link name="north"><pose>14 2.5 0.5 0 0 0</pose><collision name="c"><geometry><box><size>30 0.1 1</size></box></geometry></collision><visual name="v"><geometry><box><size>30 0.1 1</size></box></geometry></visual></link>
    <link name="south"><pose>14 -2.5 0.5 0 0 0</pose><collision name="c"><geometry><box><size>30 0.1 1</size></box></geometry></collision><visual name="v"><geometry><box><size>30 0.1 1</size></box></geometry></visual></link>
    <link name="east"><pose>29 0 0.5 0 0 0</pose><collision name="c"><geometry><box><size>0.1 5 1</size></box></geometry></collision><visual name="v"><geometry><box><size>0.1 5 1</size></box></geometry></visual></link>
    <link name="west"><pose>-1 0 0.5 0 0 0</pose><collision name="c"><geometry><box><size>0.1 5 1</size></box></geometry></collision><visual name="v"><geometry><box><size>0.1 5 1</size></box></geometry></visual></link></model>"""
    
    body = ""

    # --- 1. 静态障碍物 (固定 40 个，交错排列) ---
    # 定义一些固定的位置和尺寸 (x, y, sx, sy, sz, shape)
    static_obs_list = []
    for i in range(1, 21):
        # 左侧障碍
        static_obs_list.append((i * 1.4, 1.5, 0.6, 0.4, 1.2, 'box'))
        # 右侧障碍
        static_obs_list.append((i * 1.4 + 0.7, -1.5, 0.5, 0.5, 1.5, 'cylinder'))

    for i, obs in enumerate(static_obs_list):
        x, y, sx, sy, sz, stype = obs
        if stype == 'box':
            geom = f"<box><size>{sx} {sy} {sz}</size></box>"
        else:
            geom = f"<cylinder><radius>{sx/2}</radius><length>{sz}</length></cylinder>"
        
        body += f"""
    <model name="static_{i}"><pose>{x} {y} {sz/2} 0 0 0</pose><static>true</static>
    <link name="l"><collision name="c"><geometry>{geom}</geometry></collision>
    <visual name="v"><geometry>{geom}</geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual></link></model>"""

    # --- 2. 动态障碍物 (固定 10 个圆柱体, 2m/s) ---
    for i in range(10):
        x_fixed = 3.0 + i * 2.5 # 每个动态障碍物在 X 轴上的固定位置
        y_start = -1.8
        y_end = 1.8
        duration = 3.6 / 2.0 # 距离 3.6m / 速度 2m/s = 1.8秒
        
        body += f"""
    <actor name="dynamic_cyl_{i}">
      <link name="link"><visual name="v"><geometry><cylinder><radius>0.3</radius><length>1.2</length></cylinder></geometry>
      <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script></material></visual></link>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="linear">
          <waypoint><time>0</time><pose>{x_fixed} {y_start} 0.6 0 0 0</pose></waypoint>
          <waypoint><time>{duration}</time><pose>{x_fixed} {y_end} 0.6 0 0 0</pose></waypoint>
          <waypoint><time>{duration*2}</time><pose>{x_fixed} {y_start} 0.6 0 0 0</pose></waypoint>
        </trajectory></script></actor>"""

    with open(world_path, "w") as f:
        f.write(header + body + "</world></sdf>")

if __name__ == "__main__":
    generate()