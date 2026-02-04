#!/usr/bin/env python3
import os
import math
import rospy

# 路径设置
current_dir = os.path.dirname(os.path.abspath(__file__))
world_path = os.path.join(current_dir, "../worlds/test.world")

def generate():
    # --- 修改点 1: 加入物理引擎优化参数 ---
    header = """<?xml version="1.0" ?>
<sdf version="1.6">
<world name="default">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin name="ros_interface" filename="libgazebo_ros_api_plugin.so"/>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <model name="room_frame"><static>true</static>
 <link name="north"><pose>19 2.5 1.5 0 0 0</pose><collision name="c"><geometry><box><size>40 0.1 3.0</size></box></geometry></collision><visual name="v"><geometry><box><size>40 0.1 3.0</size></box></geometry></visual></link>
    <link name="south"><pose>19 -2.5 1.5 0 0 0</pose><collision name="c"><geometry><box><size>40 0.1 3.0</size></box></geometry></collision><visual name="v"><geometry><box><size>40 0.1 3.0</size></box></geometry></visual></link>
    <link name="east"><pose>39 0 1.5 0 0 0</pose><collision name="c"><geometry><box><size>0.1 5 3.0</size></box></geometry></collision><visual name="v"><geometry><box><size>0.1 5 3.0</size></box></geometry></visual></link>
    <link name="west"><pose>-1 0 1.5 0 0 0</pose><collision name="c"><geometry><box><size>0.1 5 3.0</size></box></geometry></collision><visual name="v"><geometry><box><size>0.1 5 3.0</size></box></geometry></visual></link></model>"""
    
    body = ""

    # --- 1. 静态障碍物 (完全保留你的逻辑) ---
    for i in range(50):
        x = 1.2 + i * 0.68
        y = 2.0 * math.sin(i * 1.5)
        sx = 0.15 if i % 2 == 0 else 0.2
        sz = 1.0 if i % 3 == 0 else 1.6
        z_offset = 1.3 if i % 4 == 0 else sz/2
        
        shape = 'box' if i % 2 == 0 else 'cylinder'
        geom = f"<box><size>{sx} {sx} {sz}</size></box>" if shape == 'box' else f"<cylinder><radius>{sx}</radius><length>{sz}</length></cylinder>"
        
        body += f"""
    <model name="static_{i}"><pose>{x:.2f} {y:.2f} {z_offset:.2f} 0 0 0</pose><static>true</static>
    <link name="l"><collision name="c"><geometry>{geom}</geometry></collision>
    <visual name="v"><geometry>{geom}</geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual></link></model>"""

    # --- 2. 动态障碍物 (保留运动方向，减少到 5 个) ---
    # for i in range(5):
    #     center_x = 3.0 + i * 2.5
    #     center_y = 0.5 * math.cos(i)
    #     angle = i * (math.pi / 3)
    #     dist = 1.5 + (i % 3) * 1.0
        
    #     dx, dy = (dist/2)*math.cos(angle), (dist/2)*math.sin(angle)
    #     s_x, s_y = center_x - dx, center_y - dy
    #     e_x, e_y = center_x + dx, center_y + dy
        
    #     duration = dist / 2.0
    #     z_p = 0.8 + (i % 3) * 0.4

    #     body += f"""
    # <actor name="dynamic_{i}">
    #   <link name="link"><visual name="v"><geometry><cylinder><radius>0.12</radius><length>0.8</length></cylinder></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script></material></visual></link>
    #   <script><loop>true</loop><auto_start>true</auto_start>
    #     <trajectory id="{i}" type="linear"> <waypoint><time>0.0</time><pose>{s_x:.2f} {s_y:.2f} {z_p:.2f} 0 0 0</pose></waypoint>
    #       <waypoint><time>{duration:.2f}</time><pose>{e_x:.2f} {e_y:.2f} {z_p:.2f} 0 0 0</pose></waypoint>
    #       <waypoint><time>{duration*2:.2f}</time><pose>{s_x:.2f} {s_y:.2f} {z_p:.2f} 0 0 0</pose></waypoint>
    #     </trajectory></script></actor>"""

    with open(world_path, "w") as f:
        f.write(header + body + "</world></sdf>")

if __name__ == "__main__":
    try:
        generate()
        rospy.init_node('world_generator')
        rospy.loginfo("World file 'test.world' generated with Physics Fix.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass