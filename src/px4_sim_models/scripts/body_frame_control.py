#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from pynput import keyboard

class DroneBodyController:
    def __init__(self):
        rospy.init_node("drone_auto_takeoff_control")

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.vel_msg = TwistStamped()
        self.pos_msg = PoseStamped()
        
        self.is_taking_off = False
        self.takeoff_complete = False
        self.target_alt = 1.0

        # 订阅
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback=self.state_cb)
        self.local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.pose_cb)

        # 发布
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # 服务
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("脚本就绪。步骤：1.按[空格]自动起飞 2.达到2m后用WASD控制")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def on_press(self, key):
        if not self.takeoff_complete:
            if key == keyboard.Key.space:
                self.start_takeoff()
            return

        try:
            if hasattr(key, 'char'):
                if key.char == 'w': self.vel_msg.twist.linear.x = 0.8
                elif key.char == 's': self.vel_msg.twist.linear.x = -0.8
                elif key.char == 'a': self.vel_msg.twist.linear.y = 0.8
                elif key.char == 'd': self.vel_msg.twist.linear.y = -0.8
                elif key.char == 'q': self.vel_msg.twist.angular.z = 0.5
                elif key.char == 'e': self.vel_msg.twist.angular.z = -0.5
                elif key.char == 'l': self.land()

            if key == keyboard.Key.up: self.vel_msg.twist.linear.z = 0.5
            elif key == keyboard.Key.down: self.vel_msg.twist.linear.z = -0.5
        except: pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char'):
                if key.char in ['w', 's']: self.vel_msg.twist.linear.x = 0
                elif key.char in ['a', 'd']: self.vel_msg.twist.linear.y = 0
                elif key.char in ['q', 'e']: self.vel_msg.twist.angular.z = 0
            if key in [keyboard.Key.up, keyboard.Key.down]:
                self.vel_msg.twist.linear.z = 0
        except: pass

    def start_takeoff(self):
        rospy.loginfo("正在请求 OFFBOARD 并解锁...")
        self.is_taking_off = True
        # 初始化起飞点为当前位置上方 2.0m
        self.pos_msg.pose.position.x = self.current_pose.pose.position.x
        self.pos_msg.pose.position.y = self.current_pose.pose.position.y
        self.pos_msg.pose.position.z = self.target_alt

    def land(self):
        self.set_mode_client.call(custom_mode='AUTO.LAND')
        self.takeoff_complete = False

    def run(self):
        rate = rospy.Rate(20)
        
        # OFFBOARD 预热：在切换模式前必须先发送一些点
        for i in range(20):
            self.pos_pub.publish(self.pos_msg)
            rate.sleep()

        while not rospy.is_shutdown():
            if self.is_taking_off and not self.takeoff_complete:
                # 模式切换逻辑
                if self.current_state.mode != "OFFBOARD":
                    self.set_mode_client.call(custom_mode='OFFBOARD')
                if not self.current_state.armed:
                    self.arming_client.call(value=True)

                # 发布起飞位置指令
                self.pos_pub.publish(self.pos_msg)
                
                # 到达高度判断
                if self.current_pose.pose.position.z >= (self.target_alt - 0.1):
                    self.takeoff_complete = True
                    self.is_taking_off = False
                    rospy.loginfo("起飞完成！已切换至机体系键盘控制。")
            
            elif self.takeoff_complete:
                # 发布机体系速度指令
                self.vel_msg.header.stamp = rospy.Time.now()
                self.vel_msg.header.frame_id = "base_link"
                self.vel_pub.publish(self.vel_msg)
            
            else:
                # 待机状态，持续发送当前点以维持 OFFBOARD
                self.pos_pub.publish(self.pos_msg)

            rate.sleep()

if __name__ == "__main__":
    controller = DroneBodyController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass