#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from pynput import keyboard

class DroneSuperController:
    def __init__(self):
        rospy.init_node("drone_ego_manager_node")

        # 数据变量
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        
        # 逻辑状态
        self.state_machine = "IDLE"  # IDLE, TAKING_OFF, HOVER, EGO_CONTROL
        self.target_alt = 1.2        # 目标起飞高度 (米)

        # 订阅
        self.state_sub = rospy.Subscriber("/mavros/state", State, callback=self.state_cb)
        self.local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.pose_cb)

        # 发布 (与 ego_planner 同一话题)
        self.pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # 服务
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # 键盘监听
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        rospy.loginfo("--- 无人机协同控制器已就绪 ---")
        rospy.loginfo("1. [空格键]: 起飞 / 在 '悬停' 与 'EGO控制' 间切换")
        rospy.loginfo("2. [L 键]: 降落")

    def state_cb(self, msg):
        self.current_state = msg
        # 如果外部操作导致模式变为降落，重置脚本状态
        if msg.mode == "AUTO.LAND" and self.state_machine != "IDLE":
            self.state_machine = "IDLE"
            rospy.logwarn("检测到外部降落指令，重置控制器状态。")

    def pose_cb(self, msg):
        self.current_pose = msg

    def on_press(self, key):
        try:
            if key == keyboard.Key.space:
                self.handle_spacebar()
            elif hasattr(key, 'char') and key.char == 'l':
                self.request_land()
        except Exception as e:
            rospy.logerr(f"按键错误: {e}")

    def handle_spacebar(self):
        if self.state_machine == "IDLE":
            rospy.loginfo(">>> 准备起飞...")
            self.target_pose.pose.position.x = self.current_pose.pose.position.x
            self.target_pose.pose.position.y = self.current_pose.pose.position.y
            self.target_pose.pose.position.z = self.target_alt
            self.state_machine = "TAKING_OFF"
            
        elif self.state_machine == "HOVER":
            rospy.logwarn(">>> 切换模式：交出控制权给 EGO_PLANNER")
            self.state_machine = "EGO_CONTROL"
            
        elif self.state_machine == "EGO_CONTROL":
            rospy.loginfo(">>> 切换模式：强制接管控制权，原地悬停")
            self.target_pose.pose = self.current_pose.pose # 锁定当前点
            self.state_machine = "HOVER"

    def request_land(self):
        rospy.loginfo("发送降落请求...")
        self.set_mode_client.call(custom_mode='AUTO.LAND')
        self.state_machine = "IDLE"

    def run(self):
        rate = rospy.Rate(20)
        
        # OFFBOARD 预热
        for i in range(20):
            if rospy.is_shutdown(): break
            self.pos_pub.publish(self.target_pose)
            rate.sleep()

        while not rospy.is_shutdown():
            # 更新时间戳防止指令过期
            self.target_pose.header.stamp = rospy.Time.now()

            if self.state_machine == "TAKING_OFF":
                # 确保进入 OFFBOARD 并解锁
                if self.current_state.mode != "OFFBOARD":
                    self.set_mode_client.call(custom_mode='OFFBOARD')
                if not self.current_state.armed:
                    self.arming_client.call(value=True)

                self.pos_pub.publish(self.target_pose)

                # 判断起飞是否完成
                if abs(self.current_pose.pose.position.z - self.target_alt) < 0.2:
                    rospy.loginfo("起飞成功，当前处于悬停锁定状态。")
                    self.state_machine = "HOVER"

            elif self.state_machine == "HOVER":
                # 持续发布锁定点，这会覆盖 ego_planner 的指令
                self.pos_pub.publish(self.target_pose)

            elif self.state_machine == "EGO_CONTROL":
                # 重要：在此模式下【不发布】任何指令
                # 此时 ego_planner 的指令将成为话题的唯一来源
                pass

            elif self.state_machine == "IDLE":
                # 待机状态不发布指令，允许手动操作
                pass

            rate.sleep()

if __name__ == "__main__":
    controller = DroneSuperController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass