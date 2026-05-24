#!/usr/bin/env python3
# coding=utf-8

import sys
import rospy
import moveit_commander
from std_msgs.msg import Bool
import copy
import time
import threading

class MassageMoveItController:
    def __init__(self):
        # 1. 初始化 MoveIt 和 ROS 节点
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('my_massage_core_node', anonymous=True)

        # 2. 连接到机械臂规划组
        self.arm_group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.arm_group_name)

        # 丝滑运动参数设置
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.2)

        # 防误触逻辑变量
        self.detection_count = 0      
        self.last_reset_time = time.time() 
        self.threshold_count = 3      
        self.window_size = 5.0        
        self.is_massaging = False

        self.leg_sub = rospy.Subscriber('/leg_detected', Bool, self.vision_callback)
        rospy.loginfo("🚀 [5自由度·关节控制版] 节点已就绪！")

        # ================= 以下为新增的初始化姿态控制代码 =================
        # 根据 GUI 面板提取的待命姿态关节角度 (单位: 弧度)
        ready_joints = [0.000, 0.412, 1.124, 0.178, 0.000]
        
        rospy.loginfo("⚙️ 正在将机械臂移动至设定的初始待命姿态...")
        try:
            self.move_group.go(ready_joints, wait=True)
            rospy.loginfo("✅ 初始姿态已就位，进入监听状态。")
        except Exception as e:
            rospy.logerr(f"❌ 初始化姿态移动失败: {str(e)}")
        # ==================================================================



    def vision_callback(self, msg):
        if self.is_massaging:
            return

        current_time = time.time()
        if current_time - self.last_reset_time > self.window_size:
            self.detection_count = 0
            self.last_reset_time = current_time

        if msg.data == True:
            self.detection_count += 1
            rospy.loginfo(f"🔍 疑似目标！检测计数: {self.detection_count}/{self.threshold_count}")

            if self.detection_count >= self.threshold_count:
                rospy.loginfo("🎯 确认为腿部目标！启动舒缓按摩模式...")
                self.is_massaging = True
                self.detection_count = 0 
                
                # 启动新线程去按摩
                massage_thread = threading.Thread(target=self.execute_massage_sequence)
                massage_thread.start()

    def execute_massage_sequence(self):
        try:
            rospy.loginfo("👉 启动降维打击：直接控制关节角度...")
            
            # 直接读取当前 5 个关节的角度
            current_joints = self.move_group.get_current_joint_values()
            
            for i in range(3):
                rospy.loginfo(f"🔨 第 {i+1} 组按压 (筋膜枪冲击)...")
                
                # 动作 1：向下压（点头）
                target_joints = copy.deepcopy(current_joints)
                
                # 核心微操：控制第 3 个关节（手肘）弯曲
                # 0.15 弧度大约是 8.5 度，这是一个平滑的点头动作
                target_joints[3] += 0.75  
                
                # 直接转动关节，完美绕过逆向运动学计算，绝对不报 5 秒超时！
                self.move_group.go(target_joints, wait=True) 
                
                rospy.sleep(0.8) # 按到底稍微停顿，让筋膜枪震一会儿
                
                # 动作 2：收回原位
                self.move_group.go(current_joints, wait=True)
                rospy.sleep(0.5)

            rospy.loginfo("✅ 按摩完成，进入 10 秒冷静期...")
            rospy.sleep(10.0) 
            
        except Exception as e:
            rospy.logerr(f"❌ 机械臂运动出错: {str(e)}")
            
        finally:
            self.is_massaging = False
            self.last_reset_time = time.time()
            rospy.loginfo("🔋 冷静期结束，系统回到待命状态。")

if __name__ == '__main__':
    try:
        MassageMoveItController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
