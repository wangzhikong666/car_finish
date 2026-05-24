#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, Float32
import math
import time
import copy

# ====================== 核心参数修改区 ======================
# 已经设为 False，准备真实上车跑！
SIMULATE_MODE = False  

CRUISE_DIST = 0.3     # 第一阶段巡航 0.3 米
TARGET_BOX_WIDTH = 180  # 停车距离阈值（如果机械臂够不到，可以把这个值调大，比如改成 200 或 220）
NORMAL_VEL = 0.2      
SLOW_VEL = 0.1        
STOP_DIST = 0.25      
# ==========================================================

# 状态定义
STATE_IDLE = 0
STATE_GLOBAL_CRUISE = 1   
STATE_VISUAL_TRACKING = 2 
STATE_SEARCHING = 3       
STATE_FINISH = 4
current_state = STATE_IDLE

# 全局变量
leg_detected = False
stop_signal = False
current_pose = None  # 初始为None
obstacle_front = False
center_error = 0.0
box_width = 0.0
detect_frame_count = 0
last_valid_detect_time = 0.0

# 发布器
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
massage_pub = rospy.Publisher('/massage/start', Bool, queue_size=10)
massage_pub = rospy.Publisher('/leg_detected', Bool, queue_size=10)

# 回调函数
def odom_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def leg_detect_callback(msg):
    global leg_detected, detect_frame_count, last_valid_detect_time
    if msg.data:
        detect_frame_count += 1
        if detect_frame_count >= 3:
            leg_detected = True
            last_valid_detect_time = time.time()
    else:
        detect_frame_count = 0
        if time.time() - last_valid_detect_time > 3.0:
            leg_detected = False

def scan_callback(msg):
    global obstacle_front
    if msg.ranges:
        front_ranges = msg.ranges[len(msg.ranges)//2-20 : len(msg.ranges)//2+20]
        valid_ranges = [r for r in front_ranges if 0.01 < r < 2.0]
        obstacle_front = any(r < STOP_DIST for r in valid_ranges)

def center_error_callback(msg):
    global center_error
    center_error = msg.data

def box_width_callback(msg):
    global box_width
    box_width = msg.data

# 核心控制函数
def emergency_stop():
    cmd_vel_pub.publish(Twist())

def calculate_distance(pose1, pose2):
    if pose1 is None or pose2 is None: return 0.0
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    return math.sqrt(dx*dx + dy*dy)

def move_straight_odom():
    """执行直线巡航逻辑"""
    global current_pose, SIMULATE_MODE
    
    # 如果没收到位置，造一个假位置让程序不卡死
    if current_pose is None:
        from geometry_msgs.msg import Pose
        current_pose = Pose()
        rospy.logwarn("⚠️ 未检测到里程计，使用虚拟坐标启动...")

    start_pose = copy.deepcopy(current_pose)
    rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = NORMAL_VEL
    
    rospy.loginfo(f"🚗 正在执行巡航：目标 {CRUISE_DIST} 米")
    
    # 模拟模式下的距离自增逻辑
    sim_dist = 0.0
    
    while not rospy.is_shutdown() and not stop_signal:
        if not SIMULATE_MODE and obstacle_front:
            emergency_stop()
            continue
            
        if SIMULATE_MODE:
            sim_dist += NORMAL_VEL * 0.1 # 模拟每0.1秒走的距离
            dist = sim_dist
        else:
            dist = calculate_distance(current_pose, start_pose)
            
        rospy.loginfo_throttle(1, f"进度: {dist:.2f}m / {CRUISE_DIST}m")
        
        if dist >= CRUISE_DIST:
            emergency_stop()
            return True
            
        cmd_vel_pub.publish(twist)
        rate.sleep()
    return False

def visual_tracking_loop():
    global current_state, leg_detected, center_error, box_width
    rate = rospy.Rate(10)
    rospy.loginfo("👁️ 进入视觉跟踪模式...")
    
    while not rospy.is_shutdown() and not stop_signal:
        if not leg_detected:
            emergency_stop()
            rospy.loginfo_throttle(2, "🔍 目标丢失，原地等待...")
            rate.sleep()
            continue

        twist = Twist()
        # 根据误差转弯（系数可微调）
        twist.angular.z = -0.003 * center_error 
        
        # 核心修改：靠得不够近就继续走，够近了就停车并连发唤醒信号
        if box_width < TARGET_BOX_WIDTH - 20:
            twist.linear.x = SLOW_VEL
        else:
            emergency_stop()
            rospy.loginfo("🎯 已就位！开始连续发送启动信号唤醒机械臂...")
            
            # 【终极修复】连续发送 5 次信号，每次间隔 0.5 秒，强行冲破机械臂防误触锁！
            for _ in range(5):
                massage_pub.publish(Bool(True))
                rospy.sleep(0.5)
                
            return True
            
        cmd_vel_pub.publish(twist)
        rate.sleep()
    return False

def run():
    global current_state, stop_signal, leg_detected, obstacle_front, current_pose, center_error, box_width, detect_frame_count, last_valid_detect_time
    
    if current_state == STATE_GLOBAL_CRUISE:
        if move_straight_odom():
            current_state = STATE_VISUAL_TRACKING
            rospy.loginfo("✅ 第一阶段完成，开启视觉跟踪")
            
    elif current_state == STATE_VISUAL_TRACKING:
        if visual_tracking_loop():
            current_state = STATE_FINISH
            rospy.loginfo("🎉 本次按摩任务交接完毕！大脑准备休息并重置...")
            
    elif current_state == STATE_FINISH:
        # 给机械臂留出充足的时间去按摩（比如 15 秒）
        rospy.loginfo("⏳ 等待机械臂按摩中 (20秒)...")
        rospy.sleep(20) 
        
        # 按摩结束，老板睡醒了！重置所有变量，准备下一轮寻找！
        rospy.loginfo("🔄 系统重置！重新开始新一轮的巡航与寻找！")
        
        # 重置关键变量
        stop_signal = False
        leg_detected = False
        obstacle_front = False
        center_error = 0.0
        box_width = 0.0
        detect_frame_count = 0
        last_valid_detect_time = 0.0
        current_pose = None 
        
        # 将状态直接切换到视觉跟踪阶段，跳过盲走！
        current_state = STATE_VISUAL_TRACKING

if __name__ == '__main__':
    rospy.init_node('massage_nav_node')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('/yolo/leg_detect', Bool, leg_detect_callback)
    rospy.Subscriber('/yolo/center_error', Float32, center_error_callback)
    rospy.Subscriber('/yolo/box_width', Float32, box_width_callback)
    
    rospy.loginfo("✅ 导航与任务调度节点已启动")
    
    # 直接设置状态，不再卡死等待传感器
    current_state = STATE_GLOBAL_CRUISE
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        run()
        rate.sleep()
