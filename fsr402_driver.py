#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 模块名称：FSR402 薄膜压力传感器力控反馈节点 (含 HAL 层模拟)
# 功    能：实时采集末端筋膜枪与人体的接触压力，实现柔性力控闭环与过载保护

import rospy
import random  # 引入随机库模拟真实物理底噪
from std_msgs.msg import Float32, Bool

# ================= 硬件与阈值配置 =================
# 假设阻抗肌肉过紧或物理干涉的危险阈值（单位：牛顿 N）
MAX_SAFE_FORCE = 30.0  
# 建议按摩的最佳力度区间
OPTIMAL_FORCE_MIN = 10.0
OPTIMAL_FORCE_MAX = 20.0

def read_adc_from_stm32():
    """
    硬件抽象层 (HAL)：从 STM32F407 读取 FSR402 的 ADC 转换数值。
    敏捷开发阶段：注入随机白噪声以模拟真实的传感器高频物理波动。
    """
    # 基础物理受力约为 15.5 N，叠加 -0.5 到 0.8 N 的随机环境底噪
    base_force = 15.5
    noise = random.uniform(-0.5, 0.8)
    return base_force + noise

def main():
    rospy.init_node('fsr402_pressure_node', anonymous=True)
    
    # 发布实时压力值，供 OLED 屏幕显示和健康大模型使用
    force_pub = rospy.Publisher('/massage/current_force', Float32, queue_size=10)
    # 发布超载警报，直接切断机械臂动力
    alert_pub = rospy.Publisher('/massage/force_alert', Bool, queue_size=10)
    
    # 设定采样频率为 20Hz (50ms响应时间)
    rate = rospy.Rate(20) 
    
    rospy.loginfo("[FSR402-Node] 压力传感器力控反馈系统已启动，正在采集接触力数据...")

    while not rospy.is_shutdown():
        try:
            # 1. 采集并计算实际受力 (带有真实物理波动)
            current_force = read_adc_from_stm32()
            force_pub.publish(Float32(current_force))
            
            # 2. 核心逻辑：安全阈值诊断与过载熔断
            if current_force > MAX_SAFE_FORCE:
                rospy.logerr("[FSR402-Node] 警告！检测到末端压力过载 (%.2f N)，触发物理熔断机制！", current_force)
                alert_pub.publish(Bool(True))
            else:
                alert_pub.publish(Bool(False))
                
            # 3. 健康管家逻辑：输出肌肉状态建议
            if current_force > OPTIMAL_FORCE_MAX:
                rospy.logwarn("[Health-Monitor] 当前阻抗偏大，判定该区域肌肉高度僵硬，建议增加放松时长。")
                
        except Exception as e:
            rospy.logerr("[FSR402-Node] 传感器读取异常: %s", str(e))
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
