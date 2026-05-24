#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无限制满血版：YOLOv8腿部检测 + 完整的ROS信号发布（包含偏移量和距离宽度）
"""
from ultralytics import YOLO
import cv2
import os
import rospy
from std_msgs.msg import Bool, Float32

# ==================== 加载模型 ====================
def load_model(weight_path='best.pt'):
    # 尝试多个可能存放模型权重的路径
    alt_paths = [
        'leg_detection/train/weights/best.pt', 
        'runs/detect/train/weights/best.pt', 
        'runs/detect/train2/weights/best.pt', 
        'last.pt', 
        'best.pt'
    ]
    for path in alt_paths:
        if os.path.exists(path):
            weight_path = path
            break
            
    print(f"✅ 成功加载模型: {weight_path}")
    model = YOLO(weight_path)
    return model

# ==================== 核心视频流识别 ====================
def predict_video(model, source=0, leg_pub=None, center_pub=None, width_pub=None):
    results = model.predict(
        source=source,
        conf=0.4,       # 置信度设为0.4，避免把椅子腿认成人的腿
        iou=0.45,
        save=False,     # 实时运行不需要保存图片
        show=True,      # 显示实时画面
        stream=True     # 流式处理，极大地节省内存
    )
    
    for result in results:
        boxes = result.boxes
        leg_detected = len(boxes) > 0
        
        # 1. 向大脑发布：是否看到腿了？(True/False)
        if leg_pub is not None:
            leg_pub.publish(leg_detected)
            
        if leg_detected:
            # 提取第一个目标（通常是最清晰的那个腿）的坐标
            box = boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            
            # 计算检测框宽度（宽度越大，代表离得越近）
            box_w = x2 - x1
            
            # 计算画面中心偏移量 (假设标准摄像头宽度为640，中心为320)
            center_x = (x1 + x2) / 2
            error = center_x - 320.0
            
            # 2. 向大脑发布：偏移量和宽度数据
            if center_pub is not None:
                center_pub.publish(float(error))
            if width_pub is not None:
                width_pub.publish(float(box_w))
                
            rospy.loginfo(f"🎯 锁定目标! 偏移量: {error:.1f}, 宽度(距离): {box_w:.1f}")

# ==================== 主程序入口 ====================
if __name__ == '__main__':
    # 初始化视觉节点
    rospy.init_node('leg_detection_node', anonymous=True)
    
    # 初始化三个关键的发布器
    leg_pub = rospy.Publisher('/yolo/leg_detect', Bool, queue_size=10)
    center_pub = rospy.Publisher('/yolo/center_error', Float32, queue_size=10)
    width_pub = rospy.Publisher('/yolo/box_width', Float32, queue_size=10)
    
    rospy.loginfo("👀 满血版视觉节点已启动，算力全开，等待画面...")
    
    # 加载模型
    model = load_model()
    
    try:
        # source=0 代表使用默认外接摄像头
        predict_video(model, source=0, leg_pub=leg_pub, center_pub=center_pub, width_pub=width_pub)
    except KeyboardInterrupt:
        rospy.loginfo("🛑 视觉识别被手动停止")
    except Exception as e:
        rospy.logerr(f"❌ 视觉识别出错：{str(e)}")
        
    # 释放资源
    cv2.destroyAllWindows()
