#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 模块名称：跨公网 MQTT 云边协同控制节点 (IoT Edge Bridge)
# 功    能：监听手机终端跨公网下发的指令，将其转化为底层的紧急启停控制信号

import rospy
from std_msgs.msg import Bool
import paho.mqtt.client as mqtt

# ================= 配置 MQTT 云端参数 =================
BROKER_ADDRESS = "broker.emqx.io"  # 免费公共测试中转站
PORT = 1883                        # MQTT 标准端口
CMD_TOPIC = "qrs/car/cmd"          # 我们约定的云端指令下发通道

# 全局变量：ROS 话题发布者
stop_pub = None

def on_connect(client, userdata, flags, rc):
    """
    回调函数：当成功连接到云端服务器时触发
    """
    if rc == 0:
        rospy.loginfo("[IoT-Bridge] 成功连接到云端 MQTT 服务器!")
        # 连接成功后，立刻订阅手机端的指令话题
        client.subscribe(CMD_TOPIC)
    else:
        rospy.logerr("[IoT-Bridge] 连接云端失败，错误码: %d", rc)

def on_message(client, userdata, msg):
    """
    回调函数：当收到手机端发来的消息时触发
    """
    # 将接收到的网络字节流解码为字符串
    payload = msg.payload.decode('utf-8')
    rospy.loginfo("[IoT-Bridge] 收到跨网指令: %s", payload)

    # 核心安全逻辑：解析手机指令并切断/恢复动力
    if payload == "STOP":
        rospy.logwarn("[IoT-Bridge] 触发最高优先级急停指令！立即锁死底盘与机械臂。")
        # 向底层急停话题发布 True 信号
        stop_pub.publish(Bool(True))
        
    elif payload == "START":
        rospy.loginfo("[IoT-Bridge] 收到启动指令，解除系统急停状态。")
        # 向底层急停话题发布 False 信号
        stop_pub.publish(Bool(False))

def main():
    global stop_pub
    
    # 初始化 ROS 节点
    rospy.init_node('iot_mqtt_bridge', anonymous=True)

    # 创建一个 ROS 发布者，负责向底层电机/MoveIt 发送急停布尔值
    stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)

    # 实例化 MQTT 客户端
    client = mqtt.Client()
    
    # 绑定回调函数
    client.on_connect = on_connect
    client.on_message = on_message

    rospy.loginfo("[IoT-Bridge] 正在尝试连接公网中转服务器: %s ...", BROKER_ADDRESS)
    
    try:
        # 连接到 Broker
        client.connect(BROKER_ADDRESS, PORT, 60)
        
        # 开启后台独立线程，自动处理网络收发与重连，不阻塞 ROS 主进程
        client.loop_start()
    except Exception as e:
        rospy.logerr("[IoT-Bridge] 网络连接异常，请检查板卡是否具备 4G/5G 联网能力: %s", str(e))

    # 保持 ROS 节点存活，等待系统退出信号
    rospy.spin()

    # 当节点被关闭（如按下 Ctrl+C）时，优雅地断开网络连接
    client.loop_stop()
    client.disconnect()
    rospy.loginfo("[IoT-Bridge] 网络连接已安全断开。")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
