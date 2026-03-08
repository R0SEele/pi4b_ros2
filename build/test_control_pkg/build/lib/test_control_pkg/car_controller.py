#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import struct
import math
import time
import os
import sys

# 工具函数
def clamp_i16(v: float) -> int:
    iv = int(round(v))
    if iv > 32767:
        return 32767
    if iv < -32768:
        return -32768
    return iv

def clamp_i32(v: float) -> int:
    iv = int(round(v))
    if iv > 2147483647:
        return 2147483647
    if iv < -2147483648:
        return -2147483648
    return iv

def pack_cmd_frame(vx_mps: float, vy_mps: float, wz_radps: float) -> bytes:
    vx_mm_s = clamp_i16(vx_mps * 1000.0)
    vy_mm_s = clamp_i16(vy_mps * 1000.0)
    wz_mrad_s = clamp_i16(wz_radps * 1000.0)
    payload = struct.pack('<Bhhh', 6, vx_mm_s, vy_mm_s, wz_mrad_s)
    checksum = 0
    for b in payload:
        checksum ^= b
    return bytes([0xAA, 0x55]) + payload + bytes([checksum, 0x0D, 0x0A])

def pack_host_odom_frame(x_m: float, y_m: float, theta_rad: float, flags: int = 1) -> bytes:
    msg_id = 0x21
    x_mm = clamp_i32(x_m * 1000.0)
    y_mm = clamp_i32(y_m * 1000.0)
    theta_mdeg = clamp_i32(theta_rad * 180000.0 / math.pi)
    payload = struct.pack('<BiiiB', msg_id, x_mm, y_mm, theta_mdeg, (flags & 0xFF))
    length = 14
    frame_wo_tail = bytes([0xAA, 0x55, length]) + payload
    checksum = 0
    for b in frame_wo_tail[2:]:
        checksum ^= b
    return frame_wo_tail + bytes([checksum, 0x0D, 0x0A])

# 节点类
class CarControllerNode(Node):
    def __init__(self):
        super().__init__('car_controller_node')
        
        # 参数声明
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('priority_file', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_priority.txt')
        
        # 获取参数
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = self.get_parameter('serial_baud').get_parameter_value().integer_value
        self.priority_file = self.get_parameter('priority_file').get_parameter_value().string_value
        
        # 小车速度
        self.speed_linear = 0.2
        self.speed_angular = 0.8
        
        # 串口初始化
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.02)
            self.get_logger().info(f"串口已连接：{self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"串口连接失败：{e}")
            sys.exit(1)
        
        # 状态变量
        self.host_x = 0.0
        self.host_y = 0.0
        self.host_theta = 0.0
        self.host_odom_ok = False
        self.executing_action = False
        self.action_end_time = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.action_queue = []  # 动作队列
        
        # 订阅话题
        self.sub_auto = self.create_subscription(Twist, '/cmd_vel', self.cb_auto, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        
        # 定时器
        self.timer = self.create_timer(0.05, self.loop)
        self.feedback_timer = self.create_timer(0.05, self.send_odom_feedback)
        
        # 强制等待优先级文件生成
        self.wait_for_priority_file()
        
        # 加载动作队列
        self.load_action_queue()
        
        self.get_logger().info("小车控制节点启动成功")
        self.get_logger().info(f"已加载{len(self.action_queue)}个景点动作，开始依次执行")

    def wait_for_priority_file(self):
        """等待优先级文件生成，最多等5分钟"""
        timeout = 300
        start_time = time.time()
        while not os.path.exists(self.priority_file):
            if time.time() - start_time > timeout:
                self.get_logger().error(f"等待优先级文件超时（{timeout}秒）！")
                sys.exit(1)
            time.sleep(2)  # 每2秒检查一次

    def load_action_queue(self):
        """加载优先级文件到动作队列"""
        if not os.path.exists(self.priority_file):
            self.get_logger().error("优先级文件不存在！")
            return
        
        with open(self.priority_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            for line in lines:
                line = line.strip()
                if line:
                    spot_id, priority, action, duration = line.split(',')
                    self.action_queue.append({
                        'spot_id': spot_id,
                        'action': action,
                        'duration': float(duration)
                    })
        
        # 删除文件避免重复加载
        os.remove(self.priority_file)

    # 里程计回调
    def cb_odom(self, msg: Odometry):
        self.host_x = msg.pose.pose.position.x
        self.host_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.host_theta = math.atan2(siny_cosp, cosy_cosp)
        self.host_odom_ok = True

    # 自动模式回调
    def cb_auto(self, msg: Twist):
        if not self.executing_action:
            self.current_vx = msg.linear.x
            self.current_vy = msg.linear.y
            self.current_wz = msg.angular.z
            self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))

    # 发送里程计反馈
    def send_odom_feedback(self):
        flags = 1 if self.host_odom_ok else 0
        frame = pack_host_odom_frame(self.host_x, self.host_y, self.host_theta, flags)
        self.ser.write(frame)

    # 启动动作
    def start_action(self, action: str, duration: float):
        self.get_logger().info(f"执行动作：{action}（持续{duration}秒）")
        self.executing_action = True
        self.action_end_time = time.monotonic() + duration
        
        # 动作映射
        if action == "forward_1m":
            self.current_vx, self.current_vy, self.current_wz = self.speed_linear, 0.0, 0.0
        elif action == "backward_1m":
            self.current_vx, self.current_vy, self.current_wz = -self.speed_linear, 0.0, 0.0
        elif action == "left_1m":
            self.current_vx, self.current_vy, self.current_wz = 0.0, self.speed_linear, 0.0
        elif action == "right_1m":
            self.current_vx, self.current_vy, self.current_wz = 0.0, -self.speed_linear, 0.0
        elif action == "spin_left_1圈":
            self.current_vx, self.current_vy, self.current_wz = 0.0, 0.0, self.speed_angular

    # 停止动作
    def stop(self):
        self.executing_action = False
        self.current_vx = self.current_vy = self.current_wz = 0.0
        self.ser.write(pack_cmd_frame(0.0, 0.0, 0.0))
        self.get_logger().info("小车已停止")

    # 主循环
    def loop(self):
        now = time.monotonic()
        
        # 执行队列里的动作
        if not self.executing_action and len(self.action_queue) > 0:
            next_action = self.action_queue.pop(0)
            self.start_action(next_action['action'], next_action['duration'])
            self.get_logger().info(f"执行第{len(self.action_queue)+1}个动作（景点{next_action['spot_id']}）")
        
        # 执行当前动作
        if self.executing_action:
            if now <= self.action_end_time:
                self.ser.write(pack_cmd_frame(self.current_vx, self.current_vy, self.current_wz))
            else:
                self.stop()

    # 销毁节点
    def destroy_node(self):
        self.ser.close()
        self.stop()
        super().destroy_node()

# 主函数
def main(args=None):
    rclpy.init(args=args)
    node = CarControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()