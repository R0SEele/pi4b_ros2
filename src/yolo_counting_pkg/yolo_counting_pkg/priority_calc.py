#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import os
import time

class PriorityCalcNode(Node):
    def __init__(self):
        super().__init__('priority_calc_node')
        
        # 参数声明
        self.declare_parameter('config_path', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/config/params.yaml')
        self.declare_parameter('detection_file', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_detections.txt')
        self.declare_parameter('priority_file', '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_priority.txt')
        
        # 获取参数
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.detection_file = self.get_parameter('detection_file').get_parameter_value().string_value
        self.priority_file = self.get_parameter('priority_file').get_parameter_value().string_value
        
        # 强制等待YOLO文件生成（最多等5分钟）
        self.wait_for_yolo_file()
        
        # 计算优先级
        self.calc_priority()
        
        self.get_logger().info(f"优先级计算完成，结果保存至：{self.priority_file}")

    def wait_for_yolo_file(self):
        """等待YOLO检测文件生成，最多等300秒"""
        timeout = 300
        start_time = time.time()
        while not os.path.exists(self.detection_file):
            if time.time() - start_time > timeout:
                self.get_logger().error(f"等待YOLO检测文件超时（{timeout}秒）！")
                rclpy.shutdown()
                exit(1)
            time.sleep(2)  # 每2秒检查一次

    def calc_priority(self):
        # 读取检测结果
        detection = {}
        with open(self.detection_file, 'r', encoding='utf-8') as f:
            for line in f.readlines():
                line = line.strip()
                if line:
                    parts = line.split(',')
                    spot_id = int(parts[0])
                    count = int(parts[1])
                    level = parts[2]
                    distance = float(parts[3]) if len(parts)>=4 else 1.0
                    detection[spot_id] = {
                        'count': count,
                        'level': level,
                        'level_score': 3 if level == 'A' else 2 if level == 'B' else 1,
                        'distance': distance
                    }
        
        # 计算优先级（人数占70%，距离占30%）
        priority_dict = {}
        for spot_id, data in detection.items():
            # 人数得分（归一化）
            max_count = max([d['count'] for d in detection.values()]) if detection else 1
            count_score = data['count'] / max_count if max_count > 0 else 0
            
            # 距离得分（归一化，距离越近得分越高）
            max_dist = max([d['distance'] for d in detection.values()]) if detection else 1
            dist_score = 1 - (data['distance'] / max_dist) if max_dist > 0 else 1
            
            # 综合优先级
            priority = (count_score * 0.7) + (dist_score * 0.3)
            priority_dict[spot_id] = priority
        
        # 按优先级排序（降序）
        sorted_spots = sorted(priority_dict.items(), key=lambda x: x[1], reverse=True)
        
        # 动作配置
        actions = {
            1: 'forward_1m',
            2: 'backward_1m',
            3: 'left_1m',
            4: 'right_1m',
            5: 'spin_left_1圈'
        }
        durations = {
            'forward_1m':5.0,
            'backward_1m':5.0,
            'left_1m':5.0,
            'right_1m':5.0,
            'spin_left_1圈':7.0
        }
        
        # 保存优先级结果
        with open(self.priority_file, 'w', encoding='utf-8') as f:
            for spot_id, priority in sorted_spots:
                action = actions.get(spot_id, 'forward_1m')
                duration = durations.get(action, 5.0)
                f.write(f"{spot_id},{priority:.4f},{action},{duration}\n")

def main(args=None):
    rclpy.init(args=args)
    node = PriorityCalcNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()