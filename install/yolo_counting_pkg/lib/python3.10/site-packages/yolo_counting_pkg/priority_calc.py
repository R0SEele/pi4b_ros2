#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import os
import time

try:
    import yaml
except ImportError:
    yaml = None

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

        self.config = self.load_config()
        self.weights = self.config.get('weights', {'person_level': 0.6, 'distance': 0.4})
        self.person_levels = self.config.get('person_levels', {})
        self.scenic_spots = self.config.get('scenic_spots', {})
        
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

    def load_config(self):
        if yaml is None:
            self.get_logger().warn('未安装PyYAML，使用默认权重与动作配置。')
            return {}
        if not os.path.exists(self.config_path):
            self.get_logger().warn(f'配置文件不存在: {self.config_path}，使用默认配置。')
            return {}

        with open(self.config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    def get_level_score(self, level):
        level_info = self.person_levels.get(level, {})
        if isinstance(level_info, dict):
            return float(level_info.get('score', 1))
        return 1.0

    def calc_priority(self):
        detection = {}
        with open(self.detection_file, 'r', encoding='utf-8') as f:
            for line in f.readlines():
                line = line.strip()
                if line:
                    parts = line.split(',')
                    if len(parts) < 4:
                        continue
                    spot_id = int(parts[0])
                    count = int(parts[1])
                    level = parts[2]
                    distance = float(parts[3])
                    detection[spot_id] = {
                        'count': count,
                        'level': level,
                        'level_score': self.get_level_score(level),
                        'distance': distance
                    }

        if not detection:
            self.get_logger().error(f'检测文件为空或格式错误: {self.detection_file}')
            return
        
        person_w = float(self.weights.get('person_level', 0.6))
        dist_w = float(self.weights.get('distance', 0.4))

        priority_dict = {}
        max_level_score = max([d['level_score'] for d in detection.values()]) if detection else 1
        max_dist = max([d['distance'] for d in detection.values()]) if detection else 1

        for spot_id, data in detection.items():
            level_score = data['level_score'] / max_level_score if max_level_score > 0 else 0
            dist_score = 1 - (data['distance'] / max_dist) if max_dist > 0 else 1
            priority = (level_score * person_w) + (dist_score * dist_w)
            priority_dict[spot_id] = priority

        sorted_spots = sorted(priority_dict.items(), key=lambda x: x[1], reverse=True)

        with open(self.priority_file, 'w', encoding='utf-8') as f:
            for spot_id, priority in sorted_spots:
                spot_cfg = self.scenic_spots.get(spot_id, self.scenic_spots.get(str(spot_id), {}))
                action = str(spot_cfg.get('action', 'forward_1m'))
                duration = float(spot_cfg.get('action_duration', 5.0))
                f.write(f"{spot_id},{priority:.4f},{action},{duration}\n")

def main(args=None):
    rclpy.init(args=args)
    node = PriorityCalcNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()