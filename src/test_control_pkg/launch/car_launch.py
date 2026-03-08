#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
import os

def generate_launch_description():
    # 1. 定义所有路径（修改为你的实际路径）
    config_path = "/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/config/params.yaml"
    detection_file = "/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_detections.txt"
    priority_file = "/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_priority.txt"
    serial_port = "/dev/ttyACM0"

    # 2. 第一步：执行YOLO检测（跑完退出）
    run_yolo = ExecuteProcess(
        name="run_yolo",
        cmd=[
            'ros2', 'run', 'yolo_counting_pkg', 'yolo_detector',
            '--ros-args', '-p', f'config_path:={config_path}'
        ],
        output="screen",
        # 关键：必须等YOLO退出，才执行下一步
        shell=False
    )

    # 3. 第二步：执行优先级计算（跑完退出）
    run_priority = ExecuteProcess(
        name="run_priority",
        cmd=[
            'ros2', 'run', 'yolo_counting_pkg', 'priority_calc',
            '--ros-args',
            '-p', f'config_path:={config_path}',
            '-p', f'detection_file:={detection_file}',
            '-p', f'priority_file:={priority_file}'
        ],
        output="screen",
        shell=False
    )

    # 4. 第三步：启动小车控制（持续运行）
    run_car = ExecuteProcess(
        name="run_car",
        cmd=[
            'ros2', 'run', 'test_control_pkg', 'car_controller',
            '--ros-args',
            '-p', f'serial_port:={serial_port}',
            '-p', f'priority_file:={priority_file}'
        ],
        output="screen",
        shell=False
    )

    # 组装：严格串行执行
    return LaunchDescription([
        LogInfo(msg="🚀 开始执行：YOLO检测 → 优先级计算 → 小车启动"),
        run_yolo,    # 第一步：YOLO（跑完才到下一步）
        run_priority,# 第二步：优先级（跑完才到下一步）
        run_car      # 第三步：小车
    ])