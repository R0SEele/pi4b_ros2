#!/bin/bash
set -e

cd ~/pi4b_ros2/car_ws
source install/setup.bash

# 串口参数（可在执行前通过环境变量覆盖）
CTRL_PORT=${CTRL_PORT:-/dev/ttyACM0}

echo "====================================="
echo "  终极版：自动等待+严格串行执行"
echo "====================================="

# 1. 启动YOLO检测（自动等完成）
echo -e "\n[1/3] 启动YOLO检测..."
ros2 run yolo_counting_pkg yolo_detector --ros-args \
	-p config_path:=/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/config/params.yaml \
	-p video_dir:=/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/data/vedio \
	-p results_dir:=/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results \
	-p detection_file:=/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_detections.txt \
	-p frame_interval:=50 \
	-p save_only_with_detections:=true \
	-p jpeg_quality:=90

# 2. 启动优先级计算（自动等YOLO文件）
echo -e "\n[2/3] 启动优先级计算..."
ros2 run yolo_counting_pkg priority_calc --ros-args \
-p config_path:=/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/config/params.yaml \
-p detection_file:=/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_detections.txt \
-p priority_file:=/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/scenic_spot_priority.txt

# 3. 启动小车控制（自动等优先级文件）
echo -e "\n[3/3] 启动小车执行动作..."
ros2 run test_control_pkg car_controller --ros-args \
	-p serial_port:=$CTRL_PORT \
	-p enable_encoder_odom:=true \
	-p send_odom_feedback:=false

echo -e "\n✅ 所有动作执行完成！"