#!/bin/bash

# car_ws 一键启动导航功能脚本（简化版）
# 用法：在 car_ws 目录下执行 ./start_navigation_simple.sh

source /opt/ros/humble/setup.bash
source install/local_setup.bash

echo "正在启动所有导航相关节点..."

# 启动所有节点（后台运行）
ros2 launch bot_description display_robot.launch.py &
PID1=$!
sleep 2

ros2 launch lslidar_driver lsn10_net_launch.py &
PID2=$!
sleep 3

ros2 run ros2_laser_scan_matcher laser_scan_matcher &
PID3=$!
sleep 2

ros2 launch bot_localization localization_launch.py &
PID4=$!
sleep 3

ros2 run test_control_pkg car_controller &
PID5=$!
sleep 2

ros2 launch bot_navigation2 navigation2.launch.py &
PID6=$!

echo "所有节点已启动！"
echo "进程ID: $PID1 $PID2 $PID3 $PID4 $PID5 $PID6"
echo "按 Ctrl+C 终止所有节点"

# 等待用户中断
wait $PID1 $PID2 $PID3 $PID4 $PID5 $PID6