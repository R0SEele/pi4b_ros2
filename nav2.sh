#!/bin/bash

# car_ws 一键启动导航功能脚本
# 功能：顺序启动机器人模型、雷达驱动、激光匹配、定位和导航模块
# 用法：在 car_ws 目录下执行 ./start_navigation.sh

# 设置 ROS 2 环境变量（根据您的实际 ROS 版本修改，例如 foxy, humble, rolling 等）
# 如果已存在于 .bashrc 中，可注释掉下面两行
source /opt/ros/humble/setup.bash  # 将 humble 替换为您使用的 ROS 2 发行版
source install/local_setup.bash     #  sourced from car_ws

# 定义日志文件（可选）
LOG_DIR="logs"
mkdir -p $LOG_DIR
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$LOG_DIR/navigation_$TIMESTAMP.log"

# 打印带时间戳的消息函数
echo_time() {
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] $1"
}

# 记录启动开始
echo_time "开始一键启动导航功能..." | tee -a $LOG_FILE

# 启动机器人模型展示 (后台运行，并显示输出到终端和日志)
echo_time "启动机器人模型: ros2 launch bot_description display_robot.launch.py" | tee -a $LOG_FILE
ros2 launch bot_description display_robot.launch.py > >(tee -a $LOG_FILE) 2>&1 &
DISPLAY_ROBOT_PID=$!
sleep 3  # 等待启动稳定

# 启动镭神雷达驱动 (LSN10 网络版)
echo_time "启动雷达驱动: ros2 launch lslidar_driver lsn10_net_launch.py" | tee -a $LOG_FILE
ros2 launch lslidar_driver lsn10_net_launch.py > >(tee -a $LOG_FILE) 2>&1 &
LIDAR_DRIVER_PID=$!
sleep 3

# 启动激光扫描匹配器 (laser_scan_matcher)
echo_time "启动激光匹配: ros2 run ros2_laser_scan_matcher laser_scan_matcher" | tee -a $LOG_FILE
ros2 run ros2_laser_scan_matcher laser_scan_matcher > >(tee -a $LOG_FILE) 2>&1 &
LASER_MATCHER_PID=$!
sleep 2

# 启动机器人定位 (第一次)
echo_time "启动定位节点 (第一次): ros2 launch bot_localization localization_launch.py" | tee -a $LOG_FILE
ros2 launch bot_localization localization_launch.py > >(tee -a $LOG_FILE) 2>&1 &
LOCALIZATION_PID_1=$!
sleep 3

# 启动机器人定位 (第二次，根据您的命令重复，可能是冗余，但保留原样)
echo_time "启动定位节点 (第二次): ros2 launch bot_localization localization_launch.py" | tee -a $LOG_FILE
ros2 launch bot_localization localization_launch.py > >(tee -a $LOG_FILE) 2>&1 &
LOCALIZATION_PID_2=$!
sleep 3

# 启动导航2
echo_time "启动导航2: ros2 launch bot_navigation2 navigation2.launch.py" | tee -a $LOG_FILE
ros2 launch bot_navigation2 navigation2.launch.py > >(tee -a $LOG_FILE) 2>&1 &
NAVIGATION_PID=$!

echo_time "所有导航相关节点已启动！" | tee -a $LOG_FILE
echo_time "进程ID: 模型($DISPLAY_ROBOT_PID), 雷达($LIDAR_DRIVER_PID), 匹配($LASER_MATCHER_PID), 定位1($LOCALIZATION_PID_1), 定位2($LOCALIZATION_PID_2), 导航($NAVIGATION_PID)" | tee -a $LOG_FILE
echo_time "日志文件: $LOG_FILE" | tee -a $LOG_FILE
echo_time "提示: 按 Ctrl+C 终止所有节点（需手动清理后台进程）" | tee -a $LOG_FILE

# 等待用户中断，退出时清理所有后台进程
wait $NAVIGATION_PID