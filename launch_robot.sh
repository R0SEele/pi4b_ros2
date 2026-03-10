#!/bin/bash

# ======================== 配置区（根据你的实际环境修改） ========================
# ROS2工作空间路径（修改为你实际的colcon_ws路径）
ROS2_WS_PATH="$HOME/mybot"
# 终端类型（默认gnome-terminal，若用其他终端可修改，如xterm）
TERMINAL="gnome-terminal"
# ===============================================================================

# 检查终端是否可用
if ! command -v $TERMINAL &> /dev/null; then
    echo "错误：未找到 $TERMINAL 终端，请安装或修改脚本中的终端类型！"
    exit 1
fi

# 检查ROS2工作空间路径是否存在
if [ ! -d "$ROS2_WS_PATH/install" ]; then
    echo "错误：ROS2工作空间路径 $ROS2_WS_PATH 不存在，请检查配置！"
    exit 1
fi

# 定义启动函数：每个命令在独立终端窗口运行，自动source环境
launch_ros2_command() {
    local cmd=$1
    local title=$2
    $TERMINAL --title="$title" -- bash -c "
        # 激活ROS2环境和工作空间
        source /opt/ros/humble/setup.bash
        source $ROS2_WS_PATH/install/setup.bash
        # 执行命令（出错时暂停，方便排查）
        echo '正在启动：$cmd'
        $cmd || { echo '命令执行失败！按任意键退出...'; read -n1; }
        # 执行完成后不关闭终端（方便查看日志）
        exec bash
    "
}

# 逐行启动命令（每个命令一个独立终端窗口）
echo "开始启动ROS2节点..."
launch_ros2_command "ros2 launch lslidar_driver lsn10_net_launch.py" "激光雷达驱动"
sleep 2  # 延迟2秒，避免启动过快导致依赖问题
launch_ros2_command "ros2 launch bot_description display_robot.launch.py" "机器人模型显示"
sleep 1
# launch_ros2_command "ros2 run ros2_laser_scan_matcher laser_scan_matcher" "激光扫描匹配"
sleep 1
launch_ros2_command "ros2 launch bot_description backpack_2d.launch.py" "2D背包导航"

echo "所有命令已提交启动！"
echo "提示：若终端窗口闪退，可检查命令是否正确、依赖是否安装。"

# 运行示例：./launch_robot.sh