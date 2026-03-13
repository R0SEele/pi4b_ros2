#!/bin/bash
set -e

# ======================== 配置区（根据你的实际环境修改） ========================
# ROS2工作空间路径（修改为你实际的colcon_ws路径）
ROS2_WS_PATH="/home/rose/pi4b_ros2/car_ws"
# 终端类型（默认gnome-terminal，若用其他终端可修改，如xterm）
TERMINAL="gnome-terminal"
LOG_DIR="$ROS2_WS_PATH/log/launch_robot"
CLEAN_BEFORE_LAUNCH=${CLEAN_BEFORE_LAUNCH:-1}
CTRL_PORT=${CTRL_PORT:-/dev/ttyACM0}
LIDAR_NET_IF=${LIDAR_NET_IF:-eth0}
LIDAR_HOST_IP=${LIDAR_HOST_IP:-192.168.1.10}
LIDAR_DEVICE_IP=${LIDAR_DEVICE_IP:-192.168.1.200}
STRICT_LIDAR_CHECK=${STRICT_LIDAR_CHECK:-1}
# ===============================================================================

# 检查ROS2工作空间路径是否存在
if [ ! -d "$ROS2_WS_PATH/install" ]; then
    echo "错误：ROS2工作空间路径 $ROS2_WS_PATH 不存在，请检查配置！"
    exit 1
fi

mkdir -p "$LOG_DIR"

# 检查是否有图形环境可用
HAS_GUI=0
if [ -n "$DISPLAY" ] || [ -n "$WAYLAND_DISPLAY" ]; then
    HAS_GUI=1
fi

# 图形模式：每个命令在独立终端窗口运行
launch_ros2_command_gui() {
    local cmd=$1
    local title=$2

    if ! command -v "$TERMINAL" >/dev/null 2>&1; then
        echo "错误：未找到 $TERMINAL 终端，请安装或修改脚本中的终端类型！"
        exit 1
    fi

    $TERMINAL --title="$title" -- bash -c "
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
        source /opt/ros/humble/setup.bash
        source $ROS2_WS_PATH/install/setup.bash
        echo '正在启动：$cmd'
        $cmd || { echo '命令执行失败！按任意键退出...'; read -n1; }
        exec bash
    "
}

# 无图形模式：后台启动并输出日志
launch_ros2_command_headless() {
    local cmd=$1
    local title=$2
    local log_file=$3

    nohup bash -lc "
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
        source /opt/ros/humble/setup.bash
        source '$ROS2_WS_PATH/install/setup.bash'
        echo \"[\$(date +%F_%T)] 启动: $cmd\"
        exec $cmd
    " >"$log_file" 2>&1 &

    local pid=$!
    echo "[HEADLESS] $title 已启动, PID=$pid, 日志=$log_file"
}

cleanup_stale_process_and_shm() {
    echo "检测到重复启动风险，先清理旧进程与FastDDS锁..."
    # 先TERM再KILL，避免历史残留进程持续占用DDS资源
    for pattern in \
        "ros2 launch lslidar_driver lsn10_net_launch.py" \
        "ros2 launch bot_description backpack_2d.launch.py" \
        "ros2 run ros2_laser_scan_matcher laser_scan_matcher" \
        "ros2 run test_control_pkg car_controller" \
        "lslidar_driver_node" \
        "laser_scan_matcher" \
        "test_control_pkg/car_controller" \
        "ekf_node" \
        "ekf_filter_node" \
        "cartographer_node" \
        "cartographer_occupancy_grid_node" \
        "robot_state_publisher" \
        "joint_state_publisher" \
        "rviz2"
    do
        pkill -f "$pattern" || true
    done

    sleep 1

    for pattern in \
        "ros2 launch lslidar_driver lsn10_net_launch.py" \
        "ros2 launch bot_description backpack_2d.launch.py" \
        "ros2 run ros2_laser_scan_matcher laser_scan_matcher" \
        "ros2 run test_control_pkg car_controller" \
        "lslidar_driver_node" \
        "laser_scan_matcher" \
        "test_control_pkg/car_controller" \
        "ekf_node" \
        "ekf_filter_node" \
        "cartographer_node" \
        "cartographer_occupancy_grid_node" \
        "robot_state_publisher" \
        "joint_state_publisher" \
        "rviz2"
    do
        pkill -9 -f "$pattern" || true
    done

    ros2 daemon stop || true
    rm -f /dev/shm/fastrtps_* /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_* /dev/shm/sem.fastrtps_port* || true
    ros2 daemon start || true
}

check_lidar_network() {
    echo "检查雷达网口状态..."

    if ! ip link show "$LIDAR_NET_IF" >/dev/null 2>&1; then
        echo "错误：未找到网口 $LIDAR_NET_IF"
        return 1
    fi

    if ! ip link show "$LIDAR_NET_IF" | grep -q "state UP"; then
        echo "错误：网口 $LIDAR_NET_IF 未处于 UP 状态"
        return 1
    fi

    local current_ip
    current_ip=$(ip -4 -o addr show dev "$LIDAR_NET_IF" | awk '{print $4}' | head -n 1)
    if [ -z "$current_ip" ]; then
        echo "错误：网口 $LIDAR_NET_IF 没有 IPv4 地址。"
        echo "建议先配置：$LIDAR_HOST_IP/24"
        return 1
    fi

    if ! echo "$current_ip" | grep -q '^192\.168\.1\.'; then
        echo "错误：网口 $LIDAR_NET_IF 当前地址为 $current_ip，不在雷达 192.168.1.0/24 网段。"
        echo "建议改为：$LIDAR_HOST_IP/24"
        return 1
    fi

    if ! ping -c 1 -W 1 -I "$LIDAR_NET_IF" "$LIDAR_DEVICE_IP" >/dev/null 2>&1; then
        echo "警告：无法通过 $LIDAR_NET_IF ping 通雷达 $LIDAR_DEVICE_IP，后续可能收不到 /scan。"
    else
        echo "雷达网络连通：$LIDAR_NET_IF -> $LIDAR_DEVICE_IP"
    fi
}

echo "开始启动ROS2节点..."
if [ "$CLEAN_BEFORE_LAUNCH" = "1" ]; then
    cleanup_stale_process_and_shm
fi

if ! check_lidar_network; then
    if [ "$STRICT_LIDAR_CHECK" = "1" ]; then
        echo "雷达网口检查失败，停止启动。"
        echo "如需跳过网口检查（仅调试底盘/里程计），可使用：STRICT_LIDAR_CHECK=0 ./launch_robot.sh"
        exit 1
    else
        echo "警告：已跳过雷达网口严格检查（STRICT_LIDAR_CHECK=0）。"
    fi
fi

# if [ "$HAS_GUI" -eq 1 ]; then
#     echo "检测到图形环境，使用 $TERMINAL 分窗口启动。"
#     launch_ros2_command_gui "ros2 launch lslidar_driver lsn10_net_launch.py" "激光雷达驱动"
#     sleep 2
#     launch_ros2_command_gui "ros2 launch bot_description display_robot.launch.py" "机器人模型显示"
#     sleep 1
#     launch_ros2_command_gui "ros2 run ros2_laser_scan_matcher laser_scan_matcher" "激光扫描匹配"
#     sleep 1
#     launch_ros2_command_gui "ros2 run test_control_pkg car_controller --ros-args -p serial_port:=$CTRL_PORT -p enable_encoder_odom:=true -p wait_for_priority_file:=false -p auto_exit_when_done:=false -p send_odom_feedback:=false" "轮式里程计发布"
#     sleep 1
#     launch_ros2_command_gui "ros2 launch bot_description backpack_2d.launch.py" "2D背包导航"
# else
    echo "未检测到图形环境，使用后台模式启动并写入日志。"
    launch_ros2_command_headless "ros2 launch lslidar_driver lsn10_net_launch.py" "激光雷达驱动" "$LOG_DIR/01_lidar.log"
    sleep 2
    echo "HEADLESS模式下跳过RViz显示节点（无DISPLAY会崩溃）。"
    echo "如需可视化，请在图形桌面终端单独运行："
    echo "  ros2 launch bot_description display_robot.launch.py"
    sleep 1
    launch_ros2_command_headless "ros2 run ros2_laser_scan_matcher laser_scan_matcher" "激光扫描匹配" "$LOG_DIR/03_scan_matcher.log"
    sleep 1
    launch_ros2_command_headless "ros2 run test_control_pkg car_controller --ros-args -p serial_port:=$CTRL_PORT -p enable_encoder_odom:=true -p wait_for_priority_file:=false -p auto_exit_when_done:=false -p send_odom_feedback:=false" "轮式里程计发布" "$LOG_DIR/04_car_controller.log"
    sleep 1
    launch_ros2_command_headless "ros2 launch bot_description backpack_2d.launch.py" "2D背包导航" "$LOG_DIR/05_backpack_2d.log"
    echo "可用以下命令查看日志："
    echo "  tail -f $LOG_DIR/01_lidar.log"
    echo "  tail -f $LOG_DIR/04_car_controller.log"
    echo "  tail -f $LOG_DIR/05_backpack_2d.log"
# fi

echo "所有命令已提交启动！"
echo "提示：若终端窗口闪退，可检查命令是否正确、依赖是否安装。"

# 运行示例：./launch_robot.sh