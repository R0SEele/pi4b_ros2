#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('bot_localization')  # 替换为您的包名
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ekf_config_file = LaunchConfiguration('ekf_config_file', 
        default=os.path.join(pkg_share, 'config', 'ekf.yaml'))
    
    # 声明参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')
    
    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=os.path.join(pkg_share, 'config', 'ekf.yaml'),
        description='Full path to the EKF config file')
    
    # robot_localization EKF节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odom'),  # 将滤波后的里程计重映射到/odom
        ],
        arguments=['--ros-args', '--log-level', 'info']  # 设置日志级别
    )
    
    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ekf_config_file_cmd)
    
    # 添加节点
    ld.add_action(ekf_node)
    
    return ld