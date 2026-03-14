#!/usr/bin/env python3
# bot_localization/launch/localization_launch.py
# 定位系统启动文件（EKF + 传感器融合）

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ==================== 获取包路径 ====================
    pkg_bot_localization = get_package_share_directory('bot_localization')
    
    # ==================== 配置文件路径 ====================
    # EKF配置文件（你提供的路径）
    ekf_config_file = PathJoinSubstitution([
        pkg_bot_localization,
        'config',
        'ekf.yaml'
    ])
    
    # ==================== 声明启动参数 ====================
    # 是否使用仿真时间
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间（如果是回放bag文件设为true）'
    )
    
    # EKF配置文件路径（允许命令行覆盖）
    declare_ekf_config = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=ekf_config_file,
        description='EKF配置文件路径（YAML格式）'
    )
    
    # 是否发布TF
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='是否发布odom到base_link的TF变换'
    )
    
    # 是否打印诊断信息
    declare_print_diagnostics = DeclareLaunchArgument(
        'print_diagnostics',
        default_value='true',
        description='在控制台打印诊断信息'
    )
    
    # 是否使用2D模式
    declare_two_d_mode = DeclareLaunchArgument(
        'two_d_mode',
        default_value='true',
        description='使用2D模式（只融合x/y/yaw）'
    )
    
    # ==================== EKF节点配置 ====================
    # 主EKF节点（用于odom坐标系定位）
    ekf_filter_node_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config_file')],
        remappings=[
            # 如果有必要，可以重映射话题名称
            ('odometry/filtered', 'odom'),
        ],
        arguments=[
            '--ros-args',
            '--log-level', 'info'
        ]
    )
    
    # 第二个EKF节点（如果需要map坐标系定位，用于导航）
    # 注意：如果你只有一个ekf节点，可以注释掉这部分
    # ekf_filter_node_map = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node_map',
    #     output='screen',
    #     parameters=[LaunchConfiguration('ekf_config_file')],
    #     remappings=[
    #         ('odometry/filtered', 'odometry/global_filtered'),
    #     ],
    #     arguments=[
    #         '--ros-args',
    #         '--log-level', 'info'
    #     ]
    # )
    
    # ==================== Navsat转换节点（如果需要GPS融合） ====================
    # 如果你需要GPS融合，取消下面的注释
    # navsat_transform_node = Node(
    #     package='robot_localization',
    #     executable='navsat_transform_node',
    #     name='navsat_transform',
    #     output='screen',
    #     parameters=[LaunchConfiguration('ekf_config_file')],
    #     remappings=[
    #         ('imu/data', 'imu/data'),
    #         ('gps/fix', 'gps/fix'),
    #         ('odometry/gps', 'odometry/gps_filtered')
    #     ]
    # )
    
    # ==================== 状态发布器（用于查看EKF状态） ====================
    # 可选：发布诊断信息
    diagnostic_node = Node(
        condition=IfCondition(LaunchConfiguration('print_diagnostics')),
        package='diagnostic_updater',
        executable='diagnostic_aggregator',
        name='diagnostic_aggregator',
        output='screen'
    )
    
    # ==================== 静态TF（如果需要） ====================
    # 如果你的base_link到传感器有固定变换，可以在这里发布
    # 例如：base_link到laser_frame的变换
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # ==================== 启动提示信息 ====================
    log_config_path = LogInfo(
        msg=['使用EKF配置文件: ', LaunchConfiguration('ekf_config_file')]
    )
    
    # ==================== 创建LaunchDescription ====================
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_ekf_config)
    ld.add_action(declare_publish_tf)
    ld.add_action(declare_print_diagnostics)
    ld.add_action(declare_two_d_mode)
    
    # 添加日志信息
    ld.add_action(log_config_path)
    
    # 添加节点
    ld.add_action(ekf_filter_node_odom)
    # ld.add_action(ekf_filter_node_map)  # 如果不需要第二个EKF，注释掉这行
    # ld.add_action(navsat_transform_node)  # 如果不需要GPS，注释掉这行
    # ld.add_action(diagnostic_node)  # 如果需要诊断信息，取消注释
    
    # 静态TF（根据需要启用）
    # ld.add_action(static_tf_laser)
    
    return ld