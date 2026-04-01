import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    bot_nav_prefix = get_package_share_directory('bot_navigation2')
    
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
        default=os.path.join(bot_nav_prefix, 'config'))
    
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='backpack_2d_localization.lua')

    # 你的绝对地图路径
    pbstream_path = "/home/rose/pi4b_ros2/car_ws/src/bot_navigation2/maps/map.pbstream"

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Config directory'),
        
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Lua config file'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time'),

        # Cartographer 纯定位节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                '-pure_localization', 'true',
                '-load_frozen_state', 'true',
                '-load_state_filename', pbstream_path
            ],
            remappings=[
                ('scan', '/scan'),      # 激光雷达话题
                ('odom', '/odom'),      # 里程计话题
                # ('imu', '/imu'),      # 如果有 IMU，取消注释
            ]
        ),  # ← 注意这里添加了逗号和闭合括号

        # 直接启动 occupancy grid 节点，不依赖外部launch
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])