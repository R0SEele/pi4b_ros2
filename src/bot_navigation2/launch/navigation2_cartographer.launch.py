import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 你的 Nav2 参数文件
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('bot_navigation2'),
            'config',
            'nav2_params.yaml'))
 
    # 你的 launch 文件夹
    nav2_launch_file_dir = os.path.join(get_package_share_directory('bot_navigation2'), 'launch')
 
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),
 
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
 
        # 启动：Cartographer纯定位 + Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_file_dir, 'bringup_launch_cartographer.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),
    ])