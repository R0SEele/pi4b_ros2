import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('bot_navigation2'), 'config', 'nav2_params.yaml'),
    )

    # 静态地图
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': os.path.join(get_package_share_directory('bot_navigation2'), 'maps', 'map.yaml')}
        ],
    )

    # Nav2 核心节点（直接启动，不用容器）
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    recoveries_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 生命周期管理
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'map_server',
                # 'amcl'
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator'
            ]},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        
        map_server_node,
        planner_server_node,
        controller_server_node,
        recoveries_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
    ])