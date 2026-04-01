import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():
    # ===================== 你的包名（不用改） =====================
    pkg_name = 'bot_navigation2'
    pkg_share = get_package_share_directory(pkg_name)

    # ===================== 官方导航包 =====================
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ===================== 配置文件 =====================
    carto_config_dir = os.path.join(pkg_share, 'config')
    carto_lua_file = 'backpack_2d.lua'  # 必须是纯定位lua
    nav2_param_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # ===================== 地图路径（你已经正确填写） =====================
    pbstream_default_path = '/home/rose/pi4b_ros2/car_ws/src/bot_navigation2/maps/map.pbstream'
    map_yaml_path = os.path.join(pkg_share, 'maps', 'map.yaml')  # 新增！导航必须用yaml地图

    # ===================== 启动参数 =====================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    pbstream_path = LaunchConfiguration('load_state_filename', default=pbstream_default_path)
    nav2_param_path = LaunchConfiguration('params_file', default=nav2_param_file)

    # ===================== 声明参数 =====================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )
    declare_pbstream = DeclareLaunchArgument(
        'load_state_filename', default_value=pbstream_default_path
    )
    declare_nav2_params = DeclareLaunchArgument(
        'params_file', default_value=nav2_param_file
    )

    # ===================== 启动 Nav2（正确传入地图） =====================
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            "map": map_yaml_path,          # 必须给地图！不能是null
            "use_sim_time": use_sim_time,
            "params_file": nav2_param_path,
            "autostart": "true"
        }.items(),
    )

    # ===================== ✅ Cartographer 纯定位模式（关键修正） =====================
    cartographer_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", carto_config_dir,
            "-configuration_basename", carto_lua_file,
            "-load_state_filename", pbstream_path,
            "-load_frozen_state", "true"   # ✅ 必须加！纯定位模式
        ],
    )

    # ===================== 地图发布 =====================
    occupancy_grid_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution", resolution,
            "-publish_period_sec", publish_period_sec
        ],
    )

    # ===================== 启动 =====================
    return LaunchDescription([
        declare_use_sim_time,
        declare_pbstream,
        declare_nav2_params,
        
        cartographer_node,
        occupancy_grid_node,
        nav2_bringup_launch,
    ])