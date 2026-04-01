import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_robot_cartographer = get_package_share_directory('bot_navigation2')
    pkg_robot_navigation = get_package_share_directory('bot_navigation2')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_cartographer, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'load_state_filename': os.path.join(pkg_robot_cartographer, 'maps', 'map.pbstream')
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_navigation, 'launch', 'navigation.launch.py')
        ),
    )

    return LaunchDescription([
        localization_launch,
        nav2_launch,
    ])