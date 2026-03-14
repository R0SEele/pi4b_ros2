import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import launch

def generate_launch_description():
    pkg_share = FindPackageShare(package='bot_description').find('bot_description')

    # 机器人模型与TF发布（确保tf_static和robot_description始终可用）
    default_model_path = os.path.join(pkg_share, 'urdf', '4wd.urdf.xacro')
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', default_model_path]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    # # robot_localization 节点
    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': False}],
    #     remappings=[
    #         ('/odometry/filtered', '/odom'),
    #     ]
    # )

    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # resolution = LaunchConfiguration('resolution', default='0.05')
    # publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # # 配置文件路径
    # configuration_directory = LaunchConfiguration(
    #     'configuration_directory',
    #     default=os.path.join(pkg_share, 'config')
    # )
    # configuration_basename = LaunchConfiguration(
    #     'configuration_basename',
    #     default='my_robot.lua'
    # )

    # cartographer 节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/scan', '/scan'),                    # 雷达话题重映射
            ('/odom', '/odom')                     # 使用EKF融合后的标准/odom
        ]
    )

    # 占用栅格地图节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,
        cartographer_node,
        occupancy_grid_node,
    ])