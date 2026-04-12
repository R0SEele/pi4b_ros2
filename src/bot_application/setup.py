from setuptools import find_packages, setup

package_name = 'bot_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='87068644+fishros@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_robot_pose=bot_application.init_robot_pose:main',
            'init_robot_pose_door=bot_application.init_robot_pose_door:main',
            'init_robot_pose_point1=bot_application.init_robot_pose_point1:main',
            'init_robot_pose_point2=bot_application.init_robot_pose_point2:main',
            'init_robot_pose_point3=bot_application.init_robot_pose_point3:main',
            'nav_through_pose=bot_application.nav_through_pose:main',
            'waypoint_follower=bot_application.waypoint_follower:main',
            'waypoint_cost_calculator=bot_application.waypoint_cost_calculator:main',
            'get_robot_pose=bot_application.get_robot_pose:main',
            'waypoint_cost_calculator_copy=bot_application.waypoint_cost_calculator_copy:main',
            'way_to_go=bot_application.way_to_go:main',
            'auto_optimized_navigator=bot_application.auto_optimized_navigator:main',
            'optimized_navigator=bot_application.optimized_navigator:main',
            'optimized_navigator_dp=bot_application.optimized_navigator_dp:main',
            'navigator_full_dp=bot_application.navigator_full_dp:main',
            'waypoint_selector=bot_application.waypoint_selector:main',
            'navigator_full_ga=bot_application.navigator_full_ga:main',
            'voice_publisher=bot_application.voice_publisher:main',
            'navigator_unified=bot_application.navigator_unified:main',
            'voice_responder_example=bot_application.voice_responder_example:main',
            'keyboard_controller=bot_application.keyboard_controller:main'
            
        ],
    },
)
 