from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import math
import asyncio

async def main_async():
    rclpy.init()
    navigator = BasicNavigator()
    
    # 创建初始位姿
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # 西景点坐标和朝向
    initial_pose.pose.position.x = 0.993753
    initial_pose.pose.position.y = 0.62935
    initial_pose.pose.position.z = 0.0

    
    # Orientation(0, 0, -0.780975, 0.624563) = Angle: -1.79245
    yaw = -1.81449  # Angle: -1.73118
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = math.sin(yaw / 2.0)
    initial_pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    print(f"Setting initial pose at ({initial_pose.pose.position.x}, {initial_pose.pose.position.y})")
    
    # 设置初始位姿
    navigator.setInitialPose(initial_pose)
    
    # 等待导航系统激活
    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    
    # 异步等待
    await asyncio.sleep(2)
    
    print("Initial pose set successfully!")
    
    # 清理资源
    navigator.destroy_node()
    rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()