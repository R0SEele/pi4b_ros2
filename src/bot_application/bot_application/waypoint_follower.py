#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # 原始3个路点（西、中、东）
        self.waypoint_map = {
            1: self.create_pose(
                0.993753, 0.62935, 0.0, -0.787809, 0.615919
            ),
            2: self.create_pose(
                -1.26875, 1.02813, 0.0, -0.802294, 0.596929
            ),
            3: self.create_pose(
                -1.84094, 3.89227, 0.0, -0.779955, 0.625835
            )
        }

        # 订阅最优路点顺序
        self.best_order = None
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/best_waypoint_order',
            self.order_callback,
            10
        )
        self.get_logger().info('✅ 等待最优路点顺序 /best_waypoint_order...')

    def create_pose(self, x, y, z, oz, ow):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        return pose

    def order_callback(self, msg):
        # 收到最优顺序，例如 [2,1,3]
        self.best_order = msg.data
        self.get_logger().info(f'\n📥 收到最优导航顺序：{self.best_order}')
        
        # 按最优顺序重新排列路点
        goal_poses = []
        for idx in self.best_order:
            goal_poses.append(self.waypoint_map[idx])

        self.get_logger().info(f'✅ 已重新排列路点，开始导航！')
        
        # 执行路点追踪
        self.navigator.followWaypoints(goal_poses)

        # 等待导航完成
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'🚩 当前目标编号：{feedback.current_waypoint}')
            time.sleep(0.2)

        # 结果
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('🎉 导航成功！')
        elif result == TaskResult.FAILED:
            self.get_logger().error('❌ 导航失败！')

        # 退出
        self.navigator.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    follower = WaypointFollower()
    
    # 保持节点运行，直到收到最优顺序
    while rclpy.ok() and follower.best_order is None:
        rclpy.spin_once(follower, timeout_sec=0.5)
    
    follower.destroy_node()

if __name__ == '__main__':
    main()