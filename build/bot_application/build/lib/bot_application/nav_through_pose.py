from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # 设置初始位姿（修正：orientation 应该在 pose.orientation 中，而不是 pose.position）
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.399933
    initial_pose.pose.position.y = 0.931381
    initial_pose.pose.position.z = 0.0  # 添加 z 坐标，通常为 0
    # 修正：方向信息应该放在 orientation 中
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = -0.120778
    initial_pose.pose.orientation.w = 0.99268  # 这里使用 w，表示朝向（1.0 表示朝前）
    
    navigator.setInitialPose(initial_pose)
    
    navigator.waitUntilNav2Active()
    
    # 创建点集
    goal_poses = []
    
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -0.080
    goal_pose1.pose.position.y = 3.599
    goal_pose1.pose.position.z = 0.0
    goal_pose1.pose.orientation.x = 0.0
    goal_pose1.pose.orientation.y = 0.0
    goal_pose1.pose.orientation.z = 0.0
    goal_pose1.pose.orientation.w = -0.221
    goal_poses.append(goal_pose1)
    
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.533
    goal_pose2.pose.position.y = 3.412
    goal_pose2.pose.position.z = 0.0
    goal_pose2.pose.orientation.x = 0.0
    goal_pose2.pose.orientation.y = 0.0
    goal_pose2.pose.orientation.z = 0.0
    goal_pose2.pose.orientation.w = -1.777
    goal_poses.append(goal_pose2)
    
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 0.966
    goal_pose3.pose.position.y = 0.720
    goal_pose3.pose.position.z = 0.0
    goal_pose3.pose.orientation.x = 0.0
    goal_pose3.pose.orientation.y = 0.0
    goal_pose3.pose.orientation.z = 0.0
    goal_pose3.pose.orientation.w = 2.956
    goal_poses.append(goal_pose3)
    
    # 使用 goThroughPoses 方法
    navigator.goThroughPoses(goal_poses)
    
    # 判断结束及获取反馈
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        
        if feedback:
            try:
                navigator.get_logger().info(
                    f'导航中... 剩余 {feedback.number_of_poses_remaining} 个目标点'
                )
            except AttributeError:
                navigator.get_logger().info('导航进行中...')
    
    # 最终结果判断
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航结果：成功 ✓')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('导航结果：被取消 ⚠')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('导航结果：失败 ✗')
    else:
        navigator.get_logger().error('导航结果：返回状态无效')

if __name__ == '__main__':
    main()