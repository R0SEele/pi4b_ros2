#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.duration import Duration
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
import math

# ====================== 纯Python欧拉角转四元数（无任何依赖）======================
def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [qx, qy, qz, qw]

class WaypointCostCalculator:
    def __init__(self, navigator):
        self.navigator = navigator
        self.node = navigator

        self.costmap_client = self.node.create_client(
            GetCostmap, '/global_costmap/get_costmap'
        )

        while not self.costmap_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('等待代价地图服务...')

        # 长期有效的 TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.costmap_data = None
        self.costmap_metadata = None

    def get_robot_pose(self):
        """使用内部长期有效的TF监听器获取位姿（修复版）"""
        rclpy.spin_once(self.node, timeout_sec=0.5)
        target_frames = ['base_link', 'base_footprint']

        for frame in target_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    frame,
                    rclpy.time.Time(seconds=0),
                    timeout=Duration(seconds=1.0)
                )

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.node.get_clock().now().to_msg()

                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation.x = transform.transform.rotation.x
                pose.pose.orientation.y = transform.transform.rotation.y
                pose.pose.orientation.z = transform.transform.rotation.z
                pose.pose.orientation.w = transform.transform.rotation.w

                self.node.get_logger().info(f"✅ 获取机器人位姿成功: map → {frame}")
                return pose

            except Exception:
                continue

        self.node.get_logger().error("❌ 无法获取位姿，使用默认起点 (0,0)")
        return None

    def fetch_costmap(self):
        request = GetCostmap.Request()
        future = self.costmap_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.costmap_data = future.result().map
            self.costmap_metadata = self.costmap_data.metadata
            return True
        return False

    def world_to_map(self, wx, wy):
        if not self.costmap_metadata:
            return None, None

        origin_x = self.costmap_metadata.origin.position.x
        origin_y = self.costmap_metadata.origin.position.y
        resolution = self.costmap_metadata.resolution

        mx = int((wx - origin_x) / resolution)
        my = int((wy - origin_y) / resolution)

        if 0 <= mx < self.costmap_metadata.size_x and 0 <= my < self.costmap_metadata.size_y:
            return mx, my
        return None, None

    def get_cost_at_point(self, mx, my):
        if not self.costmap_data:
            return None
        index = my * self.costmap_metadata.size_x + mx
        if 0 <= index < len(self.costmap_data.data):
            return self.costmap_data.data[index]
        return None

    def get_pose_cost(self, pose):
        try:
            if pose.header.frame_id != 'map':
                transform = self.tf_buffer.lookup_transform(
                    'map', pose.header.frame_id,
                    pose.header.stamp, timeout=Duration(seconds=0.5)
                )
                tp = do_transform_pose(pose, transform)
            else:
                tp = pose

            mx, my = self.world_to_map(tp.pose.position.x, tp.pose.position.y)
            if mx is None:
                return 255

            cost = self.get_cost_at_point(mx, my)
            return cost if cost is not None else 255

        except Exception:
            return 255

    def calculate_path_cost(self, path, sample_step=5):
        if not path or len(path.poses) == 0:
            self.node.get_logger().error("路径为空")
            return None

        if not self.fetch_costmap():
            self.node.get_logger().error("无法获取代价地图")
            return None

        total_cost = 0.0
        count = 0
        sampled_poses = path.poses[::sample_step]

        for p in sampled_poses:
            cost = self.get_pose_cost(p)
            total_cost += cost
            count += 1

        print("\n" + "="*50)
        print("           路径代价计算完成")
        print("="*50)
        print(f"总路径点数: {len(path.poses)}")
        print(f"计算点数:   {count}")
        print(f"总代价:     {total_cost:.1f}")
        print(f"平均代价:   {total_cost/count:.1f}")
        print("="*50)

        return total_cost

def create_goal_pose(navigator, x, y, yaw=0.0):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]
    return goal

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("等待导航系统激活...")
    try:
        navigator.waitUntilNav2Active()
        print("导航已激活")
    except:
        print("导航启动失败")
        return

    cost_calc = WaypointCostCalculator(navigator)
    
    # 使用修复后的位姿获取方法
    current_pose = cost_calc.get_robot_pose()

    if not current_pose:
        current_pose = create_goal_pose(navigator, 0.0, 0.0)

    print(f"起点: x={current_pose.pose.position.x:.2f}, y={current_pose.pose.position.y:.2f}")

    waypoints = [
        create_goal_pose(navigator, 0.97, 0.72, 0.0)
        create_goal_pose(navigator, -0.08, 3.60, math.pi/2),
        create_goal_pose(navigator, 1.53, 3.41, -math.pi),
    ]

    def safe_get_path(start, goal):
        try:
            path = navigator.getPath(start, goal)
            if path and len(path.poses) > 0:
                print(f"✓ 规划成功：{len(path.poses)} 个点")
                return path
            else:
                print("✗ 规划失败")
                return None
        except Exception as e:
            print(f"✗ 规划异常：{e}")
            return None

    p1 = safe_get_path(current_pose, waypoints[0])
    p2 = safe_get_path(waypoints[0], waypoints[1])
    p3 = safe_get_path(waypoints[1], waypoints[2])

    combined = Path()
    for p in [p1, p2, p3]:
        if p:
            combined.poses.extend(p.poses)

    if len(combined.poses) == 0:
        print("全部路径规划失败")
        return

    total_cost = cost_calc.calculate_path_cost(combined)

    print(f"\n最终结果：")
    print(f"  总代价：{total_cost:.1f}")
    print(f"  总点数：{len(combined.poses)}")

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()