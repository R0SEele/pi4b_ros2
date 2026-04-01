#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.duration import Duration
from nav2_msgs.srv import GetCostmap
from nav_msgs.srv import GetPlan  # 使用 nav_msgs 的 GetPlan 服务
from nav_msgs.msg import Path
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
import math
import time

# ====================== 纯Python欧拉角转四元数 ======================
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

        # 创建路径规划客户端（使用 nav_msgs/GetPlan）
        self.plan_client = self.node.create_client(
            GetPlan, '/plan'
        )
        
        # 等待服务可用
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('等待路径规划服务 /plan...')
        
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

    def get_robot_pose(self, max_attempts=50):
        """获取机器人位姿，持续尝试直到成功"""
        self.node.get_logger().info('尝试获取机器人位姿...')
        
        attempt = 0
        target_frames = ['base_link', 'base_footprint']
        
        while attempt < max_attempts:
            if attempt % 10 == 0:
                try:
                    frames = self.tf_buffer.all_frames_as_string()
                    if frames:
                        self.node.get_logger().info(f"当前已知坐标系:\n{frames}")
                except:
                    pass
            
            for frame in target_frames:
                try:
                    if self.tf_buffer.can_transform('map', frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)):
                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            frame,
                            rclpy.time.Time(seconds=0),
                            timeout=Duration(seconds=0.1)
                        )

                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.header.stamp = self.node.get_clock().now().to_msg()
                        
                        pose.pose.position.x = transform.transform.translation.x
                        pose.pose.position.y = transform.transform.translation.y
                        pose.pose.position.z = transform.transform.translation.z
                        pose.pose.orientation = transform.transform.rotation

                        self.node.get_logger().info(
                            f"\n🚀 成功获取到 '{frame}' 的位姿:\n"
                            f"  平移: x={pose.pose.position.x:.3f}, "
                            f"y={pose.pose.position.y:.3f}\n"
                        )
                        return pose

                except (LookupException, ConnectivityException, ExtrapolationException):
                    continue
                except Exception as e:
                    self.node.get_logger().debug(f"'{frame}' 尝试失败: {e}")
                    continue
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
            attempt += 1
        
        self.node.get_logger().error("❌ 无法获取机器人位姿")
        return None

    def compute_path_between_poses(self, start_pose, goal_pose):
        """独立计算两点之间的路径（使用 GetPlan 服务）"""
        try:
            # 创建规划请求
            request = GetPlan.Request()
            request.start = start_pose
            request.goal = goal_pose
            
            # 发送请求，设置超时
            future = self.plan_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.plan and len(response.plan.poses) > 0:
                    self.node.get_logger().info(f"规划成功，路径点数: {len(response.plan.poses)}")
                    return response.plan
                else:
                    self.node.get_logger().warn("规划成功但路径为空")
                    return None
            else:
                self.node.get_logger().error("规划超时或失败")
                return None
                
        except Exception as e:
            self.node.get_logger().error(f"路径规划异常: {e}")
            return None

    def is_pose_valid(self, pose):
        """检查位姿是否在地图内且可达"""
        try:
            if not self.fetch_costmap():
                return False
            
            mx, my = self.world_to_map(pose.pose.position.x, pose.pose.position.y)
            if mx is None or my is None:
                self.node.get_logger().warn(f"位姿 ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) 在地图外")
                return False
            
            cost = self.get_cost_at_point(mx, my)
            if cost is None:
                return False
            
            if cost >= 253:
                self.node.get_logger().warn(f"位姿在障碍物上 (代价: {cost})")
                return False
            
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"检查位姿有效性失败: {e}")
            return False

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
        navigator.waitUntilNav2Active(localizer='amcl')
        print("导航已激活")
    except Exception as e:
        print(f"导航启动失败: {e}")
        try:
            navigator.waitUntilNav2Active()
            print("导航已激活（默认模式）")
        except Exception as e2:
            print(f"导航启动失败: {e2}")
            return

    cost_calc = WaypointCostCalculator(navigator)
    
    print("\n等待TF树建立...")
    time.sleep(3.0)
    
    print("\n获取机器人当前位置...")
    current_pose = cost_calc.get_robot_pose(max_attempts=60)

    if current_pose is None:
        print("⚠️ 无法获取机器人位姿，使用默认起点(0,0)")
        current_pose = create_goal_pose(navigator, 0.0, 0.0)
    else:
        print(f"✅ 成功获取机器人位姿: x={current_pose.pose.position.x:.2f}, y={current_pose.pose.position.y:.2f}")
    
    # 定义路点
    waypoints = [
        create_goal_pose(navigator, -0.08, 3.60, math.pi/2),
        create_goal_pose(navigator, 0.97, 0.72, 0.0),
        create_goal_pose(navigator, 1.53, 3.41, -math.pi),
    ]
    
    # 检查路点有效性
    print("\n检查路点有效性:")
    for i, wp in enumerate(waypoints):
        valid = cost_calc.is_pose_valid(wp)
        print(f"  路点{i+1} ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}): {'✅ 有效' if valid else '❌ 无效'}")

    print("\n开始独立路径规划（真正分段）...")
    
    # 真正独立的分段规划
    segments = []
    segment_names = []
    
    # 第一段：起点到路点1
    print("\n规划 起点 → 路点1")
    path1 = cost_calc.compute_path_between_poses(current_pose, waypoints[0])
    if path1:
        print(f"✓ 规划成功：{len(path1.poses)} 个点")
        segments.append(path1)
        segment_names.append("起点→路点1")
    else:
        print("✗ 规划失败")
        segments.append(None)
        segment_names.append(None)
    
    # 第二段：路点1到路点2
    print("\n规划 路点1 → 路点2")
    path2 = cost_calc.compute_path_between_poses(waypoints[0], waypoints[1])
    if path2:
        print(f"✓ 规划成功：{len(path2.poses)} 个点")
        segments.append(path2)
        segment_names.append("路点1→路点2")
    else:
        print("✗ 规划失败")
        segments.append(None)
        segment_names.append(None)
    
    # 第三段：路点2到路点3
    print("\n规划 路点2 → 路点3")
    path3 = cost_calc.compute_path_between_poses(waypoints[1], waypoints[2])
    if path3:
        print(f"✓ 规划成功：{len(path3.poses)} 个点")
        segments.append(path3)
        segment_names.append("路点2→路点3")
    else:
        print("✗ 规划失败")
        segments.append(None)
        segment_names.append(None)
    
    # 合并有效路径
    combined = Path()
    valid_count = 0
    
    for i, (seg, name) in enumerate(zip(segments, segment_names)):
        if seg is not None and len(seg.poses) > 0:
            if valid_count == 0:
                # 第一段，直接添加所有点
                combined.poses.extend(seg.poses)
            else:
                # 后续段，跳过第一个点（避免重复）
                combined.poses.extend(seg.poses[1:])
            valid_count += 1
            print(f"✅ 合并 {name}: {len(seg.poses)} 个点")
        else:
            if name:
                print(f"❌ 跳过无效段: {name}")

    if len(combined.poses) == 0:
        print("\n❌ 全部路径规划失败")
        return

    print(f"\n合并后总路径: {len(combined.poses)} 个点 ({valid_count}/3 段成功)")

    # 计算代价
    total_cost = cost_calc.calculate_path_cost(combined)

    print(f"\n最终结果：")
    print(f"  总代价：{total_cost:.1f}")
    print(f"  总点数：{len(combined.poses)}")
    print(f"  成功路段：{valid_count}/3")

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()