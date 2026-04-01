#!/usr/bin/env python3
"""
优化版导航器 - 使用动态规划TSP替代穷举法
可扩展到最多12-15个路点
"""
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from std_msgs.msg import Int32MultiArray, String
import rclpy
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
import time
import threading
import re


# ====================== 动态规划 TSP 求解器 ======================
class TSPDynamicProgramming:
    """
    支持带起始点的TSP求解（不要求返回起点）
    """
    def __init__(self, cost_matrix):
        """
        Args:
            cost_matrix: 代价矩阵 [n x n]
        """
        self.cost = cost_matrix
        self.n = len(cost_matrix)

    def solve_path_from_start(self, start_idx):
        """
        求解从起点出发，访问所有其他点的最优路径（不返回起点）

        Args:
            start_idx: 起点索引

        Returns:
            (min_cost, best_path): 最小代价和最优路径（索引列表）
        """
        n = self.n

        # dp[mask][u] = 访问mask中的点，最后到达u的最小代价
        dp = [[float('inf')] * n for _ in range(1 << n)]
        prev = [[-1] * n for _ in range(1 << n)]

        # 初始化：只访问起点
        dp[1 << start_idx][start_idx] = 0

        # 遍历所有状态
        for mask in range(1 << n):
            for u in range(n):
                if not (mask & (1 << u)):
                    continue
                if dp[mask][u] == float('inf'):
                    continue

                # 尝试扩展到v
                for v in range(n):
                    if mask & (1 << v):
                        continue
                    new_mask = mask | (1 << v)
                    new_cost = dp[mask][u] + self.cost[u][v]
                    if new_cost < dp[new_mask][v]:
                        dp[new_mask][v] = new_cost
                        prev[new_mask][v] = u

        # 找到访问所有点的最小代价
        full_mask = (1 << n) - 1
        min_cost = min(dp[full_mask][u] for u in range(n))
        last_node = min(range(n), key=lambda u: dp[full_mask][u])

        # 回溯路径
        path = []
        mask = full_mask
        current = last_node
        while current != -1:
            path.append(current)
            next_current = prev[mask][current]
            mask ^= (1 << current)
            current = next_current

        path.reverse()
        return min_cost, path


# ====================== 欧拉角转四元数 ======================
def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [qx, qy, qz, qw]


# ====================== 优化导航节点 ======================
class OptimizedWaypointNavigatorDP(BasicNavigator):
    def __init__(self):
        super().__init__()
        self.get_logger().info("✅ 动态规划TSP导航节点已启动")

        # TF监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 发布器
        self.path_pub = self.create_publisher(Int32MultiArray, '/path_segment_points', 10)
        self.best_order_pub = self.create_publisher(Int32MultiArray, '/best_waypoint_order', 10)

        # 路点（可轻松扩展）
        self.waypoint_map = {
            1: self.create_goal_pose(0.993753, 0.62935, 0.0),
            2: self.create_goal_pose(-1.26875, 1.02813, math.pi/2),
            3: self.create_goal_pose(-1.84094, 3.89227, -math.pi),
            # 4: self.create_goal_pose(x, y, yaw),  # 轻松添加第4个点
            # 5: self.create_goal_pose(x, y, yaw),  # 轻松添加第5个点
        }
        self.waypoint_ids = list(self.waypoint_map.keys())
        self.n_waypoints = len(self.waypoint_ids)

        # 路点间代价矩阵（动态更新）
        self.cost_between = {
            1: {2: 92, 3: 175},
            2: {1: 86, 3: 107},
            3: {1: 197, 2: 119},
        }

        # 拥挤度数据
        self.crowd_data = {
            1: {"level": "L1", "count": 0, "C": 1},
            2: {"level": "L1", "count": 0, "C": 1},
            3: {"level": "L1", "count": 0, "C": 1},
        }

        # 订阅 YOLO
        self.yolo_sub = self.create_subscription(
            String, '/yolov8_detections', self.yolo_detection_callback, 10
        )

        self.restart_flag = False

    def yolo_detection_callback(self, msg):
        """解析YOLO检测结果（与原代码相同）"""
        try:
            data = msg.data
            spot = None
            level = None
            count = 0

            match_spot = re.search(r'spot=(\d+)', data)
            if match_spot:
                spot = int(match_spot.group(1))

            match_count = re.search(r'count=(\d+)', data)
            if match_count:
                count = int(match_count.group(1))

            match_level = re.search(r'level=([A-Z0-9]+)', data)
            if match_level:
                level = match_level.group(1)

            if spot in self.waypoint_ids:
                if count <= 13:
                    C = 1
                elif 14 <= count <= 22:
                    C = 2
                elif 23 <= count <= 31:
                    C = 3
                elif 32 <= count <= 40:
                    C = 4
                else:
                    C = 5

                self.crowd_data[spot] = {
                    "level": level, "count": count, "C": C
                }
                self.get_logger().info(
                    f"📊 更新 spot{spot}：人数={count}, 等级={level}, C={C}"
                )
        except Exception as e:
            pass

    def create_goal_pose(self, x, y, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        return goal

    def get_current_robot_pose(self, max_attempts=200):
        self.get_logger().info('尝试获取机器人位姿...')
        attempt = 0
        target_frames = ['base_link', 'base_footprint']
        while attempt < max_attempts:
            for frame in target_frames:
                try:
                    if self.tf_buffer.can_transform('map', frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)):
                        transform = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.header.stamp = self.get_clock().now().to_msg()
                        pose.pose.position.x = transform.transform.translation.x
                        pose.pose.position.y = transform.transform.translation.y
                        pose.pose.position.z = transform.transform.translation.z
                        pose.pose.orientation = transform.transform.rotation
                        self.get_logger().info(f"✅ 成功获取位姿: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
                        return pose
                except:
                    continue
            rclpy.spin_once(self, timeout_sec=0.1)
            attempt += 1
        self.get_logger().error("❌ 无法获取位姿，使用(0,0)")
        return self.create_goal_pose(0.0, 0.0)

    def safe_get_path_len(self, start, goal, name):
        try:
            path = self.getPath(start, goal)
            length = len(path.poses) if path else 0
            self.get_logger().info(f"✓ {name}：{length} 点")
            return length
        except:
            self.get_logger().error(f"✗ {name} 规划失败")
            return 0

    def listen_keyboard(self):
        while True:
            input()
            self.get_logger().warning("\n🚨 检测到回车输入：准备打断导航并重新规划！")
            self.restart_flag = True
            self.cancelTask()

    # ====================== 核心：使用动态规划求解 ======================
    def find_optimal_order(self, start_costs):
        """
        使用动态规划寻找最优路点顺序

        Args:
            start_costs: 起点到各路点的代价 dict {waypoint_id: cost}

        Returns:
            (best_order, details): 最优路点顺序和详细信息
        """
        # 构建索引映射
        wp_ids = self.waypoint_ids
        idx_to_wp = {i: wp_id for i, wp_id in enumerate(wp_ids)}
        wp_to_idx = {wp_id: i for i, wp_id in enumerate(wp_ids)}
        n = len(wp_ids)

        # 1. 构建基础代价矩阵（仅路径）
        base_cost_matrix = [[0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if i != j:
                    wp_i = idx_to_wp[i]
                    wp_j = idx_to_wp[j]
                    base_cost_matrix[i][j] = self.cost_between[wp_i][wp_j]

        # 2. 权重配置
        weights = [1.0, 0.6, 0.2] + [0.1] * (n - 3)  # 可扩展到更多点
        crowd_multiplier = 30

        self.get_logger().info(f"\n🔍 评估所有可能顺序（使用动态规划）...")

        # 3. 对每个可能的起点，使用DP求解
        min_total = float('inf')
        best_order = None
        best_details = None

        for start_wp in wp_ids:
            start_idx = wp_to_idx[start_wp]

            # 构建综合代价矩阵
            combined_matrix = [[0] * n for _ in range(n)]
            for i in range(n):
                for j in range(n):
                    if i != j:
                        combined_matrix[i][j] = base_cost_matrix[i][j]

            # 使用DP求解从start_idx出发的最优路径
            tsp_solver = TSPDynamicProgramming(combined_matrix)
            path_cost, path_idx = tsp_solver.solve_path_from_start(start_idx)

            # 转换回路点ID
            path_wp = [idx_to_wp[i] for i in path_idx]

            # 计算拥挤度惩罚
            crowd_sum = 0
            for pos, wp in enumerate(path_wp):
                weight = weights[pos] if pos < len(weights) else 0.1
                crowd_sum += self.crowd_data[wp]["C"] * weight

            # 加上起点到第一个点的代价
            total_path_cost = start_costs[start_wp] + path_cost
            total_score = total_path_cost + crowd_sum * crowd_multiplier

            self.get_logger().info(
                f"  起点{start_wp} → 顺序{path_wp}: "
                f"路径={total_path_cost}, 拥挤×30={crowd_sum*crowd_multiplier:.1f}, "
                f"总分={total_score:.1f}"
            )

            if total_score < min_total:
                min_total = total_score
                best_order = path_wp
                best_details = {
                    "path": total_path_cost,
                    "crowd_sum": round(crowd_sum, 2),
                    "score": round(total_score, 2)
                }

        return best_order, best_details

    def run_once(self):
        self.restart_flag = False
        self.get_logger().info("\n========== 开始新一轮【DP最优规划】 ==========")

        start_pose = self.get_current_robot_pose()

        # 计算起点到各路点的代价
        start_costs = {}
        for wp_id in self.waypoint_ids:
            wp = self.waypoint_map[wp_id]
            cost = self.safe_get_path_len(start_pose, wp, f"起点→{wp_id}")
            start_costs[wp_id] = cost

        # 发布
        path_msg = Int32MultiArray()
        path_msg.data = [start_costs[wp_id] for wp_id in self.waypoint_ids]
        self.path_pub.publish(path_msg)

        # 使用动态规划找最优顺序
        best_order, best_detail = self.find_optimal_order(start_costs)

        # 发布最优顺序
        order_msg = Int32MultiArray()
        order_msg.data = list(best_order)
        self.best_order_pub.publish(order_msg)

        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"🎉 最优顺序：{best_order}")
        self.get_logger().info(f"📊 路径点数：{best_detail['path']} | 拥挤加权和：{best_detail['crowd_sum']}")
        self.get_logger().info(f"🏆 最终总分：{best_detail['score']}")
        self.get_logger().info("="*70)

        # 执行导航
        self.get_logger().info("\n开始导航……")
        self.cancelTask()
        time.sleep(0.5)

        waypoints = [self.waypoint_map[i] for i in best_order]
        self.followWaypoints(waypoints)

        while not self.isTaskComplete():
            if self.restart_flag:
                self.get_logger().warn("🟠 已被打断，准备重新规划……")
                return
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f"🚩 当前目标：{feedback.current_waypoint}")
            time.sleep(0.2)

    def run_loop(self):
        key_thread = threading.Thread(target=self.listen_keyboard, daemon=True)
        key_thread.start()
        self.get_logger().info("⌨️  键盘监听已启动：按回车 = 打断 + 重新规划")

        input("\n🚦 按回车开始第一次规划导航……")

        while True:
            self.run_once()
            if self.restart_flag:
                self.get_logger().info("\n🔄 即将重新执行全流程……\n")
                time.sleep(1)
            else:
                self.get_logger().info("\n🏁 导航完成！按回车再次执行……")
                input()


def main():
    rclpy.init()
    navigator = OptimizedWaypointNavigatorDP()

    print("等待导航系统激活...")
    try:
        navigator.lifecycleStartup()
        print("导航已激活")
    except Exception as e:
        print(f"导航启动失败: {e}")
        return

    try:
        navigator.run_loop()
    except KeyboardInterrupt:
        print("\n🛑 程序退出")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
