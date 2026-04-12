#!/usr/bin/env python3
"""
完整路径遍历导航器 - 动态规划 TSP 版本
功能：
1. RViz交互式选点（用2D Pose Estimate）或从缓存文件读取
2. 动态规划求解TSP最优路径
3. 闭环导航（最后回到起点）
"""
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Int32MultiArray, String
import rclpy
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
import time
import threading
import json
import os
import argparse
import itertools
import re


# ====================== 动态规划 TSP 求解器 ======================
class TSPDynamicProgramming:
    """
    支持带起始点的TSP求解（不要求返回起点）
    """
    def __init__(self, cost_matrix):
        self.cost = cost_matrix
        self.n = len(cost_matrix)

    def solve_path_from_start(self, start_idx):
        """
        求解从起点出发，访问所有其他点的最优路径（不返回起点）
        """
        n = self.n
        dp = [[float('inf')] * n for _ in range(1 << n)]
        prev = [[-1] * n for _ in range(1 << n)]

        dp[1 << start_idx][start_idx] = 0

        for mask in range(1 << n):
            for u in range(n):
                if not (mask & (1 << u)):
                    continue
                if dp[mask][u] == float('inf'):
                    continue
                for v in range(n):
                    if mask & (1 << v):
                        continue
                    new_mask = mask | (1 << v)
                    new_cost = dp[mask][u] + self.cost[u][v]
                    if new_cost < dp[new_mask][v]:
                        dp[new_mask][v] = new_cost
                        prev[new_mask][v] = u

        full_mask = (1 << n) - 1
        min_cost = min(dp[full_mask][u] for u in range(n))
        last_node = min(range(n), key=lambda u: dp[full_mask][u])

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
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [qx, qy, qz, qw]


# ====================== 完整路径导航节点 ======================
class FullPathNavigatorDP(BasicNavigator):
    def __init__(self, waypoint_cache_file=None, cost_cache_file=None):
        super().__init__('full_path_navigator_dp')
        self.get_logger().info("="*60)
        self.get_logger().info("🚀 完整路径导航器 (DP版本) 已启动")
        self.get_logger().info("="*60)

        # TF监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅 RViz 选点（2D Pose Estimate）
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_selection_callback,
            10
        )

        # 选中的路点列表
        self.selected_waypoints = []
        self.selection_confirmed = False
        self.restart_flag = False

        # 缓存文件路径 (可选)
        self.waypoint_cache_file = waypoint_cache_file
        self.cost_cache_file = cost_cache_file
        self.cost_matrix = None  # 从缓存读取的代价矩阵
        self.use_cache = waypoint_cache_file is not None and os.path.exists(waypoint_cache_file)

        # 起点（机器人当前位置）
        self.start_pose = None

        # ====================== 新增：拥挤度数据 ======================
        self.crowd_data = {}

        # 订阅 YOLO 检测结果
        self.yolo_sub = self.create_subscription(
            String,
            '/yolov8_detections',
            self.yolo_detection_callback,
            10
        )

        # 发布器
        self.path_pub = self.create_publisher(Int32MultiArray, '/path_segment_points', 10)
        self.best_order_pub = self.create_publisher(Int32MultiArray, '/best_waypoint_order', 10)

        self.get_logger().info("\n📌 使用说明：")
        self.get_logger().info("   1. 在RViz中点击 '2D Pose Estimate' 按钮")
        self.get_logger().info("   2. 在地图上依次点击添加路点")
        self.get_logger().info("   3. 选完后在终端按回车确认\n")

    def pose_selection_callback(self, msg):
        """处理RViz选点"""
        if self.selection_confirmed:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # 提取yaw角
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        wp_id = len(self.selected_waypoints) + 1
        self.selected_waypoints.append({
            'id': wp_id,
            'x': x,
            'y': y,
            'yaw': yaw,
            'pose': self._create_pose_from_data(x, y, yaw)
        })

        self.get_logger().info(f"✅ 已添加路点 {wp_id}: (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f})")
        self.get_logger().info(f"   当前共 {len(self.selected_waypoints)} 个路点\n")

    def _create_pose_from_data(self, x, y, yaw):
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

    # ====================== 新增：解析 yolov8 话题 ======================
    def yolo_detection_callback(self, msg):
        try:
            data = msg.data
            spot = None
            level = None
            count = 0

            # 解析 spot
            match_spot = re.search(r'spot=(\d+)', data)
            if match_spot:
                spot = int(match_spot.group(1))

            # 解析 count
            match_count = re.search(r'count=(\d+)', data)
            if match_count:
                count = int(match_count.group(1))

            # 解析 level
            match_level = re.search(r'level=([A-Z0-9]+)', data)
            if match_level:
                level = match_level.group(1)

            # 初始化拥挤度数据
            if not hasattr(self, '_crowd_initialized'):
                self._crowd_initialized = True
                # 初始化所有路点的拥挤度
                for wp in self.selected_waypoints:
                    self.crowd_data[wp['id']] = {"level": "L1", "count": 0, "C": 1}

            if spot in self.crowd_data:
                # 计算拥挤参数 C
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
                    "level": level,
                    "count": count,
                    "C": C
                }
                self.get_logger().info(f"📊 更新 spot{spot}：人数={count}, 等级={level}, 拥挤参数C={C}")
        except Exception as e:
            pass

    def load_cache(self):
        """从缓存文件加载路点和代价矩阵"""
        if not self.waypoint_cache_file or not os.path.exists(self.waypoint_cache_file):
            return False

        self.get_logger().info(f"📦 加载缓存文件: {self.waypoint_cache_file}")

        try:
            with open(self.waypoint_cache_file, 'r') as f:
                wp_data = json.load(f)
            wp_list = wp_data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"❌ 读取路点缓存失败: {e}")
            return False

        # 尝试加载代价矩阵
        if self.cost_cache_file and os.path.exists(self.cost_cache_file):
            try:
                with open(self.cost_cache_file, 'r') as f:
                    cost_data = json.load(f)
                self.cost_matrix = cost_data.get('cost_matrix')
            except Exception as e:
                self.get_logger().warning(f"⚠️  读取代价矩阵缓存失败: {e}")
                self.cost_matrix = None

        self.selected_waypoints = []
        for i, wp in enumerate(wp_list):
            self.selected_waypoints.append({
                'id': i + 1,
                'x': wp['x'],
                'y': wp['y'],
                'yaw': wp['yaw'],
                'pose': self._create_pose_from_data(wp['x'], wp['y'], wp['yaw'])
            })

        self.get_logger().info(f"✅ 加载成功: {len(self.selected_waypoints)} 个路点")
        if self.cost_matrix:
            self.get_logger().info(f"✅ 已加载代价矩阵")
        return True

    def get_current_robot_pose(self, max_attempts=200):
        self.get_logger().info('🤖 尝试获取机器人位姿...')
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
                        self.get_logger().info(f"✅ 获取成功: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
                        return pose
                except:
                    continue
            rclpy.spin_once(self, timeout_sec=0.1)
            attempt += 1
        self.get_logger().error("❌ 无法获取位姿，使用(0,0)")
        return self._create_pose_from_data(0.0, 0.0, 0.0)

    def safe_get_path_len(self, start, goal, name):
        try:
            path = self.getPath(start, goal)
            length = len(path.poses) if path else 0
            return length
        except:
            self.get_logger().error(f"✗ {name} 规划失败")
            return 0

    def listen_keyboard(self):
        while True:
            input()
            if not self.selection_confirmed:
                if len(self.selected_waypoints) < 2:
                    self.get_logger().warning("⚠️  至少需要选择2个路点！")
                    continue
                self.selection_confirmed = True
                self.get_logger().info("\n✅ 路点选择已确认！")
            else:
                self.get_logger().warning("\n🚨 检测到回车：准备打断导航！")
                self.restart_flag = True
                self.cancelTask()

    def find_optimal_order(self, start_pose, waypoints):
        """使用排列遍历 + 拥挤度加权寻找最优路点顺序"""
        n = len(waypoints)
        wp_ids = [wp['id'] for wp in waypoints]

        # 初始化拥挤度数据
        for wp_id in wp_ids:
            if wp_id not in self.crowd_data:
                self.crowd_data[wp_id] = {"level": "L1", "count": 0, "C": 1}

        # 1. 计算起点到所有路点的代价
        self.get_logger().info(f"\n🔍 计算起点到各路点的代价...")
        start_costs = []
        for i, wp in enumerate(waypoints):
            cost = self.safe_get_path_len(start_pose, wp['pose'], f"起点→{i+1}")
            start_costs.append(cost)
            self.get_logger().info(f"  起点→{i+1}: {cost} 点")

        # 发布起点到各路点的代价
        path_msg = Int32MultiArray()
        path_msg.data = start_costs
        self.path_pub.publish(path_msg)

        # 2. 获取路点之间的代价矩阵 (使用缓存或重新计算)
        if self.cost_matrix is not None and len(self.cost_matrix) == n:
            self.get_logger().info(f"✅ 使用缓存的代价矩阵")
            cost_matrix = self.cost_matrix
        else:
            self.get_logger().info(f"\n🔍 计算路径代价矩阵 (n={n})...")
            self.get_logger().info(f"   需要计算 {n*(n-1)} 次路径规划，请稍候...")
            cost_matrix = [[0] * n for _ in range(n)]
            for i in range(n):
                for j in range(n):
                    if i != j:
                        cost = self.safe_get_path_len(
                            waypoints[i]['pose'],
                            waypoints[j]['pose'],
                            f"{i+1}→{j+1}"
                        )
                        cost_matrix[i][j] = cost
                        self.get_logger().info(f"  {i+1}→{j+1}: {cost} 点")

        # 3. 构建路点间代价字典 (按ID索引)
        cost_between = {}
        for i in range(n):
            cost_between[wp_ids[i]] = {}
            for j in range(n):
                if i != j:
                    cost_between[wp_ids[i]][wp_ids[j]] = cost_matrix[i][j]

        # 4. 生成顺序权重 (根据路点数量动态生成)
        weights = []
        for i in range(n):
            # 权重递减: 1.0, 0.6, 0.2, 0.1, 0.05...
            if i == 0:
                weights.append(1.0)
            elif i == 1:
                weights.append(0.6)
            elif i == 2:
                weights.append(0.2)
            else:
                weights.append(0.1 / (i - 2))

        # 5. 使用排列遍历寻找最优顺序 (路径 + 拥挤度)
        self.get_logger().info(f"\n🧮 使用排列遍历 + 拥挤度加权求解最优路径...")
        self.get_logger().info(f"   顺序权重: {[round(w, 2) for w in weights]}")
        min_score = float('inf')
        best_order = None
        best_detail = {}

        for order in itertools.permutations(wp_ids):
            # 计算路径代价
            first_wp_idx = wp_ids.index(order[0])
            path_cost = start_costs[first_wp_idx]
            for i in range(n - 1):
                a = order[i]
                b = order[i + 1]
                path_cost += cost_between[a][b]

            # 计算拥挤度加权和
            crowd_sum = 0
            for i in range(n):
                wp_id = order[i]
                c = self.crowd_data[wp_id]["C"]
                crowd_sum += c * weights[i]

            # 最终总分 = 路径代价 + 拥挤度加权和 × 30
            total_score = path_cost + crowd_sum * 30

            # 选择最小分
            if total_score < min_score:
                min_score = total_score
                best_order = order
                best_detail = {
                    "path": path_cost,
                    "crowd_sum": round(crowd_sum, 2),
                    "score": round(total_score, 2)
                }

        # 发布最优顺序
        order_msg = Int32MultiArray()
        order_msg.data = list(best_order)
        self.best_order_pub.publish(order_msg)

        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"🎉 最优顺序：{best_order}")
        self.get_logger().info(f"📊 路径点数：{best_detail['path']} | 拥挤加权和：{best_detail['crowd_sum']}")
        self.get_logger().info(f"🏆 最终总分（越小越好）：{best_detail['score']}")
        self.get_logger().info("="*70)

        # 转换为索引列表返回
        best_order_idx = [wp_ids.index(wp_id) for wp_id in best_order]
        return best_order_idx, start_costs, cost_matrix

    def run_once(self):
        self.restart_flag = False
        self.selection_confirmed = False
        self.selected_waypoints = []

        self.get_logger().info("\n" + "="*60)

        # 尝试从缓存加载
        if self.use_cache:
            if self.load_cache():
                self.selection_confirmed = True
                self.get_logger().info("✅ 使用缓存的路点")
            else:
                self.get_logger().warning("⚠️  缓存加载失败，将使用交互式选点")
                self.use_cache = False

        if not self.selection_confirmed:
            self.get_logger().info("📍 等待在RViz中选择路点...")
            self.get_logger().info("="*60)
            self.get_logger().info("\n提示：用RViz工具栏的 '2D Pose Estimate' 点击地图添加路点")
            self.get_logger().info("      选完后按回车确认\n")

            # 等待用户选点
            while not self.selection_confirmed:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.01)
        else:
            self.get_logger().info("="*60)

        if len(self.selected_waypoints) < 2:
            self.get_logger().error("❌ 路点太少，退出")
            return

        # 获取机器人当前位置
        self.start_pose = self.get_current_robot_pose()

        # 显示所有选中的路点
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 选中的路点列表：")
        for wp in self.selected_waypoints:
            self.get_logger().info(f"   {wp['id']}: (x={wp['x']:.2f}, y={wp['y']:.2f})")
        self.get_logger().info("="*60)

        # TSP求解最优顺序
        best_order_idx, start_costs, cost_matrix = self.find_optimal_order(
            self.start_pose,
            self.selected_waypoints
        )

        # 构建导航路点列表
        nav_waypoints = [self.selected_waypoints[i]['pose'] for i in best_order_idx]

        # 显示最优顺序
        best_order_ids = [self.selected_waypoints[i]['id'] for i in best_order_idx]
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"🏆 最优访问顺序: {best_order_ids}")
        self.get_logger().info("="*60)

        # 开始导航
        self.get_logger().info("\n🚗 开始导航...")
        self.cancelTask()
        time.sleep(0.5)

        self.followWaypoints(nav_waypoints)

        while not self.isTaskComplete():
            if self.restart_flag:
                self.get_logger().warn("🟠 已被打断，准备重新开始...")
                return
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f"🚩 当前目标: {feedback.current_waypoint + 1}/{len(nav_waypoints)}")
            time.sleep(0.2)

        # 回到起点
        self.get_logger().info("\n🔄 所有路点访问完成，返回起点...")
        self.goToPose(self.start_pose)

        while not self.isTaskComplete():
            if self.restart_flag:
                self.get_logger().warn("🟠 已被打断，准备重新开始...")
                return
            time.sleep(0.2)

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("✅ 导航完成！已回到起点")
        self.get_logger().info("="*60)

    def run_loop(self):
        key_thread = threading.Thread(target=self.listen_keyboard, daemon=True)
        key_thread.start()
        self.get_logger().info("⌨️  键盘监听已启动\n")

        while True:
            self.run_once()
            if self.restart_flag:
                self.get_logger().info("\n🔄 即将重新开始...\n")
                time.sleep(1)
            else:
                self.get_logger().info("\n按回车重新开始...")
                input()


def main():
    parser = argparse.ArgumentParser(description='完整路径导航器 (DP版本) - 支持指定配置文件')
    parser.add_argument('--prefix', type=str, default='',
                        help='配置文件前缀 (例如: --prefix dp 则使用 dp_waypoint_cache.json 和 dp_cost_cache.json)')
    parser.add_argument('--waypoints', type=str, default=None,
                        help='路点缓存文件名 (例如: waypoint_cache.json，不指定则使用交互式选点)')
    parser.add_argument('--cost', type=str, default=None,
                        help='代价矩阵缓存文件名 (例如: cost_cache.json)')
    args = parser.parse_args()

    # 确定配置文件名
    waypoint_file = None
    cost_file = None
    if args.prefix:
        waypoint_file = f'{args.prefix}_waypoint_cache.json'
        cost_file = f'{args.prefix}_cost_cache.json'
    elif args.waypoints:
        waypoint_file = args.waypoints
        cost_file = args.cost

    rclpy.init()
    navigator = FullPathNavigatorDP(waypoint_cache_file=waypoint_file, cost_cache_file=cost_file)

    if waypoint_file:
        print(f"使用配置文件: {waypoint_file}" + (f", {cost_file}" if cost_file else ""))
    else:
        print("使用交互式选点模式")
    print("等待导航系统激活...")
    try:
        navigator.lifecycleStartup()
        print("✅ 导航已激活")
    except Exception as e:
        print(f"❌ 导航启动失败: {e}")
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
