#!/usr/bin/env python3
"""
路点选择器 - 独立程序
功能：
1. RViz交互式选点（用2D Pose Estimate）
2. 预计算路点间代价矩阵
3. 保存到缓存文件供导航器使用
"""
# 使用示例
# 为遗传算法版本准备配置文件：


# # 选点并保存为 ga_ 前缀的文件
# python3 waypoint_selector.py --prefix ga

# # 使用 ga_ 前缀的配置文件运行遗传算法导航器
# python3 navigator_full_ga.py --prefix ga
# 为动态规划版本准备配置文件：


# # 选点并保存为 dp_ 前缀的文件
# python3 waypoint_selector.py --prefix dp

# # 使用 dp_ 前缀的配置文件运行动态规划导航器
# python3 navigator_full_dp.py --prefix dp
# 或者分别指定文件名：


# python3 waypoint_selector.py --waypoints my_waypoints.json --cost my_cost.json
# python3 navigator_full_ga.py --waypoints my_waypoints.json --cost my_cost.json
#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
import time
import threading
import json
import os
import sys
import argparse


# ====================== 欧拉角转四元数 ======================
def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [qx, qy, qz, qw]


# ====================== 路点选择器节点 ======================
class WaypointSelector(BasicNavigator):
    def __init__(self, waypoint_cache_file='waypoint_cache.json', cost_cache_file='cost_cache.json'):
        super().__init__('waypoint_selector')
        self.get_logger().info("="*60)
        self.get_logger().info("📍 路点选择器 已启动")
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

        # 缓存文件路径
        self.waypoint_cache_file = waypoint_cache_file
        self.cost_cache_file = cost_cache_file

        self.get_logger().info("\n📌 使用说明：")
        self.get_logger().info("   1. 在RViz中点击 '2D Pose Estimate' 按钮")
        self.get_logger().info("   2. 在地图上依次点击添加路点")
        self.get_logger().info("   3. 选完后在终端按回车确认\n")

        # 检查现有缓存
        self._check_existing_cache()

    def _check_existing_cache(self):
        """检查是否有现有缓存"""
        if os.path.exists(self.waypoint_cache_file):
            try:
                with open(self.waypoint_cache_file, 'r') as f:
                    data = json.load(f)
                waypoints = data.get('waypoints', [])
                self.get_logger().info(f"📦 发现现有缓存: {len(waypoints)} 个路点")
                self.get_logger().info("   按回车使用现有缓存，或直接在RViz中选点覆盖\n")
            except Exception as e:
                self.get_logger().warning(f"⚠️  无法读取缓存文件: {e}")

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
            'yaw': yaw
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

    def safe_get_path_len(self, start, goal, name):
        try:
            path = self.getPath(start, goal, use_start=True)
            length = len(path.poses) if path else 0
            return length
        except Exception as e:
            self.get_logger().error(f"✗ {name} 规划失败: {e}")
            return 0

    def listen_keyboard(self):
        while True:
            input()
            if not self.selection_confirmed:
                # 检查是否有缓存且未选点
                if len(self.selected_waypoints) == 0 and os.path.exists(self.waypoint_cache_file):
                    self.get_logger().info("\n📦 使用现有缓存...")
                    self.selection_confirmed = True
                    self._use_existing_cache = True
                    continue

                if len(self.selected_waypoints) < 2:
                    self.get_logger().warning("⚠️  至少需要选择2个路点！")
                    continue
                self.selection_confirmed = True
                self.get_logger().info("\n✅ 路点选择已确认！")
            else:
                self.get_logger().warning("\n🚨 检测到回车：准备重新开始！")
                self.restart_flag = True

    def _load_waypoints_from_cache(self):
        """从缓存加载路点"""
        with open(self.waypoint_cache_file, 'r') as f:
            data = json.load(f)
        waypoints = data.get('waypoints', [])
        # 转换格式
        self.selected_waypoints = []
        for i, wp in enumerate(waypoints):
            self.selected_waypoints.append({
                'id': i + 1,
                'x': wp['x'],
                'y': wp['y'],
                'yaw': wp['yaw']
            })
        return waypoints

    def _waypoints_match(self, waypoints1, waypoints2, tolerance=0.1):
        """检查两组路点是否匹配"""
        if len(waypoints1) != len(waypoints2):
            return False
        for w1, w2 in zip(waypoints1, waypoints2):
            dx = abs(w1['x'] - w2['x'])
            dy = abs(w1['y'] - w2['y'])
            if dx > tolerance or dy > tolerance:
                return False
        return True

    def calculate_and_save(self):
        """计算代价矩阵并保存缓存"""
        # 检查是否使用现有缓存
        if hasattr(self, '_use_existing_cache') and self._use_existing_cache:
            self._load_waypoints_from_cache()
            # 检查是否有cost_cache
            if os.path.exists(self.cost_cache_file):
                try:
                    with open(self.cost_cache_file, 'r') as f:
                        cost_data = json.load(f)
                    if 'cost_matrix' in cost_data:
                        self.get_logger().info("✅ 代价矩阵缓存已存在，跳过计算")
                        self.get_logger().info("\n🎉 完成！现在可以运行导航器了")
                        return True
                except Exception as e:
                    self.get_logger().warning(f"⚠️  无法读取代价缓存: {e}")

        n = len(self.selected_waypoints)
        if n < 2:
            self.get_logger().error("❌ 路点太少")
            return False

        # 显示路点列表
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 选中的路点列表：")
        for wp in self.selected_waypoints:
            self.get_logger().info(f"   {wp['id']}: (x={wp['x']:.2f}, y={wp['y']:.2f})")
        self.get_logger().info("="*60)

        # 创建Pose对象
        waypoint_poses = []
        for wp in self.selected_waypoints:
            pose = self._create_pose_from_data(wp['x'], wp['y'], wp['yaw'])
            waypoint_poses.append(pose)

        # 检查是否有可用的cost_cache
        need_recalc = True
        cached_cost_matrix = None

        if os.path.exists(self.cost_cache_file) and os.path.exists(self.waypoint_cache_file):
            try:
                with open(self.waypoint_cache_file, 'r') as f:
                    old_waypoint_data = json.load(f)
                old_waypoints = old_waypoint_data.get('waypoints', [])

                # 比较路点
                new_waypoints_simple = [{'x': wp['x'], 'y': wp['y'], 'yaw': wp['yaw']}
                                        for wp in self.selected_waypoints]

                if self._waypoints_match(old_waypoints, new_waypoints_simple):
                    with open(self.cost_cache_file, 'r') as f:
                        cost_data = json.load(f)
                    if 'cost_matrix' in cost_data:
                        cached_cost_matrix = cost_data['cost_matrix']
                        if len(cached_cost_matrix) == n:
                            self.get_logger().info("✅ 路点未变，使用缓存的代价矩阵")
                            need_recalc = False
            except Exception as e:
                self.get_logger().warning(f"⚠️  检查缓存时出错: {e}")

        cost_matrix = cached_cost_matrix

        # 如果需要，重新计算
        if need_recalc:
            self.get_logger().info(f"\n🔍 计算路径代价矩阵 (n={n})...")
            self.get_logger().info(f"   需要计算 {n*(n-1)} 次路径规划，请稍候...")

            cost_matrix = [[0] * n for _ in range(n)]
            start_time = time.time()

            for i in range(n):
                for j in range(n):
                    if i != j:
                        cost = self.safe_get_path_len(
                            waypoint_poses[i],
                            waypoint_poses[j],
                            f"{i+1}→{j+1}"
                        )
                        cost_matrix[i][j] = cost
                        self.get_logger().info(f"  {i+1}→{j+1}: {cost} 点")

            elapsed = time.time() - start_time
            self.get_logger().info(f"\n⏱️  计算完成，耗时 {elapsed:.1f} 秒")

        # 保存路点缓存
        waypoint_data = {
            'waypoints': [{'x': wp['x'], 'y': wp['y'], 'yaw': wp['yaw']}
                          for wp in self.selected_waypoints],
            'timestamp': time.time()
        }
        with open(self.waypoint_cache_file, 'w') as f:
            json.dump(waypoint_data, f, indent=2)
        self.get_logger().info(f"💾 路点已保存到 {self.waypoint_cache_file}")

        # 保存代价矩阵缓存
        if cost_matrix is not None:
            cost_data = {
                'cost_matrix': cost_matrix,
                'timestamp': time.time()
            }
            with open(self.cost_cache_file, 'w') as f:
                json.dump(cost_data, f, indent=2)
            self.get_logger().info(f"💾 代价矩阵已保存到 {self.cost_cache_file}")

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🎉 完成！现在可以运行导航器了")
        self.get_logger().info("="*60)
        return True

    def run_once(self):
        self.restart_flag = False
        self.selection_confirmed = False
        self.selected_waypoints = []
        if hasattr(self, '_use_existing_cache'):
            delattr(self, '_use_existing_cache')

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📍 等待在RViz中选择路点...")
        self.get_logger().info("="*60)
        self.get_logger().info("\n提示：用RViz工具栏的 '2D Pose Estimate' 点击地图添加路点")
        self.get_logger().info("      选完后按回车确认\n")

        # 等待用户选点
        while not self.selection_confirmed:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)

        # 计算并保存
        self.calculate_and_save()

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
                self.get_logger().info("\n按回车重新选点...")
                input()


def main():
    parser = argparse.ArgumentParser(description='路点选择器 - 支持指定配置文件')
    parser.add_argument('--prefix', type=str, default='',
                        help='配置文件前缀 (例如: --prefix ga 则使用 ga_waypoint_cache.json 和 ga_cost_cache.json)')
    parser.add_argument('--waypoints', type=str, default=None,
                        help='路点缓存文件名 (默认: waypoint_cache.json)')
    parser.add_argument('--cost', type=str, default=None,
                        help='代价矩阵缓存文件名 (默认: cost_cache.json)')
    args = parser.parse_args()

    # 确定配置文件名
    if args.prefix:
        waypoint_file = f'{args.prefix}_waypoint_cache.json'
        cost_file = f'{args.prefix}_cost_cache.json'
    else:
        waypoint_file = args.waypoints or 'waypoint_cache.json'
        cost_file = args.cost or 'cost_cache.json'

    rclpy.init()
    selector = WaypointSelector(waypoint_cache_file=waypoint_file, cost_cache_file=cost_file)

    print(f"使用配置文件: {waypoint_file}, {cost_file}")
    print("等待导航系统激活...")
    try:
        selector.lifecycleStartup()
        print("✅ 导航已激活")
    except Exception as e:
        print(f"❌ 导航启动失败: {e}")
        return

    try:
        selector.run_loop()
    except KeyboardInterrupt:
        print("\n🛑 程序退出")
    finally:
        selector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
