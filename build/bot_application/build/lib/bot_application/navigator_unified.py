#!/usr/bin/env python3
"""
统一导航节点 - 支持 DP 和 GA 两种模式切换
功能：
1. 支持两种模式：全景点遍历(DP, 3路点+人流密度) 和 全路径遍历(GA, 多路点)
2. 通过语音指令切换模式
3. 支持语音指令重新规划
4. 启动自动加载缓存
"""
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Empty
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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
import random


# ====================== 动态规划 TSP 求解器 ======================
class TSPDynamicProgramming:
    """支持带起始点的TSP求解（不要求返回起点）"""
    def __init__(self, cost_matrix):
        self.cost = cost_matrix
        self.n = len(cost_matrix)

    def solve_path_from_start(self, start_idx):
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


# ====================== 遗传算法 TSP 求解器 ======================
class TSPGeneticAlgorithm:
    """遗传算法求解TSP"""
    def __init__(self, cost_matrix, population_size=150, max_iterations=800,
                 crossover_rate=0.85, mutation_rate=0.15, elite_size=5):
        self.cost = cost_matrix
        self.n = len(cost_matrix)
        self.population_size = population_size
        self.max_iterations = max_iterations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.elite_size = elite_size

    def _create_individual(self):
        individual = list(range(self.n))
        random.shuffle(individual)
        return individual

    def _create_population(self):
        return [self._create_individual() for _ in range(self.population_size)]

    def _fitness(self, individual):
        total_cost = 0
        for i in range(self.n - 1):
            total_cost += self.cost[individual[i]][individual[i+1]]
        return 1.0 / (total_cost + 1)

    def _tournament_selection(self, population, fitnesses, tournament_size=3):
        candidates = random.sample(list(zip(population, fitnesses)), tournament_size)
        candidates.sort(key=lambda x: x[1], reverse=True)
        return candidates[0][0].copy()

    def _pmx_crossover(self, parent1, parent2):
        size = self.n
        if size < 2:
            return parent1.copy()

        start = random.randint(0, size - 2)
        end = random.randint(start + 1, size - 1)

        child = [-1] * size
        child[start:end+1] = parent1[start:end+1]

        for i in range(start, end + 1):
            if parent2[i] not in child:
                value = parent2[i]
                pos = i
                while start <= pos <= end:
                    pos = parent2.index(parent1[pos])
                child[pos] = value

        for i in range(size):
            if child[i] == -1:
                child[i] = parent2[i]

        return child

    def _swap_mutation(self, individual):
        if self.n < 2:
            return individual
        i, j = random.sample(range(self.n), 2)
        individual[i], individual[j] = individual[j], individual[i]
        return individual

    def _inversion_mutation(self, individual):
        if self.n < 2:
            return individual
        start = random.randint(0, self.n - 2)
        end = random.randint(start + 1, self.n - 1)
        individual[start:end+1] = reversed(individual[start:end+1])
        return individual

    def _mutate(self, individual):
        if random.random() < 0.5:
            return self._swap_mutation(individual)
        else:
            return self._inversion_mutation(individual)

    def solve(self, verbose=True):
        if self.n <= 1:
            return 0, list(range(self.n))

        population = self._create_population()
        best_fitness = 0
        best_individual = None

        if verbose:
            print(f"\n🧬 遗传算法 TSP 求解 (n={self.n})")
            print(f"   种群大小: {self.population_size}, 最大迭代: {self.max_iterations}")
            print("-" * 50)

        for iteration in range(self.max_iterations):
            fitnesses = [self._fitness(ind) for ind in population]

            current_best_idx = fitnesses.index(max(fitnesses))
            current_best_fitness = fitnesses[current_best_idx]

            if current_best_fitness > best_fitness:
                best_fitness = current_best_fitness
                best_individual = population[current_best_idx].copy()

            if verbose and (iteration % 100 == 0 or iteration == self.max_iterations - 1):
                current_cost = 1.0 / best_fitness - 1
                print(f"   迭代 {iteration:4d}: 当前最优代价 = {current_cost:.1f}")

            new_population = []
            sorted_pop = sorted(zip(population, fitnesses), key=lambda x: x[1], reverse=True)
            elites = [ind.copy() for ind, fit in sorted_pop[:self.elite_size]]
            new_population.extend(elites)

            while len(new_population) < self.population_size:
                parent1 = self._tournament_selection(population, fitnesses)
                parent2 = self._tournament_selection(population, fitnesses)

                child = parent1.copy()
                if random.random() < self.crossover_rate:
                    child = self._pmx_crossover(parent1, parent2)

                if random.random() < self.mutation_rate:
                    child = self._mutate(child)

                new_population.append(child)

            population = new_population

        min_cost = 0
        for i in range(self.n - 1):
            min_cost += self.cost[best_individual[i]][best_individual[i+1]]

        if verbose:
            print("-" * 50)
            print(f"   最终最优代价 = {min_cost:.1f}")

        return min_cost, best_individual


# ====================== 欧拉角转四元数 ======================
def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
    return [qx, qy, qz, qw]


# ====================== 统一导航节点 ======================
class UnifiedNavigator(BasicNavigator):
    MODE_DP = "dp"    # 全景点遍历模式 - 3路点+人流密度
    MODE_GA = "ga"    # 全路径遍历模式 - 多路点

    def __init__(self):
        super().__init__('unified_navigator')
        self.get_logger().info("="*60)
        self.get_logger().info("🚀 统一导航器 (DP+GA双模式) 已启动")
        self.get_logger().info("="*60)

        # TF监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 模式状态
        self.current_mode = self.MODE_DP
        self.mode_switch_flag = False

        # 选中的路点列表
        self.selected_waypoints = []
        self.selection_confirmed = False
        self.restart_flag = False
        self.cost_matrix = None

        # 起点
        self.start_pose = None

        # 拥挤度数据 (仅DP模式使用)
        self.crowd_data = {}

        # 暂停/恢复状态
        self.is_paused = False
        self.nav_waypoints = None  # 保存当前导航路点列表（按最优排序的）
        self.nav_waypoint_ids = None  # 保存当前导航路点的ID
        self.current_waypoint_index = 0  # 保存当前路点索引
        self.priority_spot_id = None  # 优先前往的景点ID

        # 语音指令防抖
        self.last_voice_command_time = 0
        self.voice_command_debounce_seconds = 2.0

        # 使用可靠的 QoS 设置
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # 订阅话题
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.pose_selection_callback, 10)
        self.yolo_sub = self.create_subscription(
            String, '/yolov8_detections', self.yolo_detection_callback, 10)
        self.voice_sub = self.create_subscription(
            String, '/voice_commands', self.voice_command_callback, reliable_qos)

        # 发布器
        self.path_pub = self.create_publisher(Int32MultiArray, '/path_segment_points', 10)
        self.best_order_pub = self.create_publisher(Int32MultiArray, '/best_waypoint_order', 10)

        # Nav2 暂停/继续服务客户端
        self.pause_nav_client = self.create_client(Empty, '/pause_navigation')
        self.resume_nav_client = self.create_client(Empty, '/resume_navigation')

        # 等待服务可用
        self.get_logger().info("⏳ 等待 Nav2 暂停/继续服务...")
        pause_available = self.pause_nav_client.wait_for_service(timeout_sec=3.0)
        resume_available = self.resume_nav_client.wait_for_service(timeout_sec=3.0)
        if pause_available and resume_available:
            self.get_logger().info("✅ Nav2 暂停/继续服务已就绪")
        else:
            self.get_logger().warning("⚠️  Nav2 暂停/继续服务不可用，将使用手动暂停方式")

        self._print_usage()

    def _print_usage(self):
        self.get_logger().info("\n📌 使用说明：")
        self.get_logger().info("   1. 在RViz中点击 '2D Pose Estimate' 按钮选点")
        self.get_logger().info("   2. 选完后在终端按回车确认")
        self.get_logger().info("   3. 语音指令:")
        self.get_logger().info("      - '全景点遍历模式' → 3路点+人流密度(DP)")
        self.get_logger().info("      - '全路径遍历模式' → 多路点(GA)")
        self.get_logger().info("      - '重新规划路线' → 重规划")
        self.get_logger().info("      - '暂停运动' → 暂停导航")
        self.get_logger().info("      - '继续运动' → 恢复导航")
        self.get_logger().info("      - '前往景点1/2/3' → 立即前往指定景点\n")

    def _get_cache_files(self, mode):
        """获取指定模式的缓存文件名（已改为绝对路径）"""
        if mode == self.MODE_DP:
            waypoint_file = "/home/rose/pi4b_ros2/car_ws/src/bot_application/bot_application/dp_waypoint_cache.json"
            cost_file = "/home/rose/pi4b_ros2/car_ws/src/bot_application/bot_application/dp_cost_cache.json"
        else:
            waypoint_file = "/home/rose/pi4b_ros2/car_ws/src/bot_application/bot_application/ga_waypoint_cache.json"
            cost_file = "/home/rose/pi4b_ros2/car_ws/src/bot_application/bot_application/ga_cost_cache.json"
        return waypoint_file, cost_file

    def _load_cache_for_mode(self, mode):
        """加载指定模式的缓存"""
        waypoint_file, cost_file = self._get_cache_files(mode)

        if not os.path.exists(waypoint_file):
            self.get_logger().warning(f"⚠️  缓存文件不存在: {waypoint_file}")
            return False

        try:
            with open(waypoint_file, 'r') as f:
                wp_data = json.load(f)
            wp_list = wp_data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"❌ 读取路点缓存失败: {e}")
            return False

        self.cost_matrix = None
        if os.path.exists(cost_file):
            try:
                with open(cost_file, 'r') as f:
                    cost_data = json.load(f)
                self.cost_matrix = cost_data.get('cost_matrix')
            except Exception as e:
                self.get_logger().warning(f"⚠️  读取代价矩阵失败: {e}")

        self.selected_waypoints = []
        for i, wp in enumerate(wp_list):
            self.selected_waypoints.append({
                'id': i + 1,
                'x': wp['x'], 'y': wp['y'], 'yaw': wp['yaw'],
                'pose': self._create_pose_from_data(wp['x'], wp['y'], wp['yaw'])
            })

        self.get_logger().info(f"✅ 加载{mode.upper()}模式: {len(self.selected_waypoints)} 个路点")
        if self.cost_matrix:
            self.get_logger().info(f"✅ 已加载代价矩阵")
        return True

    def pose_selection_callback(self, msg):
        if self.selection_confirmed:
            return
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        wp_id = len(self.selected_waypoints) + 1
        self.selected_waypoints.append({
            'id': wp_id, 'x': x, 'y': y, 'yaw': yaw,
            'pose': self._create_pose_from_data(x, y, yaw)
        })
        self.get_logger().info(f"✅ 已添加路点 {wp_id}: (x={x:.2f}, y={y:.2f})")

    def _create_pose_from_data(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = x, y
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x, goal.pose.orientation.y = q[0], q[1]
        goal.pose.orientation.z, goal.pose.orientation.w = q[2], q[3]
        return goal

    def yolo_detection_callback(self, msg):
        if self.current_mode != self.MODE_DP:
            return
        try:
            data = msg.data
            spot, level, count = None, None, 0
            match_spot = re.search(r'spot=(\d+)', data)
            if match_spot:
                spot = int(match_spot.group(1))
            match_count = re.search(r'count=(\d+)', data)
            if match_count:
                count = int(match_count.group(1))
            match_level = re.search(r'level=([A-Z0-9]+)', data)
            if match_level:
                level = match_level.group(1)

            if not hasattr(self, '_crowd_initialized'):
                self._crowd_initialized = True
                for wp in self.selected_waypoints:
                    self.crowd_data[wp['id']] = {"level": "L1", "count": 0, "C": 1}

            if spot in self.crowd_data:
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
                self.crowd_data[spot] = {"level": level, "count": count, "C": C}
                self.get_logger().info(f"📊 更新 spot{spot}：人数={count}, C={C}")
        except Exception as e:
            pass

    def voice_command_callback(self, msg):
        try:
            data = msg.data
            self.get_logger().info(f"📥 收到语音指令: {data}")

            # 防抖检查
            current_time = time.time()
            if current_time - self.last_voice_command_time < self.voice_command_debounce_seconds:
                self.get_logger().info(f"⏳ 防抖中，忽略此指令 (距离上次: {current_time - self.last_voice_command_time:.1f}s)")
                return

            # 尝试多种匹配方式
            command = None

            # 方式1: 匹配 command=xxx 格式
            match_command = re.search(r'command=([^\s\'"]+)', data)
            if match_command:
                command = match_command.group(1)
            else:
                # 方式2: 尝试匹配单引号或双引号中的内容
                match_command = re.search(r'command=[\'"]([^\'"]+)[\'"]', data)
                if match_command:
                    command = match_command.group(1)
                else:
                    # 方式3: 直接找常见的指令关键词
                    common_commands = [
                        "全景点遍历模式", "全路径遍历模式", "重新规划路线",
                        "暂停运动", "继续运动", "前往景点1", "前往景点2", "前往景点3"
                    ]
                    for cmd in common_commands:
                        if cmd in data:
                            command = cmd
                            break

            if not command:
                self.get_logger().warning("⚠️  无法解析指令")
                return

            # 更新最后执行时间
            self.last_voice_command_time = current_time

            if command == "全景点遍历模式":
                self.get_logger().info("🔄 切换到全景点遍历模式(DP)")
                self.current_mode = self.MODE_DP
                self.mode_switch_flag = True
                self._trigger_restart()
            elif command == "全路径遍历模式":
                self.get_logger().info("🔄 切换到全路径遍历模式(GA)")
                self.current_mode = self.MODE_GA
                self.mode_switch_flag = True
                self._trigger_restart()
            elif command == "重新规划路线":
                if not self.selection_confirmed:
                    self.get_logger().warning("⚠️  路点尚未确认")
                    return
                self.get_logger().warning("🚨 语音指令：准备重规划！")
                self._trigger_restart()
            elif command == "暂停运动":
                self.get_logger().info("⏸️  语音指令：暂停运动")
                self._pause_navigation()
            elif command == "继续运动":
                self.get_logger().info("▶️  语音指令：继续运动")
                self._resume_navigation()
            elif command == "前往景点1":
                self.get_logger().info("🎯 语音指令：立即前往景点1")
                self._go_to_spot_immediately(1)
            elif command == "前往景点2":
                self.get_logger().info("🎯 语音指令：立即前往景点2")
                self._go_to_spot_immediately(2)
            elif command == "前往景点3":
                self.get_logger().info("🎯 语音指令：立即前往景点3")
                self._go_to_spot_immediately(3)
        except Exception as e:
            self.get_logger().error(f"❌ 语音指令处理失败: {e}")

    def _trigger_restart(self):
        """在独立线程中触发重规划"""
        import threading
        threading.Thread(target=self._do_restart, daemon=True).start()

    def _do_restart(self):
        self.restart_flag = True
        self.cancelTask()

    def _pause_navigation(self):
        """暂停导航 - 使用 Nav2 原生服务"""
        if self.is_paused:
            self.get_logger().info("⏸️  已经处于暂停状态")
            return
        if not self.nav_waypoints:
            self.get_logger().warning("⚠️  当前没有进行中的导航任务")
            return

        # 尝试使用 Nav2 原生暂停服务
        if self.pause_nav_client.service_is_ready():
            try:
                self.get_logger().info("🔄 调用 Nav2 暂停服务...")
                request = Empty.Request()
                future = self.pause_nav_client.call_async(request)
                # 等待服务调用完成，但不阻塞太久
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                if future.result() is not None:
                    self.is_paused = True
                    self.get_logger().info("✅ 导航已通过 Nav2 服务暂停")
                    return
            except Exception as e:
                self.get_logger().warning(f"⚠️  Nav2 暂停服务调用失败: {e}")

        # 降级方案：使用手动暂停
        self.get_logger().info("⏸️  使用手动暂停方式...")

        # 保存当前进度
        try:
            feedback = self.getFeedback()
            if feedback:
                self.current_waypoint_index = feedback.current_waypoint
            else:
                self.current_waypoint_index = 0
        except:
            self.current_waypoint_index = 0

        self.is_paused = True
        self.get_logger().info(f"⏸️  导航已暂停，当前进度: 路点 {self.current_waypoint_index + 1}/{len(self.nav_waypoints)}")

        # 取消当前导航任务
        try:
            self.cancelTask()
        except:
            pass

    def _resume_navigation(self):
        """恢复导航 - 使用 Nav2 原生服务"""
        if not self.is_paused:
            self.get_logger().info("▶️  当前未暂停，无需恢复")
            return
        if not self.nav_waypoints:
            self.get_logger().warning("⚠️  没有保存的导航任务")
            return

        # 尝试使用 Nav2 原生恢复服务
        if self.resume_nav_client.service_is_ready():
            try:
                self.get_logger().info("🔄 调用 Nav2 恢复服务...")
                request = Empty.Request()
                future = self.resume_nav_client.call_async(request)
                # 等待服务调用完成，但不阻塞太久
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                if future.result() is not None:
                    self.is_paused = False
                    self.get_logger().info("✅ 导航已通过 Nav2 服务恢复")
                    return
            except Exception as e:
                self.get_logger().warning(f"⚠️  Nav2 恢复服务调用失败: {e}")

        # 降级方案：使用手动恢复
        self.get_logger().info("▶️  使用手动恢复方式...")

        self.is_paused = False
        self.get_logger().info(f"▶️  导航已恢复，继续从路点 {self.current_waypoint_index + 1} 前进")

        # 从暂停的路点重新开始导航
        remaining_waypoints = self.nav_waypoints[self.current_waypoint_index:]
        if remaining_waypoints:
            self.get_logger().info(f"🔄 继续剩余 {len(remaining_waypoints)} 个路点")
            self.followWaypoints(remaining_waypoints)

    def _go_to_spot_immediately(self, spot_id):
        """立即前往指定景点，到达后继续剩余路点（保留所有路点）"""
        if not self.nav_waypoint_ids:
            self.get_logger().warning("⚠️  当前没有进行中的导航")
            return

        if spot_id not in self.nav_waypoint_ids:
            self.get_logger().warning(f"⚠️  景点{spot_id}不在当前导航列表中")
            return

        # 找到目标景点在当前导航队列中的位置
        spot_index = self.nav_waypoint_ids.index(spot_id)
        self.get_logger().info(f"📍 景点{spot_id}在导航队列中的位置: {spot_index + 1}/{len(self.nav_waypoint_ids)}")

        # 检查当前是否正在前往该景点
        try:
            feedback = self.getFeedback()
            if feedback:
                current_idx = feedback.current_waypoint
                if current_idx == spot_index:
                    self.get_logger().info(f"✅ 已经在前往景点{spot_id}的途中，无需重新规划")
                    return
        except:
            pass

        # 保存当前机器人位置作为新起点
        try:
            self.start_pose = self.get_current_robot_pose()
        except:
            pass

        self.get_logger().info(f"🗺️  固定景点{spot_id}为第一个路点，重新计算所有路点的最优顺序")

        # 保留所有路点，不做修改 - 让 find_optimal_order 函数来处理顺序
        self.cost_matrix = None  # 清除代价矩阵缓存，强制重新计算

        # 设置标志，触发重启
        self.priority_spot_id = spot_id
        self.restart_flag = True

        # 在后台线程中取消当前任务
        import threading
        threading.Thread(target=self._cancel_task_only, daemon=True).start()

    def _cancel_task_only(self):
        """只取消任务，不做其他操作"""
        try:
            self.cancelTask()
        except:
            pass

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
        self.get_logger().error("❌ 无法获取位姿")
        return self._create_pose_from_data(0.0, 0.0, 0.0)

    def safe_get_path_len(self, start, goal, name):
        try:
            path = self.getPath(start, goal, use_start=True)
            return len(path.poses) if path else 0
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
                self.get_logger().warning("\n🚨 检测到回车：准备重规划！")
                self._trigger_restart()

    def find_optimal_order_dp(self, start_pose, waypoints, fixed_first_spot_id=None):
        """DP模式求解 - 考虑人流密度

        Args:
            start_pose: 起点pose
            waypoints: 路点列表
            fixed_first_spot_id: 如果指定，固定该路点为第一个
        """
        n = len(waypoints)
        wp_ids = [wp['id'] for wp in waypoints]

        for wp_id in wp_ids:
            if wp_id not in self.crowd_data:
                self.crowd_data[wp_id] = {"level": "L1", "count": 0, "C": 1}

        self.get_logger().info(f"\n🔍 计算起点到各路点的代价...")
        start_costs = []
        for i, wp in enumerate(waypoints):
            cost = self.safe_get_path_len(start_pose, wp['pose'], f"起点→{i+1}")
            start_costs.append(cost)
            self.get_logger().info(f"  起点→{i+1}: {cost} 点")

        path_msg = Int32MultiArray()
        path_msg.data = start_costs
        self.path_pub.publish(path_msg)

        if self.cost_matrix is not None and len(self.cost_matrix) == n:
            self.get_logger().info(f"✅ 使用缓存的代价矩阵")
            cost_matrix = self.cost_matrix
        else:
            self.get_logger().info(f"\n🔍 计算路径代价矩阵...")
            cost_matrix = [[0] * n for _ in range(n)]
            for i in range(n):
                for j in range(n):
                    if i != j:
                        cost = self.safe_get_path_len(waypoints[i]['pose'], waypoints[j]['pose'], f"{i+1}→{j+1}")
                        cost_matrix[i][j] = cost
                        self.get_logger().info(f"  {i+1}→{j+1}: {cost} 点")

        cost_between = {}
        for i in range(n):
            cost_between[wp_ids[i]] = {}
            for j in range(n):
                if i != j:
                    cost_between[wp_ids[i]][wp_ids[j]] = cost_matrix[i][j]

        weights = []
        for i in range(n):
            if i == 0:
                weights.append(1.0)
            elif i == 1:
                weights.append(0.6)
            elif i == 2:
                weights.append(0.2)
            else:
                weights.append(0.1 / (i - 2))

        self.get_logger().info(f"\n🧮 使用排列遍历 + 拥挤度加权...")
        if fixed_first_spot_id:
            self.get_logger().info(f"   固定第一个路点为: {fixed_first_spot_id}")
        self.get_logger().info(f"   顺序权重: {[round(w, 2) for w in weights]}")
        min_score = float('inf')
        best_order = None
        best_detail = {}

        # 生成所有可能的排列
        for order in itertools.permutations(wp_ids):
            # 如果指定了固定第一个路点，只考虑以该路点开头的排列
            if fixed_first_spot_id and order[0] != fixed_first_spot_id:
                continue

            first_wp_idx = wp_ids.index(order[0])
            path_cost = start_costs[first_wp_idx]
            for i in range(n - 1):
                a, b = order[i], order[i + 1]
                path_cost += cost_between[a][b]

            crowd_sum = 0
            for i in range(n):
                wp_id = order[i]
                c = self.crowd_data[wp_id]["C"]
                crowd_sum += c * weights[i]

            total_score = path_cost + crowd_sum * 30

            if total_score < min_score:
                min_score = total_score
                best_order = order
                best_detail = {"path": path_cost, "crowd_sum": round(crowd_sum, 2), "score": round(total_score, 2)}

        order_msg = Int32MultiArray()
        order_msg.data = list(best_order)
        self.best_order_pub.publish(order_msg)

        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"🎉 最优顺序：{best_order}")
        self.get_logger().info(f"📊 路径点数：{best_detail['path']} | 拥挤加权和：{best_detail['crowd_sum']}")
        self.get_logger().info(f"🏆 最终总分（越小越好）：{best_detail['score']}")
        self.get_logger().info("="*70)

        best_order_idx = [wp_ids.index(wp_id) for wp_id in best_order]
        return best_order_idx, start_costs, cost_matrix

    def find_optimal_order_ga(self, start_pose, waypoints, cost_matrix, fixed_first_spot_id=None):
        """GA模式求解 - 不考虑人流密度

        Args:
            start_pose: 起点pose
            waypoints: 路点列表
            cost_matrix: 代价矩阵
            fixed_first_spot_id: 如果指定，固定该路点为第一个
        """
        n = len(waypoints)
        wp_ids = [wp['id'] for wp in waypoints]
        self.get_logger().info(f"\n🔍 计算起点到各路点的代价...")

        start_costs = []
        for i, wp in enumerate(waypoints):
            cost = self.safe_get_path_len(start_pose, wp['pose'], f"起点→{i+1}")
            start_costs.append(cost)
            self.get_logger().info(f"  起点→{i+1}: {cost} 点")

        if n <= 8:
            pop_size, max_iter = 100, 500
        elif n <= 12:
            pop_size, max_iter = 150, 800
        else:
            pop_size, max_iter = 200, 1000

        self.get_logger().info(f"\n🧬 使用遗传算法求解TSP...")
        if fixed_first_spot_id:
            self.get_logger().info(f"   固定第一个路点为: {fixed_first_spot_id}")
        min_total = float('inf')
        best_order = None

        # 确定起始点索引
        start_indices = []
        if fixed_first_spot_id:
            # 如果指定了固定第一个路点，只从该路点开始
            if fixed_first_spot_id in wp_ids:
                start_indices = [wp_ids.index(fixed_first_spot_id)]
            else:
                self.get_logger().warning(f"⚠️  指定的路点 {fixed_first_spot_id} 不在列表中，使用所有路点")
                start_indices = range(n)
        else:
            # 否则从所有路点开始尝试
            start_indices = range(n)

        for start_idx in start_indices:
            remaining_indices = [i for i in range(n) if i != start_idx]

            if len(remaining_indices) == 0:
                path_cost, path_idx = 0, [start_idx]
            elif len(remaining_indices) == 1:
                path_cost = cost_matrix[start_idx][remaining_indices[0]]
                path_idx = [start_idx, remaining_indices[0]]
            else:
                m = len(remaining_indices)
                sub_cost_matrix = [[0] * m for _ in range(m)]
                for i_sub, i_orig in enumerate(remaining_indices):
                    for j_sub, j_orig in enumerate(remaining_indices):
                        sub_cost_matrix[i_sub][j_sub] = cost_matrix[i_orig][j_orig]

                ga_solver = TSPGeneticAlgorithm(sub_cost_matrix, population_size=pop_size, max_iterations=max_iter)
                sub_path_cost, sub_path_idx = ga_solver.solve(verbose=False)

                path_idx = [start_idx] + [remaining_indices[i] for i in sub_path_idx]
                path_cost = start_costs[start_idx]
                for i in range(len(path_idx) - 1):
                    path_cost += cost_matrix[path_idx[i]][path_idx[i+1]]

            path_wp = [waypoints[i]['id'] for i in path_idx]
            self.get_logger().info(f"  从{waypoints[start_idx]['id']}出发: 顺序={path_wp}, 总代价={path_cost}")

            if path_cost < min_total:
                min_total = path_cost
                best_order = path_idx

        return best_order, start_costs

    def run_once(self):
        self.restart_flag = False

        # ✅ 启动自动加载缓存（修复点）
        if not self.selection_confirmed:
            self.get_logger().info(f"\n📦 尝试自动加载 {self.current_mode.upper()} 缓存...")
            if self._load_cache_for_mode(self.current_mode):
                self.selection_confirmed = True
            else:
                self.get_logger().warning(f"📦 未找到缓存，进入手动选点模式")

        # 模式切换处理
        if self.mode_switch_flag:
            self.mode_switch_flag = False
            self.selection_confirmed = False
            self.selected_waypoints = []
            if self._load_cache_for_mode(self.current_mode):
                self.selection_confirmed = True
            else:
                self.get_logger().warning("⚠️  缓存加载失败，请先选点")

        if not self.selection_confirmed:
            self.get_logger().info("\n" + "="*60)
            self.get_logger().info(f"📍 当前模式: {self.current_mode.upper()}")
            self.get_logger().info("="*60)
            self.get_logger().info("\n提示：用RViz选点，选完按回车确认\n")

            while not self.selection_confirmed:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.01)
        else:
            self.get_logger().info("\n" + "="*60)
            self.get_logger().info(f"📍 当前模式: {self.current_mode.upper()}")
            self.get_logger().info("="*60)

        if len(self.selected_waypoints) < 2:
            self.get_logger().error("❌ 路点太少")
            return

        # 如果没有设置起点，或者是优先景点模式，获取当前机器人位置
        if self.start_pose is None or self.priority_spot_id is not None:
            try:
                self.start_pose = self.get_current_robot_pose()
            except:
                pass

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 路点列表：")
        for wp in self.selected_waypoints:
            self.get_logger().info(f"   {wp['id']}: (x={wp['x']:.2f}, y={wp['y']:.2f})")
        self.get_logger().info("="*60)

        if self.current_mode == self.MODE_DP:
            best_order_idx, start_costs, cost_matrix = self.find_optimal_order_dp(
                self.start_pose, self.selected_waypoints, self.priority_spot_id)
        else:
            if self.cost_matrix is None or len(self.cost_matrix) != len(self.selected_waypoints):
                self.get_logger().info(f"\n🔍 计算路径代价矩阵...")
                n = len(self.selected_waypoints)
                cost_matrix = [[0] * n for _ in range(n)]
                for i in range(n):
                    for j in range(n):
                        if i != j:
                            cost = self.safe_get_path_len(
                                self.selected_waypoints[i]['pose'],
                                self.selected_waypoints[j]['pose'],
                                f"{i+1}→{j+1}")
                            cost_matrix[i][j] = cost
            else:
                cost_matrix = self.cost_matrix
            best_order_idx, start_costs = self.find_optimal_order_ga(
                self.start_pose, self.selected_waypoints, cost_matrix, self.priority_spot_id)

        self.nav_waypoints = [self.selected_waypoints[i]['pose'] for i in best_order_idx]
        self.nav_waypoint_ids = [self.selected_waypoints[i]['id'] for i in best_order_idx]
        self.current_waypoint_index = 0
        best_order_ids = self.nav_waypoint_ids

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"🏆 最优访问顺序: {best_order_ids}")
        self.get_logger().info("="*60)

        self.get_logger().info("\n🚗 开始导航...")
        self.is_paused = False
        self.priority_spot_id = None  # 清除优先景点标志

        self.followWaypoints(self.nav_waypoints)

        # 主导航循环 - 简化版
        while not self.isTaskComplete() or self.is_paused:
            if self.restart_flag:
                self.get_logger().warn("🟠 已被打断，准备重新开始...")
                self.is_paused = False
                return
            if self.is_paused:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.2)
                continue
            try:
                feedback = self.getFeedback()
                if feedback:
                    self.current_waypoint_index = feedback.current_waypoint
                    self.get_logger().info(f"🚩 当前目标: {feedback.current_waypoint + 1}/{len(self.nav_waypoints)}")
            except:
                pass
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        if not self.is_paused:
            self.get_logger().info("\n🔄 所有路点访问完成，返回起点...")
            self.goToPose(self.start_pose)

            while not self.isTaskComplete() or self.is_paused:
                if self.restart_flag:
                    self.get_logger().warn("🟠 已被打断，准备重新开始...")
                    self.is_paused = False
                    return
                if self.is_paused:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    time.sleep(0.2)
                    continue
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

            self.get_logger().info("\n" + "="*60)
            self.get_logger().info("✅ 导航完成！已回到起点")
            self.get_logger().info("="*60)
            self.nav_waypoints = None
            self.nav_waypoint_ids = None

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
    rclpy.init()
    navigator = UnifiedNavigator()

    try:
        navigator.run_loop()
    except KeyboardInterrupt:
        print("\n🛑 程序退出")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()