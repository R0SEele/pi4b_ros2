#!/usr/bin/env python3
"""
完整路径遍历导航器 - 遗传算法 TSP 版本
功能：
1. 读取缓存文件（路点 + 代价矩阵）
2. 遗传算法求解TSP近似最优路径
3. 闭环导航（最后回到起点）
"""
from geometry_msgs.msg import PoseStamped
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
import random


# ====================== 遗传算法 TSP 求解器 ======================
class TSPGeneticAlgorithm:
    """
    遗传算法求解TSP
    """
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
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [qx, qy, qz, qw]


# ====================== 完整路径导航节点 ======================
class FullPathNavigatorGA(BasicNavigator):
    def __init__(self):
        super().__init__('full_path_navigator_ga')
        self.get_logger().info("="*60)
        self.get_logger().info("🧬 完整路径导航器 (遗传算法版本) 已启动")
        self.get_logger().info("="*60)

        # TF监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 缓存文件路径
        self.waypoint_cache_file = 'waypoint_cache.json'
        self.cost_cache_file = 'cost_cache.json'

        # 路点数据
        self.waypoints = []
        self.cost_matrix = None
        self.restart_flag = False

        # 起点（机器人当前位置）
        self.start_pose = None

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

    def load_cache(self):
        """从缓存文件加载路点和代价矩阵"""
        self.get_logger().info("📦 加载缓存文件...")

        if not os.path.exists(self.waypoint_cache_file):
            self.get_logger().error(f"❌ 路点缓存文件不存在: {self.waypoint_cache_file}")
            self.get_logger().error("   请先运行 waypoint_selector.py 选点")
            return False

        if not os.path.exists(self.cost_cache_file):
            self.get_logger().error(f"❌ 代价矩阵缓存文件不存在: {self.cost_cache_file}")
            self.get_logger().error("   请先运行 waypoint_selector.py 预计算")
            return False

        try:
            with open(self.waypoint_cache_file, 'r') as f:
                wp_data = json.load(f)
            wp_list = wp_data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"❌ 读取路点缓存失败: {e}")
            return False

        try:
            with open(self.cost_cache_file, 'r') as f:
                cost_data = json.load(f)
            self.cost_matrix = cost_data.get('cost_matrix')
        except Exception as e:
            self.get_logger().error(f"❌ 读取代价矩阵缓存失败: {e}")
            return False

        if not wp_list or not self.cost_matrix:
            self.get_logger().error("❌ 缓存数据无效")
            return False

        if len(wp_list) != len(self.cost_matrix):
            self.get_logger().error(f"❌ 路点数量({len(wp_list)})与代价矩阵({len(self.cost_matrix)})不匹配")
            return False

        self.waypoints = []
        for i, wp in enumerate(wp_list):
            self.waypoints.append({
                'id': i + 1,
                'x': wp['x'],
                'y': wp['y'],
                'yaw': wp['yaw'],
                'pose': self._create_pose_from_data(wp['x'], wp['y'], wp['yaw'])
            })

        self.get_logger().info(f"✅ 加载成功: {len(self.waypoints)} 个路点")
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
            self.get_logger().warning("\n🚨 检测到回车：准备打断导航！")
            self.restart_flag = True
            self.cancelTask()

    def find_optimal_order(self, start_pose, waypoints, cost_matrix):
        """使用遗传算法寻找最优路点顺序"""
        n = len(waypoints)
        self.get_logger().info(f"\n🔍 计算起点到各路点的代价...")

        # 1. 计算起点到所有路点的代价
        start_costs = []
        for i, wp in enumerate(waypoints):
            cost = self.safe_get_path_len(start_pose, wp['pose'], f"起点→{i+1}")
            start_costs.append(cost)
            self.get_logger().info(f"  起点→{i+1}: {cost} 点")

        # 2. 根据路点数量调整GA参数
        if n <= 8:
            pop_size = 100
            max_iter = 500
        elif n <= 12:
            pop_size = 150
            max_iter = 800
        else:
            pop_size = 200
            max_iter = 1000

        # 3. 对每个可能的起点，用GA求解
        self.get_logger().info(f"\n🧬 使用遗传算法求解TSP...")
        min_total = float('inf')
        best_order = None

        for start_idx in range(n):
            remaining_indices = [i for i in range(n) if i != start_idx]

            if len(remaining_indices) == 0:
                path_cost = 0
                path_idx = [start_idx]
            elif len(remaining_indices) == 1:
                path_cost = cost_matrix[start_idx][remaining_indices[0]]
                path_idx = [start_idx, remaining_indices[0]]
            else:
                m = len(remaining_indices)
                sub_cost_matrix = [[0] * m for _ in range(m)]
                for i_sub, i_orig in enumerate(remaining_indices):
                    for j_sub, j_orig in enumerate(remaining_indices):
                        sub_cost_matrix[i_sub][j_sub] = cost_matrix[i_orig][j_orig]

                ga_solver = TSPGeneticAlgorithm(
                    sub_cost_matrix,
                    population_size=pop_size,
                    max_iterations=max_iter
                )
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

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🚀 开始导航")
        self.get_logger().info("="*60)

        if not self.load_cache():
            return

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 路点列表：")
        for wp in self.waypoints:
            self.get_logger().info(f"   {wp['id']}: (x={wp['x']:.2f}, y={wp['y']:.2f})")
        self.get_logger().info("="*60)

        self.start_pose = self.get_current_robot_pose()

        best_order_idx, start_costs = self.find_optimal_order(
            self.start_pose,
            self.waypoints,
            self.cost_matrix
        )

        nav_waypoints = [self.waypoints[i]['pose'] for i in best_order_idx]
        best_order_ids = [self.waypoints[i]['id'] for i in best_order_idx]
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"🏆 最优访问顺序: {best_order_ids}")
        self.get_logger().info("="*60)

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
    rclpy.init()
    navigator = FullPathNavigatorGA()

    # print("等待导航系统激活...")
    # try:
    #     navigator.lifecycleStartup()
    #     print("✅ 导航已激活")
    # except Exception as e:
    #     print(f"❌ 导航启动失败: {e}")
    #     return

    try:
        navigator.run_loop()
    except KeyboardInterrupt:
        print("\n🛑 程序退出")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
