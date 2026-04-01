#!/usr/bin/env python3
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
import itertools
import threading
import re

# ====================== 欧拉角转四元数 ======================
def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [qx, qy, qz, qw]

# ====================== 三合一最优导航节点 ======================
class OptimizedWaypointNavigator(BasicNavigator):
    def __init__(self):
        super().__init__()
        self.get_logger().info("✅ 三合一最优导航节点已启动（支持人流拥挤度）")

        # TF监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 发布器
        self.path_pub = self.create_publisher(Int32MultiArray, '/path_segment_points', 10)
        self.best_order_pub = self.create_publisher(Int32MultiArray, '/best_waypoint_order', 10)

        # 路点
        self.waypoint_map = {
            1: self.create_goal_pose(0.993753, 0.62935, 0.0),
            2: self.create_goal_pose(-1.26875, 1.02813, math.pi/2),
            3: self.create_goal_pose(-1.84094, 3.89227, -math.pi),
        }

        # 路点间代价（路径点数）
        self.cost_between = {
            1: {2: 92, 3: 175},
            2: {1: 86, 3: 107},
            3: {1: 197, 2: 119}
        }

        # ====================== 新增：拥挤度数据 ======================
        self.crowd_data = {
            1: {"level": "L1", "count": 0, "C": 1},
            2: {"level": "L1", "count": 0, "C": 1},
            3: {"level": "L1", "count": 0, "C": 1},
        }

        # 订阅 YOLO 检测结果
        self.yolo_sub = self.create_subscription(
            String,
            '/yolov8_detections',
            self.yolo_detection_callback,
            10
        )

        # 打断标志
        self.restart_flag = False

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

            if spot in [1,2,3]:
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

    # ====================== 监听键盘回车（后台线程） ======================
    def listen_keyboard(self):
        while True:
            input()
            self.get_logger().warning("\n🚨 检测到回车输入：准备打断导航并重新规划！")
            self.restart_flag = True
            self.cancelTask()

    # ====================== 单次完整规划 + 导航 ======================
    def run_once(self):
        self.restart_flag = False
        self.get_logger().info("\n========== 开始新一轮【路径+拥挤度】最优规划 ==========")

        start_pose = self.get_current_robot_pose()
        wp1 = self.waypoint_map[1]
        wp2 = self.waypoint_map[2]
        wp3 = self.waypoint_map[3]

        # 计算路径长度
        p1 = self.safe_get_path_len(start_pose, wp1, "起点→1")
        p2 = self.safe_get_path_len(start_pose, wp2, "起点→2")
        p3 = self.safe_get_path_len(start_pose, wp3, "起点→3")

        # 发布
        path_msg = Int32MultiArray()
        path_msg.data = [p1, p2, p3]
        self.path_pub.publish(path_msg)

        # ====================== 核心：新的最优计算（路径+拥挤度） ======================
        min_score = float('inf')
        best_order = None
        best_detail = {}

        # 顺序权重
        weights = [1.0, 0.6, 0.2]

        for order in itertools.permutations([1,2,3]):
            a,b,c = order
            # 1. 路径总分
            s_cost = [p1,p2,p3][a-1]
            path_cost = s_cost + self.cost_between[a][b] + self.cost_between[b][c]

            # 2. 拥挤度加权和
            c1 = self.crowd_data[a]["C"]
            c2 = self.crowd_data[b]["C"]
            c3 = self.crowd_data[c]["C"]
            crowd_sum = c1 * weights[0] + c2 * weights[1] + c3 * weights[2]

            # 3. 最终总分
            total_score = path_cost + crowd_sum * 30

            # 选择最小分
            if total_score < min_score:
                min_score = total_score
                best_order = order
                best_detail = {
                    "path": path_cost,
                    "crowd_sum": round(crowd_sum,2),
                    "score": round(total_score,2)
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

        # 重新导航
        self.get_logger().info("\n开始导航……（中途按回车可打断并重新规划）")
        self.cancelTask()
        time.sleep(0.5)

        waypoints = [self.waypoint_map[i] for i in best_order]
        self.followWaypoints(waypoints)

        # 导航循环
        while not self.isTaskComplete():
            if self.restart_flag:
                self.get_logger().warn("🟠 已被打断，准备重新规划……")
                return
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f"🚩 当前目标：{feedback.current_waypoint}")
            time.sleep(0.2)

    # ====================== 主循环：可无限重启 ======================
    def run_loop(self):
        key_thread = threading.Thread(target=self.listen_keyboard, daemon=True)
        key_thread.start()
        self.get_logger().info("⌨️  键盘监听已启动：【任何时候按回车 = 打断 + 重新规划】")

        input("\n🚦 按回车开始第一次规划导航……")

        while True:
            self.run_once()
            if self.restart_flag:
                self.get_logger().info("\n🔄 即将重新执行全流程……\n")
                time.sleep(1)
            else:
                self.get_logger().info("\n🏁 导航完成！按回车再次执行……")
                input()

# ====================== 主函数 ======================
def main():
    rclpy.init()
    navigator = OptimizedWaypointNavigator()

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