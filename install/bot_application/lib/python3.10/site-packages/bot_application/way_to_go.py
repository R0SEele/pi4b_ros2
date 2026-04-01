#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import itertools

class PathPointsSubscriber(Node):
    def __init__(self):
        super().__init__('path_points_subscriber')
        
        # 订阅路径点数
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/path_segment_points',
            self.callback,
            10
        )
        
        # ✅ 发布最优路点顺序
        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/best_waypoint_order',
            10
        )
        
        self.get_logger().info("✅ 已订阅 /path_segment_points，已准备发布 /best_waypoint_order")

        # 路点之间固定代价（你提供的数据）
        self.cost_between = {
            1: {2: 92,  3: 175},
            2: {1: 86,  3: 107},
            3: {1: 197, 2: 119}
        }

    def callback(self, msg):
        p_start_1 = msg.data[0]
        p_start_2 = msg.data[1]
        p_start_3 = msg.data[2]

        self.get_logger().info(f"\n📥 收到起点路径：1={p_start_1}, 2={p_start_2}, 3={p_start_3}")

        # 枚举所有6种顺序
        waypoints = [1,2,3]
        all_orders = list(itertools.permutations(waypoints))
        min_total = float('inf')
        best_order = None

        for order in all_orders:
            wp1, wp2, wp3 = order
            start_cost = [p_start_1, p_start_2, p_start_3][wp1 - 1]
            cost1 = self.cost_between[wp1][wp2]
            cost2 = self.cost_between[wp2][wp3]
            total = start_cost + cost1 + cost2

            if total < min_total:
                min_total = total
                best_order = order

        # 输出最优结果
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"🎉 最优顺序：起点 → {best_order[0]} → {best_order[1]} → {best_order[2]}")
        self.get_logger().info(f"🎉 最少总路径点：{min_total}")
        self.get_logger().info("="*60)

        # ✅ 发布最优顺序
        order_msg = Int32MultiArray()
        order_msg.data = list(best_order)
        self.publisher.publish(order_msg)
        self.get_logger().info(f"✅ 已发布最优顺序：/best_waypoint_order → {order_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = PathPointsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()