import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import os

# ===================== 路径配置（pkg内相对路径，更规范） =====================
# 通过ROS 2包路径获取pkg根目录，避免硬编码绝对路径
def get_pkg_path():
    """获取功能包根目录（适配ROS 2包规范）"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('yolo_counting_pkg')
    except:
        # 降级为绝对路径（防止ROS环境未加载）
        return "/home/rose/car_ws/src/yolo_counting_pkg"

PKG_PATH = get_pkg_path()
WEIGHT_CALC_FILE = os.path.join(PKG_PATH, "weight_calc_input.json")

# ===================== ROS 2 节点类 =====================
class YOLOResultPublisher(Node):
    """ROS 2节点：发布YOLO检测结果"""
    def __init__(self):
        super().__init__('yolo_result_publisher')
        
        # 1. 创建发布者
        self.avg_pub = self.create_publisher(Float32, '/yolo/average_persons', 10)
        self.level_pub = self.create_publisher(String, '/yolo/person_level', 10)
        self.get_logger().info("✅ YOLO结果发布节点已启动")
        
        # 2. 发布结果
        self.publish_results()

    def publish_results(self):
        """读取检测结果并发布"""
        # 检查文件是否存在
        if not os.path.exists(WEIGHT_CALC_FILE):
            self.get_logger().error(f"❌ 未找到检测结果文件：{WEIGHT_CALC_FILE}")
            self.get_logger().info("请先运行：python3 ~/car_ws/scripts/yolo_detect_standalone.py")
            return

        # 逐行解析并发布
        with open(WEIGHT_CALC_FILE, "r", encoding="utf-8") as f:
            for line in f.readlines():
                if not line.strip():
                    continue
                
                try:
                    data = json.loads(line)
                    video_name = data["video_name"]
                    avg_persons = data["average_persons"]
                    level = data["level"]
                except Exception as e:
                    self.get_logger().warn(f"⚠️ 解析失败：{line} | 错误：{e}")
                    continue

                # 发布平均人数
                avg_msg = Float32()
                avg_msg.data = avg_persons
                self.avg_pub.publish(avg_msg)

                # 发布等级
                level_msg = String()
                level_msg.data = f"{video_name}:{level}"
                self.level_pub.publish(level_msg)

                # 日志打印
                self.get_logger().info(f"发布 | 视频：{video_name} | 平均人数：{avg_persons} | 等级：{level}")
        
        self.get_logger().info("🎉 所有结果发布完成！")

# ===================== 主函数 =====================
def main(args=None):
    rclpy.init(args=args)
    node = YOLOResultPublisher()
    rclpy.spin_once(node, timeout_sec=3.0)  # 运行3秒后退出
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()