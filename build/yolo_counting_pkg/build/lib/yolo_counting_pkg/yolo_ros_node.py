import os
# 禁用CPU高级指令集（关键：解决非法指令）
os.environ['ONNX_RUNTIME_DISABLE_CPU_OPTIMIZATION'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'  # 限制线程数，适配树莓派性能
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import os
from ament_index_python.packages import get_package_share_directory
from .yolo_infer import process_video, get_all_video_files

class YOLOCountingNode(Node):
    """ROS 2 节点：封装YOLO检测，发布结果到ROS话题"""
    def __init__(self):
        super().__init__('yolo_counting_node')
        
        # 创建ROS 2发布者
        self.publisher_avg = self.create_publisher(Float32, '/yolo/average_persons', 10)
        self.publisher_level = self.create_publisher(String, '/yolo/person_level', 10)
        
        # 获取功能包路径（避免硬编码）
        try:
            pkg_path = get_package_share_directory('yolo_counting_pkg')
        except:
            pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        # 配置路径
        self.model_path = os.path.join(pkg_path, 'models', 'best.onnx')
        self.source_dir = os.path.join(pkg_path, 'data', 'vedio')
        self.result_dir = os.path.join(pkg_path, 'results')
        self.weight_calc_path = os.path.join(pkg_path, 'weight_calc_input.json')
        
        # 创建结果目录
        os.makedirs(self.result_dir, exist_ok=True)
        
        # 启动检测流程
        self.get_logger().info("YOLO检测节点已启动，开始处理视频...")
        self.run_detection()

    def run_detection(self):
        """批量处理视频"""
        # 获取所有待检测视频
        video_files = get_all_video_files(self.source_dir)
        if not video_files:
            self.get_logger().error("❌ 未找到支持的视频文件！")
            return
        
        self.get_logger().info(f"✅ 共找到 {len(video_files)} 个待检测视频")
        
        # 逐个处理视频
        for idx, video_path in enumerate(video_files, 1):
            self.get_logger().info(f"\n[{idx}/{len(video_files)}] 开始处理：{os.path.basename(video_path)}")
            
            # 执行检测
            result = process_video(
                self.model_path,
                video_path,
                self.result_dir,
                self.weight_calc_path
            )
            
            if result is None:
                self.get_logger().error(f"处理失败：{os.path.basename(video_path)}")
                continue
            
            # 发布结果到ROS 2话题
            avg_msg = Float32()
            avg_msg.data = result['average_persons']
            self.publisher_avg.publish(avg_msg)
            
            level_msg = String()
            level_msg.data = f"{result['file_name']}:{result['level']}"
            self.publisher_level.publish(level_msg)
            
            # 打印结果
            self.get_logger().info(
                f"✅ 处理完成 | 平均人数：{result['average_persons']} | 等级：{result['level']}"
            )
        
        self.get_logger().info(f"\n🎉 所有视频处理完成！结果保存至：{self.result_dir}")

def main(args=None):
    """节点主函数"""
    rclpy.init(args=args)
    node = YOLOCountingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()