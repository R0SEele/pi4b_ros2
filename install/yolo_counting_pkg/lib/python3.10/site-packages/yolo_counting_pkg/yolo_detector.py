import rclpy
from rclpy.node import Node
import cv2
import onnxruntime as ort
import numpy as np
from std_msgs.msg import String
import os

class YoloV8OnnxNode(Node):
    def __init__(self):
        super().__init__('yolov8_onnx_node')

        # 创建图像发布者
        self.publisher_ = self.create_publisher(String, 'yolov8_detections', 10)

        # 加载ONNX模型
        self.model_path = '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/models/best.onnx'  # 替换为你的ONNX模型路径
        self.ort_session = ort.InferenceSession(self.model_path)

        # 只处理单个视频
        self.video_file = '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/data/vedio/spot1.mp4'  # 视频文件路径

        # 初始化状态
        self.frame_count = 0
        self.max_people_count = 0

        # 设置定时器
        self.timer = self.create_timer(0.1, self.detect_frame)  # 每100毫秒检测一次

        # 初始化 self.cap
        self.cap = None

        # 设置保存图片的文件夹路径
        self.output_folder = '/home/rose/pi4b_ros2/car_ws/src/yolo_counting_pkg/results/detected_frames'
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)  # 如果输出文件夹不存在，创建它

        # 尝试打开视频文件
        self.cap = cv2.VideoCapture(self.video_file)

        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开视频文件: {self.video_file}")
            return

        self.get_logger().info(f"视频文件成功打开: {self.video_file}")

    def detect_frame(self):
        # 读取视频帧
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error(f"无法读取视频的帧: {self.video_file}")
            self.cap.release()  # 确保释放资源
            self.destroy_timer(self.timer)
            return

        self.frame_count += 1

        if self.frame_count % 100 == 0:  # 每100帧检测一次
            # 预处理图像（调整尺寸并归一化）
            input_frame = cv2.resize(frame, (640, 640))  # 调整为640x640
            input_frame = input_frame.transpose(2, 0, 1)  # BGR to RGB
            input_frame = input_frame.astype(np.float32)
            input_frame /= 255.0  # 归一化到[0, 1]

            # 执行推理
            inputs = {self.ort_session.get_inputs()[0].name: input_frame[None, ...]}
            outputs = self.ort_session.run(None, inputs)

            # 解析检测结果
            result = outputs[0]  # 输出是一个3D数组，包含检测框的信息
            detections = result[0]  # 提取每个检测框

            people_count = 0  # 初始化人数计数
            for det in detections:
                # 提取类别、置信度及坐标
                class_id = int(det[5])  # 获取类别ID
                confidence = det[4]  # 获取物体置信度

                # 打印每个检测框的类别ID和置信度
                self.get_logger().info(f"检测框：类别 {class_id}, 置信度 {confidence:.2f}, 坐标 ({det[0]}, {det[1]}) 到 ({det[2]}, {det[3]})")

                # 不做置信度筛选，框出所有检测框当作人
                people_count += 1
                x1, y1, x2, y2 = int(det[0] * frame.shape[1]), int(det[1] * frame.shape[0]), int(det[2] * frame.shape[1]), int(det[3] * frame.shape[0])
                # 画出检测框
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 显示类别和置信度
                cv2.putText(frame, f'Person: {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 更新最大人数
            self.max_people_count = max(self.max_people_count, people_count)

            self.get_logger().info(f"视频 {self.video_file} 帧数 {self.frame_count}，检测到人数：{people_count}, 最大人数：{self.max_people_count}")

            # 保存带框的图像到指定文件夹
            output_filename = os.path.join(self.output_folder, f'{os.path.basename(self.video_file)}_frame_{self.frame_count}.jpg')
            cv2.imwrite(output_filename, frame)  # 保存图像


    def shutdown(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YoloV8OnnxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()